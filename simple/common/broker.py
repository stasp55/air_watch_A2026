#!/usr/bin/env python3
"""Запасной MQTT-брокер на stdlib — на случай, когда mosquitto недоступен.

Основной путь — контейнер из `simulation/compose/docker-compose.sim.mqtt.yml`.
Этот модуль нужен в двух ситуациях:
  1. на площадке нет интернета и образ eclipse-mosquitto не скачан заранее;
  2. автотесты шины (`python3 -m fleet.selftest`) — поднимается на свободном
     порту, не требует ни docker, ни сети.

Умеет ровно то, на чём стоит протокол шины: QoS 0/1, retain, LWT, wildcard-
подписки (+ и #), keepalive. Не умеет: QoS 2, TLS, persistent-сессии,
websockets. Для веб-UI это не нужно — он ходит за данными в fleet_hub по SSE,
а не в брокер напрямую.

    python3 fleet/broker_min.py --port 1883
"""
from __future__ import annotations

import argparse
import socket
import struct
import threading
import time

from mqtt import (CONNACK, CONNECT, DISCONNECT, PINGREQ, PINGRESP, PUBACK,
                   PUBLISH, SUBACK, SUBSCRIBE, MqttError, enc_len, enc_str,
                   topic_matches)

UNSUBSCRIBE, UNSUBACK = 10, 11


class _Session:
    """Одно клиентское подключение."""

    def __init__(self, sock, addr, broker):
        self.sock, self.addr, self.broker = sock, addr, broker
        self.client_id = f"{addr[0]}:{addr[1]}"
        self.subs: dict[str, int] = {}
        self.will = None
        self.lock = threading.Lock()
        self.alive = True
        self.buf = b""
        self.pid = 0

    def send(self, data: bytes):
        with self.lock:
            if self.alive:
                try:
                    self.sock.sendall(data)
                except OSError:
                    self.alive = False

    def next_pid(self) -> int:
        self.pid = self.pid % 65535 + 1
        return self.pid

    def deliver(self, topic: str, payload: bytes, qos: int, retain: bool):
        sub_qos = max((q for f, q in self.subs.items() if topic_matches(f, topic)),
                      default=None)
        if sub_qos is None:
            return
        qos = min(qos, sub_qos)
        flags = (qos << 1) | (0x01 if retain else 0)
        body = enc_str(topic)
        if qos:
            body += struct.pack("!H", self.next_pid())
        body += payload
        self.send(bytes([(PUBLISH << 4) | flags]) + enc_len(len(body)) + body)


class MiniBroker:
    def __init__(self, host="127.0.0.1", port=1883, logger=None):
        self.host, self.port = host, int(port)
        self._log = logger or (lambda m: None)
        self.sessions: list[_Session] = []
        self.retained: dict[str, tuple[bytes, int]] = {}
        self._lock = threading.RLock()
        self._server = None
        self._stopped = threading.Event()
        self._thread = None

    # ---------------------------------------------------------------- публично

    def start(self):
        self._server = socket.socket()
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.host, self.port))
        self._server.listen(64)
        self.port = self._server.getsockname()[1]     # для port=0 в тестах
        self._server.settimeout(0.5)
        self._thread = threading.Thread(target=self._accept_loop, name="broker",
                                        daemon=True)
        self._thread.start()
        self._log(f"broker: слушаю {self.host}:{self.port}")
        return self

    def stop(self):
        self._stopped.set()
        try:
            self._server.close()
        except Exception:
            pass
        with self._lock:
            sessions = list(self.sessions)
        for s in sessions:
            s.alive = False
            try:
                s.sock.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(2)

    # ------------------------------------------------------------------- цикл

    def _accept_loop(self):
        while not self._stopped.is_set():
            try:
                sock, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            sock.settimeout(1.0)
            session = _Session(sock, addr, self)
            with self._lock:
                self.sessions.append(session)
            threading.Thread(target=self._client_loop, args=(session,),
                             daemon=True).start()

    def _client_loop(self, s: _Session):
        graceful = False
        try:
            while not self._stopped.is_set() and s.alive:
                try:
                    ptype, flags, body = self._read_packet(s)
                except socket.timeout:
                    continue
                if ptype == CONNECT:
                    self._on_connect(s, body)
                elif ptype == PUBLISH:
                    self._on_publish(s, flags, body)
                elif ptype == SUBSCRIBE:
                    self._on_subscribe(s, body)
                elif ptype == UNSUBSCRIBE:
                    self._on_unsubscribe(s, body)
                elif ptype == PINGREQ:
                    s.send(bytes([PINGRESP << 4]) + enc_len(0))
                elif ptype == PUBACK:
                    pass                       # QoS1 вниз по потоку — без ретраев
                elif ptype == DISCONNECT:
                    graceful = True            # штатный выход — will НЕ публикуем
                    break
        except (OSError, MqttError):
            pass
        finally:
            s.alive = False
            with self._lock:
                if s in self.sessions:
                    self.sessions.remove(s)
            if not graceful and s.will:
                topic, payload, qos, retain = s.will
                self._log(f"broker: LWT от {s.client_id} -> {topic}")
                self._route(topic, payload, qos, retain)
            try:
                s.sock.close()
            except Exception:
                pass

    # ---------------------------------------------------------------- пакеты

    def _on_connect(self, s: _Session, body: bytes):
        pos = 0

        def take_str():
            nonlocal pos
            n = struct.unpack("!H", body[pos:pos + 2])[0]
            pos += 2
            out = body[pos:pos + n]
            pos += n
            return out

        take_str()                                    # protocol name
        pos += 1                                      # protocol level
        flags = body[pos]
        pos += 1
        pos += 2                                      # keepalive
        s.client_id = take_str().decode("utf-8", "replace") or s.client_id
        if flags & 0x04:                              # will
            wt = take_str().decode("utf-8", "replace")
            wp = take_str()
            s.will = (wt, wp, (flags >> 3) & 3, bool(flags & 0x20))
        if flags & 0x80:
            take_str()                                # username
            if flags & 0x40:
                take_str()                            # password
        s.send(bytes([CONNACK << 4]) + enc_len(2) + bytes([0, 0]))
        self._log(f"broker: подключился {s.client_id}")

    def _on_subscribe(self, s: _Session, body: bytes):
        pid = struct.unpack("!H", body[:2])[0]
        pos, granted = 2, []
        while pos < len(body):
            n = struct.unpack("!H", body[pos:pos + 2])[0]
            pos += 2
            filt = body[pos:pos + n].decode("utf-8", "replace")
            pos += n
            qos = body[pos] & 3
            pos += 1
            s.subs[filt] = qos
            granted.append(qos)
        s.send(bytes([SUBACK << 4]) + enc_len(2 + len(granted))
               + struct.pack("!H", pid) + bytes(granted))
        # Retained отдаём сразу после подписки — новый агент мгновенно узнаёт
        # состояние остальных, не дожидаясь их следующей публикации.
        with self._lock:
            snapshot = list(self.retained.items())
        for topic, (payload, qos) in snapshot:
            if any(topic_matches(f, topic) for f in s.subs):
                s.deliver(topic, payload, qos, retain=True)

    def _on_unsubscribe(self, s: _Session, body: bytes):
        pid = struct.unpack("!H", body[:2])[0]
        pos = 2
        while pos < len(body):
            n = struct.unpack("!H", body[pos:pos + 2])[0]
            pos += 2
            s.subs.pop(body[pos:pos + n].decode("utf-8", "replace"), None)
            pos += n
        s.send(bytes([UNSUBACK << 4]) + enc_len(2) + struct.pack("!H", pid))

    def _on_publish(self, s: _Session, flags: int, body: bytes):
        qos, retain = (flags >> 1) & 3, bool(flags & 0x01)
        tlen = struct.unpack("!H", body[:2])[0]
        topic = body[2:2 + tlen].decode("utf-8", "replace")
        rest = body[2 + tlen:]
        if qos:
            pid = struct.unpack("!H", rest[:2])[0]
            rest = rest[2:]
            s.send(bytes([PUBACK << 4]) + enc_len(2) + struct.pack("!H", pid))
        self._route(topic, rest, qos, retain)

    def _route(self, topic: str, payload: bytes, qos: int, retain: bool):
        if retain:
            with self._lock:
                if payload:
                    self.retained[topic] = (payload, qos)
                else:
                    self.retained.pop(topic, None)   # пустой retain = очистить
        with self._lock:
            targets = list(self.sessions)
        for sess in targets:
            sess.deliver(topic, payload, qos, retain=False)

    # ------------------------------------------------------------------ чтение

    def _read_exact(self, s: _Session, n: int) -> bytes:
        while len(s.buf) < n:
            chunk = s.sock.recv(65536)
            if not chunk:
                raise MqttError("клиент отключился")
            s.buf += chunk
        out, s.buf = s.buf[:n], s.buf[n:]
        return out

    def _read_packet(self, s: _Session):
        head = self._read_exact(s, 1)[0]
        multiplier, length = 1, 0
        while True:
            byte = self._read_exact(s, 1)[0]
            length += (byte & 0x7F) * multiplier
            if not byte & 0x80:
                break
            multiplier *= 128
            if multiplier > 128 ** 3:
                raise MqttError("испорченный Remaining Length")
        body = self._read_exact(s, length) if length else b""
        return head >> 4, head & 0x0F, body


def main():
    ap = argparse.ArgumentParser(description="Запасной MQTT-брокер шины агентов")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=1883)
    args = ap.parse_args()
    broker = MiniBroker(args.host, args.port, logger=print).start()
    print(f"Запасной брокер на {args.host}:{args.port}. Ctrl+C — выход.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        broker.stop()


if __name__ == "__main__":
    main()
