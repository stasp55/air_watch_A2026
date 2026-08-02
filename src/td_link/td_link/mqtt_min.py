#!/usr/bin/env python3
"""Минимальный MQTT 3.1.1 клиент на одной стандартной библиотеке.

Зачем свой, а не paho: этот модуль уезжает на борт дрона и на ноутбук
диспетчера, где pip install может быть недоступен или нежелателен.
`dispatcher.py` по той же причине живёт на голом stdlib. Мост ровера
(`fleet_text_bridge_ros2`) — вендорский код, он остаётся на paho, мы его
не трогаем; протокол на проводе один и тот же.

Поддержано ровно то, что нужно шине агентов:
  CONNECT/CONNACK, SUBSCRIBE/SUBACK, PUBLISH QoS 0 и 1, PUBACK,
  retain, LWT (last will), keepalive+PINGREQ, автопереподключение
  с восстановлением подписок и досылкой неподтверждённых QoS 1.

Не поддержано осознанно: QoS 2 (шине не нужен — команды идемпотентны по
message_id, дубликаты режет DuplicateCache), TLS, сессии с clean_session=0.

    c = MqttClient("127.0.0.1", client_id="mon-01",
                   will=("fleet/v1/robots/mon-01/availability", b'{"online":false}', 1, True))
    c.on_message = lambda topic, payload: print(topic, payload)
    c.start()
    c.wait_connected(5)
    c.subscribe("fleet/v1/#")
    c.publish("fleet/v1/chat", b'{"text":"hi"}', qos=1)
"""
from __future__ import annotations

import socket
import struct
import threading
import time

CONNECT, CONNACK, PUBLISH, PUBACK = 1, 2, 3, 4
SUBSCRIBE, SUBACK, PINGREQ, PINGRESP, DISCONNECT = 8, 9, 12, 13, 14

_CONNACK_ERRORS = {
    1: "unacceptable protocol version",
    2: "identifier rejected",
    3: "server unavailable",
    4: "bad user name or password",
    5: "not authorized",
}


class MqttError(Exception):
    pass


def enc_len(n: int) -> bytes:
    """Remaining Length — 7 бит на байт, старший бит = «есть продолжение»."""
    out = bytearray()
    while True:
        byte = n % 128
        n //= 128
        if n:
            byte |= 0x80
        out.append(byte)
        if not n:
            return bytes(out)


def enc_str(s: str) -> bytes:
    b = s.encode("utf-8")
    return struct.pack("!H", len(b)) + b


def topic_matches(filt: str, topic: str) -> bool:
    """Совпадение топика с фильтром подписки (+ — один уровень, # — хвост)."""
    if filt == "#":
        return not topic.startswith("$")
    f, t = filt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            # '#' обязан быть последним и покрывает остаток, включая пустой.
            return i == len(f) - 1 and (i > 0 or not topic.startswith("$"))
        if i >= len(t):
            return False
        if part != "+" and part != t[i]:
            return False
    return len(f) == len(t)


class MqttClient:
    def __init__(self, host="127.0.0.1", port=1883, client_id=None, keepalive=30,
                 will=None, username=None, password=None,
                 on_message=None, on_connect=None, on_disconnect=None,
                 reconnect_min=1.0, reconnect_max=15.0, logger=None):
        self.host, self.port = host, int(port)
        self.client_id = client_id or f"py-{int(time.time() * 1000) % 10 ** 9}"
        self.keepalive = int(keepalive)
        self.will = will
        self.username, self.password = username, password
        self.on_message = on_message
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.reconnect_min, self.reconnect_max = reconnect_min, reconnect_max
        self._log = logger or (lambda msg: None)

        self._sock = None
        self._send_lock = threading.Lock()
        self._connected = threading.Event()
        self._stopped = threading.Event()
        self._thread = None
        self._subs: dict[str, int] = {}          # фильтр -> qos, переигрывается при реконнекте
        self._pid = 0
        self._unacked: dict[int, tuple] = {}     # QoS1 без PUBACK -> досылаем
        self._last_send = 0.0
        self._buf = b""

    # ---------------------------------------------------------------- публично

    def start(self):
        if self._thread and self._thread.is_alive():
            return self
        self._stopped.clear()
        self._thread = threading.Thread(target=self._run, name=f"mqtt-{self.client_id}",
                                        daemon=True)
        self._thread.start()
        return self

    def stop(self, timeout=2.0):
        self._stopped.set()
        try:
            if self._connected.is_set():
                self._send(bytes([DISCONNECT << 4]) + enc_len(0))
        except Exception:
            pass
        self._close_socket()
        if self._thread:
            self._thread.join(timeout)

    def wait_connected(self, timeout=5.0) -> bool:
        return self._connected.wait(timeout)

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def subscribe(self, topic_filter: str, qos: int = 1):
        self._subs[topic_filter] = qos
        if self._connected.is_set():
            self._send_subscribe({topic_filter: qos})

    def publish(self, topic: str, payload, qos: int = 0, retain: bool = False):
        """Опубликовать. Не блокирует; при обрыве QoS1 досылается после реконнекта."""
        if isinstance(payload, str):
            payload = payload.encode("utf-8")
        pid = None
        if qos:
            pid = self._next_pid()
            self._unacked[pid] = (topic, payload, retain, qos)
        try:
            self._send(self._publish_packet(topic, payload, qos, retain, pid))
        except Exception as exc:
            # Не роняем вызывающего: QoS1 уйдёт при переподключении, QoS0 теряем.
            self._log(f"publish deferred ({exc})")

    # ------------------------------------------------------------------ сборка

    def _publish_packet(self, topic, payload, qos, retain, pid, dup=False):
        flags = (0x08 if dup else 0) | (qos << 1) | (0x01 if retain else 0)
        body = enc_str(topic)
        if qos:
            body += struct.pack("!H", pid)
        body += payload
        return bytes([(PUBLISH << 4) | flags]) + enc_len(len(body)) + body

    def _connect_packet(self):
        flags = 0x02                                     # clean session
        body = enc_str("MQTT") + bytes([4])
        payload = enc_str(self.client_id)
        if self.will:
            wt, wp, wq, wr = self.will
            if isinstance(wp, str):
                wp = wp.encode("utf-8")
            flags |= 0x04 | ((wq & 3) << 3) | (0x20 if wr else 0)
            payload += enc_str(wt) + struct.pack("!H", len(wp)) + wp
        if self.username:
            flags |= 0x80
            payload += enc_str(self.username)
            if self.password is not None:
                flags |= 0x40
                payload += enc_str(self.password)
        body += bytes([flags]) + struct.pack("!H", self.keepalive) + payload
        return bytes([CONNECT << 4]) + enc_len(len(body)) + body

    def _send_subscribe(self, subs: dict):
        body = struct.pack("!H", self._next_pid())
        for filt, qos in subs.items():
            body += enc_str(filt) + bytes([qos & 3])
        self._send(bytes([(SUBSCRIBE << 4) | 0x02]) + enc_len(len(body)) + body)

    def _next_pid(self) -> int:
        self._pid = self._pid % 65535 + 1
        return self._pid

    def _send(self, data: bytes):
        with self._send_lock:
            sock = self._sock
            if sock is None:
                raise MqttError("not connected")
            sock.sendall(data)
            self._last_send = time.monotonic()

    def _close_socket(self):
        with self._send_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    # ------------------------------------------------------------------- цикл

    def _run(self):
        delay = self.reconnect_min
        while not self._stopped.is_set():
            try:
                self._connect_once()
                delay = self.reconnect_min          # успешный коннект сбрасывает backoff
                self._read_loop()
            except Exception as exc:
                if not self._stopped.is_set():
                    self._log(f"mqtt: {exc}; переподключение через {delay:.0f} с")
            was_connected = self._connected.is_set()
            self._connected.clear()
            self._close_socket()
            if was_connected and self.on_disconnect:
                self._safe(self.on_disconnect)
            if self._stopped.is_set():
                return
            self._stopped.wait(delay)
            delay = min(delay * 2, self.reconnect_max)

    def _connect_once(self):
        sock = socket.create_connection((self.host, self.port), timeout=10)
        sock.settimeout(1.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self._send_lock:
            self._sock, self._buf = sock, b""
        self._send(self._connect_packet())

        ptype, flags, body = self._read_packet(deadline=time.monotonic() + 10)
        if ptype != CONNACK or len(body) < 2:
            raise MqttError("нет CONNACK")
        if body[1] != 0:
            raise MqttError(f"CONNACK rc={body[1]}: {_CONNACK_ERRORS.get(body[1], 'unknown')}")

        self._connected.set()
        self._log(f"mqtt: подключён к {self.host}:{self.port} как {self.client_id}")
        if self._subs:
            self._send_subscribe(self._subs)
        for pid, (topic, payload, retain, qos) in list(self._unacked.items()):
            self._send(self._publish_packet(topic, payload, qos, retain, pid, dup=True))
        if self.on_connect:
            self._safe(self.on_connect)

    def _read_loop(self):
        while not self._stopped.is_set():
            try:
                ptype, flags, body = self._read_packet()
            except socket.timeout:
                self._maybe_ping()
                continue
            if ptype == PUBLISH:
                self._handle_publish(flags, body)
            elif ptype == PUBACK and len(body) >= 2:
                self._unacked.pop(struct.unpack("!H", body[:2])[0], None)
            elif ptype in (SUBACK, PINGRESP):
                pass
            self._maybe_ping()

    def _maybe_ping(self):
        if self.keepalive and time.monotonic() - self._last_send > self.keepalive / 2:
            self._send(bytes([PINGREQ << 4]) + enc_len(0))

    def _handle_publish(self, flags, body):
        qos = (flags >> 1) & 3
        tlen = struct.unpack("!H", body[:2])[0]
        topic = body[2:2 + tlen].decode("utf-8", "replace")
        rest = body[2 + tlen:]
        if qos:
            pid = struct.unpack("!H", rest[:2])[0]
            rest = rest[2:]
            self._send(bytes([PUBACK << 4]) + enc_len(2) + struct.pack("!H", pid))
        if self.on_message:
            self._safe(self.on_message, topic, rest)

    def _safe(self, fn, *args):
        # Исключение в пользовательском колбэке не должно рвать соединение.
        try:
            fn(*args)
        except Exception as exc:
            # Само логирование тоже в try: repr(UnicodeEncodeError) содержит
            # ту самую строку с непечатаемым символом, и печать репра валила
            # поток чтения повторно — уже мимо этого перехвата.
            try:
                self._log(f"callback error: {type(exc).__name__}: "
                          f"{str(exc)[:120]}")
            except Exception:
                pass

    # ------------------------------------------------------------------- чтение

    def _recv_into_buf(self):
        sock = self._sock
        if sock is None:
            raise MqttError("сокет закрыт")
        chunk = sock.recv(65536)
        if not chunk:
            raise MqttError("соединение закрыто брокером")
        self._buf += chunk

    def _read_exact(self, n, deadline=None):
        while len(self._buf) < n:
            if deadline and time.monotonic() > deadline:
                raise MqttError("таймаут чтения")
            self._recv_into_buf()
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def _read_packet(self, deadline=None):
        head = self._read_exact(1, deadline)[0]
        multiplier, length = 1, 0
        while True:
            byte = self._read_exact(1, deadline)[0]
            length += (byte & 0x7F) * multiplier
            if not byte & 0x80:
                break
            multiplier *= 128
            if multiplier > 128 ** 3:
                raise MqttError("испорченный Remaining Length")
        body = self._read_exact(length, deadline) if length else b""
        return head >> 4, head & 0x0F, body
