#!/usr/bin/env python3
"""Computer-only MQTT dispatcher and dashboard; no ROS dependency."""
from __future__ import annotations
import itertools
import json
import math
import os
import re
import socket
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "common"))
from mqtt import MqttClient
from broker import MiniBroker

cfg = json.loads((Path(__file__).with_name("device.json")).read_text())
field = json.loads((ROOT / "field.json").read_text())
LOCK = threading.RLock()
STOP = threading.Event()
seen = {"dispatcher": time.time(), "drone": 0.0, "rover": 0.0}
state = {
    "mode": "manual", "phase": "idle", "active": False,
    "rover": None, "drone": None, "drone_map": None, "drone_field": None, "drone_nav_ready": False,
    "cheburashka": "not checked", "vision": "not checked", "vlm": "not checked", "label": "not checked", "ocr": "not checked", "enemy": None, "enemy_tracking": False,
    "events": [], "event_seq": 0, "last_result": None, "last_detection": None,
}

def note(text: str) -> None:
    with LOCK:
        state["events"] = [f"{time.strftime('%H:%M:%S')} — {text}"] + state["events"][:99]

def topic(agent: str, suffix: str) -> str:
    return f"air-watch/v1/{agent}/{suffix}"

def publish(agent: str, name: str, **args) -> None:
    message = {"id": uuid.uuid4().hex, "name": name, "args": args, "time": time.time()}
    mqtt.publish(topic(agent, "cmd"), json.dumps(message), qos=1)
    note(f"→ {agent}: {name} {args}" if args else f"→ {agent}: {name}")

def on_message(_topic: str, raw: str) -> None:
    try:
        msg = json.loads(raw)
    except Exception:
        return
    agent, name, data = msg.get("agent", "?"), msg.get("name", "event"), msg.get("data", {})
    with LOCK:
        state["event_seq"] += 1
        sequence = state["event_seq"]
        if name == "status":
            if agent == "dispatcher":
                seen["dispatcher"] = time.time()
            else:
                key = "drone" if agent == cfg["drone_id"] else "rover"
                seen[key] = time.time()
                state[key] = data.get("cell")
            if agent == cfg["drone_id"]:
                state["drone_map"], state["drone_field"] = data.get("map"), data.get("field")
                state["drone_nav_ready"] = bool(data.get("nav_ready", False))
        elif name == "result":
            state["last_result"] = {"seq": sequence, **data}
            if data.get("command") == "enemy_track":
                state["enemy_tracking"] = bool(data.get("enabled"))
        elif name == "cheburashka":
            state["cheburashka"] = data.get("text", "unknown")
            state["last_detection"] = {"seq": sequence, **data}
        elif name == "vision":
            state["vision"] = data.get("text", "unknown")
        elif name == "vlm":
            state["vlm"] = data.get("text", "unknown")
        elif name == "label":
            state["label"] = data.get("text", "unknown")
        elif name == "ocr":
            state["ocr"] = data.get("text", "unknown")
        elif name == "enemy":
            state["enemy"] = data.get("cell") or data.get("text", "unknown")
    if name != "status":
        note(f"← {agent}: {name} {data.get('text', '')}")

mqtt = MqttClient(cfg["mqtt_host"], cfg["mqtt_port"], client_id=f"air-watch-computer-{os.getpid()}", on_message=on_message, logger=note)

def dispatcher_heartbeat() -> None:
    """Make dispatcher presence visible both in MQTT and terminal logs."""
    payload = json.dumps({"agent": "dispatcher", "name": "status", "data": {"online": True}})
    while True:
        mqtt.publish(topic("dispatcher", "evt"), payload, qos=1, retain=True)
        time.sleep(1.0)

def cell_xy(cell: str) -> tuple[int, int]:
    cell = cell.upper()
    return ord(cell[0]) - ord("A"), int(cell[1:]) - 1

def route_from(start: str, candidates: list[str]) -> list[str]:
    """Exact shortest Manhattan route through the candidate cells."""
    unique = list(dict.fromkeys(str(c).upper() for c in candidates))
    def distance(a: str, b: str) -> int:
        ax, ay = cell_xy(a); bx, by = cell_xy(b)
        return abs(ax - bx) + abs(ay - by)
    return list(min(itertools.permutations(unique), key=lambda p: sum(distance(a, b) for a, b in zip((start, *p), p))))

def wait_for(predicate, timeout: float, description: str) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if STOP.is_set():
            return False
        with LOCK:
            if predicate():
                return True
        time.sleep(0.15)
    note(f"AUTO timeout: {description}")
    return False

def event_seq() -> int:
    with LOCK:
        return int(state["event_seq"])

def command_result(command: str, after: int, timeout: float) -> bool:
    def completed():
        result = state.get("last_result") or {}
        return result.get("seq", 0) > after and result.get("command") == command
    if not wait_for(completed, timeout, f"result {command}"):
        return False
    with LOCK:
        result = dict(state["last_result"])
    if not result.get("success", False):
        note(f"AUTO command failed: {result.get('text', command)}")
        return False
    note(f"AUTO command confirmed: {result.get('text', command)}")
    return True

def fresh_drone_cell() -> str | None:
    with LOCK:
        if time.time() - seen["drone"] > 2.5:
            return None
        cell = state.get("drone") if state.get("drone_nav_ready") else None
    return str(cell).upper() if cell else None

def scan_cell(cell: str) -> bool:
    """Hover up to five seconds; leave early only for a positive detection."""
    started = time.monotonic()
    attempt = 0
    while time.monotonic() - started < 5.0 and not STOP.is_set():
        attempt += 1
        before = event_seq()
        note(f"AUTO {cell}: vision scan {attempt}")
        publish(cfg["drone_id"], "check_cheburashka", debug=False)
        def received():
            detection = state.get("last_detection") or {}
            return detection.get("seq", 0) > before
        wait_for(received, min(1.5, max(0.1, 5.0 - (time.monotonic() - started))), f"vision {cell}")
        with LOCK:
            detection = dict(state.get("last_detection") or {})
        if detection.get("seq", 0) > before and detection.get("detected") is True:
            note(f"AUTO {cell}: Cheburashka visible; proceeding immediately")
            return True
        remaining = 5.0 - (time.monotonic() - started)
        if remaining > 0.0:
            time.sleep(min(1.0, remaining))
    note(f"AUTO {cell}: hover complete, object not detected")
    return False

def auto_worker() -> None:
    airborne = False
    try:
        note("AUTO mission started")
        with LOCK: state["phase"] = "takeoff"
        before = event_seq(); publish(cfg["drone_id"], "takeoff")
        if not command_result("takeoff", before, 12):
            raise RuntimeError("takeoff was not confirmed")
        airborne = True
        with LOCK: state["phase"] = "localization"
        if not wait_for(lambda: fresh_drone_cell() is not None, 25, "ArUco localization after takeoff"):
            raise RuntimeError("drone cell is unknown")
        start = fresh_drone_cell()
        route = route_from(start, field.get("target_candidates", []))
        if not route:
            raise RuntimeError("target_candidates is empty")
        note(f"AUTO start cell {start}; shortest route: {' → '.join(route)}")
        for index, cell in enumerate(route, 1):
            if STOP.is_set():
                raise RuntimeError("mission aborted")
            with LOCK: state["phase"] = f"goto {cell} ({index}/{len(route)})"
            note(f"AUTO {index}/{len(route)}: command goto {cell}")
            before = event_seq(); publish(cfg["drone_id"], "goto", cell=cell)
            if not command_result("goto", before, 12):
                raise RuntimeError(f"goto {cell} was rejected")
            with LOCK: state["phase"] = f"arriving {cell}"
            if not wait_for(lambda: fresh_drone_cell() == cell, 45, f"arrival {cell}"):
                raise RuntimeError(f"arrival {cell} was not confirmed")
            note(f"AUTO {cell}: arrival confirmed by ArUco status")
            with LOCK: state["phase"] = f"scan {cell}"
            scan_cell(cell)
        with LOCK: state["phase"] = "landing"
        before = event_seq(); publish(cfg["drone_id"], "land")
        if not command_result("land", before, 20):
            raise RuntimeError("landing was not confirmed")
        airborne = False
        with LOCK:
            state.update(phase="complete", active=False, mode="manual")
        note("AUTO mission complete")
    except Exception as exc:
        note(f"AUTO mission stopped: {exc}")
        if airborne and not STOP.is_set():
            before = event_seq(); publish(cfg["drone_id"], "land")
            command_result("land", before, 20)
        with LOCK:
            state.update(phase="aborted" if STOP.is_set() else "failed", active=False, mode="manual")

def start_auto() -> None:
    with LOCK:
        if state["active"]:
            note("AUTO ignored: mission is already active")
            return
        state.update(mode="auto", phase="preparing", active=True)
    STOP.clear()
    threading.Thread(target=auto_worker, name="air-watch-auto", daemon=True).start()

def abort() -> None:
    STOP.set()
    with LOCK: state.update(mode="manual", phase="abort requested", active=False)
    publish(cfg["drone_id"], "land")
    note("AUTO abort requested; land command dispatched")

# === DEAD CODE: CHEBURASHKA -> ROVER OCR CONTINUATION ======================
# Planned auto-mode tail, deliberately NOT wired into auto_worker yet.
# Flow: positive Cheburashka detection -> the drone lands immediately (the
# remaining route cells are dropped) -> the found cell becomes the rover's
# current target -> the rover drives there, turns to face the map edge on the
# A-row side (back to row F, face to row A) -> OCR reads the label cyclically
# for two seconds -> the dispatcher parses the letter and the digit out of
# every sample and logs the cell the rover saw most often (majority vote).
# Wiring notes for later:
#   * in auto_worker, when scan_cell() returns True, break the route loop and
#     call _cheburashka_rover_ocr(detection["cell"]);
#   * the rover adapter must learn a "face" command that ends the rover
#     facing the given map-frame yaw (GoToCell goal yaw_deg / face_wall).

def _rover_cell() -> str | None:
    """Fresh rover cell from its status stream, like fresh_drone_cell()."""
    with LOCK:
        if time.time() - seen["rover"] > 2.5:
            return None
        cell = state.get("rover")
    return str(cell).upper() if cell else None

def _face_a_edge_yaw_deg() -> float:
    """Map-frame yaw (deg) looking at the map edge on the A-row side.

    Row A lies toward field +y; field_to_map has no scale, so the facing
    vector is the field +y unit vector rotated into the map frame:
    (-sin yaw, cos yaw).  The rover ends with its back to row F.
    """
    t = field.get("field_to_map", {})
    theta = float(t.get("yaw", 0.0))
    return math.degrees(math.atan2(math.cos(theta), -math.sin(theta)))

def _parse_ocr_cell(text: str) -> str | None:
    """First A-F, 1-6 cell reference found in an OCR string."""
    match = re.search(r"([A-Fa-f])\s*([1-6])", text or "")
    if not match:
        return None
    return f"{match.group(1).upper()}{int(match.group(2))}"

def _wait_ocr_result(after: int, timeout: float) -> str:
    """Wait for the next read_text result and return its message text."""
    def received():
        result = state.get("last_result") or {}
        return result.get("seq", 0) > after and result.get("command") == "read_text"
    if not wait_for(received, timeout, "read_text result"):
        return ""
    with LOCK:
        return str(dict(state["last_result"]).get("text", ""))

def _cyclic_ocr_vote(duration: float = 2.0) -> str | None:
    """DEAD CODE: sample OCR as often as possible for `duration` seconds and
    majority-vote the parsed cells (each sample is a fresh read_text)."""
    counts: dict[str, int] = {}
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and not STOP.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        before = event_seq()
        publish(cfg["rover_id"], "read_text", timeout=8.0)
        text = _wait_ocr_result(before, min(remaining, 2.5))
        cell = _parse_ocr_cell(text)
        if not cell:
            continue
        counts[cell] = counts.get(cell, 0) + 1
        note(f"AUTO rover-OCR: sample {sum(counts.values())}: {text!r} -> {cell}")
    if not counts:
        return None
    winner = max(counts, key=counts.get)
    note(f"AUTO rover-OCR: votes {counts}; winner {winner}")
    return winner

def _cheburashka_rover_ocr(cell: str) -> bool:
    """DEAD CODE: full continuation after Cheburashka is found at `cell`."""
    if not cell:
        note("AUTO rover-OCR: cheburashka cell is unknown; aborting tail")
        return False
    # 1. The drone lands at once; leftover route cells are dropped.
    with LOCK: state["phase"] = "landing (cheburashka found)"
    before = event_seq(); publish(cfg["drone_id"], "land")
    if not command_result("land", before, 20):
        note("AUTO rover-OCR: landing was not confirmed")
        return False
    # 2. The found cell becomes the rover's current target.
    note(f"AUTO rover-OCR: cheburashka cell {cell} -> rover target")
    with LOCK: state["phase"] = f"rover goto {cell}"
    before = event_seq(); publish(cfg["rover_id"], "goto", cell=cell)
    if not command_result("goto", before, 90):
        note(f"AUTO rover-OCR: rover did not accept goto {cell}")
        return False
    with LOCK: state["phase"] = f"rover arriving {cell}"
    if not wait_for(lambda: _rover_cell() == cell, 120, "rover arrival"):
        note(f"AUTO rover-OCR: arrival at {cell} was not confirmed")
        return False
    # 3. Face the A-row side edge: back to row F, face to row A.
    with LOCK: state["phase"] = f"rover face A-edge at {cell}"
    before = event_seq()
    publish(cfg["rover_id"], "face", yaw_deg=_face_a_edge_yaw_deg())
    command_result("face", before, 15)
    # 4. Cyclic OCR scan: read the label repeatedly for two seconds, then
    #    majority-vote the parsed cells (most frequent A-F/1-6 wins).
    with LOCK: state["phase"] = f"rover OCR {cell}"
    seen_cell = _cyclic_ocr_vote(2.0)
    if seen_cell is None:
        note("AUTO rover-OCR: no readable cell in the OCR samples")
        return False
    note(f"AUTO rover-OCR: rover saw cell {seen_cell}")
    with LOCK: state.update(phase="complete", active=False, mode="manual")
    return True

PAGE = '''<!doctype html><meta charset=utf-8><style>body{background:#0b1220;color:#eef;font:16px system-ui;margin:auto;max-width:1000px;padding:25px}.lamp{display:inline-block;width:12px;height:12px;border-radius:50%;background:#a33;margin:0 7px}.on{background:#2c5}.card{background:#152039;padding:15px;margin:12px 0;border-radius:12px}button,input{padding:9px;margin:4px}pre{white-space:pre-wrap}</style><h1>Air Watch</h1><div class=card><span id=ld class=lamp></span>Диспетчер <span id=la class=lamp></span>Дрон <span id=lr class=lamp></span>Ровер <label><input id=auto type=checkbox onchange="send('/mode',{auto:this.checked})"> Автоматический режим</label><button onclick="cmd('drone','emergency_stop')">Экстренная остановка</button></div><div class=card>Миссия: <b id=phase>—</b> · карта: <b id=map>—</b> · поле: <b id=field>—</b> · клетка: <b id=cell>—</b> · чебурашка: <b id=cheb>—</b></div><div class=card>Дрон <input id=dc placeholder=A1><button onclick="cmd('drone','goto')">Лететь</button><button onclick="cmd('drone','takeoff')">Взлёт</button><button onclick="cmd('drone','land')">Посадка</button><label><input id=debug type=checkbox> Публиковать debug-кадр</label><button onclick="cmd('drone','check_cheburashka')">Проверить чебурашку</button><button onclick="cmd('drone','vision_debug')">Что видит зрение?</button><button onclick="cmd('drone','navigation_diagnostics')">Координаты</button></div><div class=card><pre id=vision></pre><pre id=events></pre></div><script>const g=x=>document.getElementById(x);async function send(u,d){await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});refresh()}function cmd(a,n){send('/cmd',{agent:a,name:n,cell:g('dc').value,debug:g('debug').checked})}function lamp(id,on){g(id).className='lamp '+(on?'on':'')}async function refresh(){let x=await(await fetch('/state')).json();g('phase').textContent=x.phase;g('map').textContent=JSON.stringify(x.drone_map)||'—';g('field').textContent=JSON.stringify(x.drone_field)||'—';g('cell').textContent=x.drone||'—';g('cheb').textContent=x.cheburashka;g('vision').textContent=x.vision;g('events').textContent=x.events.join('\n');lamp('ld',true);lamp('la',x.drone_map!=null);lamp('lr',x.rover!=null);g('auto').checked=x.mode=='auto'}refresh();setInterval(refresh,600)</script>'''
# The HTML script needs a literal JavaScript "\\n", not a real newline inside
# a quoted JS string (which would stop all dashboard scripts from parsing).
PAGE = PAGE.replace("<pre id=vision></pre><pre id=events></pre>", "<div class=card>Ровер <input id=rc placeholder=A1><button onclick=\"cmd('rover','goto')\">Ехать</button><button onclick=\"cmd('rover','stop')\">Стоп</button></div><b>OpenCV</b><pre id=vision></pre><b>VLM</b><pre id=vlm></pre><b>Журнал</b><pre id=events></pre>")
PAGE = PAGE.replace("g('vision').textContent=x.vision;", "g('vision').textContent=x.vision;g('vlm').textContent=x.vlm;")
PAGE = PAGE.replace("x.events.join('\n')", "x.events.join(String.fromCharCode(10))")
PAGE += '''<script>document.getElementById('debug').addEventListener('change',function(){send('/cmd',{agent:'drone',name:'set_debug',cell:'',debug:this.checked})});const oldCmd=cmd;cmd=function(a,n){let cell=a==='rover'?g('rc').value:g('dc').value;send('/cmd',{agent:a,name:n,cell:cell,debug:g('debug').checked})}</script>'''
PAGE += '''<script>let network={dispatcher:false,drone:false,rover:false};const baseLamp=lamp;lamp=function(id,on){baseLamp(id,id==='ld'?network.dispatcher:id==='la'?network.drone:id==='lr'?network.rover:on)};setInterval(async()=>{const x=await(await fetch('/state')).json();network=x.online;lamp('ld');lamp('la');lamp('lr')},400)</script>'''

# The dashboard deliberately stays dependency-free: one self-contained page
# with the journal given most of the usable screen height.
PAGE = r'''<!doctype html>
<html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Air Watch</title>
<style>
:root{color-scheme:dark;--bg:#08111f;--panel:#111f34;--panel2:#0d192b;--line:#263957;--text:#eef5ff;--muted:#9cb0cf;--blue:#388bfd;--red:#f85149;--green:#3fb950}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.4 system-ui,sans-serif}.shell{max-width:1280px;margin:auto;padding:20px}.top{display:flex;gap:16px;align-items:center;justify-content:space-between;margin-bottom:14px}h1{font-size:23px;margin:0}.sub{color:var(--muted);font-size:13px}.network{display:flex;gap:14px;flex-wrap:wrap}.online{display:flex;gap:7px;align-items:center}.lamp{width:10px;height:10px;border-radius:50%;background:#74353a;box-shadow:0 0 0 3px #742f3628}.lamp.on{background:var(--green);box-shadow:0 0 10px #3fb950a0}.grid{display:grid;grid-template-columns:1.25fr .75fr;gap:14px}.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:15px}.title{color:var(--muted);font-size:12px;font-weight:700;letter-spacing:.08em;text-transform:uppercase;margin-bottom:10px}.actions{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.row{display:flex;gap:9px;align-items:center;flex-wrap:wrap;margin-top:10px}.row:first-of-type{margin-top:0}input{width:78px;background:#091423;color:var(--text);border:1px solid #35517b;border-radius:7px;padding:8px}button{border:0;border-radius:7px;background:#245ea8;color:white;padding:8px 11px;font-weight:600;cursor:pointer}button:hover{background:#3175c8}.danger{background:#a32d38}.danger:hover{background:#c43d49}.toggle{margin-left:auto;color:var(--muted);white-space:nowrap}.facts{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.fact{background:var(--panel2);border-radius:8px;padding:9px}.fact span{display:block;font-size:12px;color:var(--muted)}.fact b{display:block;overflow-wrap:anywhere}.vision{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:14px}pre{margin:0;white-space:pre-wrap;overflow-wrap:anywhere;font:13px/1.45 ui-monospace,Consolas,monospace;color:#dbeaff}.log{margin-top:14px}.log pre{height:min(55vh,620px);min-height:360px;overflow:auto;padding:12px;background:#070e19;border-radius:8px;border:1px solid #1d2b43}@media(max-width:760px){.shell{padding:12px}.top{align-items:flex-start;flex-direction:column}.grid,.vision{grid-template-columns:1fr}.toggle{margin-left:0}.log pre{height:52vh;min-height:300px}}
</style>
<body><main class="shell">
<header class="top"><div><h1>Air Watch</h1><div class="sub">Управление миссией и наблюдение за агентами</div></div><div class="network"><div class="online"><i id="ld" class="lamp"></i>Диспетчер</div><div class="online"><i id="la" class="lamp"></i>Дрон</div><div class="online"><i id="lr" class="lamp"></i>Ровер</div></div></header>
<div class="grid"><section class="card"><div class="title">Управление</div><div class="row"><b>Дрон</b><input id="dc" placeholder="A1"><button onclick="cmd('drone','goto')">Лететь</button><button onclick="cmd('drone','takeoff')">Взлёт</button><button onclick="cmd('drone','land')">Посадка</button></div><div class="row"><button onclick="cmd('drone','check_cheburashka')">Проверить чебурашку</button><button onclick="cmd('drone','vision_debug')">Что видит зрение</button><button onclick="cmd('drone','navigation_diagnostics')">Координаты</button><button onclick="cmd('drone','enemy_track')">Следить за противником</button><label><input id="debug" type="checkbox" style="width:auto"> Debug‑кадр</label></div><div class="row"><b>Ровер</b><input id="rc" placeholder="A1"><button onclick="cmd('rover','goto')">Ехать</button><button onclick="cmd('rover','stop')">Стоп</button><label class="toggle"><input id="auto" type="checkbox"> Автоматический режим</label><button class="danger" onclick="cmd('drone','emergency_stop')">Экстренная остановка</button></div></section>
<section class="card"><div class="title">Состояние</div><div class="facts"><div class="fact"><span>Миссия</span><b id="phase">—</b></div><div class="fact"><span>Клетка дрона</span><b id="cell">—</b></div><div class="fact"><span>Карта (ArUco)</span><b id="map">—</b></div><div class="fact"><span>Поле</span><b id="field">—</b></div><div class="fact"><span>Чебурашка</span><b id="cheb">—</b></div><div class="fact"><span>Клетка ровера</span><b id="rovercell">—</b></div><div class="fact"><span>Противник</span><b id="enemycell">—</b></div><div class="fact"><span>Слежение за противником</span><b id="enemytrk">выкл</b></div></div></section></div>
<div class="vision"><section class="card"><div class="title">OpenCV</div><pre id="vision">—</pre></section><section class="card"><div class="title">VLM</div><pre id="vlm">—</pre></section></div>
<section class="card log"><div class="title">Журнал событий</div><pre id="events">Ожидание событий…</pre></section>
</main><script>
const g=id=>document.getElementById(id);let state={};
async function send(url,data){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});if(!r.ok)throw Error('command HTTP '+r.status);await refresh()}
function cmd(agent,name){const cell=agent==='rover'?g('rc').value:g('dc').value;send('/cmd',{agent,name,cell,debug:g('debug').checked}).catch(e=>g('events').textContent='Command error: '+e+'\n'+g('events').textContent)}
function setLamp(id,on){g(id).className='lamp '+(on?'on':'')}
async function refresh(){try{state=await (await fetch('/state')).json();g('phase').textContent=state.phase||'—';g('cell').textContent=state.drone||'—';g('rovercell').textContent=state.rover||'—';g('map').textContent=state.drone_map?JSON.stringify(state.drone_map):'—';g('field').textContent=state.drone_field?JSON.stringify(state.drone_field):'—';g('cheb').textContent=state.cheburashka||'—';g('enemycell').textContent=state.enemy||'—';g('enemytrk').textContent=state.enemy_tracking?'вкл':'выкл';g('vision').textContent=state.vision||'—';g('vlm').textContent=state.vlm||'—';g('events').textContent=(state.events||[]).join('\n');g('auto').checked=state.mode==='auto';setLamp('ld',true);setLamp('la',!!state.online?.drone);setLamp('lr',!!state.online?.rover)}catch(e){g('events').textContent='Dashboard connection error: '+e}}
g('auto').addEventListener('change',()=>send('/mode',{auto:g('auto').checked}).catch(console.error));g('debug').addEventListener('change',()=>cmd('drone','set_debug'));refresh();setInterval(refresh,700);
</script></body></html>'''

# Keep the OCR control self-contained and ASCII-only: this also avoids any
# browser/terminal encoding ambiguity in the Russian dashboard labels above.
PAGE = PAGE.replace('<label class="toggle">', '<button onclick="cmd(\'rover\',\'read_text\')">OCR</button><label class="toggle">', 1)
PAGE = PAGE.replace('<section class="card log">', '<section class="card"><div class="title">Rover OCR</div><pre id="ocr">—</pre></section><section class="card log">', 1)
PAGE = PAGE.replace('</body></html>', r'''<script>
async function refreshOcr(){try{const s=await (await fetch('/state')).json();const e=document.getElementById('ocr');if(e)e.textContent=s.ocr||'—'}catch(_){}}
refreshOcr();setInterval(refreshOcr,700);
</script></body></html>''', 1)

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args): pass
    def do_GET(self):
        if self.path == "/state":
            with LOCK:
                online = {"dispatcher": True, "drone": time.time() - seen["drone"] < 2.5, "rover": time.time() - seen["rover"] < 2.5}
                body = json.dumps({**state, "events": list(state["events"]), "online": online}).encode()
            content_type = "application/json"
        else:
            body, content_type = PAGE.encode(), "text/html; charset=utf-8"
        self.send_response(200); self.send_header("Content-Type", content_type); self.end_headers(); self.wfile.write(body)
    def do_POST(self):
        data = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        if self.path == "/mode":
            start_auto() if data.get("auto") else abort()
        elif self.path == "/cmd":
            publish(cfg[data["agent"] + "_id"], data["name"], cell=data.get("cell", ""), debug=bool(data.get("debug")))
        self.send_response(204); self.end_headers()

if __name__ == "__main__":
    try:
        socket.create_connection((cfg["mqtt_host"], cfg["mqtt_port"]), timeout=1).close()
    except OSError:
        if cfg["mqtt_host"] not in ("127.0.0.1", "localhost", "192.168.1.74"):
            raise SystemExit("MQTT broker unavailable")
        MiniBroker(host="0.0.0.0", port=cfg["mqtt_port"]).start(); note("local MQTT broker started")
    mqtt.start(); assert mqtt.wait_connected(10), "MQTT unavailable"
    mqtt.subscribe("air-watch/v1/+/evt", qos=1)
    note("dispatcher active; MQTT subscription ready")
    print(f"dispatcher active; dashboard http://0.0.0.0:{cfg['http_port']}", flush=True)
    threading.Thread(target=dispatcher_heartbeat, name="dispatcher-heartbeat", daemon=True).start()
    if "--auto" in sys.argv: start_auto()
    ThreadingHTTPServer(("0.0.0.0", cfg["http_port"]), Handler).serve_forever()
