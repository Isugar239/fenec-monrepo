#!/usr/bin/env python3
"""Веб-пульт. http://<ip-pi>:8080

Походка (сервис должен быть запущен, SPI не трогаем):
  sudo python3 motor_web.py

Моторы по одному (сервис остановлен, этот процесс хозяин SPI):
  sudo systemctl stop robot.service
  sudo python3 motor_web.py
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spine_spi_test import (  # noqa: E402
    ACTUATORS,
    SpineBus,
    _board_ok,
    _global_to_spine_cmd,
    _states_from_results,
)

HOST = "0.0.0.0"
PORT = 8080
STEP = 0.10
MAX_ERROR = 0.50
LOOP_DT = 0.02
ROS_CMD = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /home/pi/pupperv3-monorepo/ros2_ws/install/local_setup.bash && "
)

_gait_lock = threading.Lock()
_gait_cache = {"value": "неизвестно", "at": 0.0}


def _robot_active() -> bool:
    active = subprocess.run(
        ["systemctl", "is-active", "robot.service"],
        capture_output=True,
        text=True,
    )
    return active.stdout.strip() == "active"


def _ros2(args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["runuser", "-u", "pi", "--", "bash", "-lc", ROS_CMD + args],
        capture_output=True,
        text=True,
        timeout=25,
    )


def gait_status(force: bool = False) -> str:
    now = time.monotonic()
    with _gait_lock:
        if not force and now - _gait_cache["at"] < 2.0:
            return _gait_cache["value"]
        _gait_cache["at"] = now
    if not _robot_active():
        value = "нет сервиса"
    else:
        result = _ros2("ros2 control list_controllers")
        plain = re.sub(r"\x1b\[[0-9;]*m", "", result.stdout)
        value = "не найден"
        for line in plain.splitlines():
            parts = line.split()
            if parts and parts[0] == "neural_controller":
                value = "active" if "active" in parts else "inactive"
                break
        if result.returncode != 0 and value == "не найден":
            value = (result.stderr or "ros2 не ответил").strip().splitlines()[-1]
    with _gait_lock:
        _gait_cache["value"] = value
        _gait_cache["at"] = time.monotonic()
    return value


def set_gait(on: bool) -> str | None:
    if not _robot_active():
        return "сначала запусти robot.service, этот пульт SPI не занимает"
    flag = "--activate" if on else "--deactivate"
    result = _ros2(f"ros2 control switch_controllers {flag} neural_controller")
    text = (result.stdout + result.stderr).strip()
    if result.returncode != 0:
        return text.splitlines()[-1] if text else "не переключился"
    with _gait_lock:
        _gait_cache["value"] = "active" if on else "inactive"
        _gait_cache["at"] = time.monotonic()
    return None


class Hold:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.bus = SpineBus()
        self.zero = [0.0] * 12
        self.fb = [0.0] * 12
        self.target = [0.0] * 12
        self.enabled = False
        self.kp = 5.5
        self.kd = 0.5
        self.note = "выключено"
        self.ce0_ok = False
        self.ce1_ok = False

    def close(self) -> None:
        self.enabled = False
        self._send(False)
        time.sleep(0.05)
        self._send(False)
        self.bus.close()

    def _send(self, torque: bool) -> None:
        flags = [1, 1, 1, 1] if torque else [0, 0, 0, 0]
        kp = [self.kp] * 12 if torque else [0.0] * 12
        kd = [self.kd] * 12 if torque else [0.0] * 12
        vel = [0.0] * 12
        c0 = _global_to_spine_cmd(0, flags, self.target, vel, kp, kd, self.zero)
        c1 = _global_to_spine_cmd(2, flags, self.target, vel, kp, kd, self.zero)
        results = self.bus.exchange_both(c0, c1)
        pos, _ = _states_from_results(results, self.zero)
        self.fb = pos
        self.ce0_ok = _board_ok(results[0])
        self.ce1_ok = _board_ok(results[1])

    def loop(self) -> None:
        while not self.stop.is_set():
            with self.lock:
                self._send(self.enabled)
                if not self.enabled:
                    self.target = list(self.fb)
            time.sleep(LOOP_DT)

    def snapshot(self) -> dict:
        with self.lock:
            motors = []
            for i, act in enumerate(ACTUATORS):
                motors.append({
                    "i": i,
                    "name": act["name"],
                    "can": act["can_ch"],
                    "id": act["can_id"],
                    "fb": round(self.fb[i], 3),
                    "target": round(self.target[i], 3),
                })
            return {
                "enabled": self.enabled,
                "note": self.note,
                "ce0_ok": self.ce0_ok,
                "ce1_ok": self.ce1_ok,
                "motors": motors,
            }

    def set_enabled(self, on: bool) -> None:
        with self.lock:
            self.enabled = on
            if on:
                self.target = list(self.fb)
                self.note = "держу текущие положения"
            else:
                self.note = "момент снят"
            self._send(on)

    def nudge(self, index: int, delta: float) -> str | None:
        if index < 0 or index >= 12:
            return "нет такого мотора"
        if abs(delta) > STEP + 1e-6:
            return "шаг больше 0.1 рад"
        with self.lock:
            if not self.enabled:
                return "сначала включи момент"
            proposed = self.target[index] + delta
            if abs(proposed - self.fb[index]) > MAX_ERROR:
                return "энкодер не едет за целью, шаг отменён"
            self.target[index] = proposed
            name = ACTUATORS[index]["name"]
            self.note = f"{name} цель {proposed:+.3f}"
            self._send(True)
        return None


PAGE = """<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Моторы Pupper</title>
<style>
  body { font-family: sans-serif; margin: 1.2rem; background: #111; color: #eee; }
  button { font-size: 1rem; margin: 0.15rem; padding: 0.45rem 0.7rem; }
  table { border-collapse: collapse; width: 100%; max-width: 52rem; }
  td, th { border-bottom: 1px solid #333; padding: 0.35rem 0.4rem; text-align: left; }
  .bad { color: #f66; }
  .ok { color: #8d8; }
</style>
</head>
<body>
<h1>Pupper</h1>
<p id="gait">походка: …</p>
<p>
  <button id="gait-on">Включить походку</button>
  <button id="gait-off">Выключить походку</button>
</p>
<h2>Моторы</h2>
<p id="note">…</p>
<p id="torque">
  <button id="on">Включить момент</button>
  <button id="off">Отпустить</button>
</p>
<table>
  <thead><tr><th>Сустав</th><th>CAN</th><th>id</th><th>энкодер</th><th>цель</th><th></th></tr></thead>
  <tbody id="rows"></tbody>
</table>
<script>
const note = document.getElementById("note");
const rows = document.getElementById("rows");
async function api(path, body) {
  const opt = body ? {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)} : {};
  const r = await fetch(path, opt);
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || r.statusText);
  return data;
}
function draw(state) {
  document.getElementById("gait").textContent = "походка: " + state.gait;
  const torque = document.getElementById("torque");
  const table = document.querySelector("table");
  if (!state.motors) {
    note.textContent = state.note;
    torque.hidden = true;
    table.hidden = true;
    rows.innerHTML = "";
    return;
  }
  torque.hidden = false;
  table.hidden = false;
  note.textContent = (state.enabled ? "момент включён" : "момент выключен")
    + " · CE0 " + (state.ce0_ok ? "ok" : "нет")
    + " · CE1 " + (state.ce1_ok ? "ok" : "нет")
    + " · " + state.note;
  rows.innerHTML = "";
  for (const m of state.motors) {
    const tr = document.createElement("tr");
    const dead = Math.abs(m.fb) < 0.001;
    tr.innerHTML = `<td>${m.name}</td><td>${m.can}</td><td>${m.id}</td>`
      + `<td class="${dead ? "bad" : "ok"}">${m.fb.toFixed(3)}</td><td>${m.target.toFixed(3)}</td>`
      + `<td><button data-i="${m.i}" data-d="-0.1">−0.1</button>`
      + `<button data-i="${m.i}" data-d="0.1">+0.1</button></td>`;
    rows.appendChild(tr);
  }
}
async function refresh() {
  try { draw(await api("/api/state")); }
  catch (e) { note.textContent = String(e); }
}
document.getElementById("gait-on").onclick = async () => {
  try { draw(await api("/api/gait", {on: true})); }
  catch (e) { document.getElementById("gait").textContent = String(e); }
};
document.getElementById("gait-off").onclick = async () => {
  try { draw(await api("/api/gait", {on: false})); }
  catch (e) { document.getElementById("gait").textContent = String(e); }
};
document.getElementById("on").onclick = async () => { await api("/api/enable", {}); refresh(); };
document.getElementById("off").onclick = async () => { await api("/api/release", {}); refresh(); };
rows.onclick = async (ev) => {
  const b = ev.target.closest("button");
  if (!b) return;
  try {
    await api("/api/nudge", {index: Number(b.dataset.i), delta: Number(b.dataset.d)});
  } catch (e) { note.textContent = String(e); }
  refresh();
};
refresh();
setInterval(refresh, 500);
</script>
</body>
</html>
"""


def main() -> None:
    if os.geteuid() != 0:
        print("Нужен root: sudo python3 motor_web.py", file=sys.stderr)
        sys.exit(1)
    active = subprocess.run(
        ["systemctl", "is-active", "robot.service"],
        capture_output=True,
        text=True,
    )
    service_up = active.stdout.strip() == "active"
    hold: Hold | None = None
    if service_up:
        print("robot.service запущен: на странице только походка, SPI не открываю.")
    else:
        hold = Hold()
        threading.Thread(target=hold.loop, daemon=True).start()
        print("robot.service остановлен: кнопки моторов живые, походка недоступна.")

    def state_payload() -> dict:
        gait = gait_status()
        if hold is None:
            return {
                "enabled": False,
                "note": "моторы у robot.service",
                "ce0_ok": False,
                "ce1_ok": False,
                "motors": None,
                "gait": gait,
            }
        snap = hold.snapshot()
        snap["gait"] = gait
        return snap

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:
            return

        def _json(self, code: int, payload: dict) -> None:
            raw = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _read_body(self) -> dict:
            n = int(self.headers.get("Content-Length", "0"))
            if n <= 0:
                return {}
            return json.loads(self.rfile.read(n).decode())

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/api/state":
                self._json(200, state_payload())
                return
            if self.path in ("/", "/index.html"):
                raw = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
                return
            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            try:
                body = self._read_body()
            except json.JSONDecodeError:
                self._json(400, {"error": "плохой json"})
                return
            if self.path == "/api/gait":
                err = set_gait(bool(body.get("on")))
                if err:
                    self._json(400, {"error": err})
                    return
                self._json(200, state_payload())
                return
            if hold is None:
                self._json(400, {"error": "моторы заняты robot.service"})
                return
            if self.path == "/api/enable":
                hold.set_enabled(True)
                self._json(200, state_payload())
                return
            if self.path == "/api/release":
                hold.set_enabled(False)
                self._json(200, state_payload())
                return
            if self.path == "/api/nudge":
                err = hold.nudge(int(body.get("index", -1)), float(body.get("delta", 0)))
                if err:
                    self._json(400, {"error": err})
                    return
                self._json(200, state_payload())
                return
            self.send_error(404)

    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Пульт: http://0.0.0.0:{PORT}  (с другого компьютера — IP этой Pi)")
    print("Ctrl+C — выйти. Момент снимается только если эта страница держала SPI.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nВыход.")
    finally:
        if hold is not None:
            hold.stop.set()
            hold.close()
        server.shutdown()


if __name__ == "__main__":
    main()
