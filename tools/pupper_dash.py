#!/usr/bin/env python3
"""Пульт Pupper в браузере: гироскоп, суставы, хоуминг, походка, движение.

Работает рядом с robot.service (SPI не трогает, только ROS-топики):
  source /opt/ros/jazzy/setup.bash
  source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
  python3 ~/pupperv3-monorepo/tools/pupper_dash.py

С компьютера: http://10.2.0.235:8090

Движение идёт в /llm_cmd_vel (второй приоритет у cmd_vel_mux, джойстик главнее).
Команда живёт 0.4 с: если браузер перестал слать, робот встаёт.
Координаты на карте — счисление по командам скорости и курсу IMU, одометрии нет:
реальный путь из-за проскальзывания лап будет отличаться.
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, String

HOST = "0.0.0.0"
PORT = 8090
PUBLISH_HZ = 20.0
STREAM_HZ = 15.0
DRIVE_TTL = 0.4
MAX_VX = 0.75
MAX_VY = 0.5
MAX_WZ = 2.0
GOAL_VX = 0.35
GOAL_WZ = 0.9
GOAL_POS_TOL = 0.10
GOAL_YAW_TOL = 0.05
GAIT_CONTROLLER = "neural_controller"

COMPONENTS_XACRO = os.path.expanduser(
    "~/pupperv3-monorepo/ros2_ws/src/pupper_v3_description/description/components.xacro"
)
JOINT_ORDER = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]
# default_joint_pos из neural_controller/launch/config.yaml — поза, в которой политика стоит.
DEFAULT_POSE = [0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52]


def load_joint_limits() -> dict[str, dict[str, float]]:
    text = open(COMPONENTS_XACRO).read()
    out: dict[str, dict[str, float]] = {}
    for m in re.finditer(r'<joint name="(\w+)">(.*?)</joint>', text, re.S):
        params = dict(re.findall(r'<param name="(\w+)">([^<]*)</param>', m.group(2)))
        out[m.group(1)] = {
            "min": float(params["position_min"]),
            "max": float(params["position_max"]),
            "post": float(params["post_homing_position"]),
        }
    missing = [j for j in JOINT_ORDER if j not in out]
    if missing:
        raise RuntimeError(f"в {COMPONENTS_XACRO} нет суставов {missing}")
    return out


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def read_homing_journal() -> dict:
    """HOMING_RESULT текущего старта robot.service из journald."""
    since = subprocess.run(
        ["systemctl", "show", "robot.service", "-p", "ActiveEnterTimestamp", "--value"],
        capture_output=True, text=True,
    ).stdout.strip()
    args = ["journalctl", "-u", "robot.service", "--no-pager", "-o", "cat"]
    args += ["--since", since] if since else ["-b"]
    text = subprocess.run(args, capture_output=True, text=True).stdout
    rows = {}
    notes = []
    pat = re.compile(
        r"HOMING_RESULT (\w+) CAN(\d) id(\d) start=([-+\d.]+) end=([-+\d.]+) d=([-+\d.]+) (\S+) (\S+)"
    )
    for line in text.splitlines():
        m = pat.search(line)
        if m:
            rows[m.group(1)] = {
                "can": int(m.group(2)), "id": int(m.group(3)),
                "start": float(m.group(4)), "end": float(m.group(5)), "delta": float(m.group(6)),
                "status": m.group(7), "hold": m.group(8),
            }
            continue
        if "Skipping homing" in line or "HOMING_SKIPPED" in line or "стартовал в упоре" in line:
            notes.append(re.sub(r"^.*?\]: ", "", line))
    return {"since": since, "joints": rows, "notes": notes[-12:]}


class Dash(Node):
    def __init__(self) -> None:
        super().__init__("pupper_dash")
        self.lock = threading.Lock()
        self.limits = load_joint_limits()

        self.imu = None
        self.imu_times: list[float] = []
        self.joints: dict[str, tuple[float, float, float]] = {}
        self.joint_times: list[float] = []
        self.cmd_vel = (0.0, 0.0, 0.0)
        self.cmd_vel_at = 0.0
        self.mux_source = "?"
        self.logs: list[dict] = []

        self.drive = (0.0, 0.0, 0.0)
        self.drive_at = 0.0
        self.goal: dict | None = None
        self.was_publishing = False

        self.yaw_zero = 0.0
        self.pose = [0.0, 0.0]
        self.path: list[tuple[float, float]] = [(0.0, 0.0)]
        self.last_odom_t = time.monotonic()

        self.gait = "?"
        self.controllers: list[dict] = []
        self.homing = {"since": "", "joints": {}, "notes": []}

        self.pub = self.create_publisher(Twist, "/llm_cmd_vel", 10)
        self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(String, "/cmd_vel_mux/active_source", self.on_mux, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 100)

        self.list_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.switch_cli = self.create_client(SwitchController, "/controller_manager/switch_controller")

        self.create_timer(1.0 / PUBLISH_HZ, self.tick)
        self.create_timer(1.0, self.poll_controllers)
        self.create_timer(5.0, self.poll_homing)
        self.poll_homing()

    # ---- подписки ----
    def on_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        q = msg.orientation
        with self.lock:
            self.imu = {
                "q": [q.x, q.y, q.z, q.w],
                "gyro": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                "acc": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                "at": now,
            }
            self.imu_times.append(now)
            self.imu_times = [t for t in self.imu_times if now - t < 1.0]

    def on_joints(self, msg: JointState) -> None:
        now = time.monotonic()
        with self.lock:
            for i, name in enumerate(msg.name):
                eff = msg.effort[i] if i < len(msg.effort) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                self.joints[name] = (msg.position[i], vel, eff)
            self.joint_times.append(now)
            self.joint_times = [t for t in self.joint_times if now - t < 1.0]

    def on_cmd_vel(self, msg: Twist) -> None:
        with self.lock:
            self.cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.cmd_vel_at = time.monotonic()

    def on_mux(self, msg: String) -> None:
        with self.lock:
            self.mux_source = msg.data

    def on_log(self, msg: Log) -> None:
        if msg.level < Log.WARN or msg.name == "pupper_dash":
            return
        level = {Log.WARN: "WARN", Log.ERROR: "ERROR", Log.FATAL: "FATAL"}.get(msg.level, str(msg.level))
        with self.lock:
            self.logs.append({"t": time.strftime("%H:%M:%S"), "level": level, "node": msg.name,
                              "text": msg.msg[:300]})
            self.logs = self.logs[-60:]

    # ---- курс и счисление ----
    def yaw_raw(self) -> float | None:
        if self.imu is None:
            return None
        return quat_to_rpy(*self.imu["q"])[2]

    def yaw(self) -> float | None:
        raw = self.yaw_raw()
        return None if raw is None else wrap(raw - self.yaw_zero)

    def reset_odom(self) -> None:
        with self.lock:
            raw = self.yaw_raw()
            self.yaw_zero = raw if raw is not None else 0.0
            self.pose = [0.0, 0.0]
            self.path = [(0.0, 0.0)]
            self.goal = None

    # ---- управление ----
    def set_drive(self, vx: float, vy: float, wz: float) -> None:
        with self.lock:
            self.goal = None
            self.drive = (clamp(vx, MAX_VX), clamp(vy, MAX_VY), clamp(wz, MAX_WZ))
            self.drive_at = time.monotonic()

    def set_goal(self, goal: dict) -> str | None:
        with self.lock:
            yaw = self.yaw()
            if yaw is None:
                return "нет данных IMU, цели по курсу невозможны"
            kind = goal.get("type")
            if kind == "point":
                self.goal = {"type": "point", "x": float(goal["x"]), "y": float(goal["y"])}
            elif kind == "forward":
                d = float(goal["distance"])
                self.goal = {"type": "point",
                             "x": self.pose[0] + d * math.cos(yaw),
                             "y": self.pose[1] + d * math.sin(yaw)}
            elif kind == "turn":
                self.goal = {"type": "yaw", "yaw": wrap(yaw + math.radians(float(goal["degrees"])))}
            else:
                return f"неизвестная цель {kind}"
            self.goal["started"] = time.monotonic()
            self.drive_at = 0.0
        return None

    def stop(self) -> None:
        with self.lock:
            self.goal = None
            self.drive_at = 0.0

    def goal_command(self, yaw: float) -> tuple[float, float, float] | None:
        g = self.goal
        if g is None:
            return None
        if g["type"] == "yaw":
            err = wrap(g["yaw"] - yaw)
            if abs(err) < GOAL_YAW_TOL:
                g["done"] = True
                return None
            return 0.0, 0.0, clamp(2.0 * err, GOAL_WZ)
        dx, dy = g["x"] - self.pose[0], g["y"] - self.pose[1]
        dist = math.hypot(dx, dy)
        if dist < GOAL_POS_TOL:
            g["done"] = True
            return None
        err = wrap(math.atan2(dy, dx) - yaw)
        wz = clamp(2.0 * err, GOAL_WZ)
        if abs(err) > 0.35:
            return 0.0, 0.0, wz
        return min(GOAL_VX, 0.8 * dist + 0.08), 0.0, wz

    def tick(self) -> None:
        now = time.monotonic()
        with self.lock:
            yaw = self.yaw()
            dt = now - self.last_odom_t
            self.last_odom_t = now
            gait_on = self.gait == "active"
            vx, vy, wz = self.cmd_vel if now - self.cmd_vel_at < 0.5 else (0.0, 0.0, 0.0)
            if gait_on and yaw is not None and (vx or vy):
                self.pose[0] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
                self.pose[1] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
                last = self.path[-1]
                if math.hypot(self.pose[0] - last[0], self.pose[1] - last[1]) > 0.02:
                    self.path.append((self.pose[0], self.pose[1]))
                    self.path = self.path[-2000:]

            cmd = None
            if self.goal is not None and yaw is not None:
                cmd = self.goal_command(yaw)
                if cmd is None:
                    self.goal = None
            elif now - self.drive_at < DRIVE_TTL:
                cmd = self.drive

        if cmd is not None:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = cmd
            self.pub.publish(msg)
            self.was_publishing = True
        elif self.was_publishing:
            self.pub.publish(Twist())
            self.was_publishing = False

    # ---- контроллеры ----
    def poll_controllers(self) -> None:
        if not self.list_cli.service_is_ready():
            with self.lock:
                self.gait = "controller_manager не отвечает"
                self.controllers = []
            return
        fut = self.list_cli.call_async(ListControllers.Request())
        fut.add_done_callback(self._on_list)

    def _on_list(self, fut) -> None:
        res = fut.result()
        ctrls = [{"name": c.name, "state": c.state} for c in res.controller]
        gait = next((c["state"] for c in ctrls if c["name"] == GAIT_CONTROLLER), "не найден")
        with self.lock:
            self.controllers = ctrls
            self.gait = gait

    def switch_gait(self, on: bool) -> str | None:
        if not self.switch_cli.service_is_ready():
            return "controller_manager не отвечает (robot.service запущен?)"
        req = SwitchController.Request()
        if on:
            req.activate_controllers = [GAIT_CONTROLLER]
            req.deactivate_controllers = [c["name"] for c in self.controllers
                                          if c["state"] == "active" and c["name"] != GAIT_CONTROLLER
                                          and "broadcaster" not in c["name"]]
        else:
            req.deactivate_controllers = [GAIT_CONTROLLER]
        req.strictness = SwitchController.Request.BEST_EFFORT
        fut = self.switch_cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(5.0):
            return "switch_controller не ответил за 5 с"
        if not fut.result().ok:
            return "controller_manager отказал в переключении"
        self.poll_controllers()
        return None

    def estop(self) -> None:
        self.stop()
        self.estop_pub.publish(Empty())

    def poll_homing(self) -> None:
        data = read_homing_journal()
        with self.lock:
            self.homing = data

    # ---- снимок для браузера ----
    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            imu = None
            if self.imu is not None:
                r, p, y = quat_to_rpy(*self.imu["q"])
                qx, qy, qz, qw = self.imu["q"]
                # Мировая ось z (вверх) в осях корпуса: как политика видит гравитацию.
                gx = 2 * (qx * qz - qw * qy)
                gy = 2 * (qy * qz + qw * qx)
                gz = 1 - 2 * (qx * qx + qy * qy)
                imu = {
                    "q": self.imu["q"], "gyro": self.imu["gyro"], "acc": self.imu["acc"],
                    "roll": math.degrees(r), "pitch": math.degrees(p), "yaw": math.degrees(y),
                    "yaw_rel": math.degrees(wrap(y - self.yaw_zero)),
                    "up_in_body": [gx, gy, gz],
                    "age": now - self.imu["at"], "hz": len(self.imu_times),
                }
            joints = []
            for idx, name in enumerate(JOINT_ORDER):
                pos, vel, eff = self.joints.get(name, (float("nan"), 0.0, 0.0))
                lim = self.limits[name]
                h = self.homing["joints"].get(name)
                joints.append({
                    "name": name, "pos": pos, "vel": vel, "eff": eff,
                    "min": lim["min"], "max": lim["max"], "post": lim["post"],
                    "default": DEFAULT_POSE[idx], "homing": h,
                })
            goal = None
            if self.goal is not None:
                goal = {k: v for k, v in self.goal.items() if k != "started"}
                if "yaw" in goal:
                    goal["yaw_deg"] = math.degrees(wrap(goal["yaw"]))
            driving = now - self.drive_at < DRIVE_TTL
            return {
                "imu": imu,
                "joints": joints,
                "joint_hz": len(self.joint_times),
                "gait": self.gait,
                "controllers": self.controllers,
                "mux": self.mux_source,
                "cmd_vel": list(self.cmd_vel) if now - self.cmd_vel_at < 0.5 else [0, 0, 0],
                "sending": list(self.drive) if driving else None,
                "goal": goal,
                "pose": list(self.pose),
                "path": self.path[-400:],
                "homing": {"since": self.homing["since"], "notes": self.homing["notes"]},
                "logs": self.logs[-25:],
            }


PAGE = r"""<!DOCTYPE html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pupper — пульт</title>
<style>
:root { --bg:#0f1115; --card:#181b22; --line:#2a2f3a; --fg:#e6e8ee; --dim:#8a93a6;
        --ok:#5fd38d; --warn:#f2c14e; --bad:#ff6b6b; --acc:#6aa9ff; }
* { box-sizing:border-box; }
body { margin:0; font:14px/1.4 system-ui, sans-serif; background:var(--bg); color:var(--fg); }
header { display:flex; gap:1rem; align-items:center; padding:.7rem 1rem; border-bottom:1px solid var(--line);
         position:sticky; top:0; background:var(--bg); z-index:5; flex-wrap:wrap; }
header h1 { font-size:1.1rem; margin:0 .5rem 0 0; }
.pill { padding:.2rem .6rem; border-radius:999px; background:var(--card); border:1px solid var(--line); }
.ok { color:var(--ok); } .warn { color:var(--warn); } .bad { color:var(--bad); } .dim { color:var(--dim); }
main { display:grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr)); gap:1rem; padding:1rem; }
.card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:.8rem 1rem; }
.card h2 { font-size:1rem; margin:0 0 .6rem; }
button { background:#232835; color:var(--fg); border:1px solid var(--line); border-radius:7px;
         padding:.45rem .8rem; font-size:.95rem; cursor:pointer; }
button:hover { border-color:var(--acc); }
button.big { padding:.7rem 1rem; min-width:3.2rem; font-size:1.05rem; }
button.stop { background:#3a1d22; border-color:#6b2a33; }
button.estop { background:#7a1020; border-color:#b0203a; font-weight:700; }
button.on { background:#1d3a2a; border-color:#2f6b47; }
button.held { background:var(--acc); color:#000; }
canvas { background:#0b0d11; border:1px solid var(--line); border-radius:8px; width:100%; display:block; }
table { width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }
td, th { padding:.25rem .35rem; border-bottom:1px solid var(--line); text-align:left; white-space:nowrap; }
th { color:var(--dim); font-weight:500; }
.bar { position:relative; height:12px; background:#0b0d11; border:1px solid var(--line); border-radius:4px; min-width:140px; }
.bar i { position:absolute; top:-2px; bottom:-2px; width:3px; border-radius:2px; }
.row { display:flex; gap:.5rem; align-items:center; flex-wrap:wrap; margin:.35rem 0; }
.grid3 { display:grid; grid-template-columns: repeat(3, auto); gap:.35rem; justify-content:start; }
input[type=number] { width:5.5rem; background:#0b0d11; color:var(--fg); border:1px solid var(--line); border-radius:6px; padding:.35rem; }
input[type=range] { width:10rem; }
.kv { display:grid; grid-template-columns: auto 1fr; gap:.15rem .8rem; font-variant-numeric:tabular-nums; }
.log { max-height:220px; overflow:auto; font:12px/1.35 ui-monospace, monospace; }
.log div { border-bottom:1px solid #1f232c; padding:.15rem 0; }
.hint { color:var(--dim); font-size:.85rem; }
</style></head>
<body>
<header>
  <h1>Pupper</h1>
  <span class="pill" id="conn">связь…</span>
  <span class="pill" id="gait">походка: ?</span>
  <span class="pill" id="mux">источник cmd_vel: ?</span>
  <span class="pill" id="rates">IMU ? Гц · суставы ? Гц</span>
  <span style="flex:1"></span>
  <button class="on" id="gaitOn">Включить походку</button>
  <button id="gaitOff">Выключить походку</button>
  <button class="estop" id="estop">E-STOP</button>
</header>
<main>
  <section class="card">
    <h2>Гироскоп / IMU — как робот себя видит</h2>
    <canvas id="body3d" height="300"></canvas>
    <div class="row" style="margin-top:.6rem">
      <canvas id="horizon" width="180" height="180" style="width:180px"></canvas>
      <div class="kv" id="imuKv"></div>
    </div>
    <p class="hint">Модель повёрнута так, как её видит БНО после поправки монтажа.
    Положи робота ровно: крен и тангаж должны быть около 0°, «верх» — (0, 0, 1).
    Наклони нос вниз: тангаж должен стать положительным, вектор «верх» уйти в +x.</p>
    <canvas id="gyroChart" height="120"></canvas>
    <p class="hint">Угловая скорость, рад/с: <span style="color:#ff6b6b">x (крен)</span>
    <span style="color:#5fd38d">y (тангаж)</span> <span style="color:#6aa9ff">z (рыскание)</span></p>
  </section>

  <section class="card">
    <h2>Движение</h2>
    <div class="row">
      <div class="grid3">
        <button class="big" data-k="q" title="Q">⟲</button>
        <button class="big" data-k="w" title="W">▲</button>
        <button class="big" data-k="e" title="E">⟳</button>
        <button class="big" data-k="a" title="A">◀</button>
        <button class="big stop" id="stopBtn" title="Пробел">■</button>
        <button class="big" data-k="d" title="D">▶</button>
        <span></span>
        <button class="big" data-k="s" title="S">▼</button>
        <span></span>
      </div>
      <div style="margin-left:1rem">
        <div class="row">скорость <input type="range" id="speed" min="0.1" max="1" step="0.05" value="0.4"><span id="speedV"></span></div>
        <div class="hint">Держи кнопку или клавиши W/S — вперёд/назад,<br>A/D — вбок, Q/E — поворот, пробел — стоп.</div>
        <div class="kv" id="driveKv" style="margin-top:.4rem"></div>
      </div>
    </div>
    <div class="row">
      <button id="fwdBtn">Пройти вперёд</button><input type="number" id="fwdM" value="0.5" step="0.1"> м
      <button id="turnBtn">Повернуть на</button><input type="number" id="turnDeg" value="90" step="15"> °
    </div>
    <canvas id="map" height="360"></canvas>
    <div class="row">
      <span class="hint">Кликни по карте — робот пойдёт в эту точку. Сетка 0.5 м, старт в центре, нос — вверх.</span>
      <button id="resetOdom">Сбросить карту и курс</button>
    </div>
    <p class="hint">Координаты — оценка по командам скорости и курсу IMU; одометрии нет, лапы проскальзывают.</p>
  </section>

  <section class="card" style="grid-column: 1 / -1">
    <h2>Суставы</h2>
    <table><thead><tr><th>сустав</th><th>угол, рад</th><th>диапазон <span class="dim">(| — поза политики, ◆ — после хоуминга)</span></th>
    <th>момент, Н·м</th><th>хоуминг (CAN id · Δ · итог)</th></tr></thead><tbody id="joints"></tbody></table>
    <div class="log" id="homingNotes" style="margin-top:.5rem"></div>
  </section>

  <section class="card" style="grid-column: 1 / -1">
    <h2>Предупреждения ROS</h2>
    <div class="log" id="logs"></div>
  </section>
</main>
<script>
const $ = id => document.getElementById(id);
let S = null, gyroHist = [];

async function post(path, body) {
  const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body||{})});
  const d = await r.json();
  if (!r.ok) { alert(d.error || r.statusText); throw new Error(d.error); }
  return d;
}

// ---------- поток состояния ----------
function connect() {
  const es = new EventSource("/events");
  es.onopen = () => { $("conn").textContent = "связь есть"; $("conn").className = "pill ok"; };
  es.onerror = () => { $("conn").textContent = "нет связи с роботом"; $("conn").className = "pill bad"; };
  es.onmessage = ev => { S = JSON.parse(ev.data); draw(); };
}
connect();

function fmt(v, n=2) { return (v>=0?"+":"") + v.toFixed(n); }

function draw() {
  const g = $("gait");
  g.textContent = "походка: " + S.gait;
  g.className = "pill " + (S.gait === "active" ? "ok" : S.gait === "inactive" ? "warn" : "bad");
  $("mux").textContent = "источник cmd_vel: " + S.mux;
  const imuHz = S.imu ? S.imu.hz : 0;
  $("rates").textContent = `IMU ${imuHz} Гц · суставы ${S.joint_hz} Гц`;
  $("rates").className = "pill " + (imuHz > 20 && S.joint_hz > 20 ? "ok" : "bad");
  drawImu(); drawDrive(); drawMap(); drawJoints(); drawLogs();
}

// ---------- IMU ----------
function qmat(q) {
  const [x,y,z,w] = q;
  return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
          [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
          [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]];
}
function mv(m, v) { return [0,1,2].map(i => m[i][0]*v[0]+m[i][1]*v[1]+m[i][2]*v[2]); }

function drawImu() {
  const kv = $("imuKv");
  if (!S.imu) { kv.innerHTML = '<span class="bad">нет данных /imu_sensor_broadcaster/imu</span>'; return; }
  const m = S.imu;
  const up = m.up_in_body;
  const ageCls = m.age < 0.1 ? "ok" : "bad";
  kv.innerHTML = `
    <span>крен (roll)</span><b>${fmt(m.roll,1)}°</b>
    <span>тангаж (pitch)</span><b>${fmt(m.pitch,1)}°</b>
    <span>курс (yaw)</span><b>${fmt(m.yaw,1)}° <span class="dim">(от старта карты ${fmt(m.yaw_rel,1)}°)</span></b>
    <span>«верх» в осях робота</span><b>${up.map(v=>v.toFixed(2)).join(", ")}</b>
    <span>гиро, рад/с</span><b>${m.gyro.map(v=>fmt(v,2)).join(" ")}</b>
    <span>кватернион xyzw</span><b class="dim">${m.q.map(v=>v.toFixed(3)).join(" ")}</b>
    <span>возраст пакета</span><b class="${ageCls}">${(m.age*1000).toFixed(0)} мс</b>`;

  // 3D-корпус: мировая система, камера сбоку-сверху. Курс убираем, чтобы робот смотрел вправо.
  const c = $("body3d"), ctx = c.getContext("2d");
  c.width = c.clientWidth; const W = c.width, H = c.height;
  ctx.clearRect(0,0,W,H);
  const yaw = m.yaw * Math.PI/180;
  const qy = [0,0,Math.sin(-yaw/2),Math.cos(-yaw/2)];
  const R = mulM(qmat(qy), qmat(m.q));
  const cam = v => { // вид: азимут -35°, возвышение 25°
    const az = -35*Math.PI/180, el = 25*Math.PI/180;
    const x1 = v[0]*Math.cos(az) - v[1]*Math.sin(az), y1 = v[0]*Math.sin(az) + v[1]*Math.cos(az);
    const sx = y1, sy = v[2]*Math.cos(el) - x1*Math.sin(el);
    return [W/2 - sx*110, H/2 - sy*110];
  };
  // пол
  ctx.strokeStyle = "#222835"; ctx.lineWidth = 1;
  for (let i=-3;i<=3;i++) {
    line(ctx, cam([i*0.4,-1.2,-0.5]), cam([i*0.4,1.2,-0.5]));
    line(ctx, cam([-1.2,i*0.4,-0.5]), cam([1.2,i*0.4,-0.5]));
  }
  const L=0.9, Wd=0.45, Hh=0.22;
  const corners = [];
  for (const sx of [-1,1]) for (const sy of [-1,1]) for (const sz of [-1,1])
    corners.push(mv(R, [sx*L/2, sy*Wd/2, sz*Hh/2]));
  const P = corners.map(cam);
  const faces = [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]];
  const depth = f => f.reduce((s,i)=>s+corners[i][0]*Math.cos(-35*Math.PI/180)-corners[i][1]*Math.sin(-35*Math.PI/180),0);
  faces.sort((a,b)=>depth(b)-depth(a));
  for (const f of faces) {
    ctx.beginPath(); f.forEach((i,k)=> k?ctx.lineTo(...P[i]):ctx.moveTo(...P[i])); ctx.closePath();
    const front = f.every(i => corners[i] && [4,5,6,7].includes(i));
    ctx.fillStyle = front ? "rgba(106,169,255,.35)" : "rgba(160,170,190,.18)";
    ctx.strokeStyle = "#8a93a6"; ctx.fill(); ctx.stroke();
  }
  // лапы
  for (const [sx,sy] of [[1,1],[1,-1],[-1,1],[-1,-1]]) {
    const top = mv(R,[sx*L*0.42, sy*Wd*0.55, -Hh/2]);
    const bot = mv(R,[sx*L*0.42, sy*Wd*0.55, -Hh/2-0.3]);
    ctx.strokeStyle = sx>0 ? "#f2c14e" : "#8a93a6"; ctx.lineWidth = 4;
    line(ctx, cam(top), cam(bot)); ctx.lineWidth = 1;
  }
  const o = mv(R,[0,0,0]);
  arrow(ctx, cam(o), cam(mv(R,[0.75,0,0])), "#ff6b6b", "x вперёд");
  arrow(ctx, cam(o), cam(mv(R,[0,0.5,0])), "#5fd38d", "y влево");
  arrow(ctx, cam(o), cam(mv(R,[0,0,0.5])), "#6aa9ff", "z вверх");
  arrow(ctx, cam([0,0,0]), cam([0,0,-0.7]), "#bbbbbb", "гравитация");

  // авиагоризонт
  const h = $("horizon"), hc = h.getContext("2d"), r = 88;
  hc.save(); hc.clearRect(0,0,180,180); hc.translate(90,90);
  hc.beginPath(); hc.arc(0,0,r,0,2*Math.PI); hc.clip();
  hc.rotate(-m.roll*Math.PI/180);
  const off = m.pitch*2;
  hc.fillStyle = "#274a7a"; hc.fillRect(-200,-400+off,400,400);
  hc.fillStyle = "#6b4a2a"; hc.fillRect(-200,off,400,400);
  hc.strokeStyle="#fff"; hc.beginPath(); hc.moveTo(-200,off); hc.lineTo(200,off); hc.stroke();
  for (let d=-30; d<=30; d+=10) if (d) { hc.beginPath(); hc.moveTo(-15,off-d*2); hc.lineTo(15,off-d*2); hc.stroke(); }
  hc.restore();
  hc.strokeStyle="#f2c14e"; hc.lineWidth=3; hc.beginPath(); hc.moveTo(50,90); hc.lineTo(80,90); hc.moveTo(100,90); hc.lineTo(130,90); hc.stroke();
  hc.lineWidth=1;

  // график гиро
  gyroHist.push(m.gyro); if (gyroHist.length > 225) gyroHist.shift();
  const gc = $("gyroChart"), g2 = gc.getContext("2d");
  gc.width = gc.clientWidth; const GW = gc.width, GH = gc.height;
  g2.clearRect(0,0,GW,GH);
  const maxAbs = Math.max(0.5, ...gyroHist.flat().map(Math.abs));
  g2.strokeStyle="#2a2f3a"; line(g2,[0,GH/2],[GW,GH/2]);
  ["#ff6b6b","#5fd38d","#6aa9ff"].forEach((col,k) => {
    g2.strokeStyle = col; g2.beginPath();
    gyroHist.forEach((v,i) => { const x=i*GW/225, y=GH/2 - v[k]/maxAbs*(GH/2-4); i?g2.lineTo(x,y):g2.moveTo(x,y); });
    g2.stroke();
  });
  g2.fillStyle="#8a93a6"; g2.fillText(`±${maxAbs.toFixed(2)}`, 4, 12);
}
function mulM(a,b){ return [0,1,2].map(i=>[0,1,2].map(j=>a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j])); }
function line(ctx,a,b){ ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); }
function arrow(ctx,a,b,col,label){
  ctx.strokeStyle=col; ctx.fillStyle=col; ctx.lineWidth=2; line(ctx,a,b);
  const ang=Math.atan2(b[1]-a[1],b[0]-a[0]);
  ctx.beginPath(); ctx.moveTo(...b);
  ctx.lineTo(b[0]-9*Math.cos(ang-0.4), b[1]-9*Math.sin(ang-0.4));
  ctx.lineTo(b[0]-9*Math.cos(ang+0.4), b[1]-9*Math.sin(ang+0.4)); ctx.fill();
  ctx.lineWidth=1; ctx.fillText(label, b[0]+5, b[1]-5);
}

// ---------- движение ----------
const held = new Set();
const keyMap = {w:"w", s:"s", a:"a", d:"d", q:"q", e:"e", "ц":"w", "ы":"s", "ф":"a", "в":"d", "й":"q", "у":"e"};
function speed() { return parseFloat($("speed").value); }
function driveVec() {
  const k = speed();
  let vx=0, vy=0, wz=0;
  if (held.has("w")) vx += 0.75*k; if (held.has("s")) vx -= 0.75*k;
  if (held.has("a")) vy += 0.5*k;  if (held.has("d")) vy -= 0.5*k;
  if (held.has("q")) wz += 2.0*k;  if (held.has("e")) wz -= 2.0*k;
  return [vx,vy,wz];
}
setInterval(() => {
  document.querySelectorAll("[data-k]").forEach(b => b.classList.toggle("held", held.has(b.dataset.k)));
  if (!held.size) return;
  const [vx,vy,wz] = driveVec();
  fetch("/api/drive", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({vx,vy,wz})});
}, 100);
document.addEventListener("keydown", ev => {
  if (ev.target.tagName === "INPUT") return;
  const k = keyMap[ev.key.toLowerCase()];
  if (k) { held.add(k); ev.preventDefault(); }
  if (ev.key === " ") { held.clear(); post("/api/stop"); ev.preventDefault(); }
});
document.addEventListener("keyup", ev => { const k = keyMap[ev.key.toLowerCase()]; if (k) held.delete(k); });
window.addEventListener("blur", () => held.clear());
document.querySelectorAll("[data-k]").forEach(b => {
  const k = b.dataset.k;
  b.addEventListener("pointerdown", ev => { held.add(k); b.setPointerCapture(ev.pointerId); });
  b.addEventListener("pointerup", () => held.delete(k));
  b.addEventListener("pointercancel", () => held.delete(k));
});
$("stopBtn").onclick = () => { held.clear(); post("/api/stop"); };
$("speed").oninput = () => $("speedV").textContent = " " + Math.round(speed()*100) + "%";
$("speed").oninput();
$("fwdBtn").onclick = () => post("/api/goal", {type:"forward", distance: parseFloat($("fwdM").value)});
$("turnBtn").onclick = () => post("/api/goal", {type:"turn", degrees: parseFloat($("turnDeg").value)});
$("resetOdom").onclick = () => post("/api/reset_odom");
$("gaitOn").onclick = () => post("/api/gait", {on:true});
$("gaitOff").onclick = () => post("/api/gait", {on:false});
$("estop").onclick = () => { held.clear(); post("/api/estop"); };

function drawDrive() {
  const s = S.sending, c = S.cmd_vel;
  let goal = "нет";
  if (S.goal) goal = S.goal.type === "yaw" ? `поворот к курсу ${S.goal.yaw_deg.toFixed(0)}°`
                                           : `точка (${S.goal.x.toFixed(2)}, ${S.goal.y.toFixed(2)}) м`;
  $("driveKv").innerHTML = `
    <span>пульт шлёт</span><b>${s ? s.map(v=>fmt(v,2)).join(" ") : '<span class="dim">ничего</span>'}</b>
    <span>робот получает /cmd_vel</span><b>vx ${fmt(c[0])} vy ${fmt(c[1])} wz ${fmt(c[2])}</b>
    <span>цель</span><b>${goal}</b>
    <span>оценка позиции</span><b>x ${fmt(S.pose[0])} y ${fmt(S.pose[1])} м</b>
    ${S.gait !== "active" ? '<span></span><b class="warn">походка выключена — робот не пойдёт</b>' : ""}`;
}

const MAP_SCALE = 80; // пикселей на метр
function mapXY(c, x, y) { return [c.width/2 - y*MAP_SCALE, c.height/2 - x*MAP_SCALE]; }
function drawMap() {
  const c = $("map"), ctx = c.getContext("2d");
  c.width = c.clientWidth; const W=c.width, H=c.height;
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle = "#1c212b";
  for (let m=-10; m<=10; m+=0.5) {
    const [x0,y0] = mapXY(c, m, 0), [x1,y1] = mapXY(c, 0, m);
    line(ctx,[0,y0],[W,y0]); line(ctx,[x1,0],[x1,H]);
  }
  ctx.strokeStyle = "#2f3645"; const [ox,oy] = mapXY(c,0,0); line(ctx,[0,oy],[W,oy]); line(ctx,[ox,0],[ox,H]);
  ctx.strokeStyle = "#6aa9ff"; ctx.beginPath();
  S.path.forEach((p,i) => { const [x,y] = mapXY(c,p[0],p[1]); i?ctx.lineTo(x,y):ctx.moveTo(x,y); }); ctx.stroke();
  if (S.goal && S.goal.type === "point") {
    const [gx,gy] = mapXY(c, S.goal.x, S.goal.y);
    ctx.strokeStyle="#f2c14e"; ctx.beginPath(); ctx.arc(gx,gy,8,0,2*Math.PI); ctx.stroke();
    line(ctx,[gx-12,gy],[gx+12,gy]); line(ctx,[gx,gy-12],[gx,gy+12]);
  }
  const yaw = S.imu ? S.imu.yaw_rel*Math.PI/180 : 0;
  const [rx,ry] = mapXY(c, S.pose[0], S.pose[1]);
  ctx.save(); ctx.translate(rx,ry); ctx.rotate(-yaw);
  ctx.fillStyle = "#5fd38d"; ctx.beginPath(); ctx.moveTo(0,-16); ctx.lineTo(9,10); ctx.lineTo(-9,10); ctx.closePath(); ctx.fill();
  ctx.restore();
}
$("map").addEventListener("click", ev => {
  const c = $("map"), r = c.getBoundingClientRect();
  const px = (ev.clientX - r.left) * c.width / r.width, py = (ev.clientY - r.top) * c.height / r.height;
  const x = (c.height/2 - py) / MAP_SCALE, y = (c.width/2 - px) / MAP_SCALE;
  post("/api/goal", {type:"point", x, y});
});

// ---------- суставы ----------
const NAMES = {"front_r":"пер. правая","front_l":"пер. левая","back_r":"зад. правая","back_l":"зад. левая"};
const PART = {"1":"abad","2":"hip","3":"knee"};
function drawJoints() {
  const rows = S.joints.map(j => {
    const m = j.name.match(/leg_(\w+_\w)_(\d)/);
    const label = `${NAMES[m[1]]} · ${PART[m[2]]}`;
    const span = j.max - j.min, pct = v => Math.max(0, Math.min(100, (v - j.min)/span*100));
    const h = j.homing;
    let hcell = '<span class="dim">нет данных</span>', bad = false;
    if (h) {
      bad = h.hold !== "ДЕРЖИТ";
      hcell = `CAN${h.can} id${h.id} · Δ ${fmt(h.delta)} · <span class="${bad?"bad":"ok"}">${h.status} ${h.hold}</span>`;
    }
    const nan = j.pos === null;
    const outside = !nan && (j.pos < j.min - 0.05 || j.pos > j.max + 0.05);
    return `<tr${bad ? ' style="background:#2a1418"' : ""}>
      <td>${label} <span class="dim">${j.name}</span></td>
      <td class="${nan||outside ? "bad" : ""}">${nan ? "—" : fmt(j.pos,3)}</td>
      <td><div class="bar">
        <i style="left:${pct(j.default)}%;background:#8a93a6"></i>
        <i style="left:${pct(j.post)}%;background:#f2c14e;width:5px"></i>
        ${nan ? "" : `<i style="left:${pct(j.pos)}%;background:${outside?"#ff6b6b":"#6aa9ff"};width:5px"></i>`}
      </div><span class="dim">${j.min.toFixed(2)} … ${j.max.toFixed(2)}</span></td>
      <td>${fmt(j.eff,2)}</td>
      <td>${hcell}</td></tr>`;
  });
  $("joints").innerHTML = rows.join("");
  $("homingNotes").innerHTML = `<div class="dim">хоуминг: старт robot.service ${S.homing.since || "?"}</div>`
    + S.homing.notes.map(n => `<div class="warn">${esc(n)}</div>`).join("");
}
function esc(s){ return s.replace(/[&<>]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;"}[c])); }
function drawLogs() {
  $("logs").innerHTML = S.logs.slice().reverse().map(l =>
    `<div><span class="dim">${l.t}</span> <span class="${l.level==="WARN"?"warn":"bad"}">${l.level}</span> <span class="dim">${esc(l.node)}</span> ${esc(l.text)}</div>`).join("")
    || '<div class="dim">пусто</div>';
}
</script>
</body></html>
"""


def make_handler(dash: Dash):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args) -> None:
            return

        def _json(self, code: int, payload: dict) -> None:
            raw = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                raw = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
                return
            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    while True:
                        data = json.dumps(sanitize(dash.snapshot()))
                        self.wfile.write(f"data: {data}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(1.0 / STREAM_HZ)
                except (BrokenPipeError, ConnectionResetError):
                    return
            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            n = int(self.headers.get("Content-Length", "0"))
            try:
                body = json.loads(self.rfile.read(n).decode()) if n > 0 else {}
            except json.JSONDecodeError:
                self._json(400, {"error": "плохой json"})
                return
            err = None
            if self.path == "/api/drive":
                dash.set_drive(float(body.get("vx", 0)), float(body.get("vy", 0)), float(body.get("wz", 0)))
            elif self.path == "/api/stop":
                dash.stop()
            elif self.path == "/api/goal":
                err = dash.set_goal(body)
            elif self.path == "/api/reset_odom":
                dash.reset_odom()
            elif self.path == "/api/gait":
                err = dash.switch_gait(bool(body.get("on")))
            elif self.path == "/api/estop":
                dash.estop()
            else:
                self.send_error(404)
                return
            if err:
                self._json(400, {"error": err})
            else:
                self._json(200, {"ok": True})

    return Handler


def sanitize(obj):
    if isinstance(obj, float):
        return None if math.isnan(obj) or math.isinf(obj) else obj
    if isinstance(obj, dict):
        return {k: sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [sanitize(v) for v in obj]
    return obj


def main() -> None:
    rclpy.init()
    dash = Dash()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(dash)
    threading.Thread(target=executor.spin, daemon=True).start()

    server = ThreadingHTTPServer((HOST, PORT), make_handler(dash))
    server.daemon_threads = True
    print(f"Пульт: http://10.2.0.235:{PORT}  (Ctrl+C — выход, робот при выходе встанет)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        dash.stop()
        dash.pub.publish(Twist())
        server.server_close()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
