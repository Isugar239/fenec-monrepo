#!/usr/bin/env python3
"""Пульт Фенека. Открыть в браузере: http://<ip-робота>:8091

Работает рядом с robot.service, SPI не трогает: только ROS-топики и сервисы.
Скорость шлёт в /llm_cmd_vel — джойстик в cmd_vel_mux главнее и перебивает пульт.
Экран HDMI тоже обслуживается здесь (раньше был отдельный сервис на :8090).

  source /opt/ros/jazzy/setup.bash
  source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
  python3 ~/pupperv3-monorepo/tools/pupper_dashboard/dashboard.py
"""

from __future__ import annotations

import glob
import json
import math
import os
import re
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import sys

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float32, Float32MultiArray, Float64MultiArray, String, Bool
from tf2_ros import Buffer, TransformListener

PORT = 8091
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from open_loop_gait import GaitIK, smooth  # noqa: E402
from pupper_kinematics import LEG_INDEX, Kinematics  # noqa: E402
from pupper_record import Recorder  # noqa: E402
import screen_ctl  # noqa: E402

VPN_UNIT = "awg-quick@awg0.service"


def vpn_active() -> bool:
    r = subprocess.run(
        ["systemctl", "is-active", VPN_UNIT],
        capture_output=True,
        text=True,
    )
    return r.stdout.strip() == "active"


def vpn_set(on: bool) -> str | None:
    action = "start" if on else "stop"
    r = subprocess.run(
        ["sudo", "-n", "systemctl", action, VPN_UNIT],
        capture_output=True,
        text=True,
        timeout=60,
    )
    if r.returncode != 0:
        err = (r.stderr or r.stdout or f"systemctl {action} failed").strip()
        return err
    return None

JOINTS = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]
DEFAULT_POS = [0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52]
LEGS = ["front_r", "front_l", "back_r", "back_l"]
# Точка лапы в кадре leg_*_3 (foot_site из mujoco_xml).
FOOT = {
    "front_r": (0.06231, -0.06216, 0.018),
    "front_l": (0.06231, 0.06216, 0.018),
    "back_r": (0.06231, -0.06216, 0.018),
    "back_l": (0.06231, 0.06216, 0.018),
}

GAIT = "neural_controller"
POLICY_CONTROLLERS = ("neural_controller", "neural_controller_three_legged")
POLICY_OFFSETS = ("roll_offset_deg", "pitch_offset_deg")
FORWARD = ["forward_position_controller", "forward_kp_controller", "forward_kd_controller"]
# Стойка без нейросети. ПД считает сам мотор: момент ≈ kp·(цель − угол), поэтому
# «добавить момент» = поднять kp и/или увести цель дальше (интегральная добавка).
STAND_KP = 12.0
STAND_KP_MAX = 20.0  # не выше kp_max в components.xacro, иначе железо обрежет
# На шину этому суставу уходит удвоенный kp, не выше своего kp_max.
# leg_front_l_2: 8.8° против 1.3° у заднего левого отведения.
STAND_KP_MAX_JOINT = {"leg_front_l_2": 40.0}
WEAK_KP_SCALE = 2.0
WEAK_JOINTS = {name: JOINTS.index(name) for name in STAND_KP_MAX_JOINT}
STAND_KP_RATE = 4.0  # на сколько kp растёт за секунду, пока сустав не дотягивает
STAND_KD_PER_KP = 0.033
STAND_KI = 1.5  # рад добавки в секунду на радиан ошибки
STAND_I_MAX = 0.6
STAND_ERR_OK = 0.06
STAND_RELAX = 3.0  # с, за сколько добавка спадает в e раз, когда сустав в допуске
STAND_RAMP = 3.0
TRIM_MAX = 1.6
XACRO = os.path.join(
    os.path.dirname(os.path.dirname(HERE)),
    "ros2_ws/src/pupper_v3_description/description/components.xacro",
)
# Генератор траекторий (рысь без нейросети): траектории лап в метрах + обратная
# кинематика по URDF, см. tools/open_loop_gait.py. Поверх — удержание корпуса по IMU.
TG_DT = 0.02
TG_TO_STAND = 2.5
TG_RAMP = 1.5
TG_LIMITS = {
    "freq": (0.3, 3.0),
    "stride": (0.0, 0.06),
    "lift": (0.0, 0.06),
    "height": (-0.03, 0.03),
    "swing": (0.3, 0.7),
    "turn": (-1.0, 1.0),
    "kp": (4.0, 40.0),
    "kd": (0.1, 1.0),
    "imu_gain": (0.0, 1.5),
    "imu_ki": (0.0, 2.0),
    "imu_damp": (0.0, 0.2),
}
# Медленная рысь, которая реально сдвигается с места: stride 2 см часто буксовал,
# 6 см + IMU (запись 20260929_191137) уводили ЗП колено в −3.4 рад и валили набок.
# IMU-баланс всегда включён. Накопление почти выключено — иначе интегратор качает шаг.
TG_DEFAULTS = {
    "freq": 0.9, "stride": 0.032, "lift": 0.028, "height": 0.0, "swing": 0.5, "turn": 0.0,
    "kp": 12.0, "kd": 0.55,
    "imu_on": True, "imu_gain": 0.2, "imu_ki": 0.05, "imu_damp": 0.05,
}
TG_IMU_TAU = 0.15  # с, сглаживание углов IMU: колебания корпуса от шага не должны раскачивать ноги
TG_IMU_STALE = 0.2  # с без сообщений IMU — коррекции нет, шаг гасится
TG_IMU_I_MAX = 0.15  # рад, предел интегральной части (раньше 0.3 — слишком много на шаге)
TG_DZ_MAX = 0.015  # м, предел поправки высоты одной лапы (±1.5 см; ±3 см ломали IK)
POSE_GROUND_Z = -0.14  # м, пол модели в кадре корпуса (не «под самой нижней лапой»)
# Удержание рысканья. Курс — тот, с которого начался шаг. Ошибка > 0 значит ушёл влево
# (рысканье против часовой), а turn > 0 в генераторе — тоже влево, поэтому знак минус.
TG_YAW_KP = 1.2
TG_YAW_KI = 0.6
TG_YAW_KD = 0.08
TG_YAW_I_MAX = 0.35
TG_YAW_MAX = 0.45
# Демонстрационный режим: треугольник обратной кинематики ОДНОЙ ноги. Выбранная нога
# (плечо → колено → лапа) двигается по параметрам треугольника, остальные три стоят в обычной
# стойке. Положение лапы задаётся в метрах от стойки, углы считает IK по URDF
# (open_loop_gait.GaitIK.pose_offsets).
# Траектория — как в Stanford CS123 Lab 3 (ролик «Inverse Kinematics and Trajectory Tracking»):
# лапа идёт по рёбрам треугольника вперёд-низ → назад-низ → верх. В лабораторной
# углы (±5 см, низ на 12 см под плечом, верх на 6 см выше низа). Верх на 6 см
# упирает колено в ограничитель, поэтому по умолчанию низ опущен на DEMO_TRI_DROP,
# а высота треугольника 4 см: это самый высокий такой треугольник, который колено ещё достаёт.
DEMO_BASE_KEYS = ("height", "shift_x", "spread")
DEMO_LIMITS = {
    "shift_x": (-0.03, 0.03),
    "spread": (-0.02, 0.03),
    "anim_h": (0.0, 0.05),
    "anim_x": (0.0, 0.05),
    "anim_T": (2.0, 20.0),
    "kp": (4.0, 20.0),
    "kd": (0.1, 1.0),
    "ramp": (1.0, 8.0),
    "follow": (0.005, 0.10),
}
# Удлинение ноги относительно стойки (+ лапа ниже). Дальше +3 см колено выпрямляется,
# выше −2.5 см колено упирается в ограничитель (KNEE_MARGIN в open_loop_gait.py), IK промахивается.
DEMO_EXT = (-0.025, 0.03)
DEMO_TRI_DROP = 0.02  # насколько нижнее ребро ниже обычной стойки, м
DEMO_DEFAULTS = {
    "leg": "front_r", "height": None, "shift_x": 0.0, "spread": 0.0,
    "anim_h": 0.04, "anim_x": 0.05, "anim_T": 4.5,
    "kp": 12.0, "kd": 0.4, "ramp": 3.0, "follow": 0.03,
}
DEMO_ANIM_FADE = 1.5  # с на плавный вход и «погашение» анимации
# Уши: hobby-сервы, значение от −1 до 1.
# Сигнал — аппаратный ШИМ RP1: GPIO 12 = канал 0 (физ. контакт 32), GPIO 13 = канал 1 (контакт 33).
# Программный импульс сюда не ставим: он дрожит, серва не держит ноль и греется.
# Края A и B задаются ползунками. Анимация только ходит между этими числами.
# swap 0: левое на 12, правое на 13. swap 1: наоборот.
EAR_CHANNELS = {12: 0, 13: 1}
EAR_PERIOD_NS = 20_000_000
EAR_MODES = ("zero", "a", "b", "sync", "mirror", "alt")
EAR_LIMITS = {
    "a_l": (-1.0, 1.0), "b_l": (-1.0, 1.0),
    "a_r": (-1.0, 1.0), "b_r": (-1.0, 1.0),
    "period": (0.6, 8.0),
    "swap": (0.0, 1.0),
}
EAR_DEFAULTS = {
    "a_l": 0.5, "b_l": -0.5,
    "a_r": 0.5, "b_r": -0.5,
    "period": 1.6,
    "swap": 0.0,
}


def _sysfs_chmod(path: str) -> None:
    if os.access(path, os.W_OK):
        return
    proc = subprocess.run(["sudo", "-n", "chmod", "0666", path], capture_output=True, text=True)
    if proc.returncode != 0:
        detail = proc.stderr.strip() or f"код {proc.returncode}"
        raise OSError(f"нет права писать {path}: {detail}")


def _sysfs_write(path: str, text: str) -> None:
    _sysfs_chmod(path)
    with open(path, "w") as f:
        f.write(text)


def _ear_pwm_chip() -> str:
    """Чип pwm@98000, который включает dtoverlay=pwm-2chan на GPIO 12 и 13."""
    found = []
    for chip in sorted(glob.glob("/sys/class/pwm/pwmchip*")):
        dev = os.path.realpath(os.path.join(chip, "device"))
        if "98000" in dev:
            found.append(chip)
    if len(found) == 1:
        return found[0]
    if not found:
        chips = ", ".join(sorted(glob.glob("/sys/class/pwm/pwmchip*"))) or "пусто"
        raise OSError(
            "нет аппаратного ШИМ pwm@98000 в /sys/class/pwm "
            f"(сейчас: {chips}). В /boot/firmware/config.txt нужна строка "
            "dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4 и перезагрузка. "
            "Программный импульс на эти пины не включаю: от него сервы дрожат и греются."
        )
    raise OSError(f"несколько чипов pwm@98000: {', '.join(found)}")


class _HwPwm:
    """Один канал аппаратного ШИМ. value от −1 до 1, None — канал выключен, импульса нет."""

    def __init__(self, chip: str, channel: int, gpio: int) -> None:
        self.chip = chip
        self.channel = channel
        self.gpio = gpio
        self.path = os.path.join(chip, f"pwm{channel}")
        self.value: float | None = None
        self._duty: int | None = None

    def prepare(self) -> None:
        if not os.path.isdir(self.path):
            _sysfs_write(os.path.join(self.chip, "export"), str(self.channel))
            deadline = time.monotonic() + 1.0
            while not os.path.isdir(self.path):
                if time.monotonic() > deadline:
                    raise OSError(f"ядро не создало {self.path} для GPIO{self.gpio}")
                time.sleep(0.02)
        for name in ("enable", "period", "duty_cycle", "polarity"):
            _sysfs_chmod(os.path.join(self.path, name))
        # Сначала гасим канал: иначе ядро не даёт сменить period, если duty больше нового периода.
        _sysfs_write(os.path.join(self.path, "enable"), "0")
        _sysfs_write(os.path.join(self.path, "period"), str(EAR_PERIOD_NS))
        _sysfs_write(os.path.join(self.path, "duty_cycle"), "0")
        _sysfs_write(os.path.join(self.path, "polarity"), "normal")
        self.value = None
        self._duty = None

    def detach(self) -> None:
        # enable=0 обрывает импульс сразу. Канал не освобождаем: пин остаётся в функции ШИМ,
        # но импульсов нет, серва обмякает.
        if self.value is None and self._duty is None:
            return
        _sysfs_write(os.path.join(self.path, "enable"), "0")
        self.value = None
        self._duty = None

    def set(self, value: float) -> None:
        value = clamp(value, -1.0, 1.0)
        # −1 → 500 мкс, 0 → 1500 мкс, +1 → 2500 мкс.
        duty = int(round(1_500_000 + value * 1_000_000))
        duty = int(clamp(duty, 500_000, 2_500_000))
        if self._duty == duty and self.value is not None:
            self.value = value
            return
        _sysfs_write(os.path.join(self.path, "duty_cycle"), str(duty))
        if self.value is None:
            _sysfs_write(os.path.join(self.path, "enable"), "1")
        self.value = value
        self._duty = duty
REHOME_MODES = {"rear_first": 0.0, "all": 1.0}
REHOME_ACK_TIMEOUT = 6.0
CMD_TOPIC = "/llm_cmd_vel"
MANUAL_TIMEOUT = 0.35
CTRL_DT = 0.05
# IMU, /joint_states и /tf приходят по ~250 Гц. В rclpy каждое сообщение через
# executor — пробуждение и пересборка wait set на Python: это целое ядро Pi.
# Поэтому эти подписки живут на узле, который никто не спинит, а таймер раз в
# POLL_DT забирает накопившееся пачкой. Не реже TG_DT: tg_step балансирует по IMU.
POLL_DT = 0.02
HISTORY_SECONDS = 12.0


def quat_rotate(q, v):
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def euler(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def imu_identity(q) -> bool:
    return abs(q[3]) > 0.99999 and abs(q[0]) + abs(q[1]) + abs(q[2]) < 1e-4


def imu_usable(q, age: float) -> bool:
    """False только когда сообщений нет. Наклон и единичный кватернион не отключают IMU."""
    return q is not None and age <= TG_IMU_STALE


def read_homing() -> dict:
    # Журнал забит логом политики (50 Гц), поиск по всему запуску идёт минутами.
    # Хоуминг — первые пару минут после старта сервиса, ищем только там.
    started_us = subprocess.run(
        ["systemctl", "show", "robot.service", "-p", "ActiveEnterTimestampMonotonic", "--value"],
        capture_output=True,
        text=True,
    ).stdout.strip()
    if not started_us or started_us == "0":
        return {"when": None, "rows": [], "notes": ["robot.service не запущен"]}
    started = time.time() - (time.monotonic() - int(started_us) / 1e6)
    since = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(started - 5))
    until = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(started + 150))
    out = subprocess.run(
        [
            "journalctl", "-u", "robot.service", "-o", "short-iso", "--no-pager",
            "--since", since, "--until", until,
            "-g", "Homing|HOMING_|Finished homing|упоре|Skipping homing|homing timeout",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    ).stdout.splitlines()
    start = None
    for i, line in enumerate(out):
        if "Homing actuators..." in line:
            start = i
    if start is None:
        return {"when": None, "rows": [], "notes": ["в журнале robot.service этого запуска хоуминга нет"]}
    rows = []
    notes = []
    row_re = re.compile(
        r"HOMING_RESULT (\S+) CAN(\d) id(\d) start=(\S+) end=(\S+) d=(\S+) (\S+) (\S+)"
    )
    for line in out[start:]:
        m = row_re.search(line)
        if m:
            rows.append({
                "name": m.group(1), "can": int(m.group(2)), "id": int(m.group(3)),
                "d": float(m.group(6)), "moved": m.group(7), "hold": m.group(8),
            })
            continue
        if "[ControlBoardHardwareInterface]" in line and (
            "Skipping" in line or "упоре" in line or "HOMING_SKIPPED" in line or "timeout" in line
        ):
            notes.append(line.split("]: ", 1)[-1])
        if "Finished homing!" in line:
            break
    return {"when": out[start][:19], "rows": rows, "notes": notes}


class Dashboard(Node):
    def __init__(self) -> None:
        super().__init__("pupper_dashboard")
        self.lock = threading.Lock()

        self.imu_q = None
        self.imu_gyro = (0.0, 0.0, 0.0)
        self.imu_t = 0.0
        self.imu_hist: deque = deque()
        self.imu_rate_count = 0
        self.imu_rate = 0.0

        self.joint_pos = [None] * 12
        self.joint_vel = [None] * 12
        self.joint_eff = [None] * 12
        self.joint_t = 0.0
        self.pos_cmd = None
        self.policy_gravity = None
        self.policy_offsets = {name: None for name in POLICY_OFFSETS}
        self.policy_offsets_note = "читаю параметры контроллера…"
        self._offset_get_pending = False
        self.lat_policy = None
        self.lat_imu = None

        self.cmd_out = (0.0, 0.0, 0.0)
        self.cmd_out_t = 0.0
        self.mux_source = "?"

        self.controllers: dict[str, str] = {}
        self.controllers_t = 0.0
        self.gait_note = ""

        self.manual = (0.0, 0.0, 0.0)
        self.manual_t = 0.0
        self.mission: list[dict] = []
        self.mission_note = ""
        self.sending = False
        self.stand: dict | None = None
        # Поправка к стойке: сустав физически стоит правильно при pos = DEFAULT_POS + trim.
        self.trim = [0.0] * 12
        # Уже записано в xacro, но robot.service ещё не перезапущен — нули те же.
        self.trim_saved = [0.0] * 12
        self.trim_note = ""
        self.tg: dict | None = None
        self.tg_params = dict(TG_DEFAULTS)
        self.tg_gait = GaitIK(Kinematics.from_xacro())
        # Отдельный решатель: у него своё приближение IK, генератор его не сбивает.
        self.demo_ik = GaitIK(self.tg_gait.kin)
        self.demo_ik_lock = threading.Lock()
        self.demo: dict | None = None
        self.demo_nom_h = sum(
            float(self.tg_gait.kin.fk_links(leg, DEFAULT_POS[i:i + 3])[0][2] - self.demo_ik.nominal[leg][2])
            for leg, i in LEG_INDEX.items()
        ) / len(LEG_INDEX)
        self.demo_params = dict(DEMO_DEFAULTS, height=self.demo_nom_h + DEMO_TRI_DROP)
        self.demo_note = ""
        self.ear_lock = threading.Lock()
        self.ear_servos = None
        self.ear_params = dict(EAR_DEFAULTS)
        self.ear_mode: str | None = None
        self.ear_phase = 0.0
        self.ear_t_last = 0.0
        self.ear_note = ""
        self.ear_err = ""
        self.homing_status: dict | None = None
        self.rehome_req: dict | None = None
        self.recorder = Recorder()
        self.rec_t0 = 0.0
        self.rec_note = ""
        self.tg_note = ""
        self.vmax = 0.30
        self.vymax = 0.20
        self.wmax = 1.0
        self.odom_scale = 1.0

        self.x = 0.0
        self.y = 0.0
        self.yaw0 = None
        self.odom_t = time.monotonic()
        self.trail: deque = deque(maxlen=3000)

        self.log: deque = deque(maxlen=60)
        self.homing = {"when": None, "rows": [], "notes": ["читаю журнал…"]}
        self.service = "?"

        self.poll_node = Node("pupper_dashboard_poll")
        self.tf_buffer = Buffer()
        # Глубина очереди /tf (100) и IMU (50) должна покрывать 250 Гц × POLL_DT с запасом,
        # иначе теряются кадры TF и занижается imu_rate.
        self.tf_listener = TransformListener(self.tf_buffer, self.poll_node)
        self.imu_sub = self.poll_node.create_subscription(
            Imu, "/imu_sensor_broadcaster/imu", lambda _msg: None, 50
        )
        # Глубина 1: нужно только последнее значение.
        self.latest_subs = [
            self.poll_node.create_subscription(JointState, "/joint_states", self.on_joints, 1),
            self.poll_node.create_subscription(
                Float32MultiArray, "/neural_controller/position_command", self.on_pos_cmd, 1
            ),
            self.poll_node.create_subscription(
                Float32MultiArray, "/neural_controller/observation", self.on_obs, 1
            ),
            self.poll_node.create_subscription(
                Float32, "/neural_controller/policy_inference_latency_seconds", self.on_lat_policy, 1
            ),
            self.poll_node.create_subscription(
                Float32, "/neural_controller/imu_latency_seconds", self.on_lat_imu, 1
            ),
        ]
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_out, 10)
        self.create_subscription(String, "/cmd_vel_mux/active_source", self.on_mux, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 50)

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
        self.revive_pub = self.create_publisher(Empty, "/motor_revive", 10)
        self.fwd_pubs = [
            self.create_publisher(Float64MultiArray, f"/{name}/commands", 10) for name in FORWARD
        ]
        self.rehome_pub = self.create_publisher(Float32MultiArray, "/motor_rehome", 10)
        self.imu_power_pub = self.create_publisher(Bool, "/imu_power", 10)
        self.imu_power_on = True
        # transient_local, как у издателя: пульт, открытый после запуска, сразу видит итог хоуминга.
        self.create_subscription(
            String, "/motor_homing/status", self.on_homing_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            Bool, "/imu_power_state", self.on_imu_power_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.list_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.switch_cli = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.offset_get = {
            name: self.create_client(GetParameters, f"/{name}/get_parameters")
            for name in POLICY_CONTROLLERS
        }
        self.offset_set = {
            name: self.create_client(SetParameters, f"/{name}/set_parameters")
            for name in POLICY_CONTROLLERS
        }

        self.create_timer(POLL_DT, self.drain_topics)
        self.create_timer(CTRL_DT, self.control_step)
        self.create_timer(TG_DT, self.tg_step)
        self.create_timer(TG_DT, self.demo_step)
        self.create_timer(TG_DT, self.ear_step)
        self.create_timer(1.0, self.poll_controllers)
        self.create_timer(2.0, self.poll_policy_offsets)
        self.create_timer(1.0, self.count_rates)
        threading.Thread(target=self.system_loop, daemon=True).start()

    # --- подписки -------------------------------------------------------
    @staticmethod
    def _take(sub, raw: bool):
        with sub.handle:
            got = sub.handle.take_message(sub.msg_type, raw)
        return None if got is None else got[0]

    def drain_topics(self) -> None:
        # IMU забираем сырыми байтами: считаем все (для imu_rate), разбираем только последнее.
        last, count = None, 0
        while (raw := self._take(self.imu_sub, True)) is not None:
            last, count = raw, count + 1
        if last is not None:
            self.on_imu(deserialize_message(last, Imu), count)
        for sub in self.latest_subs:
            msg = self._take(sub, False)
            if msg is not None:
                sub.callback(msg)
        while (msg := self._take(self.tf_listener.tf_sub, False)) is not None:
            self.tf_listener.callback(msg)
        while (msg := self._take(self.tf_listener.tf_static_sub, False)) is not None:
            self.tf_listener.static_callback(msg)

    def on_imu(self, msg: Imu, count: int) -> None:
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        g = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        now = time.monotonic()
        with self.lock:
            self.imu_rate_count += count
            self.imu_q = q
            self.imu_gyro = g
            if self.yaw0 is None:
                self.yaw0 = euler(q)[2]
            # Порог 1.5·POLL_DT: в график идёт каждый второй опрос (25 Гц) даже при дрожании таймера.
            if not self.imu_hist or now - self.imu_hist[-1][0] >= 1.5 * POLL_DT:
                r, p, y = euler(q)
                self.imu_hist.append((now, math.degrees(r), math.degrees(p), math.degrees(y), *g))
                while self.imu_hist and now - self.imu_hist[0][0] > HISTORY_SECONDS:
                    self.imu_hist.popleft()
            self.imu_t = now

    def on_joints(self, msg: JointState) -> None:
        with self.lock:
            for k, name in enumerate(msg.name):
                if name in JOINTS:
                    i = JOINTS.index(name)
                    self.joint_pos[i] = msg.position[k]
                    self.joint_vel[i] = msg.velocity[k]
                    if len(msg.effort) == len(msg.name):
                        self.joint_eff[i] = msg.effort[k]
            self.joint_t = time.monotonic()

    def on_pos_cmd(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.pos_cmd = list(msg.data)

    def on_obs(self, msg: Float32MultiArray) -> None:
        with self.lock:
            self.policy_gravity = list(msg.data[3:6])

    def on_lat_policy(self, msg: Float32) -> None:
        self.lat_policy = msg.data

    def on_lat_imu(self, msg: Float32) -> None:
        self.lat_imu = msg.data

    def on_cmd_out(self, msg: Twist) -> None:
        with self.lock:
            self.cmd_out = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.cmd_out_t = time.monotonic()

    def on_mux(self, msg: String) -> None:
        self.mux_source = msg.data

    def on_homing_status(self, msg: String) -> None:
        # Поле state: running / done / none (хоуминг при запуске не выполнялся).
        data = json.loads(msg.data)
        data["rx_t"] = time.monotonic()
        with self.lock:
            self.homing_status = data
            req = self.rehome_req
            if req is not None:
                if data["seq"] != req["seq0"]:
                    req["acked"] = True
                if req["acked"] and data["state"] == "done":
                    self.rehome_req = None
                    self.demo_note = "хоуминг завершён" + (
                        ": без нуля " + ", ".join(data["skipped"]) if data["skipped"] else ": все суставы нашли ноль"
                    )

    def on_imu_power_state(self, msg: Bool) -> None:
        with self.lock:
            self.imu_power_on = bool(msg.data)

    def on_log(self, msg: Log) -> None:
        if msg.level < Log.WARN or msg.name == self.get_name():
            return
        text = msg.msg
        if "No joint state updates detected" in text:
            return
        with self.lock:
            self.log.append({
                "t": time.strftime("%H:%M:%S"),
                "level": "ERROR" if msg.level >= Log.ERROR else "WARN",
                "name": msg.name,
                "msg": text[:300],
            })

    def count_rates(self) -> None:
        with self.lock:
            self.imu_rate = float(self.imu_rate_count)
            self.imu_rate_count = 0

    # --- контроллеры ----------------------------------------------------
    def poll_controllers(self) -> None:
        if not self.list_cli.service_is_ready():
            return
        fut = self.list_cli.call_async(ListControllers.Request())
        fut.add_done_callback(self._on_controllers)

    def _on_controllers(self, fut) -> None:
        res = fut.result()
        with self.lock:
            self.controllers = {c.name: c.state for c in res.controller}
            self.controllers_t = time.monotonic()

    def gait_active(self) -> bool:
        return self.controllers.get(GAIT) == "active"

    def poll_policy_offsets(self) -> None:
        client = self.offset_get[GAIT]
        if self._offset_get_pending or not client.service_is_ready():
            if not client.service_is_ready():
                with self.lock:
                    self.policy_offsets_note = "контроллер ещё без этих параметров: после сборки перезапусти robot.service"
            return
        req = GetParameters.Request()
        req.names = list(POLICY_OFFSETS)
        self._offset_get_pending = True
        fut = client.call_async(req)
        fut.add_done_callback(self._on_policy_offsets)

    def _on_policy_offsets(self, fut) -> None:
        self._offset_get_pending = False
        try:
            res = fut.result()
        except Exception as exc:
            with self.lock:
                self.policy_offsets_note = f"не прочитал оффсеты: {exc}"
            return
        if len(res.values) != len(POLICY_OFFSETS):
            with self.lock:
                self.policy_offsets_note = "запущенный контроллер собран без оффсетов: перезапусти robot.service после сборки"
            return
        values = {}
        for name, value in zip(POLICY_OFFSETS, res.values):
            if value.type != ParameterType.PARAMETER_DOUBLE:
                with self.lock:
                    self.policy_offsets_note = "запущенный контроллер собран без оффсетов: перезапусти robot.service после сборки"
                return
            values[name] = value.double_value
        with self.lock:
            self.policy_offsets.update(values)
            self.policy_offsets_note = ""

    def api_policy_offset(self, body: dict) -> str | None:
        params = []
        for name in POLICY_OFFSETS:
            if name not in body:
                continue
            value = clamp(float(body[name]), -45.0, 45.0)
            param = Parameter()
            param.name = name
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
            params.append(param)
        if not params:
            return "пустой запрос"
        wrote = []
        missing = []
        for controller, client in self.offset_set.items():
            if not client.service_is_ready():
                missing.append(controller)
                continue
            req = SetParameters.Request()
            req.parameters = params
            done = threading.Event()
            fut = client.call_async(req)
            fut.add_done_callback(lambda _f: done.set())
            if not done.wait(2.0) or fut.result() is None:
                return f"{controller} не ответил на запись оффсета"
            failed = [r.reason for r in fut.result().results if not r.successful]
            if failed:
                return f"{controller}: {failed[0]}"
            wrote.append(controller)
        if GAIT not in wrote:
            return "neural_controller не запущен. Сначала перезапусти robot.service после сборки"
        with self.lock:
            self.policy_offsets.update({p.name: p.value.double_value for p in params})
            self.policy_offsets_note = ""
        if missing:
            return None
        return None

    def switch(self, activate: list[str], deactivate: list[str]) -> str | None:
        if not self.switch_cli.wait_for_service(timeout_sec=2.0):
            return "controller_manager не отвечает — robot.service запущен?"
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        done = threading.Event()
        fut = self.switch_cli.call_async(req)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(5.0):
            return "switch_controller не ответил за 5 с"
        if not fut.result().ok:
            return "controller_manager отказал в переключении (смотри журнал ниже)"
        self.poll_controllers()
        return None

    def set_gait(self, on: bool) -> str | None:
        with self.lock:
            active = [n for n, s in self.controllers.items() if s == "active"]
        if on:
            with self.lock:
                self.stand = None
                self.tg = None
                self.demo = None
            others = [n for n in active if n != GAIT and "broadcaster" not in n]
            return self.switch([GAIT], others)
        self.stop_motion("походка выключена")
        return self.switch([], [GAIT])

    def set_stand(self) -> str | None:
        with self.lock:
            if any(p is None for p in self.joint_pos):
                return "нет /joint_states — не знаю, откуда плавно ехать"
            start = list(self.joint_pos)
            active = [n for n, s in self.controllers.items() if s == "active"]
            self.tg = None
            self.demo = None
        self.stop_motion("стойка без нейросети")
        self._publish_forward(start, [STAND_KP] * 12, [STAND_KP * STAND_KD_PER_KP] * 12)
        others = [n for n in active if n not in FORWARD and "broadcaster" not in n]
        err = self.switch(FORWARD, others)
        if err:
            return err
        with self.lock:
            self.stand = {
                "start": start,
                "t0": time.monotonic(),
                "t_last": time.monotonic(),
                "kp": [STAND_KP] * 12,
                "i": [0.0] * 12,
                "cmd": list(start),
            }
        return None

    def set_tg(self, on: bool) -> str | None:
        if not on:
            with self.lock:
                if self.tg is None:
                    return "генератор и так выключен"
                if self.tg["stop_t"] is None:
                    self.tg["stop_t"] = time.monotonic()
                self.tg_note = "останавливаюсь: шаг плавно гаснет, потом стойка"
            return None
        with self.lock:
            if any(p is None for p in self.joint_pos):
                return "нет /joint_states — не знаю, откуда плавно ехать"
            if self.tg is not None:
                return "генератор уже идёт"
            start = list(self.joint_pos)
            active = [n for n, s in self.controllers.items() if s == "active"]
            self.stand = None
            self.demo = None
        self.stop_motion("генератор траекторий: без нейросети, корпус по IMU")
        p = self.tg_params
        self._publish_forward(start, [p["kp"]] * 12, [p["kd"]] * 12)
        others = [n for n in active if n not in FORWARD and "broadcaster" not in n]
        err = self.switch(FORWARD, others)
        if err:
            return err
        now = time.monotonic()
        self.tg_gait.reset()
        with self.lock:
            self.tg = {
                "start": start, "t0": now, "t_last": now, "phase": 0.0, "amp": 0.0,
                "stop_t": None, "cmd": list(start), "stage": "еду в стойку",
                "att": None, "i_roll": 0.0, "i_pitch": 0.0,
                "dz": {leg: 0.0 for leg in LEGS}, "ik_err": 0.0,
                "yaw_ref": None, "yaw": None, "yaw_i": 0.0, "yaw_turn": 0.0,
            }
            self.tg_params["imu_on"] = True
            self.tg_note = "еду в стойку, потом шаг плавно набирается"
        return None

    # --- демонстрационный режим -----------------------------------------
    def demo_limits(self) -> dict:
        return {
            "height": (self.demo_nom_h + DEMO_EXT[0], self.demo_nom_h + DEMO_EXT[1]),
            **DEMO_LIMITS,
        }

    def _homed_error(self) -> str | None:
        h = self.homing_status
        if h is None:
            return ("статус хоуминга неизвестен: нажми «Хоуминг». Если статус так и не придёт — "
                    "control_board_hardware_interface собран без /motor_rehome: пересобери и перезапусти robot.service")
        if h["state"] == "running":
            return "хоуминг ещё идёт"
        if h["state"] != "done":
            return "при запуске сервиса хоуминга не было — нажми «Хоуминг»"
        if h["skipped"]:
            return "суставы без нуля, момента на них нет: " + ", ".join(h["skipped"])
        return None

    def api_demo_homing(self, body: dict) -> str | None:
        mode = REHOME_MODES.get(body.get("mode"))
        if mode is None:
            return f"режим хоуминга: {' или '.join(REHOME_MODES)}"
        delay = clamp(float(body.get("delay", 1.5)), 0.0, 10.0)
        with self.lock:
            now = time.monotonic()
            req = self.rehome_req
            # Хоуминг ограничен 90 с; без ответа за REHOME_ACK_TIMEOUT запрос считаем потерянным.
            lost = req is not None and (now - req["t"] > 150.0 or (not req["acked"] and now - req["t"] > REHOME_ACK_TIMEOUT))
            if req is not None and not lost:
                return "запрос хоуминга уже отправлен"
            st = self.homing_status
            if st is not None and st["state"] == "running" and not lost:
                return "хоуминг уже идёт"
            seq0 = None if st is None else st["seq"]
            # Хоуминг занимает цикл контроллеров, а после него старые команды не должны дёрнуть ноги.
            active = [n for n, s in self.controllers.items() if s == "active" and "broadcaster" not in n]
            self.stand = None
            self.tg = None
            self.demo = None
        self.stop_motion("хоуминг: моторы обмякнут, ноги едут в упоры")
        if active:
            err = self.switch([], active)
            if err:
                return err
        with self.lock:
            self.rehome_req = {"t": time.monotonic(), "seq0": seq0, "acked": False}
            self.demo_note = "хоуминг запрошен: " + (
                "все ноги сразу" if mode == REHOME_MODES["all"] else f"сначала задние, пауза {delay:.1f} с, потом передние"
            )
        self.rehome_pub.publish(Float32MultiArray(data=[float(mode), float(delay)]))
        return None

    def api_demo_params(self, body: dict) -> str | None:
        limits = self.demo_limits()
        with self.lock:
            if body.get("reset"):
                self.demo_params = dict(DEMO_DEFAULTS, height=self.demo_nom_h + DEMO_TRI_DROP)
                return None
            for k, v in body.items():
                if k == "leg":
                    if v not in LEGS:
                        return f"нога {v!r}: нужна одна из {', '.join(LEGS)}"
                    self.demo_params[k] = v
                    continue
                if k not in limits:
                    return f"неизвестный параметр {k!r}"
                lo, hi = limits[k]
                self.demo_params[k] = clamp(float(v), lo, hi)
        return None

    def set_demo_stance(self) -> str | None:
        err = self._homed_error()
        if err:
            return err
        with self.lock:
            if self.rehome_req is not None:
                return "хоуминг ещё не подтверждён"
            if any(p is None for p in self.joint_pos):
                return "нет /joint_states — не знаю, откуда плавно ехать"
            start = list(self.joint_pos)
            active = [n for n, s in self.controllers.items() if s == "active"]
            self.stand = None
            self.tg = None
            self.demo = None
            p = dict(self.demo_params)
        self.stop_motion("демо: стойка по обратной кинематике")
        with self.demo_ik_lock:
            self.demo_ik.reset()
        self._publish_forward(start, [p["kp"]] * 12, [p["kd"]] * 12)
        others = [n for n in active if n not in FORWARD and "broadcaster" not in n]
        err = self.switch(FORWARD, others)
        if err:
            return err
        now = time.monotonic()
        with self.lock:
            self.demo = {
                "start": start, "t0": now, "t_last": now, "cmd": list(start),
                "cur": self._demo_targets(p), "ik_err": 0.0, "clamped": False,
                "anim_on": False, "anim_amp": 0.0, "anim_phase": 0.0,
            }
            self.demo_note = f"выезжаю в стойку: лапа {p['leg']} встанет на середину нижнего ребра"
        return None

    def api_demo_anim(self, on: bool) -> str | None:
        """Запустить анимацию (при необходимости сначала выехать в стойку) или погасить её."""
        if on:
            with self.lock:
                running = self.demo is not None
            if not running:
                err = self.set_demo_stance()
                if err:
                    return err
            with self.lock:
                self.demo["anim_on"] = True
                self.demo_note = "анимация: лапа идёт по рёбрам — вперёд-низ, назад-низ, верх"
            return None
        with self.lock:
            if self.demo is None:
                return "анимация не идёт: стойка не включена"
            self.demo["anim_on"] = False
            self.demo_note = "гашу анимацию: лапа возвращается на середину нижнего ребра"
        return None

    def _demo_targets(self, p: dict) -> dict:
        """Цель каждой ноги [dx, dy, ext] до анимации: только выбранная нога уходит от стойки."""
        targets = {leg: [0.0, 0.0, 0.0] for leg in LEGS}
        left = 1.0 if p["leg"].endswith("_l") else -1.0
        targets[p["leg"]] = [p["shift_x"], left * p["spread"], p["height"] - self.demo_nom_h]
        return targets

    def demo_release(self) -> str | None:
        with self.lock:
            if any(p is None for p in self.joint_pos):
                return "нет /joint_states"
            pos = list(self.joint_pos)
            active = [n for n, s in self.controllers.items() if s == "active"]
            self.stand = None
            self.tg = None
            self.demo = None
        self.stop_motion("демо: момент снят")
        self._publish_forward(pos, [0.0] * 12, [0.1] * 12)
        others = [n for n in active if n not in FORWARD and "broadcaster" not in n]
        err = self.switch(FORWARD, others)
        if err:
            return err
        with self.lock:
            self.demo_note = "момент снят: ноги обмякли, робот должен лежать или висеть"
        return None

    def demo_triangle(self, p: dict) -> list[list[float]]:
        """Три угла траектории в base_link: вперёд-низ, назад-низ, верх."""
        nom = self.demo_ik.nominal[p["leg"]]
        left = 1.0 if p["leg"].endswith("_l") else -1.0
        bx = float(nom[0]) + p["shift_x"]
        by = float(nom[1]) + left * p["spread"]
        bz = float(nom[2]) - (p["height"] - self.demo_nom_h)
        ax, ah = p["anim_x"], p["anim_h"]
        return [
            [round(bx + ax, 4), round(by, 4), round(bz, 4)],
            [round(bx - ax, 4), round(by, 4), round(bz, 4)],
            [round(bx, 4), round(by, 4), round(bz + ah, 4)],
        ]

    @staticmethod
    def _tri_delta(phase: float, anim_x: float, anim_h: float) -> tuple[float, float]:
        """Смещение (dx, dext) по рёбрам. dext < 0 — лапа выше нижнего ребра.

        Порядок углов как в CS123 lab 3: вперёд-низ → назад-низ → верх → снова вперёд.
        phase 0..1 — один обход, на каждое ребро уходит треть периода.
        """
        corners = ((anim_x, 0.0), (-anim_x, 0.0), (0.0, -anim_h))
        u = (phase * 3.0) % 3.0
        seg = int(u)
        a = u - seg
        x0, e0 = corners[seg]
        x1, e1 = corners[(seg + 1) % 3]
        return (1.0 - a) * x0 + a * x1, (1.0 - a) * e0 + a * e1

    @staticmethod
    def _demo_offsets(cur: dict, leg: str, anim_dx: float, anim_ext: float) -> tuple[dict, bool]:
        """Смещения лап от стойки (dx, dy, dz), м, и признак «дальше предела ноги».

        anim_dx и anim_ext — добавка анимации, применяется только к выбранной ноге.
        """
        offsets, clamped = {}, False
        for name, (dx, dy, ext) in cur.items():
            if name == leg:
                dx += anim_dx
                ext += anim_ext
            limited = clamp(ext, *DEMO_EXT)
            clamped = clamped or abs(limited - ext) > 1e-6
            offsets[name] = (dx, dy, -limited)
        return offsets, clamped

    def demo_step(self) -> None:
        now = time.monotonic()
        with self.lock:
            demo = self.demo
            if demo is None:
                return
            fwd_active = all(self.controllers.get(n) == "active" for n in FORWARD)
            if not fwd_active and now - demo["t0"] > 3.0:
                self.demo = None
                self.demo_note = "демо снято: forward-контроллеры выключены"
                return
            p = dict(self.demo_params)
            dt = min(0.1, now - demo["t_last"])
            demo["t_last"] = now
            trim = list(self.trim)
            # Ползунок не прыгает ногой: цель догоняется со скоростью follow (м/с).
            # Смена ноги работает так же: прежняя плавно возвращается в стойку, новая выходит.
            step = p["follow"] * dt
            for leg, target in self._demo_targets(p).items():
                cur_leg = demo["cur"][leg]
                for k in range(3):
                    cur_leg[k] += clamp(target[k] - cur_leg[k], -step, step)
            cur = {leg: list(v) for leg, v in demo["cur"].items()}
            el = now - demo["t0"]
            # Анимация стартует после выезда в стойку и входит/гаснет за DEMO_ANIM_FADE.
            want_anim = demo["anim_on"] and el >= p["ramp"]
            fade = dt / DEMO_ANIM_FADE
            demo["anim_amp"] = clamp(demo["anim_amp"] + (fade if want_anim else -fade), 0.0, 1.0)
            if demo["anim_amp"] > 0.0:
                demo["anim_phase"] = (demo["anim_phase"] + dt / p["anim_T"]) % 1.0
            amp, phase = smooth(demo["anim_amp"]), demo["anim_phase"]
        a = smooth(min(1.0, el / p["ramp"]))
        dx, dext = self._tri_delta(phase, p["anim_x"], p["anim_h"])
        offsets, clamped = self._demo_offsets(cur, p["leg"], amp * dx, amp * dext)
        with self.demo_ik_lock:
            q = self.demo_ik.pose_offsets(offsets)
            ik_err = self.demo_ik.ik_err
        cmd = [s + a * (q[i] + trim[i] - s) for i, s in enumerate(demo["start"])]
        with self.lock:
            if self.demo is not demo:
                return
            demo["cmd"] = cmd
            demo["ik_err"] = ik_err
            demo["clamped"] = clamped
            self._publish_forward(cmd, [p["kp"]] * 12, [p["kd"]] * 12)

    # --- сервы ушей -----------------------------------------------------
    def api_ears_params(self, body: dict) -> str | None:
        with self.ear_lock:
            if body.get("reset"):
                self.ear_params = dict(EAR_DEFAULTS)
                return None
            for k, v in body.items():
                if k not in EAR_LIMITS:
                    return f"неизвестный параметр {k!r}"
                lo, hi = EAR_LIMITS[k]
                number = clamp(float(v), lo, hi)
                if k == "swap":
                    number = 1.0 if number >= 0.5 else 0.0
                self.ear_params[k] = number
        return None

    def api_ears(self, mode: str) -> str | None:
        if mode == "off":
            self.ears_off()
            return None
        if mode not in EAR_MODES:
            return f"режим {mode!r}: нужен один из {', '.join(EAR_MODES)} или off"
        err = self._ears_open()
        if err:
            with self.ear_lock:
                self.ear_err = err
                self.ear_note = err
            return err
        with self.ear_lock:
            self.ear_mode = mode
            self.ear_phase = 0.0
            self.ear_t_last = time.monotonic()
            self.ear_err = ""
            self.ear_note = {
                "zero": "оба уха в 0",
                "a": "держу оба уха в крае A",
                "b": "держу оба уха в крае B",
                "sync": "вместе: оба уха ходят от A к B и обратно",
                "mirror": "зеркально: левое A→B, правое B→A",
                "alt": "по очереди: одно ухо в A, потом в B и отпуск, затем другое",
            }[mode]
        return None

    def ears_off(self) -> None:
        with self.ear_lock:
            was = self.ear_mode is not None or self.ear_servos is not None
            self.ear_mode = None
            servos = list(self.ear_servos.values()) if self.ear_servos else []
            for servo in servos:
                servo.detach()
            if was:
                self.ear_note = "импульс снят: уши обмякли, дай им остыть"

    def _ears_open(self) -> str | None:
        """Подготовить каналы ШИМ при первой команде. До set() импульса нет."""
        with self.ear_lock:
            if self.ear_servos is not None:
                return None
            try:
                chip = _ear_pwm_chip()
                npwm = int(open(os.path.join(chip, "npwm")).read())
                servos: dict[int, _HwPwm] = {}
                for gpio, channel in EAR_CHANNELS.items():
                    if channel >= npwm:
                        raise OSError(f"у {chip} каналов {npwm}, а GPIO{gpio} ждёт канал {channel}")
                    servo = _HwPwm(chip, channel, gpio)
                    servo.prepare()
                    servos[gpio] = servo
                self.ear_servos = servos
            except OSError as e:
                return str(e)
        return None

    def ear_step(self) -> None:
        now = time.monotonic()
        with self.ear_lock:
            servos = self.ear_servos
            mode = self.ear_mode
            if servos is None or mode is None:
                return
            p = dict(self.ear_params)
            dt = min(0.1, now - self.ear_t_last) if self.ear_t_last else 0.0
            self.ear_t_last = now
            if mode in ("sync", "mirror", "alt"):
                self.ear_phase = (self.ear_phase + dt / p["period"]) % 1.0
            phase = self.ear_phase
        left, right = self._ear_targets(mode, phase, p)
        try:
            self._ear_apply(left, right)
        except OSError as e:
            self.ears_off()
            with self.ear_lock:
                self.ear_err = f"ШИМ оборвался, импульс снят: {e}"
                self.ear_note = self.ear_err

    @staticmethod
    def _ear_targets(mode: str, phase: float, p: dict) -> tuple[float | None, float | None]:
        """(левое, правое). None — отпустить серву. Числа — текущие края A/B."""
        if mode == "zero":
            return 0.0, 0.0
        if mode == "a":
            return p["a_l"], p["a_r"]
        if mode == "b":
            return p["b_l"], p["b_r"]
        # 0 на крае A, 1 на крае B, обратно к A. Косинус, чтобы на развороте не было удара.
        w = 0.5 - 0.5 * math.cos(2.0 * math.pi * phase)
        if mode == "sync":
            return (
                p["a_l"] + (p["b_l"] - p["a_l"]) * w,
                p["a_r"] + (p["b_r"] - p["a_r"]) * w,
            )
        if mode == "mirror":
            return (
                p["a_l"] + (p["b_l"] - p["a_l"]) * w,
                p["a_r"] + (p["b_r"] - p["a_r"]) * (1.0 - w),
            )
        # Четыре равные доли периода, как tools/ears_cycle.py, но края берутся из ползунков.
        q = (phase * 4.0) % 4.0
        seg = int(q)
        a = q - seg
        if seg == 0:
            return None, p["a_r"]
        if seg == 1:
            return None, p["a_r"] + (p["b_r"] - p["a_r"]) * a
        if seg == 2:
            return p["a_l"], None
        return p["a_l"] + (p["b_l"] - p["a_l"]) * a, None

    def _ear_sides(self) -> tuple[int, int]:
        """GPIO левого и правого уха с учётом свапа. Читать под ear_lock."""
        if self.ear_params["swap"] >= 0.5:
            return 13, 12
        return 12, 13

    def _ear_apply(self, left: float | None, right: float | None) -> None:
        with self.ear_lock:
            servos = self.ear_servos
            if servos is None or self.ear_mode is None:
                return
            gpio_l, gpio_r = self._ear_sides()
            # Оба пина в одном тике: при свапе старый пин не остаётся со старым импульсом.
            for gpio, value in ((gpio_l, left), (gpio_r, right)):
                servo = servos[gpio]
                if value is None:
                    servo.detach()
                else:
                    servo.set(value)

    def api_tg_params(self, body: dict) -> str | None:
        with self.lock:
            for k, v in body.items():
                if k in TG_LIMITS:
                    lo, hi = TG_LIMITS[k]
                    self.tg_params[k] = clamp(float(v), lo, hi)
                elif k == "imu_on":
                    # Баланс по IMU всегда включён; параметр оставлен в meta для старых клиентов.
                    self.tg_params[k] = True
                else:
                    return f"неизвестный параметр {k!r}"
            if self.recorder.active:
                self.recorder.log_event({"tg_params": dict(self.tg_params)})
        return None

    def api_record(self, body: dict) -> str | None:
        """Запись ros2 bag для sysid/обучения (tools/pupper_record.py, ai/rl/REAL_DATA.md)."""
        if bool(body.get("on")):
            if self.recorder.active:
                return "запись уже идёт"
            with self.lock:
                extra = {
                    "source": "dashboard",
                    "mode": "tg" if self.tg is not None else "demo" if self.demo is not None else "gait" if self.gait_active() else "stand" if self.stand is not None else "idle",
                    "tg_params": dict(self.tg_params),
                    "trim": list(self.trim),
                }
            # RuntimeError здесь — это сообщение для пользователя (мало места, запись уже идёт), а не сбой.
            try:
                session = self.recorder.start(str(body.get("note", "")), extra)
            except RuntimeError as e:
                return str(e)
            self.rec_t0 = time.monotonic()
            self.rec_note = f"пишу: {session}"
            return None
        if not self.recorder.active:
            return "запись не идёт"
        try:
            session = self.recorder.stop()
        except RuntimeError as e:
            self.rec_note = str(e)
            return str(e)
        self.rec_note = f"сохранено: {session}  (экспорт: python3 tools/pupper_record.py export {session})"
        return None

    def _tg_balance(self, tg: dict, p: dict, imu_q, gyro, imu_age: float, dt: float) -> dict[str, float] | None:
        """Поправки высоты лап, м. None — сообщений IMU нет, шаг гасится."""
        if not imu_usable(imu_q, imu_age):
            return None
        roll, pitch, _ = euler(imu_q)
        a = dt / (TG_IMU_TAU + dt)
        if tg["att"] is None:
            tg["att"] = [roll, pitch]
        tg["att"][0] += a * (roll - tg["att"][0])
        tg["att"][1] += a * (pitch - tg["att"][1])
        r, pt = tg["att"]
        tg["i_roll"] = clamp(tg["i_roll"] + p["imu_ki"] * r * dt, -TG_IMU_I_MAX, TG_IMU_I_MAX)
        tg["i_pitch"] = clamp(tg["i_pitch"] + p["imu_ki"] * pt * dt, -TG_IMU_I_MAX, TG_IMU_I_MAX)
        corr_roll = p["imu_gain"] * r + tg["i_roll"] + p["imu_damp"] * gyro[0]
        corr_pitch = p["imu_gain"] * pt + tg["i_pitch"] + p["imu_damp"] * gyro[1]
        # roll > 0 — правый бок (y < 0) ниже: правые лапы опускаем (dz < 0).
        # pitch > 0 — нос ниже: передние лапы (x > 0) опускаем.
        dz = {}
        for leg, p0 in self.tg_gait.nominal.items():
            x, y = float(p0[0]), float(p0[1])
            dz[leg] = clamp(y * corr_roll - x * corr_pitch, -TG_DZ_MAX, TG_DZ_MAX)
        return dz

    def _tg_yaw_turn(self, tg: dict, imu_q, imu_age: float, dt: float) -> float:
        """Добавка к turn, чтобы держать курс старта шага. 0, если IMU плохой."""
        if not imu_usable(imu_q, imu_age) or dt <= 0.0:
            return 0.0
        yaw = euler(imu_q)[2]
        if tg["yaw_ref"] is None:
            tg["yaw_ref"] = yaw
            tg["yaw"] = yaw
        err = wrap(yaw - tg["yaw_ref"])
        rate = wrap(yaw - tg["yaw"]) / dt
        tg["yaw"] = yaw
        tg["yaw_i"] = clamp(tg["yaw_i"] + err * dt, -TG_YAW_I_MAX, TG_YAW_I_MAX)
        tg["yaw_turn"] = clamp(-TG_YAW_KP * err - TG_YAW_KI * tg["yaw_i"] - TG_YAW_KD * rate,
                               -TG_YAW_MAX, TG_YAW_MAX)
        return tg["yaw_turn"]

    def tg_step(self) -> None:
        now = time.monotonic()
        with self.lock:
            tg = self.tg
            if tg is None:
                return
            fwd_active = all(self.controllers.get(n) == "active" for n in FORWARD)
            if not fwd_active and now - tg["t0"] > 3.0:
                self.tg = None
                self.tg_note = "генератор снят: forward-контроллеры выключены"
                return
            p = dict(self.tg_params)
            dt = min(0.1, now - tg["t_last"])
            tg["t_last"] = now
            trim = list(self.trim)
            imu_q, gyro, imu_age = self.imu_q, self.imu_gyro, now - self.imu_t
            el = now - tg["t0"]
            if el >= TG_TO_STAND:
                if tg["stop_t"] is None:
                    tg["amp"] = min(1.0, tg["amp"] + dt / TG_RAMP)
                    tg["stage"] = "иду" if tg["amp"] >= 1.0 else "набираю шаг"
                else:
                    tg["amp"] = max(0.0, tg["amp"] - dt / TG_RAMP)
                    tg["stage"] = "останавливаюсь"
                tg["phase"] = (tg["phase"] + p["freq"] * dt) % 1.0

        if el < TG_TO_STAND:
            a = smooth(el / TG_TO_STAND)
            base = [DEFAULT_POS[i] + trim[i] for i in range(12)]
            cmd = [s + a * (b - s) for s, b in zip(tg["start"], base)]
        else:
            amp = smooth(tg["amp"])
            dz = self._tg_balance(tg, p, imu_q, gyro, imu_age, dt)
            if dz is None:
                with self.lock:
                    if tg["stop_t"] is None:
                        tg["stop_t"] = now
                    self.tg_note = f"IMU молчит {imu_age:.2f} с — коррекции нет, гашу шаг"
            else:
                dz = {leg: v * amp for leg, v in dz.items()}
            tg["dz"] = dz if dz is not None else {leg: 0.0 for leg in LEGS}
            yaw_turn = self._tg_yaw_turn(tg, imu_q, imu_age, dt)
            q = self.tg_gait.pose(
                tg["phase"], amp, stride=p["stride"], lift=p["lift"], swing=p["swing"],
                turn=clamp(p["turn"] + yaw_turn, -1.0, 1.0), height=p["height"], dz_extra=dz,
            )
            tg["ik_err"] = self.tg_gait.ik_err
            cmd = [q[i] + trim[i] for i in range(12)]
        tg["cmd"] = cmd
        # Проверка и отправка под одной блокировкой: если пока считали углы, нажали
        # аварийный стоп или «Стойку», генератор уже снят и момент подавать нельзя.
        with self.lock:
            if self.tg is not tg:
                return
            self._publish_forward(cmd, [p["kp"]] * 12, [p["kd"]] * 12)
            if el >= TG_TO_STAND and tg["stop_t"] is not None and tg["amp"] == 0.0:
                # Шаг погас — дальше держит обычная «Стойка».
                self.tg = None
                self.stand = {
                    "start": list(cmd), "t0": now - STAND_RAMP, "t_last": now,
                    "kp": [STAND_KP] * 12, "i": [0.0] * 12, "cmd": list(cmd),
                }
                self.tg_note = "генератор остановлен, держу стойку"

    def _publish_forward(self, pos: list[float], kp: list[float], kd: list[float]) -> None:
        kp, kd = list(kp), list(kd)
        for name, idx in WEAK_JOINTS.items():
            if kp[idx] > 0.0:
                kp[idx] = min(STAND_KP_MAX_JOINT[name], kp[idx] * WEAK_KP_SCALE)
                kd[idx] = min(1.0, kd[idx] * WEAK_KP_SCALE)
        for pub, data in zip(self.fwd_pubs, (pos, kp, kd)):
            pub.publish(Float64MultiArray(data=[float(v) for v in data]))

    def stand_step(self, now: float) -> None:
        with self.lock:
            stand = self.stand
            fwd_active = all(self.controllers.get(n) == "active" for n in FORWARD)
            if stand is not None and not fwd_active and now - stand["t0"] > 3.0:
                self.stand = None
                self.mission_note = "стойка снята: forward-контроллеры выключены"
                return
            if stand is None:
                return
            dt = min(0.2, now - stand["t_last"])
            stand["t_last"] = now
            ramp = min(1.0, (now - stand["t0"]) / STAND_RAMP)
            a = 0.5 - 0.5 * math.cos(math.pi * ramp)
            cmd, kp = [], stand["kp"]
            for i in range(12):
                goal = DEFAULT_POS[i] + self.trim[i]
                ref = stand["start"][i] + a * (goal - stand["start"][i])
                q = self.joint_pos[i]
                err = ref - q
                # Добавка только пока сустав не дотягивает до стойки, и не больше STAND_I_MAX.
                # Команду нельзя привязывать к фактическому углу: иначе уступившее колено
                # утаскивает цель за собой до упора и само уже не возвращается.
                if abs(err) > STAND_ERR_OK:
                    if ramp >= 1.0:
                        kp[i] = min(STAND_KP_MAX_JOINT.get(JOINTS[i], STAND_KP_MAX), kp[i] + STAND_KP_RATE * dt)
                    excess = err - math.copysign(STAND_ERR_OK, err)
                    stand["i"][i] = clamp(stand["i"][i] + STAND_KI * excess * dt, -STAND_I_MAX, STAND_I_MAX)
                elif ramp >= 1.0:
                    stand["i"][i] *= math.exp(-dt / STAND_RELAX)
                    kp[i] = max(STAND_KP, kp[i] - STAND_KP_RATE * 0.5 * dt)
                cmd.append(ref + stand["i"][i])
            stand["cmd"] = cmd
            kps = list(kp)
        self._publish_forward(cmd, kps, [k * STAND_KD_PER_KP for k in kps])

    def api_trim(self, body: dict) -> str | None:
        with self.lock:
            if self.stand is None:
                return "поправки крутятся только в режиме «Стойка»"
            if body.get("reset"):
                self.trim = list(self.trim_saved)
                return None
            i = int(body["index"])
            self.trim[i] = clamp(self.trim[i] + float(body["delta"]), -TRIM_MAX, TRIM_MAX)
        return None

    def api_trim_save(self) -> str | None:
        with self.lock:
            delta = [t - s for t, s in zip(self.trim, self.trim_saved)]
        if all(abs(d) < 1e-6 for d in delta):
            return "нечего записывать: поправки нулевые"
        with open(XACRO, encoding="utf-8") as f:
            text = f.read()
        changed = []
        # Каждый сустав из JOINTS: abad (_1), hip (_2) и knee (_3). Нулевая поправка не пишется.
        for name, d in zip(JOINTS, delta):
            if abs(d) < 1e-6:
                continue
            m = re.search(rf'<joint name="{name}">.*?</joint>', text, re.S)
            if m is None:
                return f"в {XACRO} нет блока {name}"
            block = m.group(0)
            vals = {
                k: float(re.search(rf'<param name="{k}">([^<]+)</param>', block).group(1))
                for k in ("homed_position", "post_homing_position", "position_min", "position_max")
            }
            # Упор физически тот же, но в новых координатах он на d меньше;
            # вместе с ним сдвигаются пределы, а post_homing — если он стоял на упоре.
            new = {
                "homed_position": vals["homed_position"] - d,
                "position_min": vals["position_min"] - d,
                "position_max": vals["position_max"] - d,
                "post_homing_position": vals["post_homing_position"]
                - (d if vals["post_homing_position"] == vals["homed_position"] else 0.0),
            }
            for k, v in new.items():
                block = re.sub(rf'(<param name="{k}">)[^<]+(</param>)', rf"\g<1>{v:.3f}\g<2>", block)
            text = text[: m.start()] + block + text[m.end():]
            changed.append(f"{name} {vals['homed_position']:+.3f}→{new['homed_position']:+.3f}")
        with open(XACRO, "w", encoding="utf-8") as f:
            f.write(text)
        with self.lock:
            self.trim_saved = list(self.trim)
            self.trim_note = "записано (" + ", ".join(changed) + "). Применится после sudo systemctl restart robot.service и нового хоуминга; поправки в пульте тогда станут нулевыми."
        return None

    # --- движение -------------------------------------------------------
    def stop_motion(self, note: str) -> None:
        with self.lock:
            self.mission.clear()
            self.manual_t = 0.0
            self.mission_note = note
        self._publish(0.0, 0.0, 0.0)

    def revive_motors(self) -> None:
        self.revive_pub.publish(Empty())

    def set_imu_power(self, on: bool) -> None:
        msg = Bool()
        msg.data = bool(on)
        self.imu_power_pub.publish(msg)
        with self.lock:
            self.imu_power_on = bool(on)

    def estop(self) -> None:
        self.ears_off()
        self.stop_motion("АВАРИЙНЫЙ СТОП: момент ослаблен, включи походку заново")
        self.estop_pub.publish(Empty())
        with self.lock:
            was_standing = self.stand is not None or self.tg is not None or self.demo is not None
            self.stand = None
            self.demo = None
            if self.tg is not None:
                self.tg = None
                self.tg_note = "генератор снят аварийным стопом"
            pos = [p if p is not None else 0.0 for p in self.joint_pos]
        if was_standing:
            self._publish_forward(pos, [0.0] * 12, [0.1] * 12)

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def odom_yaw(self) -> float:
        if self.imu_q is None or self.yaw0 is None:
            return 0.0
        return wrap(euler(self.imu_q)[2] - self.yaw0)

    def control_step(self) -> None:
        now = time.monotonic()
        self.stand_step(now)
        with self.lock:
            dt = now - self.odom_t
            self.odom_t = now
            yaw = self.odom_yaw()
            gait = self.gait_active()
            if gait and now - self.cmd_out_t < 0.5:
                vx, vy, _ = self.cmd_out
                s = self.odom_scale
                self.x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt * s
                self.y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt * s
                if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.01:
                    self.trail.append((round(self.x, 3), round(self.y, 3)))

            cmd = None
            if now - self.manual_t < MANUAL_TIMEOUT:
                if self.mission:
                    self.mission.clear()
                    self.mission_note = "маршрут отменён: ручное управление"
                cmd = self.manual
            elif self.mission:
                if not gait:
                    self.mission.clear()
                    self.mission_note = "маршрут отменён: походка не включена"
                else:
                    cmd = self._mission_cmd(yaw)

            if cmd is not None and gait:
                self.sending = True
                vx, vy, wz = cmd
            elif self.sending:
                self.sending = False
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                return
        self._publish(vx, vy, wz)

    def _mission_cmd(self, yaw: float):
        step = self.mission[0]
        kind = step["kind"]
        if kind == "turn":
            if "target" not in step:
                step["target"] = wrap(yaw + math.radians(step["deg"]))
            err = wrap(step["target"] - yaw)
            if abs(err) < math.radians(3):
                return self._next_step()
            wz = clamp(2.0 * err, -self.wmax, self.wmax)
            if abs(wz) < 0.25:
                wz = math.copysign(0.25, wz)
            self.mission_note = f"поворот: осталось {math.degrees(err):+.0f}°"
            return (0.0, 0.0, wz)

        if kind == "straight":
            if "x0" not in step:
                step["x0"], step["y0"], step["h0"] = self.x, self.y, yaw
            h0 = step["h0"]
            progress = (self.x - step["x0"]) * math.cos(h0) + (self.y - step["y0"]) * math.sin(h0)
            rem = step["m"] - progress
            if abs(rem) < 0.03:
                return self._next_step()
            v = clamp(0.8 * abs(rem) + 0.08, 0.0, self.vmax)
            self.mission_note = f"прямо: осталось {rem:+.2f} м (по оценке)"
            return (math.copysign(v, rem), 0.0, clamp(2.0 * wrap(h0 - yaw), -self.wmax, self.wmax))

        dx = step["x"] - self.x
        dy = step["y"] - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.06:
            return self._next_step()
        err = wrap(math.atan2(dy, dx) - yaw)
        wz = clamp(2.0 * err, -self.wmax, self.wmax)
        if abs(err) > math.radians(25):
            self.mission_note = f"разворот к точке ({step['x']:+.2f}, {step['y']:+.2f}): {math.degrees(err):+.0f}°"
            return (0.0, 0.0, wz)
        v = clamp(0.8 * dist + 0.08, 0.0, self.vmax) * math.cos(err)
        self.mission_note = f"иду к ({step['x']:+.2f}, {step['y']:+.2f}): {dist:.2f} м"
        return (v, 0.0, wz)

    def _next_step(self):
        self.mission.pop(0)
        if not self.mission:
            self.mission_note = "маршрут пройден (по оценке одометрии)"
        return (0.0, 0.0, 0.0)

    # --- фон ------------------------------------------------------------
    def system_loop(self) -> None:
        while rclpy.ok():
            self.service = subprocess.run(
                ["systemctl", "is-active", "robot.service"], capture_output=True, text=True
            ).stdout.strip()
            homing = read_homing()
            with self.lock:
                if self.homing["when"] is not None and homing["when"] != self.homing["when"]:
                    # Новый хоуминг уже с записанными нулями — старые поправки стали бы двойными.
                    self.trim = [0.0] * 12
                    self.trim_saved = [0.0] * 12
                    self.trim_note = "robot.service перезапущен: поправки обнулены, нули из xacro"
                self.homing = homing
            time.sleep(15.0)

    def legs(self) -> dict:
        out = {}
        for leg in LEGS:
            hip = self.tf_buffer.lookup_transform("base_link", f"leg_{leg}_1", Time())
            knee = self.tf_buffer.lookup_transform("base_link", f"leg_{leg}_3", Time())
            kt = knee.transform
            kq = (kt.rotation.x, kt.rotation.y, kt.rotation.z, kt.rotation.w)
            f = quat_rotate(kq, FOOT[leg])
            ht = hip.transform.translation
            out[leg] = {
                "hip": [ht.x, ht.y, ht.z],
                "knee": [kt.translation.x, kt.translation.y, kt.translation.z],
                "foot": [kt.translation.x + f[0], kt.translation.y + f[1], kt.translation.z + f[2]],
            }
        return out

    def legs_from_q(self, q12: list[float]) -> dict:
        """Лапки по команде (не по факту) — призрак на 3D-модели."""
        from pupper_kinematics import LEG_INDEX
        out = {}
        for leg, i in LEG_INDEX.items():
            hip, knee, foot = self.tg_gait.kin.fk_links(leg, q12[i:i + 3])
            out[leg] = {
                "hip": [float(v) for v in hip],
                "knee": [float(v) for v in knee],
                "foot": [float(v) for v in foot],
            }
        return out

    def snapshot(self) -> dict:
        now = time.monotonic()
        legs = None
        if all(self.tf_buffer.can_transform("base_link", f"leg_{leg}_3", Time()) for leg in LEGS):
            legs = self.legs()
        with self.ear_lock:
            gpio_l, gpio_r = self._ear_sides()
            ear_state = {
                "mode": self.ear_mode,
                "phase": self.ear_phase,
                "params": dict(self.ear_params),
                "limits": {k: list(v) for k, v in EAR_LIMITS.items()},
                "pins": {"left": gpio_l, "right": gpio_r},
                "note": self.ear_note,
                "error": self.ear_err,
                "open": self.ear_servos is not None,
            }
        with self.lock:
            imu = None
            if self.imu_q is not None:
                q = self.imu_q
                r, p, y = euler(q)
                grav = quat_rotate((-q[0], -q[1], -q[2], q[3]), (0.0, 0.0, -1.0))
                imu = {
                    "q": q,
                    "roll": math.degrees(r),
                    "pitch": math.degrees(p),
                    "yaw": math.degrees(y),
                    "gyro": self.imu_gyro,
                    "gravity": grav,
                    "age": now - self.imu_t,
                    "rate": self.imu_rate,
                    "identity": imu_identity(q),
                    "usable": imu_usable(q, now - self.imu_t),
                }
            hist = [[round(h[0] - now, 2), *[round(v, 3) for v in h[1:]]] for h in self.imu_hist]
            joints = []
            stand = self.stand
            tg = self.tg
            cmd12 = None
            if stand is not None:
                cmd12 = list(stand["cmd"])
            elif tg is not None:
                cmd12 = list(tg["cmd"])
            elif self.demo is not None:
                cmd12 = list(self.demo["cmd"])
            elif self.gait_active() and self.pos_cmd and len(self.pos_cmd) == 12:
                cmd12 = list(self.pos_cmd)
            for i, name in enumerate(JOINTS):
                cmd = cmd12[i] if cmd12 is not None else None
                joints.append({
                    "name": name,
                    "pos": self.joint_pos[i],
                    "vel": self.joint_vel[i],
                    "eff": self.joint_eff[i],
                    "kp": stand["kp"][i] if stand is not None else None,
                    "cmd": cmd,
                    "def": DEFAULT_POS[i],
                    "trim": self.trim[i],
                    "trim_saved": self.trim_saved[i],
                })
            legs_cmd = self.legs_from_q(cmd12) if cmd12 is not None else None
            req = self.rehome_req
            homing_error = None
            if req is not None and not req["acked"] and now - req["t"] > REHOME_ACK_TIMEOUT:
                homing_error = (f"робот не ответил на /motor_rehome за {REHOME_ACK_TIMEOUT:.0f} с: "
                                "control_board_hardware_interface собран без повторного хоуминга "
                                "или robot.service не запущен")
            hs = self.homing_status
            demo_state = {
                "active": self.demo is not None,
                "note": self.demo_note,
                "params": dict(self.demo_params),
                "limits": {k: list(v) for k, v in self.demo_limits().items()},
                "nom_h": self.demo_nom_h,
                "ik_err": None if self.demo is None else self.demo["ik_err"],
                "clamped": False if self.demo is None else self.demo["clamped"],
                "anim_on": self.demo is not None and self.demo["anim_on"],
                "anim_amp": 0.0 if self.demo is None else self.demo["anim_amp"],
                "tri": self.demo_triangle(self.demo_params),
                "homing": {
                    "status": None if hs is None else {k: v for k, v in hs.items() if k != "rx_t"},
                    "requested": req is not None,
                    "error": homing_error,
                    "homed": hs is not None and hs["state"] == "done" and not hs["skipped"],
                },
            }
            return {
                "service": self.service,
                "controllers": self.controllers,
                "controllers_age": now - self.controllers_t if self.controllers_t else None,
                "gait": self.gait_active(),
                "stand": self.stand is not None,
                "demo": demo_state,
                "ears": ear_state,
                "trim_note": self.trim_note,
                "tg": None if tg is None else {
                    "stage": tg["stage"], "amp": tg["amp"], "phase": tg["phase"],
                    "att": None if tg["att"] is None else [math.degrees(v) for v in tg["att"]],
                    "dz": tg["dz"], "ik_err": tg["ik_err"],
                    "yaw_err": None if tg["yaw_ref"] is None else math.degrees(wrap(tg["yaw"] - tg["yaw_ref"])),
                    "yaw_turn": tg["yaw_turn"],
                },
                "tg_params": dict(self.tg_params),
                "tg_note": self.tg_note,
                "record": {"active": self.recorder.active, "t": now - self.rec_t0 if self.recorder.active else None,
                           "note": self.rec_note},
                "imu": imu,
                "imu_power": self.imu_power_on,
                "imu_hist": hist,
                "joints": joints,
                "joints_age": now - self.joint_t if self.joint_t else None,
                "legs": legs,
                "legs_cmd": legs_cmd,
                "ground_z": POSE_GROUND_Z,
                "policy_gravity": self.policy_gravity if self.gait_active() else None,
                "policy_offsets": dict(self.policy_offsets),
                "policy_offsets_note": self.policy_offsets_note,
                "latency": {"policy": self.lat_policy, "imu": self.lat_imu},
                "cmd_out": self.cmd_out if now - self.cmd_out_t < 0.6 else None,
                "mux_source": self.mux_source,
                "manual": now - self.manual_t < MANUAL_TIMEOUT,
                "mission": [
                    {k: v for k, v in s.items() if k in ("kind", "x", "y", "deg", "m")} for s in self.mission
                ],
                "mission_note": self.mission_note,
                "odom": {"x": self.x, "y": self.y, "yaw": self.odom_yaw(), "trail": list(self.trail)},
                "limits": {"vmax": self.vmax, "vymax": self.vymax, "wmax": self.wmax, "scale": self.odom_scale},
                "homing": self.homing,
                "log": list(self.log),
            }

    # --- API ------------------------------------------------------------
    def api_cmd(self, body: dict) -> None:
        with self.lock:
            self.manual = (
                clamp(float(body.get("vx", 0.0)), -1.0, 1.0) * self.vmax,
                clamp(float(body.get("vy", 0.0)), -1.0, 1.0) * self.vymax,
                clamp(float(body.get("wz", 0.0)), -1.0, 1.0) * self.wmax,
            )
            self.manual_t = time.monotonic()

    def api_mission(self, body: dict) -> str | None:
        steps = []
        for s in body.get("steps", []):
            kind = s.get("kind")
            if kind == "goal":
                steps.append({"kind": "goal", "x": float(s["x"]), "y": float(s["y"])})
            elif kind == "turn":
                steps.append({"kind": "turn", "deg": float(s["deg"])})
            elif kind == "straight":
                steps.append({"kind": "straight", "m": float(s["m"])})
            else:
                return f"неизвестный шаг {kind!r}"
        with self.lock:
            if not self.gait_active():
                return "сначала включи походку"
            if not body.get("append"):
                self.mission.clear()
            self.mission.extend(steps)
            self.mission_note = f"маршрут: {len(self.mission)} шаг(ов)"
        return None

    def api_limits(self, body: dict) -> None:
        with self.lock:
            self.vmax = clamp(float(body.get("vmax", self.vmax)), 0.05, 0.75)
            self.vymax = clamp(float(body.get("vymax", self.vymax)), 0.05, 0.5)
            self.wmax = clamp(float(body.get("wmax", self.wmax)), 0.2, 2.0)
            self.odom_scale = clamp(float(body.get("scale", self.odom_scale)), 0.2, 2.0)

    def api_odom_reset(self) -> None:
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.trail.clear()
            if self.imu_q is not None:
                self.yaw0 = euler(self.imu_q)[2]


def make_handler(node: Dashboard):
    page_path = os.path.join(HERE, "index.html")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args) -> None:
            return

        def _send(self, code: int, ctype: str, raw: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def _json(self, code: int, payload: dict) -> None:
            self._send(code, "application/json", json.dumps(payload).encode())

        def _path(self) -> str:
            return self.path.split("?", 1)[0]

        def do_GET(self) -> None:  # noqa: N802
            if screen_ctl.handle_get(self):
                return
            path = self._path()
            if path == "/api/state":
                self._json(200, node.snapshot())
            elif path == "/api/vpn":
                self._json(200, {"active": vpn_active(), "unit": VPN_UNIT})
            elif path in ("/", "/index.html"):
                with open(page_path, "rb") as f:
                    self._send(200, "text/html; charset=utf-8", f.read())
            else:
                self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            path = self._path()
            # Экранные POST сами читают тело (upload / json).
            if path in (
                "/api/upload",
                "/api/youtube",
                "/api/show",
                "/api/show_live",
                "/api/screen_stop",
                "/api/open_screen",
            ):
                if screen_ctl.handle_post(self):
                    return
                self.send_error(404)
                return
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n).decode()) if n > 0 else {}
            err = None
            if path == "/api/cmd":
                node.api_cmd(body)
            elif path == "/api/stop":
                node.ears_off()
                node.stop_motion("стоп")
                if node.tg is not None:
                    node.set_tg(False)
            elif path == "/api/tg":
                err = node.set_tg(bool(body.get("on")))
            elif path == "/api/tg_params":
                err = node.api_tg_params(body)
            elif path == "/api/record":
                err = node.api_record(body)
            elif path == "/api/estop":
                node.estop()
            elif path == "/api/revive":
                node.revive_motors()
            elif path == "/api/imu_power":
                node.set_imu_power(bool(body.get("on")))
            elif path == "/api/gait":
                err = node.set_gait(bool(body.get("on")))
            elif path == "/api/policy_offset":
                err = node.api_policy_offset(body)
            elif path == "/api/mission":
                err = node.api_mission(body)
            elif path == "/api/limits":
                node.api_limits(body)
            elif path == "/api/stand":
                err = node.set_stand()
            elif path == "/api/demo_homing":
                err = node.api_demo_homing(body)
            elif path == "/api/demo_stance":
                err = node.set_demo_stance()
            elif path == "/api/demo_anim":
                err = node.api_demo_anim(bool(body.get("on")))
            elif path == "/api/demo_params":
                err = node.api_demo_params(body)
            elif path == "/api/demo_release":
                err = node.demo_release()
            elif path == "/api/ears":
                err = node.api_ears(str(body.get("mode", "")))
            elif path == "/api/ears_params":
                err = node.api_ears_params(body)
            elif path == "/api/trim":
                err = node.api_trim(body)
            elif path == "/api/trim_save":
                err = node.api_trim_save()
            elif path == "/api/odom_reset":
                node.api_odom_reset()
            elif path == "/api/vpn":
                err = vpn_set(bool(body.get("on")))
            else:
                self.send_error(404)
                return
            if err:
                self._json(400, {"error": err})
            else:
                self._json(200, {"ok": True})

        def do_DELETE(self) -> None:  # noqa: N802
            if screen_ctl.handle_delete(self):
                return
            self.send_error(404)

    return Handler


def main() -> None:
    screen_ctl.ensure_library()
    rclpy.init()
    node = Dashboard()
    # Все колбэки в одной MutuallyExclusive-группе узла, так что потоки
    # MultiThreadedExecutor параллелизма не давали, только накладные расходы.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), make_handler(node))
    print(f"Пульт Фенека: http://<ip-робота>:{PORT}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        screen_ctl.stop_chromium()
        node.ears_off()
        node.stop_motion("пульт закрыт")
        server.shutdown()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
