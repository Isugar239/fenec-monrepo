#!/usr/bin/env python3
"""

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

import cv2
import numpy as np
import rclpy
import zmq
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
from sensor_msgs.msg import CompressedImage, Imu, JointState, Joy, JoyFeedback, JoyFeedbackArray
from std_msgs.msg import Empty, Float32, Float32MultiArray, Float64MultiArray, String, Bool
from tf2_ros import Buffer, TransformListener

PORT = 8091
EAR_POSE_ENDPOINT = "ipc:///run/user/1000/pupper-ears.sock"
HERE = os.path.dirname(os.path.abspath(__file__))
# prefs: vmax/уши, лежат вне репо
PREFS_PATH = os.path.expanduser("~/.config/pupper-dashboard/prefs.json")
# Превью в пульте: меньше JPEG + меньше FPS → меньше очередь по сети.
CAM_PREVIEW_MAX_W = 480
CAM_PREVIEW_FPS = 8.0
CAM_JPEG_QUALITY = 45
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from open_loop_gait import GaitIK, smooth  # noqa: E402
from pupper_kinematics import LEG_INDEX, Kinematics  # noqa: E402
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
# лапа в кадре leg_*_3
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
# стойка без политики
STAND_KP = 12.0
STAND_KP_MAX = 20.0  # потолок kp_max в xacro
# слабое отведение: удвоенный kp
STAND_KP_MAX_JOINT = {"leg_front_l_2": 40.0}
WEAK_KP_SCALE = 2.0
WEAK_JOINTS = {name: JOINTS.index(name) for name in STAND_KP_MAX_JOINT}
STAND_KP_RATE = 4.0  # kp/с при ошибке
STAND_KD_PER_KP = 0.033
STAND_KI = 1.5
STAND_I_MAX = 0.6
STAND_ERR_OK = 0.06
STAND_RELAX = 3.0  # спад интеграла, с
STAND_RAMP = 3.0
TRIM_MAX = 1.6
XACRO = os.path.join(
    os.path.dirname(os.path.dirname(HERE)),
    "ros2_ws/src/pupper_v3_description/description/components.xacro",
)
# рысь без политики (open_loop_gait + IMU)
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
# рабочие дефолты рыси; imu_ki маленький — иначе качает
TG_DEFAULTS = {
    "freq": 0.9, "stride": 0.032, "lift": 0.028, "height": 0.0, "swing": 0.5, "turn": 0.0,
    "kp": 12.0, "kd": 0.55,
    "imu_on": True, "imu_gain": 0.2, "imu_ki": 0.05, "imu_damp": 0.05,
}
TG_IMU_TAU = 0.15  # сглаживание IMU, с
TG_IMU_STALE = 0.2  # без IMU гасим шаг
TG_IMU_I_MAX = 0.15
TG_DZ_MAX = 0.015  # м, поправка высоты лапы
POSE_GROUND_Z = -0.14  # пол в кадре корпуса, м
# удержание курса: минус — turn и ошибка yaw в одну сторону
TG_YAW_KP = 1.2
TG_YAW_KI = 0.6
TG_YAW_KD = 0.08
TG_YAW_I_MAX = 0.35
TG_YAW_MAX = 0.45
# демо: треугольник IK одной ноги (вперёд-низ → назад-низ → верх)
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
# пределы удлинения ноги относительно стойки
DEMO_EXT = (-0.025, 0.03)
DEMO_TRI_DROP = 0.02  # низ треугольника ниже стойки, м
DEMO_DEFAULTS = {
    "leg": "front_r", "height": None, "shift_x": 0.0, "spread": 0.0,
    "anim_h": 0.04, "anim_x": 0.05, "anim_T": 4.5,
    "kp": 12.0, "kd": 0.4, "ramp": 3.0, "follow": 0.03,
}
DEMO_ANIM_FADE = 1.5
# уши: hw PWM GPIO12/13, −1..1; soft PWM греет сервы
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
    """pwmchip для dtoverlay=pwm-2chan."""
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
    """Канал hw PWM. None = импульс снят."""

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
        # enable=0 до смены period
        _sysfs_write(os.path.join(self.path, "enable"), "0")
        _sysfs_write(os.path.join(self.path, "period"), str(EAR_PERIOD_NS))
        _sysfs_write(os.path.join(self.path, "duty_cycle"), "0")
        _sysfs_write(os.path.join(self.path, "polarity"), "normal")
        self.value = None
        self._duty = None

    def detach(self) -> None:
        # импульс снять, unexport не трогаем
        if self.value is None and self._duty is None:
            return
        _sysfs_write(os.path.join(self.path, "enable"), "0")
        self.value = None
        self._duty = None

    def set(self, value: float) -> None:
        value = clamp(value, -1.0, 1.0)
        # −1..1 → 500..2500 мкс
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
ROBOT_UNIT = "robot.service"
# /joy: круг/треуг/квадрат/R1/R2 (крест — joy_util)
JOY_CIRCLE = 1
JOY_TRIANGLE = 2
JOY_SQUARE = 3
JOY_L1 = 4
JOY_R1 = 5
JOY_R2 = 7
JOY_DPAD_Y_AXIS = 7
JOY_DPAD_THRESHOLD = 0.5
# R2: отпускать уши только после удержания
JOY_R2_HOLD_S = 0.5
JOY_SQUARE_HOLD_S = 0.5
JOINT_STATE_MAX_AGE = 0.25
# ждать /motor_homing/status после restart
REHOME_ACK_TIMEOUT = 60.0
CMD_TOPIC = "/llm_cmd_vel"
MANUAL_TIMEOUT = 0.35
CTRL_DT = 0.05
# тяжёлые топики (~250 Гц) — на poll_node, читаем пачкой раз в POLL_DT
POLL_DT = 0.02
HISTORY_SECONDS = 12.0
FALL_ALARM_ANGLE = math.radians(90.0)
LIGHTBAR_EFFORT_FULL_SCALE_NM = 3.0
SPEED_MODES = (
    ("eco", "Eco", 0.75, 0.5, 2.0, (0, 255, 0)),
    ("comfort", "Comfort", 1.5, 1.0, 3.0, (0, 128, 255)),
    ("sport", "Sport", 2.25, 2.0, 6.0, (190, 0, 255)),
)
SPEED_MODE_FLASH_SECONDS = 1.0 / 3.0


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
    """Есть свежий IMU."""
    return q is not None and age <= TG_IMU_STALE


def read_homing() -> dict:
    # хоуминг только в начале запуска сервиса
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
        self.fall_alarm = False
        self.fall_rumble_on = False
        lightbars = glob.glob("/sys/class/leds/input*:rgb:indicator")
        self.lightbar_dir = lightbars[0] if len(lightbars) == 1 else None
        self.lightbar_rgb: tuple[int, int, int] | None = None

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
        # trim к DEFAULT_POS
        self.trim = [0.0] * 12
        # trim, уже в xacro (до restart ещё старые нули)
        self.trim_saved = [0.0] * 12
        self.trim_note = ""
        self.tg: dict | None = None
        self.tg_params = dict(TG_DEFAULTS)
        self.tg_gait = GaitIK(Kinematics.from_xacro())
        # свой IK, отдельно от генератора
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
        self.ear_pose = (None, None)
        self.ear_pose_dirty = False
        self.ear_pose_context = zmq.Context()
        self.ear_pose_pub = self.ear_pose_context.socket(zmq.PUB)
        self.ear_pose_pub.set_hwm(1)
        self.ear_pose_pub.bind(EAR_POSE_ENDPOINT)
        self.ear_phase = 0.0
        self.ear_t_last = 0.0
        self.ear_note = ""
        self.ear_err = ""
        self.homing_status: dict | None = None
        self.rehome_req: dict | None = None
        self.joy_prev: list[int] | None = None
        self.joy_busy = False
        self.joy_r2_since: float | None = None
        self.joy_r2_off = False
        self.joy_square_since: float | None = None
        self.joy_square_fired = False
        self.joy_dpad_y = 0
        self.prefs_lock = threading.Lock()
        self.prefs_timer: threading.Timer | None = None
        self.tg_note = ""
        self.speed_mode_index = 1
        self.speed_mode_pending = False
        self.speed_mode_flash_until = 0.0
        self.speed_mode_flash_rgb = (0, 0, 0)
        self.vmax = SPEED_MODES[self.speed_mode_index][2]
        self.vymax = SPEED_MODES[self.speed_mode_index][3]
        self.wmax = SPEED_MODES[self.speed_mode_index][4]
        self.speed_mode_restore_pending = False
        self.odom_scale = 1.0
        self._load_prefs()

        self.x = 0.0
        self.y = 0.0
        self.yaw0 = None
        self.odom_t = time.monotonic()
        self.trail: deque = deque(maxlen=3000)

        self.log: deque = deque(maxlen=60)
        self.homing = {"when": None, "rows": [], "notes": ["читаю журнал…"]}
        self.service = "?"
        self.camera_jpeg: bytes | None = None
        self.camera_t = 0.0
        self.camera_rx_t = 0.0
        self.camera_enc_t = 0.0
        self.camera_bytes = 0
        self.detection_jpeg: bytes | None = None
        self.detection_t = 0.0
        self.detection_rx_t = 0.0
        self.detection_enc_t = 0.0
        self.detection_bytes = 0
        self.camera_cond = threading.Condition(self.lock)

        self.poll_node = Node("pupper_dashboard_poll")
        self.tf_buffer = Buffer()
        # очередь под 250 Гц
        self.tf_listener = TransformListener(self.tf_buffer, self.poll_node)
        self.imu_sub = self.poll_node.create_subscription(
            Imu, "/imu_sensor_broadcaster/imu", lambda _msg: None, 50
        )
        # только последнее
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
            self.poll_node.create_subscription(
                CompressedImage, "/camera/image_raw/compressed", self.on_camera, 1
            ),
            self.poll_node.create_subscription(
                CompressedImage, "/glaz_detect/annotated/compressed", self.on_detection_camera, 1
            ),
        ]
        # Для кнопок важнее свежий кадр, чем гарантированная доставка старого нажатия.
        self.create_subscription(
            Joy, "/joy", self.on_joy,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_out, 10)
        self.create_subscription(String, "/cmd_vel_mux/active_source", self.on_mux, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 50)

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.estop_pub = self.create_publisher(Empty, "/emergency_stop", 10)
        self.revive_pub = self.create_publisher(Empty, "/motor_revive", 10)
        self.fwd_pubs = [
            self.create_publisher(Float64MultiArray, f"/{name}/commands", 10) for name in FORWARD
        ]
        self.imu_power_pub = self.create_publisher(Bool, "/imu_power", 10)
        self.joy_feedback_pub = self.create_publisher(JoyFeedbackArray, "/joy/set_feedback", 10)
        self.animation_pub = self.create_publisher(
            String, "/animation_controller_py/animation_select", 10
        )
        self.imu_power_on = True
        # transient_local — статус хоуминга и опоздавшим
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
        self.teleop_set = self.create_client(
            SetParameters, "/teleop_twist_joy_node/set_parameters"
        )

        self.create_timer(0.5, self._restore_speed_mode)
        self.create_timer(POLL_DT, self.drain_topics)
        self.create_timer(CTRL_DT, self.control_step)
        self.create_timer(TG_DT, self.demo_step)
        self.create_timer(TG_DT, self.ear_step)
        self.create_timer(1.0, self.poll_controllers)
        self.create_timer(2.0, self.poll_policy_offsets)
        self.create_timer(1.0, self.count_rates)
        self.create_timer(0.05, self.fall_rumble_step)
        threading.Thread(target=self.system_loop, daemon=True).start()

    # --- подписки -------------------------------------------------------
    @staticmethod
    def _take(sub, raw: bool):
        with sub.handle:
            got = sub.handle.take_message(sub.msg_type, raw)
        return None if got is None else got[0]

    def drain_topics(self) -> None:
        # IMU: считаем все, парсим последнее
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
            # ~25 Гц в историю
            if not self.imu_hist or now - self.imu_hist[-1][0] >= 1.5 * POLL_DT:
                r, p, y = euler(q)
                self.imu_hist.append((now, math.degrees(r), math.degrees(p), math.degrees(y), *g))
                while self.imu_hist and now - self.imu_hist[0][0] > HISTORY_SECONDS:
                    self.imu_hist.popleft()
            self.imu_t = now

    def _joy_rumble(self, intensity: float) -> None:
        """Вибрация DualSense через установленный joy_linux_node."""
        msg = JoyFeedbackArray()
        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.id = 0
        feedback.intensity = float(clamp(intensity, 0.0, 1.0))
        msg.array = [feedback]
        self.joy_feedback_pub.publish(msg)

    def _set_lightbar(self, red: int, green: int, blue: int) -> None:
        """Установить RGB lightbar DualSense через драйвер hid-playstation."""
        rgb = (red, green, blue)
        lightbar_value = (
            None if self.lightbar_dir is None
            else f"{self.lightbar_dir}/multi_intensity"
        )
        if (
            lightbar_value is None
            or not os.path.isdir(self.lightbar_dir)
            or not os.access(lightbar_value, os.W_OK)
        ):
            lightbars = [
                path
                for path in glob.glob("/sys/class/leds/input*:rgb:indicator")
                if os.access(f"{path}/multi_intensity", os.W_OK)
            ]
            self.lightbar_dir = lightbars[0] if len(lightbars) == 1 else None
            self.lightbar_rgb = None
        if self.lightbar_dir is None or rgb == self.lightbar_rgb:
            return
        with open(f"{self.lightbar_dir}/multi_intensity", "w", encoding="ascii") as file:
            file.write(f"{red} {green} {blue}\n")
        with open(f"{self.lightbar_dir}/brightness", "w", encoding="ascii") as file:
            file.write("255\n")
        self.lightbar_rgb = rgb

    def fall_rumble_step(self) -> None:
        """Индицировать падение непрерывной вибрацией и красным светом."""
        now = time.monotonic()
        with self.lock:
            q = self.imu_q
            fresh = q is not None and now - self.imu_t <= TG_IMU_STALE
            if fresh:
                roll, pitch, _ = euler(q)
                alarm = abs(roll) >= FALL_ALARM_ANGLE or abs(pitch) >= FALL_ALARM_ANGLE
            else:
                alarm = False
            changed = alarm != self.fall_alarm
            self.fall_alarm = alarm
            if alarm:
                self.fall_rumble_on = True
                intensity = 1.0
            else:
                was_rumbling = self.fall_rumble_on
                self.fall_rumble_on = False
                if not changed and not was_rumbling:
                    intensity = None
                else:
                    intensity = 0.0
            estimated_effort = max(
                (abs(effort) for effort in self.joint_eff if effort is not None and math.isfinite(effort)),
                default=0.0,
            )
        if intensity is not None:
            self._joy_rumble(intensity)
        if alarm:
            self._set_lightbar(255 if int(now * 4.0) % 2 == 0 else 0, 0, 0)
        elif now < self.speed_mode_flash_until:
            self._set_lightbar(*self.speed_mode_flash_rgb)
        else:
            blue = round(255.0 * min(estimated_effort / LIGHTBAR_EFFORT_FULL_SCALE_NM, 1.0))
            self._set_lightbar(0, 0, blue)

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

    def on_camera(self, msg: CompressedImage) -> None:
        self._store_preview(msg, "camera")

    def on_detection_camera(self, msg: CompressedImage) -> None:
        self._store_preview(msg, "detection")

    def _store_preview(self, msg: CompressedImage, kind: str) -> None:
        now = time.monotonic()
        with self.camera_cond:
            setattr(self, f"{kind}_rx_t", now)
            if now - getattr(self, f"{kind}_enc_t") < 1.0 / CAM_PREVIEW_FPS:
                return
        if now - getattr(self, f"{kind}_enc_t") < 1.0 / CAM_PREVIEW_FPS:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn("%s: не разобрал JPEG", kind)
            return
        h, w = img.shape[:2]
        if w > CAM_PREVIEW_MAX_W:
            nh = max(1, int(round(h * (CAM_PREVIEW_MAX_W / float(w)))))
            img = cv2.resize(img, (CAM_PREVIEW_MAX_W, nh), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), CAM_JPEG_QUALITY]
        )
        if not ok:
            self.get_logger().warn("%s: cv2.imencode не смог сжать превью", kind)
            return
        jpeg = buf.tobytes()
        with self.camera_cond:
            setattr(self, f"{kind}_jpeg", jpeg)
            setattr(self, f"{kind}_bytes", len(jpeg))
            setattr(self, f"{kind}_t", now)
            setattr(self, f"{kind}_enc_t", now)
            self.camera_cond.notify_all()

    def preview_frame(self, kind: str, wait: float = 0.0) -> tuple[bytes | None, float]:
        with self.camera_cond:
            if wait > 0.0 and getattr(self, f"{kind}_jpeg") is None:
                self.camera_cond.wait(timeout=wait)
            jpeg = getattr(self, f"{kind}_jpeg")
            stamp = getattr(self, f"{kind}_t")
            age = 0.0 if stamp <= 0.0 else time.monotonic() - stamp
            return jpeg, age

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

    def on_joy(self, msg: Joy) -> None:
        buttons = [int(b) for b in msg.buttons]
        prev = self.joy_prev
        self.joy_prev = buttons
        now = time.monotonic()
        dpad_axis = msg.axes[JOY_DPAD_Y_AXIS] if JOY_DPAD_Y_AXIS < len(msg.axes) else 0.0
        dpad_y = (
            1 if dpad_axis >= JOY_DPAD_THRESHOLD
            else -1 if dpad_axis <= -JOY_DPAD_THRESHOLD
            else 0
        )
        dpad_rose = self.joy_dpad_y == 0 and dpad_y != 0
        self.joy_dpad_y = dpad_y
        r2 = JOY_R2 < len(buttons) and bool(buttons[JOY_R2])
        square = JOY_SQUARE < len(buttons) and bool(buttons[JOY_SQUARE])
        if not r2:
            self.joy_r2_since = None
            self.joy_r2_off = False
        elif self.joy_r2_since is None:
            self.joy_r2_since = now
        elif not self.joy_r2_off and now - self.joy_r2_since >= JOY_R2_HOLD_S:
            self.joy_r2_off = True
            self.ears_off()
            self.get_logger().info("джойстик R2: уши отпущены")
        if not square:
            self.joy_square_since = None
            self.joy_square_fired = False
        elif self.joy_square_since is None:
            self.joy_square_since = now
        elif not self.joy_square_fired and now - self.joy_square_since >= JOY_SQUARE_HOLD_S:
            self.joy_square_fired = True
            self._joy_run("стойка", self.set_stand)
        if prev is None:
            return

        def rose(i: int) -> bool:
            return i < len(buttons) and i < len(prev) and not prev[i] and buttons[i]

        # □ hold 0.5 s: стойка; ○ демо; △ хоуминг; L1 скорость; R1 уши.
        if rose(JOY_CIRCLE):
            self._joy_run("кручение лапы", lambda: self.api_demo_anim(True))
        elif rose(JOY_TRIANGLE):
            self._joy_run("хоуминг", self.api_demo_homing)
        elif rose(JOY_L1):
            self._cycle_speed_mode()
        elif rose(JOY_R1):
            threading.Thread(target=self._joy_ears_on, daemon=True).start()
        elif dpad_rose and dpad_y > 0:
            self._joy_run("прыжок", lambda: self._play_animation("jump"))
        elif dpad_rose and dpad_y < 0:
            self._joy_run("посадка назад", lambda: self._play_animation("rear_sit"))

    def _joy_run(self, name: str, fn) -> None:
        with self.lock:
            if self.joy_busy:
                return
            self.joy_busy = True

        def work() -> None:
            try:
                err = fn()
                self.get_logger().info("джойстик %s: %s" % (name, err or "ок"))
            finally:
                with self.lock:
                    self.joy_busy = False

        threading.Thread(target=work, daemon=True).start()

    def _joy_ears_on(self) -> None:
        err = self.api_ears("sync")
        self.get_logger().info("джойстик R1 уши: %s" % (err or "ок"))

    def _play_animation(self, name: str) -> str | None:
        with self.lock:
            if any(position is None for position in self.joint_pos):
                return "нет /joint_states"
            if time.monotonic() - self.joint_t > JOINT_STATE_MAX_AGE:
                return "данные /joint_states устарели"
            if self.fall_alarm:
                return "робот наклонён; анимация отменена"
            self.stand = None
            self.tg = None
            self.demo = None
        if self.animation_pub.get_subscription_count() == 0:
            return "animation_controller_py не отвечает"
        self.stop_motion(name)
        self.animation_pub.publish(String(data=name))
        return None

    def _cycle_speed_mode(self) -> None:
        with self.lock:
            if self.speed_mode_pending:
                return
            mode_index = (self.speed_mode_index + 1) % len(SPEED_MODES)
            self.speed_mode_index = mode_index
            self.speed_mode_restore_pending = True
            _name, _label, vx, vy, yaw, rgb = SPEED_MODES[mode_index]
            self.vmax, self.vymax, self.wmax = vx, vy, yaw
            self.speed_mode_flash_rgb = rgb
            self.speed_mode_flash_until = time.monotonic() + SPEED_MODE_FLASH_SECONDS
        self._remember()
        self._restore_speed_mode(origin="L1")

    def _restore_speed_mode(self, origin: str = "восстановление") -> None:
        with self.lock:
            if not self.speed_mode_restore_pending or self.speed_mode_pending:
                return
            mode_index = self.speed_mode_index
        self._set_speed_mode(mode_index, flash=False, origin=origin)

    def _set_speed_mode(self, mode_index: int, flash: bool, origin: str) -> None:
        name, label, vx, vy, yaw, rgb = SPEED_MODES[mode_index]
        if not self.teleop_set.service_is_ready():
            return
        with self.lock:
            self.speed_mode_pending = True
        request = SetParameters.Request()
        request.parameters = []
        for parameter_name, value in (
            ("scale_linear.x", vx),
            ("scale_linear.y", vy),
            ("scale_angular.yaw", yaw),
        ):
            parameter = Parameter()
            parameter.name = parameter_name
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
            request.parameters.append(parameter)
        future = self.teleop_set.call_async(request)
        future.add_done_callback(
            lambda result: self._on_speed_mode_set(
                result, mode_index, name, label, vx, vy, yaw, rgb, flash, origin
            )
        )

    def _on_speed_mode_set(
        self, future, mode_index: int, name: str, label: str, vx: float, vy: float, yaw: float,
        rgb: tuple[int, int, int], flash: bool, origin: str,
    ) -> None:
        response = future.result()
        if not all(result.successful for result in response.results):
            with self.lock:
                self.speed_mode_pending = False
            self.get_logger().error("L1: %s отклонён teleop_twist_joy_node", label)
            return
        with self.lock:
            self.speed_mode_pending = False
            self.speed_mode_index = mode_index
            self.vmax, self.vymax, self.wmax = vx, vy, yaw
            self.speed_mode_restore_pending = False
            if flash:
                self.speed_mode_flash_rgb = rgb
                self.speed_mode_flash_until = time.monotonic() + SPEED_MODE_FLASH_SECONDS
        self._remember()
        self.get_logger().warn(
            f"{origin}: {label} ({name}), лимиты {vx:.2f}/{vy:.2f}/{yaw:.2f}"
        )

    def on_homing_status(self, msg: String) -> None:
        # state: running|done|none
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
                    self.demo_note = "хоуминг ок" if not data["skipped"] else (
                        "хоуминг: без нуля " + ", ".join(data["skipped"])
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
                    self.policy_offsets_note = "нет параметров оффсета — пересобери и restart robot.service"
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
                self.policy_offsets_note = "контроллер без оффсетов — пересобери и restart"
            return
        values = {}
        for name, value in zip(POLICY_OFFSETS, res.values):
            if value.type != ParameterType.PARAMETER_DOUBLE:
                with self.lock:
                    self.policy_offsets_note = "контроллер без оффсетов — пересобери и restart"
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
        enable_pose_mods = Parameter()
        enable_pose_mods.name = "pose_mods"
        enable_pose_mods.value.type = ParameterType.PARAMETER_BOOL
        enable_pose_mods.value.bool_value = True
        params.append(enable_pose_mods)
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
            return "neural_controller не запущен"
        with self.lock:
            self.policy_offsets.update(
                {p.name: p.value.double_value for p in params if p.name in POLICY_OFFSETS}
            )
            self.policy_offsets_note = ""
        if missing:
            return None
        return None

    def switch(self, activate: list[str], deactivate: list[str]) -> str | None:
        if not self.switch_cli.wait_for_service(timeout_sec=2.0):
            return "controller_manager не отвечает"
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
            return "switch_controller отказал"
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
                return "нет /joint_states"
            if time.monotonic() - self.joint_t > JOINT_STATE_MAX_AGE:
                return "данные /joint_states устарели; стойка отменена"
            start = list(self.joint_pos)
            active = [n for n, s in self.controllers.items() if s == "active"]
            self.tg = None
            self.demo = None
        self.stop_motion("стойка")
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
                self.tg_note = "гашу шаг → стойка"
            return None
        with self.lock:
            if any(p is None for p in self.joint_pos):
                return "нет /joint_states"
            if self.tg is not None:
                return "генератор уже идёт"
            start = list(self.joint_pos)
            active = [n for n, s in self.controllers.items() if s == "active"]
            self.stand = None
            self.demo = None
        self.stop_motion("генератор")
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
            self.tg_note = "в стойку, потом шаг"
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
            return "нет статуса хоуминга"
        if h["state"] == "running":
            return "хоуминг идёт"
        if h["state"] != "done":
            return "хоуминга не было"
        if h["skipped"]:
            return "без нуля: " + ", ".join(h["skipped"])
        return None

    def api_demo_homing(self) -> str | None:
        """Хоуминг через restart robot.service."""
        with self.lock:
            now = time.monotonic()
            req = self.rehome_req
            # таймаут restart+хоуминг
            lost = req is not None and now - req["t"] > REHOME_ACK_TIMEOUT + 90.0
            if req is not None and not lost:
                return "перезапуск robot.service уже запрошен"
            self.stand = None
            self.tg = None
            self.demo = None
        self.stop_motion("хоуминг: restart robot.service")
        r = subprocess.run(
            ["sudo", "-n", "systemctl", "restart", ROBOT_UNIT],
            capture_output=True,
            text=True,
            timeout=120,
        )
        if r.returncode != 0:
            return (r.stderr or r.stdout or f"systemctl restart {ROBOT_UNIT} failed").strip()
        with self.lock:
            self.homing_status = None
            self.rehome_req = {"t": time.monotonic(), "seq0": None, "acked": False}
            self.demo_note = f"{ROBOT_UNIT} перезапущен, жду хоуминг"
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
                return "нет /joint_states"
            start = list(self.joint_pos)
            active = [n for n, s in self.controllers.items() if s == "active"]
            self.stand = None
            self.tg = None
            self.demo = None
            p = dict(self.demo_params)
        self.stop_motion("стойка IK")
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
            self.demo_note = f"стойка IK · {p['leg']}"
        return None

    def api_demo_anim(self, on: bool) -> str | None:
        """Вкл/выкл анимацию треугольника."""
        if on:
            with self.lock:
                running = self.demo is not None
            if not running:
                err = self.set_demo_stance()
                if err:
                    return err
            with self.lock:
                self.demo["anim_on"] = True
                self.demo_note = "анимация треугольника"
            return None
        with self.lock:
            if self.demo is None:
                return "анимация не идёт: стойка не включена"
            self.demo["anim_on"] = False
            self.demo_note = "гашу анимацию"
        return None

    def _demo_targets(self, p: dict) -> dict:
        """Смещения ног до анимации."""
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
        self.stop_motion("момент снят")
        self._publish_forward(pos, [0.0] * 12, [0.1] * 12)
        others = [n for n in active if n not in FORWARD and "broadcaster" not in n]
        err = self.switch(FORWARD, others)
        if err:
            return err
        with self.lock:
            self.demo_note = "момент снят"
        return None

    def demo_triangle(self, p: dict) -> list[list[float]]:
        """Углы треугольника в base_link."""
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
        """(dx, dext) по рёбрам треугольника, phase 0..1."""
        corners = ((anim_x, 0.0), (-anim_x, 0.0), (0.0, -anim_h))
        u = (phase * 3.0) % 3.0
        seg = int(u)
        a = u - seg
        x0, e0 = corners[seg]
        x1, e1 = corners[(seg + 1) % 3]
        return (1.0 - a) * x0 + a * x1, (1.0 - a) * e0 + a * e1

    @staticmethod
    def _demo_offsets(cur: dict, leg: str, anim_dx: float, anim_ext: float) -> tuple[dict, bool]:
        """Смещения лап (м) + флаг clamp."""
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
                self.demo_note = "демо снято"
                return
            p = dict(self.demo_params)
            dt = min(0.1, now - demo["t_last"])
            demo["t_last"] = now
            trim = list(self.trim)
            # догоняем цель со скоростью follow
            step = p["follow"] * dt
            for leg, target in self._demo_targets(p).items():
                cur_leg = demo["cur"][leg]
                for k in range(3):
                    cur_leg[k] += clamp(target[k] - cur_leg[k], -step, step)
            cur = {leg: list(v) for leg, v in demo["cur"].items()}
            el = now - demo["t0"]
            # fade анимации
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
            else:
                for k, v in body.items():
                    if k not in EAR_LIMITS:
                        return f"неизвестный параметр {k!r}"
                    lo, hi = EAR_LIMITS[k]
                    number = clamp(float(v), lo, hi)
                    if k == "swap":
                        number = 1.0 if number >= 0.5 else 0.0
                    self.ear_params[k] = number
        self._remember()
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
                "zero": "уши в 0",
                "a": "уши в A",
                "b": "уши в B",
                "sync": "уши вместе",
                "mirror": "уши зеркально",
                "alt": "уши по очереди",
            }[mode]
        return None

    def ears_off(self) -> None:
        with self.ear_lock:
            was = self.ear_mode is not None or self.ear_servos is not None
            self.ear_mode = None
            self.ear_pose = (None, None)
            self.ear_pose_dirty = True
            servos = list(self.ear_servos.values()) if self.ear_servos else []
            for servo in servos:
                servo.detach()
            if was:
                self.ear_note = "уши отпущены"

    def _ears_open(self) -> str | None:
        """Открыть PWM при первой команде."""
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
                if not self.ear_pose_dirty:
                    return
                left, right = self.ear_pose
                self.ear_pose_dirty = False
                self._publish_ear_pose(left, right)
                return
            p = dict(self.ear_params)
            dt = min(0.1, now - self.ear_t_last) if self.ear_t_last else 0.0
            self.ear_t_last = now
            if mode in ("sync", "mirror", "alt"):
                self.ear_phase = (self.ear_phase + dt / p["period"]) % 1.0
            phase = self.ear_phase
        left, right = self._ear_targets(mode, phase, p)
        try:
            if self._ear_apply(left, right):
                self._publish_ear_pose(left, right)
        except OSError as e:
            self.ears_off()
            with self.ear_lock:
                self.ear_err = f"ШИМ оборвался, импульс снят: {e}"
                self.ear_note = self.ear_err

    @staticmethod
    def _ear_targets(mode: str, phase: float, p: dict) -> tuple[float | None, float | None]:
        """Позиции ушей; None = отпустить."""
        if mode == "zero":
            return 0.0, 0.0
        if mode == "a":
            return p["a_l"], p["a_r"]
        if mode == "b":
            return p["b_l"], p["b_r"]
        # A↔B, косинус на развороте
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
        # как ears_cycle, края с ползунков
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
        """GPIO L/R с учётом swap."""
        if self.ear_params["swap"] >= 0.5:
            return 13, 12
        return 12, 13

    def _publish_ear_pose(self, left: float | None, right: float | None) -> None:
        self.ear_pose_pub.send_json({"left": left, "right": right})

    def _ear_apply(self, left: float | None, right: float | None) -> bool:
        with self.ear_lock:
            servos = self.ear_servos
            if servos is None or self.ear_mode is None:
                return False
            gpio_l, gpio_r = self._ear_sides()
            # оба пина за один тик
            for gpio, value in ((gpio_l, left), (gpio_r, right)):
                servo = servos[gpio]
                if value is None:
                    servo.detach()
                else:
                    servo.set(value)
            return True

    def api_tg_params(self, body: dict) -> str | None:
        with self.lock:
            for k, v in body.items():
                if k in TG_LIMITS:
                    lo, hi = TG_LIMITS[k]
                    self.tg_params[k] = clamp(float(v), lo, hi)
                elif k == "imu_on":
                    # imu всегда on
                    self.tg_params[k] = True
                else:
                    return f"неизвестный параметр {k!r}"
        return None

    def _tg_balance(self, tg: dict, p: dict, imu_q, gyro, imu_age: float, dt: float) -> dict[str, float] | None:
        """dz лап по IMU; None = нет данных."""
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
        # roll/pitch → опустить низкую сторону
        dz = {}
        for leg, p0 in self.tg_gait.nominal.items():
            x, y = float(p0[0]), float(p0[1])
            dz[leg] = clamp(y * corr_roll - x * corr_pitch, -TG_DZ_MAX, TG_DZ_MAX)
        return dz

    def _tg_yaw_turn(self, tg: dict, imu_q, imu_age: float, dt: float) -> float:
        """Поправка turn под курс старта."""
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
                self.tg_note = "генератор снят"
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
                    self.tg_note = f"нет IMU {imu_age:.2f} с — гашу шаг"
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
        # публикация только если tg ещё жив
        with self.lock:
            if self.tg is not tg:
                return
            self._publish_forward(cmd, [p["kp"]] * 12, [p["kd"]] * 12)
            if el >= TG_TO_STAND and tg["stop_t"] is not None and tg["amp"] == 0.0:
                # шаг кончился → стойка
                self.tg = None
                self.stand = {
                    "start": list(cmd), "t0": now - STAND_RAMP, "t_last": now,
                    "kp": [STAND_KP] * 12, "i": [0.0] * 12, "cmd": list(cmd),
                }
                self.tg_note = "стоп, держу стойку"

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
                # интеграл по ошибке к цели, не к факту
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
        publish = None
        with self.lock:
            stand = self.stand
            if stand is None:
                return "поправки крутятся только в режиме «Стойка»"
            if body.get("reset"):
                delta = [saved - current for saved, current in zip(self.trim_saved, self.trim)]
                self.trim = list(self.trim_saved)
            else:
                i = int(body["index"])
                if not 0 <= i < len(JOINTS):
                    return f"сустав {i}: нужен индекс 0…{len(JOINTS) - 1}"
                old = self.trim[i]
                self.trim[i] = clamp(old + float(body["delta"]), -TRIM_MAX, TRIM_MAX)
                delta = [0.0] * len(JOINTS)
                delta[i] = self.trim[i] - old
            # Не ждём следующего control_step: поправка сразу уходит к мотору и
            # попадает в /api/state. Дальше stand_step продолжает держать цель.
            stand["cmd"] = [command + change for command, change in zip(stand["cmd"], delta)]
            publish = (list(stand["cmd"]), list(stand["kp"]))
        self._publish_forward(publish[0], publish[1], [kp * STAND_KD_PER_KP for kp in publish[1]])
        return None

    def api_trim_save(self) -> str | None:
        with self.lock:
            delta = [t - s for t, s in zip(self.trim, self.trim_saved)]
        if all(abs(d) < 1e-6 for d in delta):
            return "нечего записывать: поправки нулевые"
        with open(XACRO, encoding="utf-8") as f:
            text = f.read()
        changed = []
        # пишем ненулевой trim в xacro
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
            # сдвиг нуля и лимитов на d
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
            self.trim_note = "записано (" + ", ".join(changed) + "). Нужен restart robot.service + хоуминг."
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
        self.stop_motion("АВАРИЙНЫЙ СТОП")
        self.estop_pub.publish(Empty())
        with self.lock:
            was_standing = self.stand is not None or self.tg is not None or self.demo is not None
            self.stand = None
            self.demo = None
            if self.tg is not None:
                self.tg = None
                self.tg_note = "генератор снят (estop)"
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
            self.mission_note = f"прямо: осталось {rem:+.2f} м"
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
            self.mission_note = "маршрут пройден"
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
                    # новый хоуминг — trim сбросить
                    self.trim = [0.0] * 12
                    self.trim_saved = [0.0] * 12
                    self.trim_note = "restart: trim сброшен"
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
                homing_error = (f"после перезапуска {ROBOT_UNIT} нет /motor_homing/status за {REHOME_ACK_TIMEOUT:.0f} с: "
                                "смотри journalctl -u robot.service")
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
                "speed_mode": SPEED_MODES[self.speed_mode_index][0],
                "homing": self.homing,
                "log": list(self.log),
                "camera": {
                    "age": None if self.camera_rx_t <= 0.0 else now - self.camera_rx_t,
                    "preview_age": None if self.camera_t <= 0.0 else now - self.camera_t,
                    "bytes": self.camera_bytes,
                    "ok": self.camera_rx_t > 0.0 and now - self.camera_rx_t < 1.5,
                },
                "detection": {
                    "age": None if self.detection_rx_t <= 0.0 else now - self.detection_rx_t,
                    "preview_age": None if self.detection_t <= 0.0 else now - self.detection_t,
                    "bytes": self.detection_bytes,
                    "ok": self.detection_rx_t > 0.0 and now - self.detection_rx_t < 1.5,
                },
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
            self.vmax = clamp(float(body.get("vmax", self.vmax)), 0.05, 3.0)
            self.vymax = clamp(float(body.get("vymax", self.vymax)), 0.05, 2.0)
            self.wmax = clamp(float(body.get("wmax", self.wmax)), 0.2, 6.0)
            self.odom_scale = clamp(float(body.get("scale", self.odom_scale)), 0.2, 2.0)
        self._remember()

    def _load_prefs(self) -> None:
        if not os.path.isfile(PREFS_PATH):
            return
        try:
            with open(PREFS_PATH, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError, UnicodeError) as e:
            self.get_logger().error("не прочитал %s: %s" % (PREFS_PATH, e))
            return
        if not isinstance(data, dict):
            self.get_logger().error("%s: ожидался объект" % PREFS_PATH)
            return
        speeds = data.get("speeds")
        if isinstance(speeds, dict):
            for key, lo, hi in (("vmax", 0.05, 3.0), ("vymax", 0.05, 2.0), ("wmax", 0.2, 6.0)):
                if key not in speeds:
                    continue
                try:
                    number = float(speeds[key])
                except (TypeError, ValueError):
                    self.get_logger().error("%s: скорости.%s не число" % (PREFS_PATH, key))
                    continue
                setattr(self, key, clamp(number, lo, hi))
        saved_mode = data.get("speed_mode")
        for index, (name, _label, vx, vy, yaw, _rgb) in enumerate(SPEED_MODES):
            if saved_mode == name or (self.vmax, self.vymax, self.wmax) == (vx, vy, yaw):
                self.speed_mode_index = index
                self.vmax, self.vymax, self.wmax = vx, vy, yaw
                self.speed_mode_restore_pending = True
                break
        ears = data.get("ears")
        if isinstance(ears, dict):
            for key, (lo, hi) in EAR_LIMITS.items():
                if key not in ears:
                    continue
                try:
                    number = float(ears[key])
                except (TypeError, ValueError):
                    self.get_logger().error("%s: уши.%s не число" % (PREFS_PATH, key))
                    continue
                number = clamp(number, lo, hi)
                if key == "swap":
                    number = 1.0 if number >= 0.5 else 0.0
                self.ear_params[key] = number
        self.get_logger().info(
            "скорости %.2f/%.2f/%.2f и края ушей из %s" % (self.vmax, self.vymax, self.wmax, PREFS_PATH)
        )

    def _remember(self) -> None:
        with self.prefs_lock:
            if self.prefs_timer is not None:
                self.prefs_timer.cancel()
            timer = threading.Timer(0.5, self._write_prefs)
            timer.daemon = True
            self.prefs_timer = timer
            timer.start()

    def _write_prefs(self) -> None:
        with self.lock:
            speeds = {"vmax": self.vmax, "vymax": self.vymax, "wmax": self.wmax}
            speed_mode = SPEED_MODES[self.speed_mode_index][0]
        with self.ear_lock:
            ears = {key: self.ear_params[key] for key in EAR_LIMITS}
        raw = json.dumps(
            {"speed_mode": speed_mode, "speeds": speeds, "ears": ears},
            indent=2,
            ensure_ascii=False,
        ) + "\n"
        tmp = PREFS_PATH + ".tmp"
        try:
            os.makedirs(os.path.dirname(PREFS_PATH), exist_ok=True)
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(raw)
            os.replace(tmp, PREFS_PATH)
        except OSError as e:
            self.get_logger().error("не записал %s: %s" % (PREFS_PATH, e))

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

        def _preview_jpeg(self, kind: str, topic: str) -> None:
            jpeg, _age = node.preview_frame(kind, wait=0.5)
            if jpeg is None:
                self.send_error(503, explain=f"нет кадра с {topic}")
                return
            self._send(200, "image/jpeg", jpeg)

        def do_GET(self) -> None:  # noqa: N802
            if screen_ctl.handle_get(self):
                return
            path = self._path()
            if path == "/api/state":
                self._json(200, node.snapshot())
            elif path == "/api/vpn":
                self._json(200, {"active": vpn_active(), "unit": VPN_UNIT})
            elif path == "/api/camera.jpg":
                self._preview_jpeg("camera", "/camera/image_raw/compressed")
            elif path == "/api/detection.jpg":
                self._preview_jpeg("detection", "/glaz_detect/annotated/compressed")
            elif path in ("/", "/index.html"):
                with open(page_path, "rb") as f:
                    self._send(200, "text/html; charset=utf-8", f.read())
            elif path == "/leg_labels.js":
                with open(os.path.join(HERE, "leg_labels.js"), "rb") as f:
                    self._send(200, "application/javascript; charset=utf-8", f.read())
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
            if path == "/api/stop":
                node.ears_off()
                node.stop_motion("стоп")
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
            elif path == "/api/stand":
                err = node.set_stand()
            elif path == "/api/demo_homing":
                err = node.api_demo_homing()
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
    # колбэки MutualExclusive — MultiThreaded бесполезен
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
