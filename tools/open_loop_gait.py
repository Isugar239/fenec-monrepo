#!/usr/bin/env python3
"""Походка без нейросети: генератор траекторий рыси для лап + обратная кинематика по URDF.

Траектория задаётся смещением лапы от стойки по умолчанию в метрах (x вперёд, z вверх),
углы суставов считает обратная кинематика (tools/pupper_kinematics.py). IMU здесь нет —
коррекция по IMU есть в пульте :8091 (раздел «Генератор траекторий»).

Работает рядом с robot.service через forward-контроллеры, как «Стойка» в пульте.
Нейросеть (neural_controller) на время работы выключается.
Не запускай одновременно генератор или «Стойку» в пульте — оба будут слать команды.

  source /opt/ros/jazzy/setup.bash
  source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
  python3 ~/pupperv3-monorepo/tools/open_loop_gait.py                   # шаг на месте
  python3 ~/pupperv3-monorepo/tools/open_loop_gait.py --stride 0.03     # вперёд
  python3 ~/pupperv3-monorepo/tools/open_loop_gait.py --stride 0.03 --turn 0.5

Ctrl+C: амплитуда плавно спадает, робот остаётся в стойке на forward-контроллерах.
"""

from __future__ import annotations

import argparse
import math
import threading
import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from pupper_kinematics import LEG_INDEX, Kinematics

JOINTS = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]
DEFAULT_POS = [0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52]
# (сторона: +1 правая, -1 левая; сдвиг фазы). Рысь: диагонали ПП+ЗЛ и ПЛ+ЗП в противофазе.
# Знак разгибания колена: +1 — упор в минусе (левые ноги), -1 — упор в плюсе (правые).
# У задней левой упор на ~0.2 рад ближе к стойке, чем у правой, и подъём лапы упирает колено
# в ограничитель. Команду держим чуть не доходя до упора, чтобы мотор не вжимался в него.
KNEE_SIGN = {"front_r": -1.0, "front_l": +1.0, "back_r": -1.0, "back_l": +1.0}
KNEE_MARGIN = 0.05
PHASE = {
    "front_r": (+1, 0.0),
    "front_l": (-1, 0.5),
    "back_r": (+1, 0.5),
    "back_l": (-1, 0.0),
}
FORWARD = ["forward_position_controller", "forward_kp_controller", "forward_kd_controller"]
DT = 0.02
TO_STAND_S = 2.5


def smooth(s: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * s)


def foot_offsets(phase: float, swing_frac: float, stride: float, lift: float) -> tuple[float, float]:
    """(dx, dz) лапы, м. stride — полуамплитуда шага вдоль x."""
    if phase < swing_frac:
        s = phase / swing_frac
        return -stride + 2.0 * stride * smooth(s), lift * math.sin(math.pi * s)
    s = (phase - swing_frac) / (1.0 - swing_frac)
    return stride - 2.0 * stride * s, 0.0


class GaitIK:
    """Цели 12 суставов из траекторий лап. Хранит прошлое решение как начальное приближение."""

    def __init__(self, kin: Kinematics, default: list[float] = DEFAULT_POS) -> None:
        self.kin = kin
        self.default = np.array(default, dtype=float)
        self.nominal = {
            leg: kin.fk(leg, self.default[i:i + 3]) for leg, i in LEG_INDEX.items()
        }
        self.reset()

    def reset(self) -> None:
        self.q = {leg: self.default[i:i + 3].copy() for leg, i in LEG_INDEX.items()}
        self.ik_err = 0.0

    def pose(
        self,
        phase: float,
        amp: float,
        *,
        stride: float,
        lift: float,
        swing: float,
        turn: float,
        height: float,
        dz_extra: dict[str, float] | None = None,
    ) -> list[float]:
        """phase — общая фаза цикла в оборотах, amp — 0..1. turn > 0 — поворот налево."""
        out = self.default.copy()
        err = 0.0
        for leg, (side, shift) in PHASE.items():
            dx, dz = foot_offsets((phase + shift) % 1.0, swing, stride * (1.0 + turn * side) * amp, lift * amp)
            dz -= height * amp
            if dz_extra is not None:
                dz += dz_extra[leg]
            target = self.nominal[leg] + np.array([dx, 0.0, dz])
            q, leg_err = self.kin.ik(leg, target, self.q[leg])
            q = self._keep_off_stop(leg, target, q)
            err = max(err, leg_err, float(np.linalg.norm(target - self.kin.fk(leg, q))))
            self.q[leg] = q
            i = LEG_INDEX[leg]
            out[i:i + 3] = q
        self.ik_err = err
        return [float(v) for v in out]

    def pose_offsets(self, offsets: dict[str, tuple[float, float, float]]) -> list[float]:
        """Статическая поза: смещение каждой лапы от стойки (dx, dy, dz), м; x вперёд, y влево, z вверх."""
        out = self.default.copy()
        err = 0.0
        for leg, off in offsets.items():
            target = self.nominal[leg] + np.array(off)
            q, leg_err = self.kin.ik(leg, target, self.q[leg], iters=6)
            q = self._keep_off_stop(leg, target, q)
            err = max(err, leg_err, float(np.linalg.norm(target - self.kin.fk(leg, q))))
            self.q[leg] = q
            i = LEG_INDEX[leg]
            out[i:i + 3] = q
        self.ik_err = err
        return [float(v) for v in out]

    def _keep_off_stop(self, leg: str, target: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Если колено упёрлось бы в ограничитель, добираем положение лапы бедром."""
        limit = KNEE_SIGN[leg] * KNEE_MARGIN
        if KNEE_SIGN[leg] * q[2] >= KNEE_SIGN[leg] * limit:
            return q
        q = q.copy()
        q[2] = limit
        for _ in range(3):
            p, J = self.kin.fk_jac(leg, q)
            e = target - p
            step = np.linalg.solve(J[:, :2].T @ J[:, :2] + 1e-3 * np.eye(2), J[:, :2].T @ e)
            q[:2] += step
        return q


class Gait(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("open_loop_gait")
        self.args = args
        self.gait = GaitIK(Kinematics.from_xacro())
        self.lock = threading.Lock()
        self.joint_pos: list[float | None] = [None] * 12
        self.create_subscription(JointState, "/joint_states", self.on_joints, 20)
        self.pubs = [
            self.create_publisher(Float64MultiArray, f"/{name}/commands", 10) for name in FORWARD
        ]
        self.switch_cli = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.list_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")

    def on_joints(self, msg: JointState) -> None:
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in JOINTS:
                    self.joint_pos[JOINTS.index(name)] = pos

    def call(self, client, req):
        if not client.wait_for_service(timeout_sec=3.0):
            raise SystemExit("controller_manager не отвечает — robot.service запущен?")
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            raise SystemExit(f"{client.srv_name} не ответил за 5 с")
        return fut.result()

    def take_forward(self) -> None:
        res = self.call(self.list_cli, ListControllers.Request())
        active = [c.name for c in res.controller if c.state == "active"]
        others = [n for n in active if n not in FORWARD and "broadcaster" not in n]
        req = SwitchController.Request()
        req.activate_controllers = FORWARD
        req.deactivate_controllers = others
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        if not self.call(self.switch_cli, req).ok:
            raise SystemExit("controller_manager отказал в переключении на forward-контроллеры")
        print(f"forward-контроллеры включены, выключены: {others or 'ничего'}")

    def publish(self, pos: list[float]) -> None:
        kp = [self.args.kp] * 12
        kd = [self.args.kd] * 12
        # Переднее левое отведение слабее, kp_max у него 40.
        kp[4] = min(40.0, kp[4] * 2.0)
        kd[4] = min(1.0, kd[4] * 2.0)
        for pub, data in zip(self.pubs, (pos, kp, kd)):
            pub.publish(Float64MultiArray(data=[float(v) for v in data]))

    def target(self, t: float, amp: float) -> list[float]:
        a = self.args
        return self.gait.pose(
            t * a.freq, amp, stride=a.stride, lift=a.lift, swing=a.swing, turn=a.turn, height=a.height,
        )


def spin_sleep(node: Node, until: float) -> None:
    while True:
        left = until - time.monotonic()
        if left <= 0:
            return
        rclpy.spin_once(node, timeout_sec=min(left, DT))


def main() -> None:
    p = argparse.ArgumentParser(description="Рысь без нейросети: траектории лап + обратная кинематика")
    p.add_argument("--freq", type=float, default=0.9, help="шагов на ногу в секунду, Гц")
    p.add_argument("--stride", type=float, default=0.032, help="полуамплитуда шага, м (0 = на месте)")
    p.add_argument("--lift", type=float, default=0.028, help="подъём лапы в переносе, м")
    p.add_argument("--height", type=float, default=0.0, help="корпус выше стойки на, м")
    p.add_argument("--swing", type=float, default=0.5, help="доля цикла в переносе (рысь = 0.5)")
    p.add_argument("--turn", type=float, default=0.0, help="-1..1, > 0 — налево")
    p.add_argument("--kp", type=float, default=12.0)
    p.add_argument("--kd", type=float, default=0.55)
    p.add_argument("--ramp", type=float, default=2.0, help="с, плавный набор и спад амплитуды")
    p.add_argument("--duration", type=float, default=0.0, help="с ходьбы, 0 = до Ctrl+C")
    args = p.parse_args()
    if not 0.2 <= args.swing <= 0.8:
        raise SystemExit("--swing вне 0.2..0.8")
    if abs(args.turn) > 1.0:
        raise SystemExit("--turn вне -1..1")
    if args.stride > 0.06 or args.lift > 0.06 or abs(args.height) > 0.03:
        raise SystemExit("stride/lift больше 6 см или height больше 3 см — лапа выйдет из зоны досягаемости")
    if args.kp > 20.0:
        print("ВНИМАНИЕ: kp_max в components.xacro = 20, железо обрежет выше")

    # Без этого rclpy на Ctrl+C сам гасит контекст и спад амплитуды не опубликовать.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Gait(args)

    deadline = time.monotonic() + 5.0
    while any(v is None for v in node.joint_pos):
        if time.monotonic() > deadline:
            raise SystemExit("нет /joint_states по всем 12 суставам за 5 с")
        rclpy.spin_once(node, timeout_sec=0.1)
    with node.lock:
        start = list(node.joint_pos)

    node.publish(start)
    node.take_forward()

    print("еду в стойку...")
    t0 = time.monotonic()
    tick = t0
    while (now := time.monotonic()) - t0 < TO_STAND_S:
        a = smooth((now - t0) / TO_STAND_S)
        node.publish([s + a * (d - s) for s, d in zip(start, DEFAULT_POS)])
        tick += DT
        spin_sleep(node, tick)

    print(f"иду: freq={args.freq} stride={args.stride} lift={args.lift} turn={args.turn}. Ctrl+C — стоп")
    t_walk = time.monotonic()
    stop_at = None
    try:
        while True:
            now = time.monotonic()
            t = now - t_walk
            if args.duration > 0 and stop_at is None and t >= args.duration:
                stop_at = now
            if stop_at is None:
                amp = min(1.0, t / args.ramp)
            else:
                amp = max(0.0, 1.0 - (now - stop_at) / args.ramp)
                if amp == 0.0:
                    break
            node.publish(node.target(t, smooth(amp)))
            if node.gait.ik_err > 0.005:
                print(f"ВНИМАНИЕ: лапа не достаёт до цели на {node.gait.ik_err * 1000:.0f} мм")
            tick += DT
            spin_sleep(node, tick)
    except KeyboardInterrupt:
        print("\nостанавливаюсь...")
        stop = time.monotonic()
        while (now := time.monotonic()) - stop < args.ramp:
            amp = 1.0 - (now - stop) / args.ramp
            node.publish(node.target(now - t_walk, smooth(amp)))
            time.sleep(DT)

    node.publish(DEFAULT_POS)
    print("стою в стойке (forward-контроллеры). Нейросеть — кнопкой «Включить походку» в пульте.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
