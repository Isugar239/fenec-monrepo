#!/usr/bin/env python3
"""Ищет механические упоры одного сустава, медленно ведя его в обе стороны.

Остальные суставы держатся на месте. Момент ограничен: как только оценка усилия
превышает --tau, направление считается упором, сустав отступает на --backoff и
едет в другую сторону. Печатает оба упора и середину между ними.

  python3 find_stops.py                 # заднее левое колено
  python3 find_stops.py --joint leg_back_r_3 --tau 1.5

Робот должен стоять на подставке: нога в воздухе, руки убраны.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from open_loop_gait import JOINTS

FORWARD = ["forward_position_controller", "forward_kp_controller", "forward_kd_controller"]
DT = 0.02


class Finder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.idx = JOINTS.index(args.joint)
        self.pos: list[float] | None = None
        node = rclpy.create_node("find_stops")
        self.node = node
        node.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.pubs = [node.create_publisher(Float64MultiArray, f"/{n}/commands", 10) for n in FORWARD]
        self.switch_cli = node.create_client(SwitchController, "/controller_manager/switch_controller")
        self.list_cli = node.create_client(ListControllers, "/controller_manager/list_controllers")

    def on_joints(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in JOINTS):
            self.pos = [msg.position[idx[j]] for j in JOINTS]

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call(self, client, req):
        if not client.wait_for_service(timeout_sec=3.0):
            raise SystemExit("controller_manager не отвечает — robot.service запущен?")
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
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

    def publish(self, cmd: list[float]) -> None:
        kp = [self.args.hold_kp] * 12
        kp[self.idx] = self.args.kp
        # У едущего сустава kd = 0: на шине скорость всё равно 0, и kd гасил бы ход.
        # Остальные держатся, как в хоуминге.
        kd = [self.args.kd] * 12
        kd[self.idx] = 0.0
        for pub, data in zip(self.pubs, (cmd, kp, kd)):
            pub.publish(Float64MultiArray(data=[float(v) for v in data]))

    def release(self, cmd: list[float]) -> None:
        """Снимает усилие: kp и kd в ноль, сустав остаётся где стоит."""
        for pub, data in zip(self.pubs, (cmd, [0.0] * 12, [0.0] * 12)):
            pub.publish(Float64MultiArray(data=[float(v) for v in data]))

    def cruise(self, cmd: list[float], direction: float, distance: float, until_stop: bool) -> float | None:
        """Крутит цель с постоянной скоростью, как хоуминг. None — упора не было."""
        a = self.args
        origin = cmd[self.idx]
        t0 = time.monotonic()
        t_acc = a.accel_time
        while True:
            self.spin(DT)
            if self.pos is None:
                raise SystemExit("нет /joint_states")
            elapsed = time.monotonic() - t0
            if elapsed < t_acc:
                moved = 0.5 * (a.speed / t_acc) * elapsed * elapsed
            else:
                moved = 0.5 * a.speed * t_acc + a.speed * (elapsed - t_acc)
            cmd[self.idx] = origin + direction * moved
            q = self.pos[self.idx]
            tau = a.kp * (cmd[self.idx] - q)
            self.publish(cmd)
            if until_stop and abs(tau) > a.tau and moved > a.backoff:
                print(f"  упор в направлении {direction:+.0f}: угол {q:+.3f} рад, момент {tau:+.2f}")
                return q
            if moved >= distance:
                if until_stop:
                    print(f"  нет упора за {moved:.2f} рад, угол {q:+.3f}")
                    return None
                return q

    def sweep(self, cmd: list[float]) -> tuple[float, float]:
        start = self.pos[self.idx] if self.pos is not None else cmd[self.idx]
        print(f"{self.args.joint}: старт {start:+.3f} рад, порог {self.args.tau:.1f} Н·м")
        stops = []
        for direction in (+1.0, -1.0):
            cmd[self.idx] = self.pos[self.idx]
            hit = self.cruise(cmd, direction, self.args.max_travel, until_stop=True)
            if hit is not None:
                stops.append(hit)
            self.cruise(cmd, -direction, self.args.backoff, until_stop=False)
        if len(stops) < 2:
            print(f"STOP {self.args.joint} неполный")
            self.cruise_back(cmd, start)
            return float("nan"), float("nan")
        lo, hi = sorted(stops)
        print(f"STOP {self.args.joint} {lo:+.3f} {hi:+.3f} ход {hi - lo:.3f}")
        self.cruise_back(cmd, start)
        return lo, hi

    def cruise_back(self, cmd: list[float], target: float) -> None:
        if self.pos is None:
            return
        direction = 1.0 if target > cmd[self.idx] else -1.0
        self.cruise(cmd, direction, abs(target - cmd[self.idx]), until_stop=False)
        cmd[self.idx] = target
        self.publish(cmd)

    def run(self) -> None:
        self.spin(3.0)
        if self.pos is None:
            raise SystemExit("нет /joint_states — robot.service запущен?")
        self.take_forward()
        names = JOINTS if self.args.all else [self.args.joint]
        cmd = list(self.pos)
        self.publish(cmd)
        saved_tau = self.args.tau
        try:
            for name in names:
                self.args.joint = name
                self.idx = JOINTS.index(name)
                self.args.tau = saved_tau
                lo, hi = self.sweep(cmd)
                if hi == hi and hi - lo < 1.2:
                    print(f"  ход {hi - lo:.2f} рад мал для упора, повтор с порогом {saved_tau + 1.5:.1f}")
                    self.args.tau = saved_tau + 1.5
                    lo, hi = self.sweep(cmd)
                    self.args.tau = saved_tau
                if hi == hi:
                    print(f"упоры: {lo:+.3f} … {hi:+.3f} рад, середина {(lo + hi) / 2:+.3f}, ход {hi - lo:.3f} рад")
        finally:
            if self.pos is not None:
                self.release(list(self.pos))
                self.spin(0.3)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--joint", default="leg_back_l_3")
    p.add_argument("--all", action="store_true", help="все 12 суставов по очереди")
    p.add_argument("--kp", type=float, default=8.0, help="жёсткость ведомого сустава")
    p.add_argument("--hold-kp", type=float, default=12.0, help="жёсткость остальных суставов")
    p.add_argument("--kd", type=float, default=0.4)
    p.add_argument("--tau", type=float, default=1.2, help="порог момента, Н·м, по которому узнаём упор")
    p.add_argument("--speed", type=float, default=1.5, help="постоянная скорость, рад/с, как homing_velocity")
    p.add_argument("--accel-time", type=float, default=0.6, dest="accel_time", help="разгон до скорости, с")
    p.add_argument("--backoff", type=float, default=0.15, help="отступ от упора, рад")
    p.add_argument("--max-travel", type=float, default=3.8, help="макс. ход в одну сторону, рад")
    args = p.parse_args()
    if args.joint not in JOINTS:
        raise SystemExit(f"нет такого сустава: {args.joint}")

    rclpy.init()
    finder = Finder(args)
    try:
        finder.run()
    finally:
        if finder.pos is not None:
            finder.release(list(finder.pos))
            finder.spin(0.3)
        finder.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
