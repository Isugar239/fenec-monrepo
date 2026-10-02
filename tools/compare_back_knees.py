#!/usr/bin/env python3
"""Сравнение жёсткости задних коленей при одинаковом ПД.

Робот должен стоять на подставке либо быть надёжно удержан. Скрипт фиксирует
только заднее правое и заднее левое колени в их текущем положении с одинаковым
kp/kd. Все остальные суставы отпущены. Дави на голени одинаково и сравни
отклонение, которое печатается в терминал.

    python3 tools/compare_back_knees.py --kp 12 --kd 0.4

Ctrl+C снимает момент со всех суставов.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]
BACK_R_KNEE = JOINTS.index("leg_back_r_3")
BACK_L_KNEE = JOINTS.index("leg_back_l_3")
FORWARD = ["forward_position_controller", "forward_kp_controller", "forward_kd_controller"]


class KneeComparison:
    def __init__(self, kp: float, kd: float) -> None:
        self.kp = kp
        self.kd = kd
        self.pos: list[float] | None = None
        self.node = rclpy.create_node("compare_back_knees")
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.pubs = [
            self.node.create_publisher(Float64MultiArray, f"/{name}/commands", 10)
            for name in FORWARD
        ]
        self.list_cli = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.switch_cli = self.node.create_client(SwitchController, "/controller_manager/switch_controller")

    def _on_joints(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in JOINTS):
            self.pos = [by_name[name] for name in JOINTS]

    def spin(self, seconds: float) -> None:
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=3.0):
            raise SystemExit(f"{client.srv_name} недоступен")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is None:
            raise SystemExit(f"{client.srv_name} не ответил")
        return future.result()

    def take_forward(self) -> None:
        result = self.call(self.list_cli, ListControllers.Request())
        active = [item.name for item in result.controller if item.state == "active"]
        others = [name for name in active if name not in FORWARD and "broadcaster" not in name]
        request = SwitchController.Request()
        request.activate_controllers = FORWARD
        request.deactivate_controllers = others
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        if not self.call(self.switch_cli, request).ok:
            raise SystemExit("controller_manager не включил forward-контроллеры")

    def publish(self, target: list[float], enabled: bool) -> None:
        kp = [0.0] * len(JOINTS)
        kd = [0.0] * len(JOINTS)
        if enabled:
            for index in (BACK_R_KNEE, BACK_L_KNEE):
                kp[index] = self.kp
                kd[index] = self.kd
        for publisher, values in zip(self.pubs, (target, kp, kd)):
            publisher.publish(Float64MultiArray(data=values))

    def run(self) -> None:
        self.spin(3.0)
        if self.pos is None:
            raise SystemExit("нет /joint_states")
        self.take_forward()
        target = list(self.pos)
        print(
            "Фиксирую только задние колени: "
            f"правое {target[BACK_R_KNEE]:+.3f}, левое {target[BACK_L_KNEE]:+.3f}; "
            f"kp={self.kp:.1f}, kd={self.kd:.2f}. Ctrl+C — отпустить."
        )
        last_print = 0.0
        while True:
            self.publish(target, enabled=True)
            self.spin(0.02)
            if self.pos is None:
                continue
            now = time.monotonic()
            if now - last_print >= 0.2:
                right_error = self.pos[BACK_R_KNEE] - target[BACK_R_KNEE]
                left_error = self.pos[BACK_L_KNEE] - target[BACK_L_KNEE]
                print(f"ошибка: заднее правое {right_error:+.3f} рад, заднее левое {left_error:+.3f} рад")
                last_print = now

    def release(self) -> None:
        target = self.pos if self.pos is not None else [0.0] * len(JOINTS)
        for _ in range(10):
            self.publish(target, enabled=False)
            self.spin(0.02)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--kp", type=float, default=12.0)
    parser.add_argument("--kd", type=float, default=0.4)
    args = parser.parse_args()
    if not 0.0 < args.kp <= 20.0:
        raise SystemExit("kp должен быть в диапазоне (0, 20]")
    if not 0.0 <= args.kd <= 1.0:
        raise SystemExit("kd должен быть в диапазоне [0, 1]")

    rclpy.init()
    test = KneeComparison(args.kp, args.kd)
    try:
        test.run()
    except KeyboardInterrupt:
        pass
    finally:
        test.release()
        test.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
