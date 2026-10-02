#!/usr/bin/env python3
"""Снимок работы походки: что командует политика, где сустав на самом деле, какой нужен момент.

  python3 gait_capture.py [секунды]   # robot.service и neural_controller должны работать
"""

from __future__ import annotations

import json
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

JOINTS = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]
DEFAULT = np.array([0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52])
KP = 5.0
KD = 0.25


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = rclpy.create_node("gait_capture")
    rows: list[tuple[float, np.ndarray, np.ndarray]] = []
    last_cmd: dict[str, np.ndarray | None] = {"cmd": None, "obs": None, "act": None}
    cmds, obss, acts = [], [], []

    def on_js(msg: JointState) -> None:
        if last_cmd["cmd"] is None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        pos = np.array([msg.position[idx[j]] for j in JOINTS])
        vel = np.array([msg.velocity[idx[j]] for j in JOINTS])
        rows.append((time.monotonic(), pos, vel, last_cmd["cmd"].copy()))

    def on_cmd(msg) -> None:
        last_cmd["cmd"] = np.array(msg.data[:12])
        cmds.append(last_cmd["cmd"])

    def on_obs(msg) -> None:
        obss.append(np.array(msg.data[:36]))

    def on_act(msg) -> None:
        acts.append(np.array(msg.data[:12]))

    node.create_subscription(JointState, "/joint_states", on_js, qos_profile_sensor_data)
    node.create_subscription(Float32MultiArray, "/neural_controller/position_command", on_cmd, 10)
    node.create_subscription(Float32MultiArray, "/neural_controller/observation", on_obs, 10)
    node.create_subscription(Float32MultiArray, "/neural_controller/policy_output", on_act, 10)

    t_end = time.monotonic() + seconds + 3.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if rows and rows[-1][0] - rows[0][0] > seconds:
            break
    node.destroy_node()
    rclpy.shutdown()

    if not rows:
        print("нет данных: походка выключена или position_command не публикуется")
        sys.exit(1)

    pos = np.array([r[1] for r in rows])
    vel = np.array([r[2] for r in rows])
    cmd = np.array([r[3] for r in rows])
    err = cmd - pos
    tau = KP * err - KD * vel
    print(f"кадров {len(rows)} за {rows[-1][0] - rows[0][0]:.1f} с, команд {len(cmds)}, "
          f"наблюдений {len(obss)}")
    print(f"{'сустав':15s} {'поза':>6s} {'команда':>8s} {'факт':>7s} {'ошибка':>7s} "
          f"{'|τ|ср':>6s} {'|τ|max':>6s} {'дрожь cmd':>9s}")
    for i, j in enumerate(JOINTS):
        print(f"{j:15s} {DEFAULT[i]:+6.2f} {cmd[:, i].mean():+8.3f} {pos[:, i].mean():+7.3f} "
              f"{err[:, i].mean():+7.3f} {np.abs(tau[:, i]).mean():6.2f} {np.abs(tau[:, i]).max():6.2f} "
              f"{cmd[:, i].std():9.3f}")
    if obss:
        o = np.array(obss)
        print("гравитация в наблюдении (ср):", np.round(o[:, 3:6].mean(0), 3))
        print("угл. скорость (ср |.|):", np.round(np.abs(o[:, 0:3]).mean(0), 3))
        print("команда скорости (ср):", np.round(o[:, 6:9].mean(0), 3))
        print("желаемый верх в корпусе:", np.round(o[:, 9:12].mean(0), 3))
    if acts:
        a = np.array(acts)
        print("выход политики (ср):", np.round(a.mean(0), 2))
        print("выход политики (|max|):", np.round(np.abs(a).max(0), 2))
    out = "/tmp/gait_capture.json"
    json.dump({"joints": JOINTS, "pos": pos.tolist(), "cmd": cmd.tolist(),
               "obs": [x.tolist() for x in obss]}, open(out, "w"))
    print("сырые данные:", out)


if __name__ == "__main__":
    main()
