"""Кинематика ноги Pupper v3 по тому же URDF, что у robot_state_publisher.

Суставы ноги (проверено по URDF и стойке по умолчанию):
  _1 — бедро, качает ногу вперёд-назад;
  _2 — отведение, качает ногу вбок;
  _3 — колено.
Координаты — base_link: x вперёд, y влево, z вверх, метры.
"""

from __future__ import annotations

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

XACRO = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "ros2_ws/src/pupper_v3_description/description/pupper_v3.urdf.xacro",
)
LEGS = ("front_r", "front_l", "back_r", "back_l")
LEG_INDEX = {"front_r": 0, "front_l": 3, "back_r": 6, "back_l": 9}
# Точка лапы в кадре leg_*_3 (foot_site из mujoco_xml).
FOOT = {
    "front_r": (0.06231, -0.06216, 0.018),
    "front_l": (0.06231, 0.06216, 0.018),
    "back_r": (0.06231, -0.06216, 0.018),
    "back_l": (0.06231, 0.06216, 0.018),
}


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rot_z(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Kinematics:
    def __init__(self, urdf_xml: str) -> None:
        joints = {j.get("name"): j for j in ET.fromstring(urdf_xml).findall("joint")}
        self.chain: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}
        for leg in LEGS:
            links = []
            for k in (1, 2, 3):
                j = joints[f"leg_{leg}_{k}"]
                axis = [float(v) for v in j.find("axis").get("xyz").split()]
                if axis != [0.0, 0.0, 1.0]:
                    raise ValueError(f"leg_{leg}_{k}: ось {axis}, код рассчитан на ось z")
                o = j.find("origin")
                xyz = np.array([float(v) for v in o.get("xyz").split()])
                links.append((_rpy(*[float(v) for v in o.get("rpy").split()]), xyz))
            self.chain[leg] = links
        self.foot = {leg: np.array(v) for leg, v in FOOT.items()}

    @classmethod
    def from_xacro(cls, path: str = XACRO) -> "Kinematics":
        urdf = subprocess.run(["xacro", path], check=True, capture_output=True, text=True).stdout
        return cls(urdf)

    def fk_jac(self, leg: str, q) -> tuple[np.ndarray, np.ndarray]:
        """Положение лапы и якобиан 3×3 (столбец — производная по суставу)."""
        R = np.eye(3)
        t = np.zeros(3)
        origins, axes = [], []
        for (Ro, to), qi in zip(self.chain[leg], q):
            t = t + R @ to
            R = R @ Ro
            origins.append(t)
            axes.append(R[:, 2])
            R = R @ _rot_z(qi)
        p = t + R @ self.foot[leg]
        J = np.column_stack([np.cross(a, p - o) for a, o in zip(axes, origins)])
        return p, J

    def fk(self, leg: str, q) -> np.ndarray:
        return self.fk_jac(leg, q)[0]

    def fk_links(self, leg: str, q) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(бедро, колено, лапа) в base_link — для отрисовки."""
        R = np.eye(3)
        t = np.zeros(3)
        hip = knee = None
        for j, ((Ro, to), qi) in enumerate(zip(self.chain[leg], q)):
            t = t + R @ to
            R = R @ Ro
            if j == 0:
                hip = t.copy()
            if j == 2:
                knee = t.copy()
            R = R @ _rot_z(qi)
        return hip, knee, t + R @ self.foot[leg]

    def ik(self, leg: str, target: np.ndarray, q0, iters: int = 3, tol: float = 2e-4) -> tuple[np.ndarray, float]:
        """(углы, промах в м до последнего шага), чтобы лапа была в target.

        q0 — начальное приближение (прошлый шаг): при 50 Гц хватает 1–2 итераций.
        """
        q = np.array(q0, dtype=float)
        lam2 = 1e-4
        err = 0.0
        for _ in range(iters):
            p, J = self.fk_jac(leg, q)
            e = target - p
            err = float(np.linalg.norm(e))
            if err < tol:
                break
            q = q + J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), e)
        return q, err
