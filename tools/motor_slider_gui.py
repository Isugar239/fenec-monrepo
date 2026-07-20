#!/usr/bin/env python3
"""GUI: слайдеры для 3 моторов передней левой ноги (CAN ch1 / CE0 leg0).

Диапазоны из components.xacro (post_homing = «ноль» после homing).
Только leg0 включён; остальные flags=0.

Запуск (нужен root и дисплей):
  sudo systemctl stop robot.service
  sudo python3 motor_slider_gui.py

При sudo root не видит X11 — скрипт сам подставит XAUTHORITY из SUDO_USER.
Если не помогло:
  sudo XAUTHORITY=/home/pi/.Xauthority DISPLAY=:0 python3 motor_slider_gui.py
"""

from __future__ import annotations

import os
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)

from spine_spi_test import (  # noqa: E402
    ACTUATORS,
    SpineBus,
    SpineCommand,
    BoardResult,
    _actuator_state,
    _board_ok,
    _global_to_spine_cmd,
    _read_actuator_states,
    _spine_to_global_leg,
)

# leg_front_l_* — индексы 3,4,5 в ACTUATORS; лимиты из pupper_v3 components.xacro
FRONT_LEFT = (
    {
        "idx": 3,
        "name": "leg_front_l_1 (abad)",
        "pos_min": -2.41,
        "pos_max": 1.12,
        "home": 0.0,
    },
    {
        "idx": 4,
        "name": "leg_front_l_2 (hip)",
        "pos_min": -3.04,
        "pos_max": 0.32,
        "home": 0.0,
    },
    {
        "idx": 5,
        "name": "leg_front_l_3 (knee)",
        "pos_min": -0.61,
        "pos_max": 2.69,
        "home": -0.71,
    },
)

LOOP_HZ = 50
SCALE = 1000  # слайдер в миллирадианах


def _positions_from_ce0(
    r: BoardResult, zero_offsets: list[float], prev: list[float]
) -> list[float]:
    """Только ноги 0–1 с CE0 (can_ch 1 и 2); остальные индексы не трогаем."""
    pos = list(prev)
    raw_pos: dict[tuple[int, str], float] = {}
    raw_vel: dict[tuple[int, str], float] = {}
    for local, g in ((0, 0), (1, 1)):
        _spine_to_global_leg(r.data, local, g, raw_pos, raw_vel)
    for i, act in enumerate(ACTUATORS):
        if act["can_ch"] <= 2:
            pos[i] = _actuator_state(act, raw_pos, zero_offsets[i])
    return pos


def _check_root() -> None:
    if os.geteuid() != 0:
        print("Запусти с sudo: sudo python3 motor_slider_gui.py", file=sys.stderr)
        sys.exit(1)


def _setup_display() -> None:
    if not os.environ.get("DISPLAY"):
        os.environ["DISPLAY"] = ":0"
    if os.geteuid() == 0 and not os.environ.get("XAUTHORITY"):
        sudo_user = os.environ.get("SUDO_USER")
        if sudo_user:
            auth = os.path.join("/home", sudo_user, ".Xauthority")
            if os.path.isfile(auth):
                os.environ["XAUTHORITY"] = auth


class MotorSliderApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Pupper — передняя левая нога (3 мотора)")
        self.root.geometry("520x420")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.bus = SpineBus()
        self.flags = [1, 0, 0, 0]
        self.zero_offsets = [0.0] * len(ACTUATORS)
        self.cmd_pos = [0.0] * len(ACTUATORS)
        self.cmd_vel = [0.0] * len(ACTUATORS)
        self.cmd_kp = [0.0] * len(ACTUATORS)
        self.cmd_kd = [0.0] * len(ACTUATORS)
        self.actual_pos = [0.0] * len(ACTUATORS)
        self.enabled = tk.BooleanVar(value=False)
        self.kp_var = tk.DoubleVar(value=5.5)
        self.kd_var = tk.DoubleVar(value=0.5)
        self.status_var = tk.StringVar(value="Выключено")
        self._stop = threading.Event()
        self._idle = SpineCommand()
        self._idle.flags = [0, 0]

        self._sliders: list[tk.Scale] = []
        self._cmd_labels: list[tk.StringVar] = []
        self._act_labels: list[tk.StringVar] = []

        self._build_ui()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, textvariable=self.status_var, wraplength=480).pack(anchor=tk.W)
        ttk.Checkbutton(top, text="Включить моторы (flags leg0)", variable=self.enabled).pack(
            anchor=tk.W, pady=4
        )

        pd = ttk.Frame(top)
        pd.pack(anchor=tk.W)
        ttk.Label(pd, text="kp").grid(row=0, column=0)
        ttk.Spinbox(pd, from_=0.0, to=10.0, increment=0.5, width=6, textvariable=self.kp_var).grid(
            row=0, column=1, padx=4
        )
        ttk.Label(pd, text="kd").grid(row=0, column=2)
        ttk.Spinbox(pd, from_=0.0, to=2.0, increment=0.05, width=6, textvariable=self.kd_var).grid(
            row=0, column=3, padx=4
        )
        ttk.Button(top, text="В ноль (post homing)", command=self._go_home).pack(anchor=tk.W, pady=4)
        ttk.Button(top, text="Считать положение с моторов", command=self._refresh_read).pack(
            anchor=tk.W
        )

        for j in FRONT_LEFT:
            frame = ttk.LabelFrame(self.root, text=j["name"], padding=8)
            frame.pack(fill=tk.X, padx=8, pady=4)

            lo = int(j["pos_min"] * SCALE)
            hi = int(j["pos_max"] * SCALE)
            home = int(j["home"] * SCALE)

            var = tk.IntVar(value=home)
            sl = tk.Scale(
                frame,
                from_=lo,
                to=hi,
                orient=tk.HORIZONTAL,
                length=440,
                resolution=1,
                variable=var,
                command=lambda _v, idx=j["idx"]: self._on_slider(idx),
            )
            sl.set(home)
            sl.pack(fill=tk.X)
            self._sliders.append(sl)

            cmd_l = tk.StringVar(value="cmd: —")
            act_l = tk.StringVar(value="факт: —")
            self._cmd_labels.append(cmd_l)
            self._act_labels.append(act_l)
            ttk.Label(frame, textvariable=cmd_l).pack(anchor=tk.W)
            ttk.Label(frame, textvariable=act_l).pack(anchor=tk.W)
            ttk.Label(
                frame,
                text=f"диапазон {j['pos_min']:.2f} … {j['pos_max']:.2f} rad, ноль={j['home']:.2f}",
                font=("", 9),
            ).pack(anchor=tk.W)

        ttk.Label(
            self.root,
            text="Закрытие окна отпускает моторы. Держи ногу свободной от препятствий.",
            padding=8,
        ).pack(anchor=tk.W)

    def _joint_slider_index(self, actuator_idx: int) -> int:
        for i, j in enumerate(FRONT_LEFT):
            if j["idx"] == actuator_idx:
                return i
        raise KeyError(actuator_idx)

    def _slider_rad(self, joint_i: int) -> float:
        return self._sliders[joint_i].get() / SCALE

    def _on_slider(self, actuator_idx: int) -> None:
        si = self._joint_slider_index(actuator_idx)
        self.cmd_pos[actuator_idx] = self._slider_rad(si)

    def _go_home(self) -> None:
        for i, j in enumerate(FRONT_LEFT):
            home = int(j["home"] * SCALE)
            self._sliders[i].set(home)
            self.cmd_pos[j["idx"]] = j["home"]

    def _apply_gains(self) -> None:
        kp = float(self.kp_var.get())
        kd = float(self.kd_var.get())
        for j in FRONT_LEFT:
            self.cmd_kp[j["idx"]] = kp
            self.cmd_kd[j["idx"]] = kd

    def _refresh_read(self) -> None:
        pos, _, results = _read_actuator_states(self.bus, self.zero_offsets)
        if not _board_ok(results[0]):
            self.status_var.set("CE0: нет ответа spine — проверь SPIne, одну плату, питание")
            return
        self.actual_pos = pos
        for i, j in enumerate(FRONT_LEFT):
            self._act_labels[i].set(f"факт: {pos[j['idx']]:+.4f} rad")
        self.status_var.set("Положение обновлено")

    def _exchange(self) -> bool:
        if self.enabled.get():
            self._apply_gains()
            c0 = _global_to_spine_cmd(
                0, self.flags, self.cmd_pos, self.cmd_vel, self.cmd_kp, self.cmd_kd, self.zero_offsets
            )
            results = self.bus.exchange_both(c0, self._idle)
        else:
            return False
        if not _board_ok(results[0]):
            self.status_var.set("CE0: нет spine-ответа")
            return False
        self.actual_pos = _positions_from_ce0(results[0], self.zero_offsets, self.actual_pos)
        return True

    def _loop(self) -> None:
        dt = 1.0 / LOOP_HZ
        while not self._stop.is_set():
            if self.enabled.get():
                for i, j in enumerate(FRONT_LEFT):
                    self.cmd_pos[j["idx"]] = self._slider_rad(i)
                if self._exchange():
                    parts = []
                    for i, j in enumerate(FRONT_LEFT):
                        idx = j["idx"]
                        parts.append(f"{j['name'].split()[0]}:{self.actual_pos[idx]:+.2f}")
                        self.root.after(
                            0,
                            lambda ii=i, ix=idx: self._cmd_labels[ii].set(
                                f"cmd: {self.cmd_pos[ix]:+.4f} rad"
                            ),
                        )
                        self.root.after(
                            0,
                            lambda ii=i, ix=idx: self._act_labels[ii].set(
                                f"факт: {self.actual_pos[ix]:+.4f} rad"
                            ),
                        )
                    self.root.after(0, lambda p=", ".join(parts): self.status_var.set(f"OK  {p}"))
            time.sleep(dt)

    def _release(self) -> None:
        self.enabled.set(False)
        for _ in range(5):
            self.bus.exchange_both(self._idle, self._idle)
            time.sleep(0.02)

    def _on_close(self) -> None:
        self._stop.set()
        self._release()
        self.bus.close()
        self.root.destroy()

    def run(self) -> None:
        if os.environ.get("DISPLAY", "") == "":
            messagebox.showerror(
                "Нет DISPLAY",
                "Нужен графический дисплей.\n"
                "Локально на Pi с монитором:\n"
                "  sudo python3 motor_slider_gui.py\n"
                "Или явно:\n"
                "  sudo XAUTHORITY=/home/pi/.Xauthority DISPLAY=:0 python3 motor_slider_gui.py",
            )
            self._on_close()
            return
        self.root.mainloop()


def main() -> None:
    _check_root()
    _setup_display()
    app = MotorSliderApp()
    app.run()


if __name__ == "__main__":
    main()
