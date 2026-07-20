#!/usr/bin/env python3
"""
Цикл ушей без GUI:
- Левая серва расслаблена (detach), правая держится сверху.
- Затем правая вниз → detach, левая вверх.
- Затем левая вниз → detach, правая вверх.
И так по кругу.

Пины: GPIO16 (левая), GPIO26 (правая).
Требуется gpiozero (python3-rpi-lgpio). Для меньшего дёргания можно запускать с pigpio:
  GPIOZERO_PIN_FACTORY=pigpio sudo -E python3 ears_cycle.py
"""

import argparse
import time

from gpiozero import Servo

PIN_LEFT = 16
PIN_RIGHT = 26


def clamp(val: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, val))


def move(servo: Servo, value: float, settle: float) -> None:
    servo.value = clamp(value)
    time.sleep(settle)


def main() -> int:
    p = argparse.ArgumentParser(description="Цикл ушей: попеременно, с detach между фазами.")
    p.add_argument("--up", type=float, default=0.5, help="Положение вверх [-1..1]")
    p.add_argument("--down", type=float, default=-0.5, help="Положение вниз [-1..1]")
    p.add_argument("--dwell", type=float, default=0.6, help="Пауза на удержание, сек")
    p.add_argument("--settle", type=float, default=0.2, help="Время после движения, сек")
    args = p.parse_args()

    up = clamp(args.up)
    down = clamp(args.down)
    dwell = max(0.1, args.dwell)
    settle = max(0.05, args.settle)

    left = Servo(PIN_LEFT, min_pulse_width=0.000500, max_pulse_width=0.002500)
    right = Servo(PIN_RIGHT, min_pulse_width=0.000500, max_pulse_width=0.002500)

    try:
        while True:
            # Фаза A: левая отпущена, правая наверху
            left.detach()
            move(right, up, settle)
            time.sleep(dwell)

            # Правая вниз и отпуск
            move(right, down, settle)
            right.detach()

            # Фаза B: левая наверху (правая отпущена)
            move(left, up, settle)
            time.sleep(dwell)

            # Левая вниз и отпуск
            move(left, down, settle)
            left.detach()
    finally:
        left.detach()
        right.detach()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
