#!/usr/bin/env python3
"""Непрерывный SPI loopback watch: шлёт паттерн, сравнивает TX и RX.

Для проверки замыкания MOSI↔MISO, STM Hi-Z / echo, подтяжек на MISO.

  sudo python3 spi_loopback_watch.py
  sudo python3 spi_loopback_watch.py --speed 1000000 --interval 0.5
  sudo python3 spi_loopback_watch.py --ce 0
  sudo python3 spi_loopback_watch.py --ce 0 --ce 1 --bytes 17

Ctrl+C — выход.

Не путать с tools/spi_watch.c — тот шлёт spine-пакет почти из нулей
(при loopback на Pi нули в RX — нормально, бесполезно для диагностики).
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import spidev

PATTERN_START = 0xA0
PATTERN_LEN_DEFAULT = 16


def classify(tx: list[int], rx: list[int]) -> str:
    if rx == tx:
        return "WIRE_OK"
    if len(rx) == len(tx) and rx[0] == tx[0] and rx[1:] == tx[:-1]:
        return "SLAVE_ECHO"
    if all(b == 0 for b in rx):
        return "ALL_ZERO"
    if all(b == 0xFF for b in rx):
        return "ALL_FF"
    mism = sum(1 for a, b in zip(tx, rx) if a != b)
    return f"MISMATCH({mism}/{len(tx)})"


def fmt_bytes(buf: list[int], n: int = 8) -> str:
    part = " ".join(f"{b:02x}" for b in buf[:n])
    if len(buf) > n:
        part += " ..."
    return part


class CeWatch:
    def __init__(self, bus: int, cs: int, speed: int, nbytes: int) -> None:
        self.dev = f"/dev/spidev{bus}.{cs}"
        self.nbytes = nbytes
        self.tx = list(range(PATTERN_START, PATTERN_START + nbytes))
        self.spi = spidev.SpiDev()
        self.spi.open(bus, cs)
        self.spi.mode = 0
        self.spi.bits_per_word = 8
        self.spi.max_speed_hz = speed
        self.ok_wire = 0
        self.ok_echo = 0
        self.iter = 0

    def close(self) -> None:
        self.spi.close()

    def exchange(self) -> tuple[list[int], list[int], str]:
        # xfer2 перезаписывает переданный буфер — TX храним отдельно (см. spi_loopback.py)
        tx_sent = list(self.tx)
        rx = self.spi.xfer2(list(self.tx))
        tag = classify(tx_sent, rx)
        self.iter += 1
        if tag == "WIRE_OK":
            self.ok_wire += 1
        elif tag == "SLAVE_ECHO":
            self.ok_echo += 1
        return tx_sent, rx, tag


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Непрерывный SPI loopback watch")
    p.add_argument("--bus", type=int, default=0)
    p.add_argument("--ce", type=int, action="append", dest="ces", help="CE (0/1), можно несколько раз")
    p.add_argument("--speed", type=int, default=1_000_000)
    p.add_argument("--interval", type=float, default=0.25, help="пауза между кадрами, сек")
    p.add_argument("--bytes", type=int, default=PATTERN_LEN_DEFAULT, help="длина кадра (16=wire loopback)")
    p.add_argument("--verbose", action="store_true", help="каждый кадр печатать TX/RX полностью")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ces = args.ces if args.ces else [0, 1]

    watches = [CeWatch(args.bus, cs, args.speed, args.bytes) for cs in ces]

    stop = False

    def on_sig(_sig, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    print(
        f"spi_loopback_watch  bus={args.bus}  CE={ces}  "
        f"speed={args.speed}  bytes={args.bytes}  interval={args.interval}s"
    )
    print(f"TX pattern: {fmt_bytes(watches[0].tx, watches[0].nbytes)}")
    print("tags: WIRE_OK | SLAVE_ECHO | ALL_ZERO | ALL_FF | MISMATCH(n/N)")
    print("Ctrl+C — выход\n")

    while not stop:
        parts: list[str] = []
        for w in watches:
            tx_sent, rx, tag = w.exchange()
            parts.append(f"{w.dev} {tag}")
            if args.verbose or tag not in ("WIRE_OK", "SLAVE_ECHO"):
                parts.append(
                    f"tx={fmt_bytes(tx_sent, w.nbytes)} rx={fmt_bytes(rx, w.nbytes)}"
                )
        line = "  |  ".join(parts)
        print(line, flush=True)
        time.sleep(args.interval)

    print("\n--- итог ---")
    for w in watches:
        print(
            f"{w.dev}: iter={w.iter}  WIRE_OK={w.ok_wire}  "
            f"SLAVE_ECHO={w.ok_echo}"
        )
    for w in watches:
        w.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
