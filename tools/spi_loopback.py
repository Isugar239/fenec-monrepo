#!/usr/bin/env python3
"""Тест SPI loopback.

Два режима успеха:
  1. Провод MOSI↔MISO на Pi: RX == TX побайтно.
  2. STM32 echo-прошивка (stm32_spi_slave.ino): pipeline на 1 байт —
     RX[0]==TX[0], RX[1:]==TX[:-1] (нужно 17 байт в кадре).

Запуск:
  sudo python3 spi_loopback.py              # bus=0, cs=0
  sudo python3 spi_loopback.py 0 1          # bus=0, cs=1  (/dev/spidev0.1)
  sudo python3 spi_loopback.py 0 0 1000000  # + скорость в Гц
"""

import sys

import spidev

bus = int(sys.argv[1]) if len(sys.argv) > 1 else 0
cs = int(sys.argv[2]) if len(sys.argv) > 2 else 0
speed = int(sys.argv[3]) if len(sys.argv) > 3 else 1_000_000

# 17 байт: 0xA0..0xB0 — последний даёт echo TX[15] на позиции RX[16]
tx = list(range(0xA0, 0xB1))
tx_sent = list(tx)

spi = spidev.SpiDev()
spi.open(bus, cs)
spi.mode = 0
spi.bits_per_word = 8
spi.max_speed_hz = speed

rx = spi.xfer2(tx)
spi.close()

print(f"/dev/spidev{bus}.{cs}  mode=0  speed={speed} Hz")
print(f"TX: {[hex(b) for b in tx_sent]}")
print(f"RX: {[hex(b) for b in rx]}")


def is_wire_loopback(tx_buf: list[int], rx_buf: list[int]) -> bool:
    return rx_buf == tx_buf


def is_slave_echo(tx_buf: list[int], rx_buf: list[int]) -> bool:
    if len(rx_buf) != len(tx_buf):
        return False
    if rx_buf[0] != tx_buf[0]:
        return False
    return rx_buf[1:] == tx_buf[:-1]


if is_wire_loopback(tx_sent, rx):
    print("OK — проводной loopback: RX == TX")
    sys.exit(0)

if is_slave_echo(tx_sent, rx):
    print("OK — STM32 SPI echo: MOSI→MISO (pipeline 1 байт)")
    sys.exit(0)

mismatches = [(i, tx_sent[i], rx[i]) for i in range(len(tx_sent)) if tx_sent[i] != rx[i]]
print(f"FAIL — {len(mismatches)}/{len(tx_sent)} байт не совпали (ни wire, ни slave echo):")
for i, t, r in mismatches[:8]:
    print(f"  [{i}] sent 0x{t:02x}, got 0x{r:02x}")
if len(mismatches) > 8:
    print(f"  ... ещё {len(mismatches) - 8}")

if all(b == 0 for b in rx):
    print("  (все нули — MISO не подключён / slave молчит)")
elif all(b == 0xFF for b in rx):
    print("  (все 0xFF — линия подтянута вверх, slave не драйвит MISO)")
elif rx[0] == tx_sent[0] and rx[1] == tx_sent[0]:
    print("  (похоже на echo, но сдвиг не совпал — проверь PA4=NSS AF5, CE, питание)")

sys.exit(1)
