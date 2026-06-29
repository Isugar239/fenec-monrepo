#!/usr/bin/env python3
"""Тест SPI-протокола spine (Pi <-> STM) без ROS.

Повторяет логику ros2_ws/.../rt_spi.cpp:
  - 66 слов × 16 бит = 132 байта на транзакцию
  - MODE0, 6 МГц (по умолчанию)
  - byte-swap каждого 16-битного слова на проводе
  - cmd: XOR по 32 первым u32, checksum в слове 33
  - data: XOR по 14 первым u32, checksum в слове 15
  - flags[i]=1 — разрешить момент на ноге (как при activate робота)

Запуск (нужен root для /dev/spidev*):
  sudo python3 spine_spi_test.py probe
  sudo python3 spine_spi_test.py watch
  sudo python3 spine_spi_test.py enable
  sudo python3 spine_spi_test.py read
  sudo python3 spine_spi_test.py homing --seconds 3
  sudo python3 spine_spi_test.py homing-full --timeout 180

Перед тестом: sudo systemctl stop robot.service
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass, field

import spidev

WORDS_PER_MESSAGE = 66
CMD_U32_WORDS = 33
DATA_U32_WORDS = 15
MESSAGE_BYTES = WORDS_PER_MESSAGE * 2

# Индексы u32 в spine_cmd_t / spine_data_t (см. rt_spi.h)
IDX_Q_ABAD = 0
IDX_Q_HIP = 2
IDX_Q_KNEE = 4
IDX_QD_ABAD = 6
IDX_QD_HIP = 8
IDX_QD_KNEE = 10
IDX_FLAGS = 12
IDX_CHECKSUM = 14

CMD_IDX_FLAGS = 30
CMD_IDX_CHECKSUM = 32

DEFAULT_SPEED_HZ = 6_000_000
DEVICES = ("/dev/spidev0.0", "/dev/spidev0.1")
BOARD_NAMES = ("CE0 board0 legs[0,1]", "CE1 board1 legs[2,3]")

ABAD_SIDE_SIGN = [-1.0, -1.0, -1.0, -1.0]
HIP_SIDE_SIGN = [-1.0, -1.0, -1.0, -1.0]
KNEE_SIDE_SIGN = [-1.0, -1.0, -1.0, -1.0]
ABAD_OFFSET = [0.0, 0.0, 0.0, 0.0]
HIP_OFFSET = [0.0, 0.0, 0.0, 0.0]
KNEE_OFFSET = [0.0, 0.0, 0.0, 0.0]

# Порядок как в components.xacro / ControlBoardHardwareInterface
ACTUATORS = (
    {"name": "leg_front_r_1", "can_ch": 2, "can_id": 1, "homing_stage": 2,
     "homing_velocity": -1.5, "homed_position": -1.22, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_r_2", "can_ch": 2, "can_id": 2, "homing_stage": 1,
     "homing_velocity": -1.5, "homed_position": -0.42, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_r_3", "can_ch": 2, "can_id": 3, "homing_stage": 0,
     "homing_velocity": 1.5, "homed_position": 0.71, "post_homing_position": 0.71,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_l_1", "can_ch": 1, "can_id": 1, "homing_stage": 2,
     "homing_velocity": 1.5, "homed_position": 1.22, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_l_2", "can_ch": 1, "can_id": 2, "homing_stage": 1,
     "homing_velocity": 1.5, "homed_position": 0.42, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_l_3", "can_ch": 1, "can_id": 3, "homing_stage": 0,
     "homing_velocity": -1.5, "homed_position": -0.71, "post_homing_position": -0.71,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_back_r_1", "can_ch": 4, "can_id": 1, "homing_stage": 2,
     "homing_velocity": -1.5, "homed_position": -1.22, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_back_r_2", "can_ch": 4, "can_id": 2, "homing_stage": 1,
     "homing_velocity": -1.5, "homed_position": -0.42, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_back_r_3", "can_ch": 4, "can_id": 3, "homing_stage": 0,
     "homing_velocity": 1.5, "homed_position": 0.71, "post_homing_position": 0.71,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_back_l_1", "can_ch": 3, "can_id": 1, "homing_stage": 2,
     "homing_velocity": 1.5, "homed_position": 1.22, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_back_l_2", "can_ch": 3, "can_id": 2, "homing_stage": 1,
     "homing_velocity": 1.5, "homed_position": 0.42, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_back_l_3", "can_ch": 3, "can_id": 3, "homing_stage": 0,
     "homing_velocity": -1.5, "homed_position": -0.71, "post_homing_position": -0.71,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
)


def xor_checksum_u32(words: list[int], n_words: int) -> int:
    t = 0
    for i in range(n_words):
        t ^= words[i] & 0xFFFFFFFF
    return t & 0xFFFFFFFF


def swap16_bytes(buf: bytes) -> bytes:
    out = bytearray(len(buf))
    for i in range(0, len(buf), 2):
        out[i] = buf[i + 1]
        out[i + 1] = buf[i]
    return bytes(out)


def u32_list_to_wire(words: list[int]) -> list[int]:
    raw = struct.pack(f"<{len(words)}I", *words)
    swapped = swap16_bytes(raw)
    return list(swapped)


def wire_to_u32_list(wire: list[int], n_u32: int) -> list[int]:
    raw = bytes(wire[: n_u32 * 4])
    unswapped = swap16_bytes(raw)
    return list(struct.unpack(f"<{n_u32}I", unswapped))


@dataclass
class SpineCommand:
  """spine_cmd_t: 2 ноги на одной STM."""

  q_des_abad: list[float] = field(default_factory=lambda: [0.0, 0.0])
  q_des_hip: list[float] = field(default_factory=lambda: [0.0, 0.0])
  q_des_knee: list[float] = field(default_factory=lambda: [0.0, 0.0])
  qd_des_abad: list[float] = field(default_factory=lambda: [0.0, 0.0])
  qd_des_hip: list[float] = field(default_factory=lambda: [0.0, 0.0])
  qd_des_knee: list[float] = field(default_factory=lambda: [0.0, 0.0])
  kp_abad: list[float] = field(default_factory=lambda: [0.0, 0.0])
  kp_hip: list[float] = field(default_factory=lambda: [0.0, 0.0])
  kp_knee: list[float] = field(default_factory=lambda: [0.0, 0.0])
  kd_abad: list[float] = field(default_factory=lambda: [0.0, 0.0])
  kd_hip: list[float] = field(default_factory=lambda: [0.0, 0.0])
  kd_knee: list[float] = field(default_factory=lambda: [0.0, 0.0])
  tau_abad_ff: list[float] = field(default_factory=lambda: [0.0, 0.0])
  tau_hip_ff: list[float] = field(default_factory=lambda: [0.0, 0.0])
  tau_knee_ff: list[float] = field(default_factory=lambda: [0.0, 0.0])
  flags: list[int] = field(default_factory=lambda: [0, 0])

  def to_u32(self) -> list[int]:
    words: list[int] = []
    for pair in (
        self.q_des_abad,
        self.q_des_hip,
        self.q_des_knee,
        self.qd_des_abad,
        self.qd_des_hip,
        self.qd_des_knee,
        self.kp_abad,
        self.kp_hip,
        self.kp_knee,
        self.kd_abad,
        self.kd_hip,
        self.kd_knee,
        self.tau_abad_ff,
        self.tau_hip_ff,
        self.tau_knee_ff,
    ):
      words.extend(struct.unpack("<2I", struct.pack("<2f", *pair)))
    words.extend(struct.unpack("<2I", struct.pack("<2i", *self.flags)))
    assert len(words) == 32
    words.append(xor_checksum_u32(words, 32))
    return words

  def to_wire(self) -> list[int]:
    return u32_list_to_wire(self.to_u32())

  @classmethod
  def enable_only(cls) -> SpineCommand:
    return cls(flags=[1, 1])


@dataclass
class SpineData:
  q_abad: list[float]
  q_hip: list[float]
  q_knee: list[float]
  qd_abad: list[float]
  qd_hip: list[float]
  qd_knee: list[float]
  flags: list[int]
  checksum_reported: int
  checksum_calc: int
  raw_u32: list[int]
  raw_wire: list[int]

  @property
  def checksum_ok(self) -> bool:
    return self.checksum_calc == self.checksum_reported

  @property
  def all_zero(self) -> bool:
    return all(w == 0 for w in self.raw_u32[:IDX_CHECKSUM])

  @property
  def miso_floating(self) -> bool:
    """MISO подтянут вверх: все u16 слова 0xFFFF, checksum в ответе 0xFFFFFFFF."""
    if self.checksum_reported != 0xFFFFFFFF:
      return False
    return all(w == 0xFFFF for w in struct.unpack("<30H", bytes(self.raw_wire[:60])))

  @property
  def response_class(self) -> str:
    if self.all_zero:
      return "all_zero"
    if self.miso_floating:
      return "miso_0xFF (STM не драйвит MISO)"
    if self.checksum_ok:
      return "checksum_OK"
    return f"BAD_CHK calc=0x{self.checksum_calc:08x} rep=0x{self.checksum_reported:08x}"

  @classmethod
  def from_wire(cls, wire: list[int]) -> SpineData:
    u32 = wire_to_u32_list(wire, DATA_U32_WORDS)
    calc = xor_checksum_u32(u32, 14)
    reported = u32[IDX_CHECKSUM]
    raw = struct.pack(f"<{DATA_U32_WORDS}I", *u32)

    def f2(byte_offset: int) -> list[float]:
      return list(struct.unpack("<2f", raw[byte_offset : byte_offset + 8]))

    return cls(
        q_abad=f2(IDX_Q_ABAD * 4),
        q_hip=f2(IDX_Q_HIP * 4),
        q_knee=f2(IDX_Q_KNEE * 4),
        qd_abad=f2(IDX_QD_ABAD * 4),
        qd_hip=f2(IDX_QD_HIP * 4),
        qd_knee=f2(IDX_QD_KNEE * 4),
        flags=list(struct.unpack("<2i", raw[IDX_FLAGS * 4 : IDX_FLAGS * 4 + 8])),
        checksum_reported=reported,
        checksum_calc=calc,
        raw_u32=u32,
        raw_wire=wire[:MESSAGE_BYTES],
    )


@dataclass
class BoardResult:
  board: int
  device: str
  data: SpineData

  def leg_lines(self) -> list[str]:
    lines = []
    for leg in (0, 1):
      lines.append(
          f"    leg{leg}: abad={self.data.q_abad[leg]:7.4f} hip={self.data.q_hip[leg]:7.4f} "
          f"knee={self.data.q_knee[leg]:7.4f}  "
          f"qd=({self.data.qd_abad[leg]:6.3f},{self.data.qd_hip[leg]:6.3f},{self.data.qd_knee[leg]:6.3f}) "
          f"flags={self.data.flags[leg]}"
      )
    return lines


class SpineBus:
  def __init__(self, speed_hz: int = DEFAULT_SPEED_HZ) -> None:
    self.speed_hz = speed_hz
    self._spi: list[spidev.SpiDev] = []
    for bus, cs in ((0, 0), (0, 1)):
      dev = spidev.SpiDev()
      dev.open(bus, cs)
      dev.mode = 0
      dev.bits_per_word = 8
      dev.max_speed_hz = speed_hz
      self._spi.append(dev)

  def close(self) -> None:
    for dev in self._spi:
      dev.close()

  def exchange(self, board: int, cmd: SpineCommand) -> BoardResult:
    tx = cmd.to_wire()
    if len(tx) < MESSAGE_BYTES:
      tx = tx + [0] * (MESSAGE_BYTES - len(tx))
    rx = self._spi[board].xfer2(list(tx))
    data = SpineData.from_wire(rx)
    return BoardResult(board=board, device=DEVICES[board], data=data)

  def exchange_both(self, cmd0: SpineCommand, cmd1: SpineCommand) -> list[BoardResult]:
    return [self.exchange(0, cmd0), self.exchange(1, cmd1)]


def fmt_status(r: BoardResult) -> str:
  return f"[{r.device}] {BOARD_NAMES[r.board]}: {r.data.response_class}"


def print_board(r: BoardResult, verbose: bool = False) -> None:
  print(fmt_status(r))
  for line in r.leg_lines():
    print(line)
  if verbose:
    words = r.data.raw_u32
    hexes = " ".join(f"0x{w:08x}" for w in words)
    print(f"    raw u32: {hexes}")


def cmd_probe(bus: SpineBus, verbose: bool) -> int:
  print("=== probe: enable flags, одна транзакция на каждый CE ===\n")
  cmd = SpineCommand.enable_only()
  results = bus.exchange_both(cmd, cmd)
  ok = 0
  for r in results:
    print_board(r, verbose=verbose)
    print()
    if r.data.checksum_ok and not r.data.all_zero and not r.data.miso_floating:
      ok += 1
  if ok == 0:
    print("Ни одна плата не ответила валидным spine-пакетом.")
    print("Проверь: robot.service остановлен, прошивка STM, CE0/CE1, MISO.")
    return 1
  print(f"Валидный ответ: {ok}/2 плат.")
  return 0 if ok == 2 else 1


def cmd_watch(bus: SpineBus, interval: float, count: int, verbose: bool) -> None:
  print("=== watch: непрерывный обмен (Ctrl+C для выхода) ===\n")
  cmd = SpineCommand.enable_only()
  prev_q: list[list[float] | None] = [None, None]
  i = 0
  while count == 0 or i < count:
    i += 1
    results = bus.exchange_both(cmd, cmd)
    moved = []
    for r in results:
      q_flat = r.data.q_abad + r.data.q_hip + r.data.q_knee
      if prev_q[r.board] is not None and q_flat != prev_q[r.board]:
        moved.append(r.board)
      prev_q[r.board] = q_flat
    move_hint = f"  MOVED boards={moved}" if moved else ""
    line = f"iter={i:5d}  " + "  |  ".join(fmt_status(r) for r in results) + move_hint
    print(line)
    if verbose and i % 20 == 1:
      for r in results:
        print_board(r)
    time.sleep(interval)


def cmd_enable(bus: SpineBus, repeat: int) -> int:
  print(f"=== enable: flags=1 × {repeat} (как on_activate робота) ===\n")
  cmd = SpineCommand.enable_only()
  last: list[BoardResult] = []
  for n in range(repeat):
    last = bus.exchange_both(cmd, cmd)
    time.sleep(0.01)
  for r in last:
    print_board(r)
  ok = sum(1 for r in last if r.data.checksum_ok and not r.data.miso_floating)
  print(f"\nПоследний обмен: {ok}/2 плат с OK checksum.")
  return 0 if ok >= 1 else 1


def cmd_read(bus: SpineBus, repeat: int) -> int:
  print("=== read: чтение положений суставов ===\n")
  cmd = SpineCommand.enable_only()
  for _ in range(repeat):
    results = bus.exchange_both(cmd, cmd)
    time.sleep(0.01)
  for r in results:
    print_board(r, verbose=False)
    print()
  return 0


def cmd_homing(bus: SpineBus, seconds: float, kp: float, kd: float, velocity: float) -> int:
  """Упрощённый homing-тест: enable + медленное вращение knee с PD."""
  print("=== homing: enable → read → slow knee sweep → read ===\n")
  print("ВНИМАНИЕ: суставы могут чуть двинуться. Держи робота в безопасном положении.\n")

  enable = SpineCommand.enable_only()
  for _ in range(10):
    bus.exchange_both(enable, enable)
    time.sleep(0.01)

  baseline = bus.exchange_both(enable, enable)
  for r in baseline:
    print_board(r)
    if not r.data.checksum_ok:
      print(
          f"\n{r.device}: checksum не сходится — STM не принимает/не отвечает корректно. "
          "Homing не запускаем."
      )
      return 1

  print("\n--- отправка homing-команды (knee qd_des, kp/kd как у робота) ---")
  t0 = time.monotonic()
  step = 0
  while time.monotonic() - t0 < seconds:
    cmd0 = SpineCommand.enable_only()
    cmd1 = SpineCommand.enable_only()
    for cmd in (cmd0, cmd1):
      cmd.flags = [1, 1]
      cmd.kp_knee = [kp, kp]
      cmd.kd_knee = [kd, kd]
      cmd.qd_des_knee = [velocity, velocity]
    results = bus.exchange_both(cmd0, cmd1)
    if step % 10 == 0:
      print(f"  t={time.monotonic() - t0:4.1f}s  " + " | ".join(
          f"leg1_knee={r.data.q_knee[1]:.4f}" for r in results))
    step += 1
    time.sleep(0.01)

  print("\n--- после homing ---")
  final = bus.exchange_both(enable, enable)
  changed = False
  for b, (a, z) in enumerate(zip(baseline, final)):
    for leg in (0, 1):
      for name, va, vz in (
          ("abad", a.data.q_abad[leg], z.data.q_abad[leg]),
          ("hip", a.data.q_hip[leg], z.data.q_hip[leg]),
          ("knee", a.data.q_knee[leg], z.data.q_knee[leg]),
      ):
        delta = abs(vz - va)
        if delta > 0.001 and not (math.isnan(va) or math.isnan(vz)):
          changed = True
          print(f"  board{b} leg{leg} {name}: {va:.4f} -> {vz:.4f}  (Δ={delta:.4f})")
    print_board(z)
    print()

  if changed:
    print("Позиции изменились — STM принимает команды и шина CAN/моторы, возможно, живы.")
    return 0
  print("Позиции НЕ изменились — команда не дошла до моторов или обратная связь мертва.")
  return 1


def _spine_to_global_leg(data: SpineData, local_leg: int, global_leg: int,
                         out: dict[tuple[int, str], float], vel: dict[tuple[int, str], float]) -> None:
  g = global_leg
  for joint, q_attr, qd_attr in (
      ("abad", "q_abad", "qd_abad"),
      ("hip", "q_hip", "qd_hip"),
      ("knee", "q_knee", "qd_knee"),
  ):
    raw_q = getattr(data, q_attr)[local_leg]
    raw_qd = getattr(data, qd_attr)[local_leg]
    if joint == "abad":
      q = (raw_q - ABAD_OFFSET[g]) * ABAD_SIDE_SIGN[g]
      qd = raw_qd * ABAD_SIDE_SIGN[g]
    elif joint == "hip":
      q = (raw_q - HIP_OFFSET[g]) * HIP_SIDE_SIGN[g]
      qd = raw_qd * HIP_SIDE_SIGN[g]
    else:
      q = (raw_q - KNEE_OFFSET[g]) * KNEE_SIDE_SIGN[g]
      qd = raw_qd * KNEE_SIDE_SIGN[g]
    out[(g, joint)] = q
    vel[(g, joint)] = qd


def _actuator_state(act: dict, raw: dict[tuple[int, str], float], zero: float) -> float:
  leg = act["can_ch"] - 1
  joint = ("abad", "hip", "knee")[act["can_id"] - 1]
  return raw[(leg, joint)] - zero


def _read_actuator_states(bus: SpineBus, zero_offsets: list[float]) -> tuple[list[float], list[float], list[BoardResult]]:
  enable = SpineCommand.enable_only()
  results = bus.exchange_both(enable, enable)
  raw_pos: dict[tuple[int, str], float] = {}
  raw_vel: dict[tuple[int, str], float] = {}
  for board, leg_0 in ((0, 0), (1, 2)):
    r = results[board]
    for local in (0, 1):
      _spine_to_global_leg(r.data, local, leg_0 + local, raw_pos, raw_vel)
  pos = [_actuator_state(ACTUATORS[i], raw_pos, zero_offsets[i]) for i in range(12)]
  vel = [_actuator_state(ACTUATORS[i], raw_vel, 0.0) for i in range(12)]
  return pos, vel, results


def _global_to_spine_cmd(leg_0: int, flags: list[int],
                         cmd_pos: list[float], cmd_vel: list[float],
                         cmd_kp: list[float], cmd_kd: list[float],
                         zero_offsets: list[float]) -> SpineCommand:
  cmd = SpineCommand()
  for local in (0, 1):
    g = leg_0 + local
    cmd.flags[local] = flags[g]
  for i, act in enumerate(ACTUATORS):
    g = act["can_ch"] - 1
    if g not in (leg_0, leg_0 + 1):
      continue
    local = g - leg_0
    joint = ("abad", "hip", "knee")[act["can_id"] - 1]
    wire_pos = cmd_pos[i] + zero_offsets[i]
    wire_vel = cmd_vel[i]
    wire_kp = cmd_kp[i]
    wire_kd = cmd_kd[i]
    if joint == "abad":
      q_des, qd_des, kp, kd = cmd.q_des_abad, cmd.qd_des_abad, cmd.kp_abad, cmd.kd_abad
      q_des[local] = wire_pos * ABAD_SIDE_SIGN[g] + ABAD_OFFSET[g]
      qd_des[local] = wire_vel * ABAD_SIDE_SIGN[g]
    elif joint == "hip":
      q_des, qd_des, kp, kd = cmd.q_des_hip, cmd.qd_des_hip, cmd.kp_hip, cmd.kd_hip
      q_des[local] = wire_pos * HIP_SIDE_SIGN[g] + HIP_OFFSET[g]
      qd_des[local] = wire_vel * HIP_SIDE_SIGN[g]
    else:
      q_des, qd_des, kp, kd = cmd.q_des_knee, cmd.qd_des_knee, cmd.kp_knee, cmd.kd_knee
      q_des[local] = (wire_pos - KNEE_OFFSET[g]) * KNEE_SIDE_SIGN[g]
      qd_des[local] = wire_vel / KNEE_SIDE_SIGN[g]
    kp[local] = wire_kp
    kd[local] = wire_kd
  return cmd


def _board_ok(r: BoardResult) -> bool:
  return r.data.checksum_ok and not r.data.all_zero and not r.data.miso_floating


def cmd_homing_full(bus: SpineBus, timeout_s: float, dt_ms: int, board0_only: bool) -> int:
  """Полный homing как ControlBoardHardwareInterface::do_homing — ждёт до конца."""
  scope = "ноги 0–1 (CE0)" if board0_only else "все 12 суставов"
  print(f"=== homing-full: homing {scope} (как robot.service) ===\n")
  print("ВНИМАНИЕ: суставы будут двигаться. Держи робота в безопасном положении.\n")

  for _ in range(20):
    bus.exchange_both(SpineCommand.enable_only(), SpineCommand.enable_only())
    time.sleep(0.01)

  _, _, probe = _read_actuator_states(bus, [0.0] * 12)
  for r in probe:
    print_board(r)
  if not _board_ok(probe[0]):
    print(f"\n{probe[0].device}: нет валидного spine-ответа — homing не запускаем.")
    return 1
  if not board0_only and not _board_ok(probe[1]):
    print(f"\n{probe[1].device}: нет валидного spine-ответа — homing не запускаем.")
    print("Подсказка: пока CE1 мёртв, попробуй --board0-only")
    return 1
  print()

  n = len(ACTUATORS)
  zero_offsets = [0.0] * n
  is_homed = [False] * n
  if board0_only:
    for i, a in enumerate(ACTUATORS):
      if a["can_ch"] not in (1, 2):
        is_homed[i] = True
  cmd_pos = [0.0] * n
  cmd_vel = [0.0] * n
  cmd_kp = [0.0] * n
  cmd_kd = [0.0] * n
  flags = [1, 1, 1, 1]

  state_pos, state_vel, _ = _read_actuator_states(bus, zero_offsets)
  for i in range(n):
    cmd_pos[i] = state_pos[i]
  time.sleep(dt_ms * 0.001)

  filtered_torque = [0.0] * n
  alpha = 0.5
  t0 = time.monotonic()
  step = 0

  def exchange_commands() -> list[BoardResult]:
    c0 = _global_to_spine_cmd(0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
    c1 = _global_to_spine_cmd(2, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
    return bus.exchange_both(c0, c1)

  print("--- фаза 1: homing по стадиям ---")
  while not all(is_homed):
    if time.monotonic() - t0 > timeout_s:
      print(f"\nТАЙМАУТ {timeout_s:.0f}s — не все суставы захомились.")
      for i, a in enumerate(ACTUATORS):
        if not is_homed[i]:
          print(f"  НЕ ГОТОВ [{i}] {a['name']}")
      return 1

    homing_stage = 999
    for i, a in enumerate(ACTUATORS):
      if not is_homed[i] and a["homing_stage"] < homing_stage:
        homing_stage = a["homing_stage"]

    for i, a in enumerate(ACTUATORS):
      if homing_stage < a["homing_stage"]:
        cmd_pos[i] = state_pos[i]
        cmd_vel[i] = 0.0
        cmd_kp[i] = 0.0
        cmd_kd[i] = 0.0
        continue
      cmd_kp[i] = a["homing_kp"]
      cmd_kd[i] = a["homing_kd"]
      if not is_homed[i]:
        filtered_torque[i] = (
            (1.0 - alpha) * filtered_torque[i]
            + alpha * ((cmd_pos[i] - state_pos[i]) * cmd_kp[i]
                       + (cmd_vel[i] - state_vel[i]) * cmd_kd[i])
        )
        if abs(filtered_torque[i]) >= a["homing_torque_threshold"]:
          zero_offsets[i] = state_pos[i] - a["homed_position"]
          cmd_pos[i] = a["homed_position"]
          cmd_vel[i] = 0.0
          is_homed[i] = True
          print(f"  Homed [{i:2d}] {a['name']}  pos={state_pos[i]:.4f}  zero={zero_offsets[i]:.4f}")
        else:
          cmd_pos[i] += a["homing_velocity"] * dt_ms * 0.001
          cmd_vel[i] = a["homing_velocity"]

    exchange_commands()
    state_pos, state_vel, results = _read_actuator_states(bus, zero_offsets)
    if not _board_ok(results[0]):
      print(f"\n{results[0].device}: потерян spine-ответ на шаге {step} — останавливаемся.")
      return 1
    if not board0_only and not _board_ok(results[1]):
      print(f"\n{results[1].device}: потерян spine-ответ на шаге {step} — останавливаемся.")
      return 1

    step += 1
    if step % 50 == 0:
      done = sum(is_homed)
      print(f"  t={time.monotonic() - t0:5.1f}s  homed {done}/{n}")
    time.sleep(dt_ms * 0.001)

  print("\n--- фаза 2: возврат в post_homing_position ---")
  while True:
    if time.monotonic() - t0 > timeout_s:
      print(f"\nТАЙМАУТ на фазе возврата.")
      return 1
    all_returned = True
    for i, a in enumerate(ACTUATORS):
      target = a["post_homing_position"]
      vel = abs(a["homing_velocity"])
      if cmd_pos[i] < target - 0.01:
        cmd_pos[i] += vel * dt_ms * 0.001
        cmd_vel[i] = vel
        all_returned = False
      elif cmd_pos[i] > target + 0.01:
        cmd_pos[i] -= vel * dt_ms * 0.001
        cmd_vel[i] = -vel
        all_returned = False
      else:
        cmd_pos[i] = target
        cmd_vel[i] = 0.0
      cmd_kp[i] = a["homing_kp"]
      cmd_kd[i] = a["homing_kd"]

    exchange_commands()
    state_pos, state_vel, results = _read_actuator_states(bus, zero_offsets)
    if not _board_ok(results[0]):
      print(f"\n{results[0].device}: потерян spine-ответ на возврате — останавливаемся.")
      return 1
    if not board0_only and not _board_ok(results[1]):
      print(f"\n{results[1].device}: потерян spine-ответ на возврате — останавливаемся.")
      return 1
    if all_returned:
      break
    time.sleep(dt_ms * 0.001)

  elapsed = time.monotonic() - t0
  print(f"\nFinished homing! ({elapsed:.1f}s, {step} шагов)")
  for r in results:
    print_board(r)
  return 0


def cmd_hold_front_left(bus: SpineBus, seconds: float, kp: float, kd: float) -> int:
  """Держит только переднюю левую ногу (CAN ch1 / CE0 leg0). Остальные flags=0."""
  print("=== hold-front-left: удержание передней левой ноги ===\n")
  print("Только leg0 на CE0. Передняя правая и задние ноги выключены.\n")

  flags = [1, 0, 0, 0]
  zero_offsets = [0.0] * 12
  n = len(ACTUATORS)

  for _ in range(10):
    idle = SpineCommand()
    idle.flags = [0, 0]
    bus.exchange_both(idle, idle)
    time.sleep(0.01)

  pos, _, probe = _read_actuator_states(bus, zero_offsets)
  if not _board_ok(probe[0]):
    print(f"{probe[0].device}: CE0 не отвечает — не запускаем.")
    return 1
  print_board(probe[0])
  fl_idx = [i for i, a in enumerate(ACTUATORS) if a["can_ch"] == 1]
  if any(abs(pos[i]) < 1e-6 for i in fl_idx):
    print("\nПредупреждение: обратная связь по передней левой подозрительно нулевая.")
  print(f"\nУдержание {seconds:.0f}s (kp={kp}, kd={kd}). Ctrl+C — отпустить раньше.\n")

  cmd_pos = list(pos)
  cmd_vel = [0.0] * n
  cmd_kp = [0.0] * n
  cmd_kd = [0.0] * n
  for i in fl_idx:
    cmd_kp[i] = kp
    cmd_kd[i] = kd

  t0 = time.monotonic()
  step = 0
  idle = SpineCommand()
  idle.flags = [0, 0]
  try:
    while time.monotonic() - t0 < seconds:
      c0 = _global_to_spine_cmd(0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      bus.exchange_both(c0, idle)
      if step % 50 == 0:
        p, _, r = _read_actuator_states(bus, zero_offsets)
        knee = p[fl_idx[2]]
        print(f"  t={time.monotonic() - t0:4.1f}s  leg_front_l_3 knee={knee:.4f}")
        if not _board_ok(r[0]):
          print("  CE0 потерян — останавливаемся.")
          return 1
      step += 1
      time.sleep(0.01)
  except KeyboardInterrupt:
    print("\nПрервано — отпускаем моторы.")
  bus.exchange_both(idle, idle)
  print("\nГотово — моторы отпущены (flags=0).")
  return 0


def cmd_wiggle_front_left(
    bus: SpineBus, seconds: float, amplitude: float, hz: float, kp: float, kd: float,
) -> int:
  """Качает колено передней левой ноги туда-сюда вокруг текущей позиции."""
  print("=== wiggle-front-left: колено туда-сюда ===\n")
  print("Только leg0 / CAN1. Ctrl+C — стоп и отпустить.\n")

  flags = [1, 0, 0, 0]
  zero_offsets = [0.0] * 12
  n = len(ACTUATORS)
  fl_idx = [i for i, a in enumerate(ACTUATORS) if a["can_ch"] == 1]
  knee_i = fl_idx[2]

  idle = SpineCommand()
  idle.flags = [0, 0]
  for _ in range(10):
    bus.exchange_both(idle, idle)
    time.sleep(0.01)

  pos, _, probe = _read_actuator_states(bus, zero_offsets)
  if not _board_ok(probe[0]):
    print(f"{probe[0].device}: CE0 не отвечает.")
    return 1

  center_knee = pos[knee_i]
  print_board(probe[0])
  print(f"\nКолено: центр={center_knee:.3f} rad, амплитуда=±{amplitude:.3f}, {hz:.2f} Hz, {seconds:.0f}s\n")

  cmd_pos = list(pos)
  cmd_vel = [0.0] * n
  cmd_kp = [0.0] * n
  cmd_kd = [0.0] * n
  for i in fl_idx:
    cmd_kp[i] = kp
    cmd_kd[i] = kd

  t0 = time.monotonic()
  step = 0
  try:
    while time.monotonic() - t0 < seconds:
      t = time.monotonic() - t0
      cmd_pos[knee_i] = center_knee + amplitude * math.sin(2.0 * math.pi * hz * t)
      cmd_vel[knee_i] = amplitude * 2.0 * math.pi * hz * math.cos(2.0 * math.pi * hz * t)
      c0 = _global_to_spine_cmd(0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      bus.exchange_both(c0, idle)
      if step % 40 == 0:
        p, _, r = _read_actuator_states(bus, zero_offsets)
        print(f"  t={t:4.1f}s  cmd={cmd_pos[knee_i]:.3f}  knee={p[knee_i]:.3f}")
        if not _board_ok(r[0]):
          print("  CE0 потерян.")
          return 1
      step += 1
      time.sleep(0.01)
  except KeyboardInterrupt:
    print("\nПрервано.")
  bus.exchange_both(idle, idle)
  print("\nСтоп — моторы отпущены.")
  return 0


def cmd_selftest() -> int:
  print("=== selftest: кодирование пакета enable ===\n")
  cmd = SpineCommand.enable_only()
  u32 = cmd.to_u32()
  assert u32[30] == 1 and u32[31] == 1, "flags"
  assert u32[32] == 0, f"checksum expected 0 got {u32[32]}"
  wire = cmd.to_wire()
  back = wire_to_u32_list(wire, CMD_U32_WORDS)
  assert back == u32, "wire roundtrip"
  print("  flags @ u32[30,31] = 1,1")
  print(f"  cmd checksum = 0x{u32[32]:08x}")
  print(f"  wire length = {len(wire)} bytes")
  print("  OK")
  return 0


def build_parser() -> argparse.ArgumentParser:
  p = argparse.ArgumentParser(
      description="Тест SPI spine (Pi ↔ STM). Останови robot.service перед запуском.",
      formatter_class=argparse.RawDescriptionHelpFormatter,
      epilog=__doc__,
  )
  p.add_argument("--speed", type=int, default=DEFAULT_SPEED_HZ, help="SPI Hz (default 6M)")
  sub = p.add_subparsers(dest="command", required=True)

  sub.add_parser("selftest", help="Проверка кодирования без SPI")

  s = sub.add_parser("probe", help="Один обмен на CE0 и CE1")
  s.add_argument("-v", "--verbose", action="store_true")

  s = sub.add_parser("watch", help="Непрерывный мониторинг")
  s.add_argument("--interval", type=float, default=0.05)
  s.add_argument("--count", type=int, default=0, help="0 = бесконечно")
  s.add_argument("-v", "--verbose", action="store_true")

  s = sub.add_parser("enable", help="10× enable flags (как activate)")
  s.add_argument("--repeat", type=int, default=10)

  s = sub.add_parser("read", help="Прочитать положения")
  s.add_argument("--repeat", type=int, default=3)

  s = sub.add_parser("homing", help="Тест homing-подобной команды на knee")
  s.add_argument("--seconds", type=float, default=3.0)
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.2)
  s.add_argument("--velocity", type=float, default=0.3, help="qd_des_knee rad/s")

  s = sub.add_parser("homing-full", help="Полный homing 12 суставов, ждёт до конца")
  s.add_argument("--timeout", type=float, default=180.0, help="секунд")
  s.add_argument("--dt-ms", type=int, default=10, help="шаг цикла, мс")
  s.add_argument("--board0-only", action="store_true",
                 help="Только CE0 (передние ноги), пока CE1 не работает")

  s = sub.add_parser("hold-front-left",
                     help="Удержать только переднюю левую ногу (рабочую)")
  s.add_argument("--seconds", type=float, default=15.0)
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)

  s = sub.add_parser("wiggle-front-left", help="Качать колено передней левой туда-сюда")
  s.add_argument("--seconds", type=float, default=15.0)
  s.add_argument("--amplitude", type=float, default=0.12, help="рад от центра")
  s.add_argument("--hz", type=float, default=0.4, help="частота Гц")
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)

  return p


def main() -> int:
  args = build_parser().parse_args()
  bus = SpineBus(speed_hz=args.speed)
  try:
    if args.command == "selftest":
      return cmd_selftest()
    if args.command == "probe":
      return cmd_probe(bus, verbose=args.verbose)
    if args.command == "watch":
      cmd_watch(bus, args.interval, args.count, args.verbose)
      return 0
    if args.command == "enable":
      return cmd_enable(bus, args.repeat)
    if args.command == "read":
      return cmd_read(bus, args.repeat)
    if args.command == "homing":
      return cmd_homing(bus, args.seconds, args.kp, args.kd, args.velocity)
    if args.command == "homing-full":
      return cmd_homing_full(bus, args.timeout, args.dt_ms, args.board0_only)
    if args.command == "hold-front-left":
      return cmd_hold_front_left(bus, args.seconds, args.kp, args.kd)
    if args.command == "wiggle-front-left":
      return cmd_wiggle_front_left(
          bus, args.seconds, args.amplitude, args.hz, args.kp, args.kd)
    return 2
  finally:
    bus.close()


if __name__ == "__main__":
  sys.exit(main())
