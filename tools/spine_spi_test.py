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
  sudo python3 spine_spi_test.py homing          # отказ (рвёт к q=0)
  sudo python3 spine_spi_test.py homing-full     # копия ROS do_homing, только CE0

Знаки как в rt_spi.cpp (все −1). Штатный homing: sudo systemctl restart robot
(это процесс на Pi, STM/плату не ресетит).

  sudo python3 spine_spi_test.py motor-scan      # все 6 передних: ±0.5 rad, кто едет

Перед тестом: sudo systemctl stop robot.service
"""

from __future__ import annotations

import argparse
import math
import select
import struct
import sys
import time
from dataclasses import dataclass, field

import spidev
import yaml

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

# Как rt_spi.cpp: abad/hip/knee_side_sign[] = −1 на всех ногах.
ABAD_SIDE_SIGN = [-1.0, -1.0, -1.0, -1.0]
HIP_SIDE_SIGN = [-1.0, -1.0, -1.0, -1.0]
KNEE_SIDE_SIGN = [-1.0, -1.0, -1.0, -1.0]
ABAD_OFFSET = [0.0, 0.0, 0.0, 0.0]
HIP_OFFSET = [0.0, 0.0, 0.0, 0.0]
KNEE_OFFSET = [0.0, 0.0, 0.0, 0.0]

# Порядок как в components.xacro / ControlBoardHardwareInterface
ACTUATORS = (
    {"name": "leg_front_r_1", "can_ch": 2, "can_id": 1, "homing_stage": 5,
     "homing_velocity": -1.5, "homed_position": -1.22, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_r_2", "can_ch": 2, "can_id": 2, "homing_stage": 4,
     "homing_velocity": -1.5, "homed_position": -0.42, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_r_3", "can_ch": 2, "can_id": 3, "homing_stage": 3,
     "homing_velocity": 1.5, "homed_position": 0.71, "post_homing_position": 0.71,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_l_1", "can_ch": 1, "can_id": 1, "homing_stage": 5,
     "homing_velocity": 1.5, "homed_position": 1.22, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_l_2", "can_ch": 1, "can_id": 2, "homing_stage": 4,
     "homing_velocity": 1.5, "homed_position": 0.42, "post_homing_position": 0.0,
     "homing_kp": 5.5, "homing_kd": 0.2, "homing_torque_threshold": 2.0},
    {"name": "leg_front_l_3", "can_ch": 1, "can_id": 3, "homing_stage": 3,
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

MOTOR_MAP_PATH = "/home/pi/pupperv3-monorepo/robot/config/motor_map.yaml"


def apply_motor_map(path: str = MOTOR_MAP_PATH) -> None:
    """Подменяет can_ch/can_id из robot/config/motor_map.yaml. Хоуминг остаётся у сустава."""
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    joints = data.get("joints") if isinstance(data, dict) else None
    if not isinstance(joints, dict):
        raise SystemExit(f"{path}: нет словаря joints")
    by_name = {actuator["name"]: actuator for actuator in ACTUATORS}
    if set(joints) != set(by_name):
        missing = sorted(set(by_name) - set(joints))
        extra = sorted(set(joints) - set(by_name))
        raise SystemExit(f"{path}: missing={missing} extra={extra}")
    seen: set[tuple[int, int]] = set()
    for name, spec in joints.items():
        channel = int(spec["can_channel"])
        can_id = int(spec["can_id"])
        if channel not in (1, 2, 3, 4) or can_id not in (1, 2, 3):
            raise SystemExit(f"{path}: {name} CAN {channel} id {can_id} вне диапазона")
        key = (channel, can_id)
        if key in seen:
            raise SystemExit(f"{path}: CAN {channel} id {can_id} назначен дважды")
        seen.add(key)
        by_name[name]["can_ch"] = channel
        by_name[name]["can_id"] = can_id


apply_motor_map()


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


def _silent_board_result(board: int) -> BoardResult:
  """Заглушка для отсутствующей платы: без реального SPI-обмена."""
  raw = bytes(DATA_U32_WORDS * 4)
  return BoardResult(board=board, device=DEVICES[board], data=SpineData.from_wire(raw))


class SpineBus:
  def __init__(
      self,
      speed_hz: int = DEFAULT_SPEED_HZ,
      board0_only: bool = False,
  ) -> None:
    self.speed_hz = speed_hz
    self.board0_only = board0_only
    self._spi: list[spidev.SpiDev | None] = [None, None]
    boards = (0,) if board0_only else (0, 1)
    for board in boards:
      bus, cs = 0, board
      dev = spidev.SpiDev()
      dev.open(bus, cs)
      dev.mode = 0
      dev.bits_per_word = 8
      dev.max_speed_hz = speed_hz
      self._spi[board] = dev

  def close(self) -> None:
    for dev in self._spi:
      if dev is not None:
        dev.close()

  def exchange(self, board: int, cmd: SpineCommand) -> BoardResult:
    if self._spi[board] is None:
      return _silent_board_result(board)
    tx = cmd.to_wire()
    if len(tx) < MESSAGE_BYTES:
      tx = tx + [0] * (MESSAGE_BYTES - len(tx))
    rx = self._spi[board].xfer2(list(tx))
    data = SpineData.from_wire(rx)
    return BoardResult(board=board, device=DEVICES[board], data=data)

  def exchange_both(self, cmd0: SpineCommand, cmd1: SpineCommand) -> list[BoardResult]:
    # При одной STM нельзя долбить CE1: лишний CS на общей шине + риск
    # рассинхрона/EMI. board0_only → только CE0.
    r0 = self.exchange(0, cmd0)
    r1 = self.exchange(1, cmd1) if not self.board0_only else _silent_board_result(1)
    return [r0, r1]


def _disable_motors(bus: SpineBus) -> None:
  idle = SpineCommand()
  idle.flags = [0, 0]
  bus.exchange(0, idle)
  if not bus.board0_only:
    bus.exchange(1, idle)


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
  scope = "только CE0" if bus.board0_only else "CE0 и CE1"
  print(f"=== probe: enable flags, {scope} ===\n")
  cmd = SpineCommand.enable_only()
  results = bus.exchange_both(cmd, cmd)
  ok = 0
  expect = 1 if bus.board0_only else 2
  for r in results:
    if bus.board0_only and r.board == 1:
      print(f"[{r.device}] {BOARD_NAMES[r.board]}: skipped (--board0-only)\n")
      continue
    print_board(r, verbose=verbose)
    print()
    if r.data.checksum_ok and not r.data.all_zero and not r.data.miso_floating:
      ok += 1
  if ok == 0:
    print("Ни одна плата не ответила валидным spine-пакетом.")
    print("Проверь: robot.service остановлен, прошивка STM, CE0/CE1, MISO.")
    return 1
  print(f"Валидный ответ: {ok}/{expect} плат.")
  return 0 if ok == expect else 1


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
  """Не homing робота. Старая реализация рвала колена к q_des=0 — больше не шлём."""
  del seconds, kp, kd, velocity, bus
  print("СТОП: `homing` — это НЕ homing при старте Pi.")
  print()
  print("При boot homing делает robot.service (ControlBoardHardwareInterface::do_homing):")
  print("  q_des = текущая позиция, потом медленный упор, потом post_homing.")
  print()
  print("Эта команда раньше ставила q_des_knee=0 и kp=5.5 на обе STM.")
  print("Моторы рвались к нулю, часть уезжала в упор и потом не слушала wiggle")
  print("(залипали на последней PD-команде).")
  print()
  print("Нужен штатный homing:")
  print("  sudo systemctl start robot.service")
  print("Не запускай `homing-full`, пока CAN id 1/2/3 не совпадают с суставами.")
  print("После срыва — силовой цикл моторов (не только reboot Pi).")
  return 2


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


def _states_from_results(
    results: list[BoardResult], zero_offsets: list[float]
) -> tuple[list[float], list[float]]:
  raw_pos: dict[tuple[int, str], float] = {}
  raw_vel: dict[tuple[int, str], float] = {}
  for board, leg_0 in ((0, 0), (1, 2)):
    r = results[board]
    for local in (0, 1):
      _spine_to_global_leg(r.data, local, leg_0 + local, raw_pos, raw_vel)
  pos = [_actuator_state(ACTUATORS[i], raw_pos, zero_offsets[i]) for i in range(12)]
  vel = [_actuator_state(ACTUATORS[i], raw_vel, 0.0) for i in range(12)]
  return pos, vel


def _read_actuator_states(bus: SpineBus, zero_offsets: list[float]) -> tuple[list[float], list[float], list[BoardResult]]:
  enable = SpineCommand.enable_only()
  results = bus.exchange_both(enable, enable)
  pos, vel = _states_from_results(results, zero_offsets)
  return pos, vel, results


def _global_to_spine_cmd(leg_0: int, flags: list[int],
                         cmd_pos: list[float], cmd_vel: list[float],
                         cmd_kp: list[float], cmd_kd: list[float],
                         zero_offsets: list[float]) -> SpineCommand:
  cmd = SpineCommand()
  for local in (0, 1):
    g = leg_0 + local
    cmd.flags[local] = flags[g]
  for i, acd in enumerate(ACTUATORS):
    g = acd["can_ch"] - 1
    if g not in (leg_0, leg_0 + 1):
      continue
    local = g - leg_0
    joint = ("abad", "hip", "knee")[acd["can_id"] - 1]
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
      q_des[local] = wire_pos * KNEE_SIDE_SIGN[g] + KNEE_OFFSET[g]
      qd_des[local] = wire_vel * KNEE_SIDE_SIGN[g]
    kp[local] = wire_kp
    kd[local] = wire_kd
  return cmd


def _board_ok(r: BoardResult) -> bool:
  return r.data.checksum_ok and not r.data.all_zero and not r.data.miso_floating


def cmd_homing_full(bus: SpineBus, timeout_s: float, dt_ms: int, board0_only: bool) -> int:
  """Копия ROS do_homing по CE0. Плату/STM не ресетит."""
  print("=== homing-full: CE0, знаки как rt_spi.cpp (−1), ложный упор → SKIP ===\n")
  print("Ноги в воздухе, не в упоре. Ctrl+C — отпустить момент.")
  print("Левый abad id=1 с fb=0 будет SKIP, не Homed.")
  print()
  try:
    return _cmd_homing_full_body(bus, timeout_s, dt_ms, board0_only)
  finally:
    _disable_motors(bus)
    print("Моторы отпущены (flags=0).")


def _cmd_homing_full_body(bus: SpineBus, timeout_s: float, dt_ms: int, board0_only: bool) -> int:
  for _ in range(40):
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
  skipped = [False] * n
  flags = [1, 1, 0, 0] if board0_only else [1, 1, 1, 1]

  state_pos, state_vel, _ = _read_actuator_states(bus, zero_offsets)
  for i in range(n):
    cmd_pos[i] = state_pos[i]
  start_pos = list(state_pos)
  time.sleep(dt_ms * 0.001)

  filtered_torque = [0.0] * n
  alpha = 0.5
  t0 = time.monotonic()
  step = 0
  lost = 0

  idle = SpineCommand()
  idle.flags = [0, 0]

  def exchange_commands() -> list[BoardResult]:
    c0 = _global_to_spine_cmd(0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
    c1 = _global_to_spine_cmd(2, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
    return bus.exchange_both(c0, c1)

  def check_ce0(results: list[BoardResult], where: str) -> bool:
    nonlocal lost
    if _board_ok(results[0]):
      lost = 0
      return True
    lost += 1
    print(
        f"  warn {where} step={step} CE0 {results[0].data.response_class} "
        f"lost={lost}"
    )
    if lost >= 8:
      print(f"\n{results[0].device}: потерян spine-ответ ({where}) — стоп.")
      bus.exchange(0, idle)
      return False
    return True

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
          traveled = abs(state_pos[i] - start_pos[i])
          if traveled < 0.10:
            cmd_kp[i] = 0.0
            cmd_kd[i] = 0.0
            cmd_vel[i] = 0.0
            is_homed[i] = True
            skipped[i] = True
            print(
                f"  SKIP [{i:2d}] {a['name']}  fb не едет "
                f"(Δ={traveled:.4f}) — ложный упор, отпускаем"
            )
          else:
            zero_offsets[i] = state_pos[i] - a["homed_position"]
            cmd_pos[i] = a["homed_position"]
            cmd_vel[i] = 0.0
            is_homed[i] = True
            print(
                f"  Homed [{i:2d}] {a['name']}  pos={state_pos[i]:.4f}  "
                f"zero={zero_offsets[i]:.4f}"
            )
        else:
          cmd_pos[i] += a["homing_velocity"] * dt_ms * 0.001
          cmd_vel[i] = a["homing_velocity"]

    results = exchange_commands()
    if not check_ce0(results, "фаза1"):
      return 1
    if not _board_ok(results[0]):
      step += 1
      time.sleep(dt_ms * 0.001)
      continue
    if not board0_only and not _board_ok(results[1]):
      print(f"\n{results[1].device}: потерян spine-ответ на шаге {step} — останавливаемся.")
      bus.exchange(0, idle)
      return 1
    state_pos, state_vel = _states_from_results(results, zero_offsets)

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
      if skipped[i] or (board0_only and a["can_ch"] not in (1, 2)):
        cmd_kp[i] = 0.0
        cmd_kd[i] = 0.0
        cmd_vel[i] = 0.0
        continue
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

    results = exchange_commands()
    if not check_ce0(results, "возврат"):
      return 1
    if not _board_ok(results[0]):
      time.sleep(dt_ms * 0.001)
      continue
    if not board0_only and not _board_ok(results[1]):
      print(f"\n{results[1].device}: потерян spine-ответ на возврате — останавливаемся.")
      bus.exchange(0, idle)
      return 1
    state_pos, state_vel = _states_from_results(results, zero_offsets)
    if all_returned:
      break
    time.sleep(dt_ms * 0.001)

  elapsed = time.monotonic() - t0
  print(f"\nFinished homing! ({elapsed:.1f}s, {step} шагов)")
  print("HOMING_RESULT  name             CAN id   start    end      d      status  hold")
  for i, a in enumerate(ACTUATORS):
    delta = state_pos[i] - start_pos[i]
    active = not (board0_only and a["can_ch"] not in (1, 2))
    if not active:
      status = "ВЫКЛ"
    elif abs(delta) >= 0.10:
      status = "ЕХАЛ"
    else:
      status = "СТОИТ"
    hold = "ДЕРЖИТ" if cmd_kp[i] > 0.0 else "ОТПУЩЕН"
    print(
        f"HOMING_RESULT  {a['name']:<16} {a['can_ch']}   {a['can_id']}  "
        f"{start_pos[i]:+7.3f}  {state_pos[i]:+7.3f}  {delta:+7.3f}  {status:<6}  {hold}"
    )
  print("ЕХАЛ = энкодер сдвинулся ≥0.10 рад. ДЕРЖИТ = момент ещё включён.")
  print("Скрипт сразу после таблицы снимает момент со всех.")
  for r in results:
    print_board(r)
  return 0


def cmd_hold_front_left(bus: SpineBus, seconds: float, kp: float, kd: float) -> int:
  """Держит только переднюю левую ногу (CAN ch1 / CE0 leg0). Остальные flags=0."""
  print("=== hold-front-left: удержание передней левой ноги ===\n")
  print("Только CE0 / leg0. Передняя правая и задние ноги выключены.\n")

  flags = [1, 0, 0, 0]
  zero_offsets = [0.0] * 12
  n = len(ACTUATORS)

  idle = SpineCommand()
  idle.flags = [0, 0]
  for _ in range(10):
    bus.exchange(0, idle)
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
  lost = 0
  try:
    while time.monotonic() - t0 < seconds:
      c0 = _global_to_spine_cmd(0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      r = bus.exchange(0, c0)
      if step % 50 == 0:
        print(f"  t={time.monotonic() - t0:4.1f}s  knee={r.data.q_knee[0]:.4f}  {r.data.response_class}")
      if not _board_ok(r):
        lost += 1
        if lost >= 5:
          print("  CE0 потерян — останавливаемся.")
          bus.exchange(0, idle)
          return 1
      else:
        lost = 0
      step += 1
      time.sleep(0.01)
  except KeyboardInterrupt:
    print("\nПрервано — отпускаем моторы.")
  bus.exchange(0, idle)
  print("\nГотово — моторы отпущены (flags=0).")
  return 0


def cmd_hold(bus: SpineBus, seconds: float, kp: float, kd: float, force_all: bool) -> int:
  """Держит текущую позицию. С --all зажимает и моторы с нулевым энкодером."""
  print("=== hold: фиксация текущей позиции, без хоуминга ===\n")
  if force_all:
    print("Режим --all: момент на все 12, даже если энкодер 0.")
    print("Нулевой энкодер значит цель 0 рад: если он врёт, мотор поедет к этому нулю.")
  else:
    print("Мотор с энкодером около нуля не зажимается. Для всех 12 добавь --all.")
  print("Ctrl+C снимает момент со всех моторов.\n")

  zero_offsets = [0.0] * len(ACTUATORS)
  for _ in range(20):
    bus.exchange_both(SpineCommand.enable_only(), SpineCommand.enable_only())
    time.sleep(0.01)
  pos, _, probe = _read_actuator_states(bus, zero_offsets)
  for r in probe:
    print_board(r)
  if not _board_ok(probe[0]):
    print(f"\n{probe[0].device}: CE0 не отвечает — фиксацию не включаем.")
    return 1

  flags = [0, 0, 0, 0]
  cmd_kp = [0.0] * len(ACTUATORS)
  cmd_kd = [0.0] * len(ACTUATORS)
  cmd_vel = [0.0] * len(ACTUATORS)
  held = 0
  print()
  for i, actuator in enumerate(ACTUATORS):
    silent = abs(pos[i]) < 1e-3
    if silent and not force_all:
      print(
          f"  пропуск {actuator['name']}  CAN{actuator['can_ch']} "
          f"id{actuator['can_id']}  fb={pos[i]:+.3f}"
      )
      continue
    flags[actuator["can_ch"] - 1] = 1
    cmd_kp[i] = kp
    cmd_kd[i] = kd
    held += 1
    note = "  энкодер 0, цель 0" if silent else ""
    print(
        f"  держу   {actuator['name']}  CAN{actuator['can_ch']} "
        f"id{actuator['can_id']}  fb={pos[i]:+.3f}{note}"
    )
  if held == 0:
    print("\nНечего фиксировать: у всех энкодер около нуля.")
    return 1
  print(f"\nФиксация {held} моторов, kp={kp:.1f}, {seconds:.0f} с. В упор не едем.\n")

  idle = SpineCommand()
  idle.flags = [0, 0]

  def release() -> None:
    for _ in range(20):
      bus.exchange_both(idle, idle)
      time.sleep(0.01)
    print("Момент снят (flags=0).")

  t0 = time.monotonic()
  step = 0
  try:
    while time.monotonic() - t0 < seconds:
      c0 = _global_to_spine_cmd(0, flags, pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      c1 = _global_to_spine_cmd(2, flags, pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      bus.exchange_both(c0, c1)
      if step % 100 == 0:
        print(f"  t={time.monotonic() - t0:5.1f}s  держу {held}")
      step += 1
      time.sleep(0.01)
  except KeyboardInterrupt:
    print("\nCtrl+C — отпускаем моторы.")
    release()
    return 0
  print("Время вышло. Последняя команда — фиксация, момент не снимал.")
  return 0


def cmd_wiggle_front_left(
    bus: SpineBus, seconds: float, amplitude: float, hz: float, kp: float, kd: float,
) -> int:
  """Качает колено передней левой ноги туда-сюда вокруг текущей позиции."""
  print("=== wiggle-front-left: колено туда-сюда ===\n")
  print("Только CE0 / leg0 / CAN1. Ctrl+C — стоп и отпустить.\n")
  if not bus.board0_only:
    print("Подсказка: лучше --board0-only (не трогать CE1).\n")

  flags = [1, 0, 0, 0]
  zero_offsets = [0.0] * 12
  n = len(ACTUATORS)
  fl_idx = [i for i, a in enumerate(ACTUATORS) if a["can_ch"] == 1]
  knee_i = fl_idx[2]

  idle = SpineCommand()
  idle.flags = [0, 0]
  for _ in range(10):
    bus.exchange(0, idle)
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
  lost = 0
  try:
    while time.monotonic() - t0 < seconds:
      t = time.monotonic() - t0
      cmd_pos[knee_i] = center_knee + amplitude * math.sin(2.0 * math.pi * hz * t)
      cmd_vel[knee_i] = amplitude * 2.0 * math.pi * hz * math.cos(2.0 * math.pi * hz * t)
      c0 = _global_to_spine_cmd(0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      r = bus.exchange(0, c0)
      if step % 40 == 0:
        print(f"  t={t:4.1f}s  cmd={cmd_pos[knee_i]:.3f}  knee={r.data.q_knee[0]:.3f}  {r.data.response_class}")
      if not _board_ok(r):
        lost += 1
        if lost >= 5:
          print("  CE0 потерян (5 плохих кадров подряд).")
          bus.exchange(0, idle)
          return 1
      else:
        lost = 0
      step += 1
      time.sleep(0.01)
  except KeyboardInterrupt:
    print("\nПрервано.")
  bus.exchange(0, idle)
  print("\nСтоп — моторы отпущены.")
  return 0


def _countdown(label: str) -> None:
  print(f"\n>>> {label}", flush=True)
  for n in (3, 2, 1):
    line = f"    {n}..."
    print(line, flush=True)
    for tty in ("/dev/tty1", "/dev/console"):
      try:
        with open(tty, "w") as f:
          f.write(f"\n{label}\n{n}\n")
      except OSError:
        pass
    time.sleep(1)
  print("    СМОТРИ!\n", flush=True)
  for tty in ("/dev/tty1", "/dev/console"):
    try:
      with open(tty, "w") as f:
        f.write(f"\nСМОТРИ  {label}\n")
    except OSError:
      pass


def _cmd_wiggle_can_leg_all(
    bus: SpineBus,
    can_ch: int,
    seconds: float,
    amplitude: float,
    hz: float,
    kp: float,
    kd: float,
    joints: tuple[str, ...] = ("abad", "hip", "knee"),
    countdown: bool = False,
) -> int:
  """Качает указанные суставы на CAN-канале (CE0)."""
  global_leg = can_ch - 1
  flags = [0, 0, 0, 0]
  flags[global_leg] = 1
  all_idx = [i for i, a in enumerate(ACTUATORS) if a["can_ch"] == can_ch]
  joint_names = ("abad", "hip", "knee")
  want = {n: i for i, n in enumerate(joint_names) if n in joints}
  if not want:
    print(f"Нет суставов в {joints}")
    return 1
  leg_idx = [all_idx[j] for j in sorted(want.values())]
  names_run = tuple(joint_names[j] for j in sorted(want.values()))

  print(
      f"=== wiggle-can{can_ch}: {', '.join(names_run)} ===\n"
  )
  print(
      f"CE0 / leg{global_leg}: ID1 abad, ID2 hip, ID3 knee. "
      "Ctrl+C — стоп.\n"
  )

  zero_offsets = [0.0] * 12
  n = len(ACTUATORS)

  idle = SpineCommand()
  idle.flags = [0, 0]
  for _ in range(15):
    bus.exchange(0, SpineCommand.enable_only())
    time.sleep(0.01)

  pos, _, probe = _read_actuator_states(bus, zero_offsets)
  if not _board_ok(probe[0]):
    print(f"{probe[0].device}: CE0 не отвечает.")
    return 1
  print_board(probe[0])
  print(
      f"\nАмплитуда=±{amplitude:.3f} rad, {hz:.2f} Hz, "
      f"по {seconds:.0f}s на сустав\n"
  )

  cmd_pos = list(pos)
  cmd_vel = [0.0] * n
  cmd_kp = [0.0] * n
  cmd_kd = [0.0] * n
  for i in leg_idx:
    cmd_kp[i] = kp
    cmd_kd[i] = kd

  try:
    for jname in names_run:
      j = joint_names.index(jname)
      gi = all_idx[j]
      p_now, _, rr = _read_actuator_states(bus, zero_offsets)
      if not _board_ok(rr[0]):
        print("CE0 потерян перед суставом", jname)
        bus.exchange(0, idle)
        return 1
      for ii in all_idx:
        cmd_pos[ii] = p_now[ii]
        cmd_vel[ii] = 0.0
      center = cmd_pos[gi]
      if countdown:
        side = "левая" if can_ch == 1 else "правая"
        _countdown(f"{side} нога  колено" if jname == "knee" else f"{side}  {jname}")
      print(f"--- {jname} (CAN{can_ch} id={j + 1}) center={center:.3f} ---")
      t0 = time.monotonic()
      step = 0
      lost = 0
      min_fb = None
      max_fb = None
      while time.monotonic() - t0 < seconds:
        t = time.monotonic() - t0
        cmd_pos[gi] = center + amplitude * math.sin(2.0 * math.pi * hz * t)
        cmd_vel[gi] = (
            amplitude * 2.0 * math.pi * hz * math.cos(2.0 * math.pi * hz * t)
        )
        c0 = _global_to_spine_cmd(
            0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets
        )
        r = bus.exchange(0, c0)
        fb = (
            r.data.q_abad[global_leg],
            r.data.q_hip[global_leg],
            r.data.q_knee[global_leg],
        )[j]
        if min_fb is None:
          min_fb, max_fb = fb, fb
        else:
          min_fb = min(min_fb, fb)
          max_fb = max(max_fb, fb)
        if step % 40 == 0:
          print(
              f"  t={t:4.1f}s  cmd={cmd_pos[gi]:.3f}  fb={fb:.3f}  "
              f"{r.data.response_class}"
          )
        if not _board_ok(r):
          lost += 1
          if lost >= 5:
            print("  CE0 потерян.")
            bus.exchange(0, idle)
            return 1
        else:
          lost = 0
        step += 1
        time.sleep(0.01)
      cmd_pos[gi] = center
      cmd_vel[gi] = 0.0
      for _ in range(20):
        c0 = _global_to_spine_cmd(
            0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets
        )
        bus.exchange(0, c0)
        time.sleep(0.01)
      span = (max_fb - min_fb) if min_fb is not None else 0.0
      status = "OK" if span > 0.03 else "слабо/нет движения"
      print(f"  итог {jname}: span={span:.3f} rad  {status}\n")
  except KeyboardInterrupt:
    print("\nПрервано.")
  bus.exchange(0, idle)
  print("Стоп — моторы отпущены.")
  return 0


def cmd_wiggle_can1_all(
    bus: SpineBus, seconds: float, amplitude: float, hz: float, kp: float, kd: float,
    joints: tuple[str, ...] = ("abad", "hip", "knee"), countdown: bool = False,
) -> int:
  """Качает суставы на CAN1 (передняя левая / CE0 leg0)."""
  return _cmd_wiggle_can_leg_all(
      bus, 1, seconds, amplitude, hz, kp, kd, joints=joints, countdown=countdown)


def cmd_wiggle_can2_all(
    bus: SpineBus, seconds: float, amplitude: float, hz: float, kp: float, kd: float,
    joints: tuple[str, ...] = ("abad", "hip", "knee"), countdown: bool = False,
) -> int:
  """Качает суставы на CAN2 (передняя правая / CE0 leg1)."""
  return _cmd_wiggle_can_leg_all(
      bus, 2, seconds, amplitude, hz, kp, kd, joints=joints, countdown=countdown)


def cmd_wiggle_6(
    bus: SpineBus, seconds: float, amplitude: float, hz: float, kp: float, kd: float,
) -> int:
  """По очереди 6 моторов: левая abad/hip/knee, потом правая."""
  print("=== wiggle-6: 6 моторов туда-сюда ===\n")
  print("Порядок: Л-abad, Л-hip, Л-knee, П-abad, П-hip, П-knee.")
  print("Перед каждым: 3… 2… 1… СМОТРИ. Ctrl+C — стоп.\n")
  rc = _cmd_wiggle_can_leg_all(
      bus, 1, seconds, amplitude, hz, kp, kd,
      joints=("abad", "hip", "knee"), countdown=True,
  )
  if rc:
    return rc
  print("\n--- левая готова, дальше правая ---\n")
  time.sleep(1.0)
  return _cmd_wiggle_can_leg_all(
      bus, 2, seconds, amplitude, hz, kp, kd,
      joints=("abad", "hip", "knee"), countdown=True,
  )


def cmd_motor_position(
    bus: SpineBus,
    can_ch: int | None,
    motor: str | None,
    position: float | None,
    relative: bool,
    duration: float,
    hold: float,
    max_delta: float,
    tolerance: float,
    kp: float,
    kd: float,
    force: bool,
) -> int:
  """Плавно переводит один мотор передней ноги в заданное положение."""
  if can_ch is None:
    can_ch = int(input("CAN [1=левая, 2=правая]: ").strip())
  if can_ch not in (1, 2):
    print("Ошибка: доступны только CAN 1 (левая) и CAN 2 (правая).")
    return 2

  aliases = {"1": "abad", "2": "hip", "3": "knee"}
  if motor is None:
    motor = input("Мотор [1=abad, 2=hip, 3=knee]: ").strip().lower()
  motor = aliases.get(motor, motor)
  if motor not in ("abad", "hip", "knee"):
    print("Ошибка: мотор должен быть 1/abad, 2/hip или 3/knee.")
    return 2

  zero_offsets = [0.0] * len(ACTUATORS)
  for _ in range(20):
    bus.exchange(0, SpineCommand.enable_only())
    time.sleep(0.01)
  current, _, probe = _read_actuator_states(bus, zero_offsets)
  if not _board_ok(probe[0]):
    print(f"{probe[0].device}: CE0 не отвечает — движение отменено.")
    return 1

  joint_id = ("abad", "hip", "knee").index(motor) + 1
  actuator_i = next(
      i for i, a in enumerate(ACTUATORS)
      if a["can_ch"] == can_ch and a["can_id"] == joint_id
  )
  start = current[actuator_i]
  side = "левая" if can_ch == 1 else "правая"
  print(f"\n{side} / CAN{can_ch} / {motor}: сейчас {start:.4f} rad")
  if position is None:
    mode = input("Режим [r=смещение от текущего, a=абсолютный] (r): ").strip().lower()
    relative = mode != "a"
    position = float(
        input("Смещение, rad (+/-): " if relative else "Целевое положение, rad: ").strip()
    )
  if relative:
    position = start + position

  delta = position - start
  if abs(delta) > max_delta and not force:
    print(
        f"ОТМЕНА: прыжок {delta:+.3f} rad больше лимита ±{max_delta:.3f}.\n"
        "Задай более близкую позицию или явно добавь --force."
    )
    return 2
  if duration <= 0 or hold < 0:
    print("Ошибка: --duration должен быть >0, --hold >=0.")
    return 2

  flags = [0, 0, 0, 0]
  flags[can_ch - 1] = 1
  cmd_pos = list(current)
  cmd_vel = [0.0] * len(ACTUATORS)
  cmd_kp = [0.0] * len(ACTUATORS)
  cmd_kd = [0.0] * len(ACTUATORS)
  cmd_kp[actuator_i] = kp
  cmd_kd[actuator_i] = kd
  idle = SpineCommand()
  idle.flags = [0, 0]

  _countdown(f"{side} {motor}: {start:.3f} → {position:.3f} rad")
  print(
      f"Плавный ход {duration:.1f}s, удержание {hold:.1f}s, "
      f"kp={kp:.2f} kd={kd:.2f}. Ctrl+C — отпустить."
  )

  t0 = time.monotonic()
  lost = 0
  last_fb = start
  next_print = 0.0
  try:
    while True:
      elapsed = time.monotonic() - t0
      if elapsed < duration:
        x = elapsed / duration
        # Smoothstep: нулевая скорость в начале и конце.
        blend = x * x * (3.0 - 2.0 * x)
        cmd_pos[actuator_i] = start + delta * blend
        cmd_vel[actuator_i] = delta * (6.0 * x - 6.0 * x * x) / duration
      else:
        cmd_pos[actuator_i] = position
        cmd_vel[actuator_i] = 0.0
        if elapsed >= duration + hold:
          break

      c0 = _global_to_spine_cmd(
          0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      r = bus.exchange(0, c0)
      if _board_ok(r):
        lost = 0
        raw_pos: dict[tuple[int, str], float] = {}
        raw_vel: dict[tuple[int, str], float] = {}
        _spine_to_global_leg(
            r.data, can_ch - 1, can_ch - 1, raw_pos, raw_vel)
        last_fb = raw_pos[(can_ch - 1, motor)]
      else:
        lost += 1
        if lost >= 8:
          print("\nCE0 потерян (8 плохих кадров) — аварийный стоп.")
          bus.exchange(0, idle)
          return 1

      if elapsed >= next_print:
        print(
            f"\r  t={elapsed:4.1f}s  cmd={cmd_pos[actuator_i]:+.4f}  "
            f"fb={last_fb:+.4f}  {r.data.response_class}   ",
            end="", flush=True,
        )
        next_print += 0.5
      time.sleep(0.01)
  except KeyboardInterrupt:
    print("\nПрервано.")
  finally:
    for _ in range(5):
      bus.exchange(0, idle)
      time.sleep(0.01)

  error = position - last_fb
  if abs(error) <= tolerance:
    print(
        f"\nOK: target={position:+.4f}, fb={last_fb:+.4f}, "
        f"error={error:+.4f} rad ≤ {tolerance:.3f}. Мотор отпущен."
    )
    return 0
  print(
      f"\nFAIL: мотор не дошёл до цели. target={position:+.4f}, "
      f"fb={last_fb:+.4f}, error={error:+.4f} rad "
      f"(допуск ±{tolerance:.3f}). Мотор отпущен."
  )
  return 1


def cmd_motor_console(
    bus: SpineBus, speed: float, max_delta: float, kp: float, kd: float,
) -> int:
  """Интерактивный пульт одного мотора, который сохраняет удержание."""
  if speed <= 0 or max_delta <= 0:
    print("Ошибка: --speed и --max-delta должны быть больше нуля.")
    return 2

  print("=== motor-console: постоянное управление передними моторами ===")
  print("CAN1 = левая, CAN2 = правая. После target/delta выбранный мотор удерживает цель.")
  print("Команды:")
  print("  select CAN MOTOR      выбор: select 1 abad   (мотор: abad, hip, knee)")
  print("  target RAD            абсолютная цель: target -0.30")
  print("  delta RAD             сдвинуть от текущей цели: delta +0.10")
  print("  speed RAD_PER_SEC     скорость: speed 0.20")
  print("  status                текущая цель и обратная связь")
  print("  release               отпустить мотор")
  print("  quit                  отпустить и выйти")
  print(f"Лимит одной цели от обратной связи: ±{max_delta:.2f} rad.")
  print("motor> ", end="", flush=True)

  selected_can: int | None = None
  selected_motor: str | None = None
  actuator_i: int | None = None
  target: float | None = None
  last_fb: float | None = None
  command_pos: float | None = None
  last_tick = time.monotonic()
  last_status = 0.0
  lost = 0
  idle = SpineCommand()
  idle.flags = [0, 0]

  def release() -> None:
    nonlocal target, command_pos
    target = None
    command_pos = None
    for _ in range(5):
      bus.exchange(0, idle)
      time.sleep(0.01)

  def select_motor(can_ch: int, motor: str) -> None:
    nonlocal selected_can, selected_motor, actuator_i, target, command_pos, last_fb
    if can_ch not in (1, 2) or motor not in ("abad", "hip", "knee"):
      print("Ошибка: select 1|2 abad|hip|knee")
      return
    release()
    joint_id = ("abad", "hip", "knee").index(motor) + 1
    selected_can = can_ch
    selected_motor = motor
    actuator_i = next(
        i for i, a in enumerate(ACTUATORS)
        if a["can_ch"] == can_ch and a["can_id"] == joint_id
    )
    pos, _, probe = _read_actuator_states(bus, [0.0] * len(ACTUATORS))
    if not _board_ok(probe[0]):
      selected_can = selected_motor = actuator_i = None
      print("CE0 не отвечает; выбор отменён.")
      return
    last_fb = pos[actuator_i]
    side = "левая" if can_ch == 1 else "правая"
    homing_direction = ACTUATORS[actuator_i]["homing_velocity"]
    direction = "+" if homing_direction > 0 else "-"
    print(
        f"Выбрано: {side} CAN{can_ch} {motor}; fb={last_fb:+.4f} rad.\n"
        f"Для первого безопасного теста в направлении homing: delta {direction}0.15"
    )

  def set_target(value: float) -> None:
    nonlocal target, command_pos
    if actuator_i is None or last_fb is None:
      print("Сначала: select CAN MOTOR")
      return
    if abs(value - last_fb) > max_delta:
      print(
          f"ОТМЕНА: цель {value:+.3f} дальше чем ±{max_delta:.2f} rad "
          f"от fb={last_fb:+.3f}."
      )
      return
    target = value
    command_pos = last_fb
    print(f"Цель: {target:+.4f} rad; скорость {speed:.3f} rad/s.")

  try:
    while True:
      now = time.monotonic()
      dt = min(now - last_tick, 0.1)
      last_tick = now

      if target is not None and actuator_i is not None:
        assert selected_can is not None and selected_motor is not None
        assert command_pos is not None
        max_step = speed * dt
        command_pos += max(-max_step, min(max_step, target - command_pos))
        flags = [0, 0, 0, 0]
        flags[selected_can - 1] = 1
        cmd_pos = [0.0] * len(ACTUATORS)
        cmd_pos[actuator_i] = command_pos
        cmd_vel = [0.0] * len(ACTUATORS)
        remaining = target - command_pos
        if abs(remaining) > 0.001:
          # Homing задаёт ненулевую qd_des. Некоторые DAP не начинают
          # движение от одной меняющейся позиции, поэтому сохраняем
          # согласованную позицию и скорость.
          cmd_vel[actuator_i] = math.copysign(speed, remaining)
        cmd_kp = [0.0] * len(ACTUATORS)
        cmd_kd = [0.0] * len(ACTUATORS)
        cmd_kp[actuator_i] = kp
        cmd_kd[actuator_i] = kd
        response = bus.exchange(
            0, _global_to_spine_cmd(
                0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd,
                [0.0] * len(ACTUATORS),
            ),
        )
        if _board_ok(response):
          lost = 0
          raw_pos: dict[tuple[int, str], float] = {}
          raw_vel: dict[tuple[int, str], float] = {}
          _spine_to_global_leg(
              response.data, selected_can - 1, selected_can - 1, raw_pos, raw_vel)
          last_fb = raw_pos[(selected_can - 1, selected_motor)]
        else:
          lost += 1
          if lost >= 8:
            print("\nCE0 потерян — мотор отпущен.")
            release()
        if now - last_status >= 0.5:
          print(
              f"\r  cmd={command_pos:+.3f} target={target:+.3f} "
              f"fb={last_fb if last_fb is not None else float('nan'):+.3f}   motor> ",
              end="", flush=True,
          )
          last_status = now

      readable, _, _ = select.select([sys.stdin], [], [], 0.01)
      if not readable:
        continue
      line = sys.stdin.readline()
      if not line:
        break
      parts = line.strip().lower().split()
      if not parts:
        print("motor> ", end="", flush=True)
        continue
      command = parts[0]
      try:
        if command in ("quit", "exit", "q"):
          break
        if command in ("help", "h", "?"):
          print("select CAN MOTOR | target RAD | delta RAD | speed RAD/S | status | release | quit")
        elif command == "select" and len(parts) == 3:
          select_motor(int(parts[1]), parts[2])
        elif command == "target" and len(parts) == 2:
          set_target(float(parts[1]))
        elif command == "delta" and len(parts) == 2:
          if actuator_i is None or last_fb is None:
            print("Сначала: select CAN MOTOR")
          else:
            # Первая delta после select идёт от текущей обратной связи.
            set_target((target if target is not None else last_fb) + float(parts[1]))
        elif command == "speed" and len(parts) == 2:
          requested_speed = float(parts[1])
          if requested_speed <= 0:
            print("Скорость должна быть >0.")
          else:
            speed = requested_speed
            print(f"Скорость: {speed:.3f} rad/s.")
        elif command == "status":
          print(
              f"CAN={selected_can}, motor={selected_motor}, "
              f"target={target}, fb={last_fb}, speed={speed:.3f} rad/s"
          )
        elif command == "release":
          release()
          print("Мотор отпущен.")
        else:
          print("Не понял. help — список команд.")
      except ValueError:
        print("Ошибка числа. Пример: target -0.30")
      print("motor> ", end="", flush=True)
  except KeyboardInterrupt:
    print("\nПрервано.")
  finally:
    release()
  print("Все моторы отпущены.")
  return 0


def cmd_motor_probe(
    bus: SpineBus, can_ch: int, motor: str, amplitude: float, kp: float, kd: float,
) -> int:
  """Диагностика одного мотора: ±amplitude от текущей позиции, вердикт по fb."""
  if can_ch not in (1, 2) or motor not in ("abad", "hip", "knee"):
    print("Ошибка: --can 1|2, --motor abad|hip|knee")
    return 2
  if not (0.05 <= amplitude <= 0.8):
    print("Ошибка: --amplitude в пределах 0.05…0.8 rad.")
    return 2

  joint_id = ("abad", "hip", "knee").index(motor) + 1
  actuator_i = next(
      i for i, a in enumerate(ACTUATORS)
      if a["can_ch"] == can_ch and a["can_id"] == joint_id
  )
  side = "левая" if can_ch == 1 else "правая"
  n = len(ACTUATORS)
  zero_offsets = [0.0] * n
  flags = [0, 0, 0, 0]
  flags[can_ch - 1] = 1
  idle = SpineCommand()
  idle.flags = [0, 0]

  for _ in range(15):
    bus.exchange(0, SpineCommand.enable_only())
    time.sleep(0.01)
  pos, _, probe = _read_actuator_states(bus, zero_offsets)
  if not _board_ok(probe[0]):
    print(f"{probe[0].device}: CE0 не отвечает — тест отменён.")
    return 1

  fb0 = pos[actuator_i]
  print(f"=== motor-probe: {side} CAN{can_ch} {motor} ===")
  print(f"База fb={fb0:+.4f} rad; ход ±{amplitude:.2f} rad, kp={kp}, kd={kd}.")
  if abs(fb0) < 1e-4:
    print("ВНИМАНИЕ: fb≈0 — возможно, мотор вообще не отвечает по CAN (ID/проводка).")
  print("Нога должна висеть свободно. Ctrl+C — отпустить.\n")

  cmd_pos = list(pos)
  cmd_vel = [0.0] * n
  cmd_kp = [0.0] * n
  cmd_kd = [0.0] * n
  cmd_kp[actuator_i] = kp
  cmd_kd[actuator_i] = kd

  def read_fb(r: BoardResult) -> float:
    raw_pos: dict[tuple[int, str], float] = {}
    raw_vel: dict[tuple[int, str], float] = {}
    _spine_to_global_leg(r.data, can_ch - 1, can_ch - 1, raw_pos, raw_vel)
    return raw_pos[(can_ch - 1, motor)]

  def run_phase(target: float, seconds: float) -> float:
    """Плавно к target и удержание; вернуть смещение fb от базы со знаком цели."""
    start = cmd_pos[actuator_i]
    direction = 1.0 if target >= start else -1.0
    t0 = time.monotonic()
    fb = fb0
    best = 0.0
    while time.monotonic() - t0 < seconds:
      x = min(1.0, (time.monotonic() - t0) / (seconds * 0.4))
      blend = x * x * (3.0 - 2.0 * x)
      cmd_pos[actuator_i] = start + (target - start) * blend
      c0 = _global_to_spine_cmd(
          0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
      r = bus.exchange(0, c0)
      if _board_ok(r):
        fb = read_fb(r)
        best = max(best, (fb - fb0) * direction)
      time.sleep(0.01)
    return best * direction

  try:
    dev_plus = run_phase(fb0 + amplitude, 1.5)
    dev_minus = run_phase(fb0 - amplitude, 1.5)
    run_phase(fb0, 1.0)
  except KeyboardInterrupt:
    print("\nПрервано.")
  finally:
    for _ in range(5):
      bus.exchange(0, idle)
      time.sleep(0.01)

  print(f"\nРезультат: база fb={fb0:+.4f} rad")
  print(f"  цель +{amplitude:.2f}: fb сместился на {dev_plus:+.4f} rad")
  print(f"  цель -{amplitude:.2f}: fb сместился на {dev_minus:+.4f} rad")
  ok_plus = dev_plus > 0.5 * amplitude
  ok_minus = dev_minus < -0.5 * amplitude
  if ok_plus and ok_minus:
    print("ВЕРДИКТ: момент есть в обе стороны — CAN и драйвер в порядке.")
    print("Если при этом нога физически НЕ двигается — механика: муфта/шестерни/вал.")
    return 0
  if abs(dev_plus) < 0.05 and abs(dev_minus) < 0.05:
    print("ВЕРДИКТ: fb почти не двигается — мотор НЕ создаёт момент.")
    print("Проверь: силовое питание именно этого мотора, fault на драйвере,")
    print("CAN ID мотора (abad должен быть ID 1), разъём на CAN2.")
    return 1
  print("ВЕРДИКТ: момент только в одну сторону — похоже на драйвер (полумост)")
  print("или механический упор с одной стороны.")
  return 1


def cmd_motor_scan(bus: SpineBus, amplitude: float, kp: float, kd: float) -> int:
  """По очереди все CAN id 1/2/3 на CAN1 и CAN2: ±amplitude, сводка по fb."""
  if not (0.05 <= amplitude <= 0.8):
    print("Ошибка: --amplitude в пределах 0.05…0.8 rad.")
    return 2

  slots = (
      (1, "abad"),
      (1, "hip"),
      (1, "knee"),
      (2, "abad"),
      (2, "hip"),
      (2, "knee"),
  )
  print("=== motor-scan: перебор 6 слотов CE0 (CAN1/CAN2 × id 1/2/3) ===\n")
  print(f"Каждый: от текущей позиции +{amplitude:.2f} потом -{amplitude:.2f} и назад.")
  print("Имена abad/hip/knee — слот в софте, не обязательно тот сустав на ноге.")
  print("Ноги пусть висят. Ctrl+C — стоп и отпустить.\n")

  n = len(ACTUATORS)
  zero_offsets = [0.0] * n
  idle = SpineCommand()
  idle.flags = [0, 0]
  rows: list[tuple[str, str, float, float, float, float, str]] = []
  interrupted = False

  def read_slot_fb(r: BoardResult, can_ch: int, motor: str) -> float:
    raw_pos: dict[tuple[int, str], float] = {}
    raw_vel: dict[tuple[int, str], float] = {}
    _spine_to_global_leg(r.data, can_ch - 1, can_ch - 1, raw_pos, raw_vel)
    return raw_pos[(can_ch - 1, motor)]

  try:
    for can_ch, motor in slots:
      joint_id = ("abad", "hip", "knee").index(motor) + 1
      actuator_i = next(
          i for i, a in enumerate(ACTUATORS)
          if a["can_ch"] == can_ch and a["can_id"] == joint_id
      )
      side = "левая" if can_ch == 1 else "правая"
      flags = [0, 0, 0, 0]
      flags[can_ch - 1] = 1
      warm = SpineCommand()
      warm.flags = [flags[0], flags[1]]
      for _ in range(15):
        bus.exchange(0, warm)
        time.sleep(0.01)
      pos, _, probe = _read_actuator_states(bus, zero_offsets)
      if not _board_ok(probe[0]):
        print(f"{probe[0].device}: CE0 пропал — scan оборван.")
        return 1

      fb0 = pos[actuator_i]
      print(f"--- {side} CAN{can_ch} id={joint_id} ({motor})  fb0={fb0:+.4f} ---")
      cmd_pos = list(pos)
      cmd_vel = [0.0] * n
      cmd_kp = [0.0] * n
      cmd_kd = [0.0] * n
      cmd_kp[actuator_i] = kp
      cmd_kd[actuator_i] = kd

      def run_phase(target: float, seconds: float) -> tuple[float, float]:
        start = cmd_pos[actuator_i]
        t0 = time.monotonic()
        fb = fb0
        lo = fb
        hi = fb
        while time.monotonic() - t0 < seconds:
          x = min(1.0, (time.monotonic() - t0) / (seconds * 0.4))
          blend = x * x * (3.0 - 2.0 * x)
          cmd_pos[actuator_i] = start + (target - start) * blend
          c0 = _global_to_spine_cmd(
              0, flags, cmd_pos, cmd_vel, cmd_kp, cmd_kd, zero_offsets)
          r = bus.exchange(0, c0)
          if _board_ok(r):
            fb = read_slot_fb(r, can_ch, motor)
            lo = min(lo, fb)
            hi = max(hi, fb)
          time.sleep(0.01)
        return fb - fb0, hi - lo

      dev_plus, _ = run_phase(fb0 + amplitude, 1.8)
      dev_minus, _ = run_phase(fb0 - amplitude, 1.8)
      _, span = run_phase(fb0, 1.0)
      span = max(span, abs(dev_plus), abs(dev_minus))
      bus.exchange(0, idle)

      no_fb = abs(fb0) < 1e-4 and span < 0.04
      both = dev_plus > 0.5 * amplitude and dev_minus < -0.5 * amplitude
      some = span >= 0.05
      if no_fb:
        verdict = "НЕТ (fb=0)"
      elif both:
        verdict = "ЕДЕТ"
      elif some:
        verdict = "слабо / одна сторона"
      else:
        verdict = "НЕТ"
      print(
          f"    +{amplitude:.2f}: {dev_plus:+.3f}   "
          f"-{amplitude:.2f}: {dev_minus:+.3f}   span={span:.3f}  {verdict}\n"
      )
      rows.append(
          (side, f"id{joint_id}/{motor}", fb0, dev_plus, dev_minus, span, verdict)
      )
  except KeyboardInterrupt:
    interrupted = True
    print("\nПрервано.")
  finally:
    for _ in range(5):
      bus.exchange(0, idle)
      time.sleep(0.01)

  print("=== сводка ===")
  print(f"{'нога':<8} {'слот':<14} {'fb0':>8} {'span':>8}  вердикт")
  for side, slot, fb0, _p, _m, span, verdict in rows:
    print(f"{side:<8} {slot:<14} {fb0:+8.3f} {span:8.3f}  {verdict}")
  if not rows:
    print("(ни одного слота не прогнали)")
    return 1
  moving = sum(1 for r in rows if r[6] == "ЕДЕТ")
  print(f"\nЕдут {moving}/{len(rows)}. Имя слота — софт, смотри какой сустав качнулся.")
  return 1 if interrupted else 0


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
  p.add_argument(
      "--board0-only",
      action="store_true",
      help="Только CE0 (вторая STM выпаяна): не открывать/не тактировать CE1",
  )
  sub = p.add_subparsers(dest="command", required=True)

  sub.add_parser("selftest", help="Проверка кодирования без SPI")

  s = sub.add_parser("probe", help="Один обмен на CE0 (и CE1, если не --board0-only)")
  s.add_argument("-v", "--verbose", action="store_true")

  s = sub.add_parser("watch", help="Непрерывный мониторинг")
  s.add_argument("--interval", type=float, default=0.05)
  s.add_argument("--count", type=int, default=0, help="0 = бесконечно")
  s.add_argument("-v", "--verbose", action="store_true")

  s = sub.add_parser("enable", help="10× enable flags (как activate)")
  s.add_argument("--repeat", type=int, default=10)

  s = sub.add_parser("read", help="Прочитать положения")
  s.add_argument("--repeat", type=int, default=3)

  s = sub.add_parser(
      "homing",
      help="Отказ: это не homing робота (раньше срывало моторы к q=0)",
  )
  s.add_argument("--seconds", type=float, default=3.0)
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.2)
  s.add_argument("--velocity", type=float, default=0.3, help="qd_des_knee rad/s")

  s = sub.add_parser(
      "homing-full",
      help="Homing как ROS do_homing, только передние ноги (CE0)",
  )
  s.add_argument("--timeout", type=float, default=180.0, help="секунд")
  s.add_argument("--dt-ms", type=int, default=10, help="шаг цикла, мс")

  s = sub.add_parser("hold-front-left",
                     help="Удержать только переднюю левую ногу (рабочую)")
  s.add_argument("--seconds", type=float, default=15.0)
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)

  s = sub.add_parser(
      "hold",
      help="Зафиксировать текущую позицию всех моторов с живым энкодером, без хоуминга",
  )
  s.add_argument("--seconds", type=float, default=120.0)
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)
  s.add_argument(
      "--all",
      action="store_true",
      help="Зажать все 12 моторов, включая нулевой энкодер (цель станет 0 рад)",
  )

  s = sub.add_parser("wiggle-front-left", help="Качать колено передней левой туда-сюда")
  s.add_argument("--seconds", type=float, default=15.0)
  s.add_argument("--amplitude", type=float, default=0.12, help="рад от центра")
  s.add_argument("--hz", type=float, default=0.4, help="частота Гц")
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)

  for can_ch in (1, 2):
    s = sub.add_parser(
        f"wiggle-can{can_ch}-all",
        help=f"По очереди качнуть все 3 мотора CAN{can_ch} (abad→hip→knee)",
    )
    s.add_argument("--seconds", type=float, default=4.0, help="секунд на каждый сустав")
    s.add_argument("--amplitude", type=float, default=0.10, help="рад от центра")
    s.add_argument("--hz", type=float, default=0.4, help="частота Гц")
    s.add_argument("--kp", type=float, default=5.5)
    s.add_argument("--kd", type=float, default=0.5)
    s.add_argument(
        "--joint",
        choices=("abad", "hip", "knee", "all"),
        default="all",
        help="один сустав или all",
    )
    s.add_argument(
        "--countdown",
        action="store_true",
        help="перед движением: 3… 2… 1… СМОТРИ",
    )

  s = sub.add_parser(
      "wiggle-6",
      help="6 моторов туда-сюда: левая abad/hip/knee, потом правая (с 3…2…1)",
  )
  s.add_argument("--seconds", type=float, default=5.0, help="секунд на каждый мотор")
  s.add_argument("--amplitude", type=float, default=0.15, help="рад от центра")
  s.add_argument("--hz", type=float, default=0.35, help="частота Гц")
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)

  s = sub.add_parser(
      "motor-position",
      help="Выбрать один мотор и плавно дать ему положение (rad)",
  )
  s.add_argument("--can", dest="can_ch", type=int, choices=(1, 2))
  s.add_argument(
      "--motor",
      choices=("1", "2", "3", "abad", "hip", "knee"),
      help="1=abad, 2=hip, 3=knee",
  )
  s.add_argument("--position", type=float, help="абсолютная позиция, rad")
  s.add_argument(
      "--relative", action="store_true",
      help="трактовать --position как смещение от текущего положения, rad",
  )
  s.add_argument("--duration", type=float, default=2.0, help="время плавного хода, s")
  s.add_argument("--hold", type=float, default=1.0, help="удерживать цель, s")
  s.add_argument(
      "--max-delta", type=float, default=1.5,
      help="максимальный безопасный сдвиг от текущей позиции, rad",
  )
  s.add_argument(
      "--tolerance", type=float, default=0.08,
      help="допустимая ошибка позиции для OK, rad",
  )
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)
  s.add_argument(
      "--force", action="store_true",
      help="разрешить сдвиг больше --max-delta (опасно)",
  )

  s = sub.add_parser(
      "motor-console",
      help="Постоянная интерактивная консоль: выбрать мотор, цель и скорость",
  )
  s.add_argument(
      "--speed", dest="motor_speed", type=float, default=0.20,
      help="начальная скорость мотора, rad/s",
  )
  s.add_argument("--max-delta", type=float, default=1.5, help="лимит цели от fb, rad")
  s.add_argument("--kp", type=float, default=10.0)
  s.add_argument("--kd", type=float, default=0.5)

  s = sub.add_parser(
      "motor-probe",
      help="Диагностика одного мотора: ±амплитуда от текущей позиции, вердикт по fb",
  )
  s.add_argument("--can", dest="can_ch", type=int, choices=(1, 2), required=True)
  s.add_argument("--motor", choices=("abad", "hip", "knee"), required=True)
  s.add_argument("--amplitude", type=float, default=0.25, help="рад от текущей позиции")
  s.add_argument("--kp", type=float, default=8.0)
  s.add_argument("--kd", type=float, default=0.3)

  s = sub.add_parser(
      "motor-scan",
      help="Перебор всех 6 слотов CE0: ±амплитуда, кто едет по fb",
  )
  s.add_argument("--amplitude", type=float, default=0.5, help="рад туда-сюда от текущей позиции")
  s.add_argument("--kp", type=float, default=5.5)
  s.add_argument("--kd", type=float, default=0.5)

  return p


def main() -> int:
  args = build_parser().parse_args()
  # Одна STM: hold/wiggle/homing-full всегда только CE0.
  board0_only = bool(args.board0_only) or args.command in (
      "hold-front-left",
      "wiggle-front-left",
      "wiggle-can1-all",
      "wiggle-can2-all",
      "wiggle-6",
      "motor-position",
      "motor-console",
      "motor-probe",
      "motor-scan",
      "homing-full",
  )
  bus = SpineBus(speed_hz=args.speed, board0_only=board0_only)
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
      return cmd_homing_full(bus, args.timeout, args.dt_ms, board0_only=True)
    if args.command == "hold-front-left":
      return cmd_hold_front_left(bus, args.seconds, args.kp, args.kd)
    if args.command == "hold":
      return cmd_hold(bus, args.seconds, args.kp, args.kd, args.all)
    if args.command == "wiggle-front-left":
      return cmd_wiggle_front_left(
          bus, args.seconds, args.amplitude, args.hz, args.kp, args.kd)
    if args.command in ("wiggle-can1-all", "wiggle-can2-all"):
      joints = ("abad", "hip", "knee") if args.joint == "all" else (args.joint,)
      fn = cmd_wiggle_can1_all if args.command == "wiggle-can1-all" else cmd_wiggle_can2_all
      return fn(
          bus, args.seconds, args.amplitude, args.hz, args.kp, args.kd,
          joints=joints, countdown=args.countdown)
    if args.command == "wiggle-6":
      return cmd_wiggle_6(
          bus, args.seconds, args.amplitude, args.hz, args.kp, args.kd)
    if args.command == "motor-position":
      return cmd_motor_position(
          bus, args.can_ch, args.motor, args.position, args.relative,
          args.duration, args.hold, args.max_delta, args.tolerance,
          args.kp, args.kd, args.force,
      )
    if args.command == "motor-console":
      return cmd_motor_console(
          bus, args.motor_speed, args.max_delta, args.kp, args.kd,
      )
    if args.command == "motor-probe":
      return cmd_motor_probe(
          bus, args.can_ch, args.motor, args.amplitude, args.kp, args.kd,
      )
    if args.command == "motor-scan":
      return cmd_motor_scan(bus, args.amplitude, args.kp, args.kd)
    return 2
  finally:
    bus.close()


if __name__ == "__main__":
  sys.exit(main())
