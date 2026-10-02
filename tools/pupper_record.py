#!/usr/bin/env python3
"""Запись реальных данных Pupper для идентификации модели и дообучения политики.

  python3 pupper_record.py record --note "генератор, шаг 2 см, пол ламинат" [--duration 60]
  python3 pupper_record.py export ~/pupper_data/20260928_190000_note   # -> dataset.npz
  python3 pupper_record.py info   ~/pupper_data/20260928_190000_note

Сессия — каталог ~/pupper_data/<время>_<метка>/:
  bag/       — ros2 bag (mcap) без потерь: суставы и IMU ~250 Гц, все команды, kp/kd, cmd_vel, URDF;
  meta.json  — заметка, git-коммит, motor_map, параметры режима (генератор/политика) от пульта.
Экспорт — dataset.npz: сырые ряды каждого топика + сетка 50 Гц (как environment_dt в ai/rl).
Инструкция по обучению: ai/rl/REAL_DATA.md.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from open_loop_gait import JOINTS

REPO = Path(__file__).resolve().parent.parent
DATA_DIR = Path.home() / "pupper_data"
MIN_FREE_BYTES = 2 * 1024**3
GRID_DT = 0.02

TOPICS = {
    "/joint_states": "sensor_msgs/msg/JointState",
    "/imu_sensor_broadcaster/imu": "sensor_msgs/msg/Imu",
    "/forward_position_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/forward_kp_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/forward_kd_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/neural_controller/position_command": "std_msgs/msg/Float32MultiArray",
    "/neural_controller/observation": "std_msgs/msg/Float32MultiArray",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/emergency_stop": "std_msgs/msg/Empty",
    "/robot_description": "std_msgs/msg/String",
}


def _git(*args: str) -> str:
    return subprocess.run(["git", "-C", str(REPO), *args], check=True, capture_output=True, text=True).stdout.strip()


class Recorder:
    """ros2 bag record в дочернем процессе. Используется из CLI и из пульта."""

    def __init__(self) -> None:
        self.proc: subprocess.Popen | None = None
        self.session: Path | None = None
        self.meta: dict | None = None

    @property
    def active(self) -> bool:
        return self.proc is not None

    def start(self, note: str, extra: dict | None = None) -> Path:
        if self.proc is not None:
            raise RuntimeError(f"запись уже идёт: {self.session}")
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(DATA_DIR).free
        if free < MIN_FREE_BYTES:
            raise RuntimeError(f"мало места: {free / 1024**3:.1f} ГБ свободно в {DATA_DIR}")
        slug = re.sub(r"[^\w-]+", "_", note.strip(), flags=re.UNICODE)[:40].strip("_") or "rec"
        session = DATA_DIR / f"{datetime.now():%Y%m%d_%H%M%S}_{slug}"
        session.mkdir()
        self.meta = {
            "note": note,
            "start_unix": time.time(),
            "git_commit": _git("rev-parse", "HEAD"),
            "git_dirty": _git("status", "--porcelain", "--untracked-files=no").splitlines(),
            "motor_map_yaml": (REPO / "robot/config/motor_map.yaml").read_text(),
            "joints": JOINTS,
            "topics": TOPICS,
            "extra": extra or {},
            "events": [],
        }
        self._write_meta(session)
        self.proc = subprocess.Popen(
            ["ros2", "bag", "record", "-s", "mcap", "-o", str(session / "bag"), "--topics", *TOPICS],
            stdout=open(session / "record.log", "w"), stderr=subprocess.STDOUT, start_new_session=True,
        )
        self.session = session
        return session

    def log_event(self, event: dict) -> None:
        """Изменение режима во время записи; t — секунды от старта записи."""
        self.meta["events"].append({"t": time.time() - self.meta["start_unix"], **event})

    def stop(self) -> Path:
        if self.proc is None:
            raise RuntimeError("запись не идёт")
        self.proc.send_signal(signal.SIGINT)
        code = self.proc.wait(timeout=15)
        session, self.proc, self.session = self.session, None, None
        self.meta["stop_unix"] = time.time()
        self.meta["duration_s"] = self.meta["stop_unix"] - self.meta["start_unix"]
        self.meta["ros2_bag_exit_code"] = code
        self._write_meta(session)
        if code != 0:
            raise RuntimeError(f"ros2 bag завершился с кодом {code}, см. {session / 'record.log'}")
        return session

    def _write_meta(self, session: Path) -> None:
        (session / "meta.json").write_text(json.dumps(self.meta, ensure_ascii=False, indent=2))


def read_bag(session: Path) -> dict[str, list[tuple[float, object]]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session / "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out: dict[str, list[tuple[float, object]]] = {t: [] for t in TOPICS}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(data, types[topic])))
    return out


def _series(msgs: list, fn) -> tuple[np.ndarray, np.ndarray]:
    if not msgs:
        return np.zeros(0), np.zeros((0, 0))
    return np.array([t for t, _ in msgs]), np.array([fn(m) for _, m in msgs], dtype=np.float64)


def _hold(t_src: np.ndarray, v_src: np.ndarray, t_grid: np.ndarray, width: int) -> tuple[np.ndarray, np.ndarray]:
    """Последнее значение не позже t (как видит контроллер) и его возраст; NaN до первого сообщения."""
    val = np.full((len(t_grid), width), np.nan)
    age = np.full(len(t_grid), np.inf)
    if len(t_src) == 0:
        return val, age
    i = np.searchsorted(t_src, t_grid, side="right") - 1
    ok = i >= 0
    val[ok] = v_src[i[ok]]
    age[ok] = t_grid[ok] - t_src[i[ok]]
    return val, age


def export(session: Path) -> Path:
    raw = read_bag(session)
    js = raw["/joint_states"]
    if not js:
        raise RuntimeError("в записи нет /joint_states — robot.service не работал?")
    idx = {n: i for i, n in enumerate(js[0][1].name)}
    order = [idx[j] for j in JOINTS]
    d: dict[str, np.ndarray] = {}
    d["js_t"], d["js_q"] = _series(js, lambda m: [m.position[i] for i in order])
    d["js_dq"] = _series(js, lambda m: [m.velocity[i] for i in order])[1]
    d["js_tau"] = _series(js, lambda m: [m.effort[i] for i in order])[1]
    imu = raw["/imu_sensor_broadcaster/imu"]
    d["imu_t"], d["imu_quat_xyzw"] = _series(imu, lambda m: [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w])
    d["imu_gyro"] = _series(imu, lambda m: [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])[1]
    d["imu_acc"] = _series(imu, lambda m: [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])[1]
    arr = lambda m: list(m.data)
    d["fwd_t"], d["fwd_q"] = _series(raw["/forward_position_controller/commands"], arr)
    d["kp_t"], d["kp"] = _series(raw["/forward_kp_controller/commands"], arr)
    d["kd_t"], d["kd"] = _series(raw["/forward_kd_controller/commands"], arr)
    d["nn_t"], d["nn_q"] = _series(raw["/neural_controller/position_command"], arr)
    d["obs_t"], d["obs"] = _series(raw["/neural_controller/observation"], arr)
    d["cmdvel_t"], d["cmdvel"] = _series(raw["/cmd_vel"], lambda m: [m.linear.x, m.linear.y, m.angular.z])
    d["estop_t"] = np.array([t for t, _ in raw["/emergency_stop"]])

    t0 = d["js_t"][0]
    grid = np.arange(t0, d["js_t"][-1], GRID_DT)
    d["t"] = grid - t0
    for key, src in [("q", "js_q"), ("dq", "js_dq"), ("tau", "js_tau")]:
        d[key] = _hold(d["js_t"], d[src], grid, 12)[0]
    d["quat_xyzw"], d["imu_age"] = _hold(d["imu_t"], d["imu_quat_xyzw"], grid, 4)
    d["gyro"] = _hold(d["imu_t"], d["imu_gyro"], grid, 3)[0]
    # Активный источник команды — тот, чьё сообщение свежее: forward-контроллеры (генератор/стойка) или политика.
    fwd, fwd_age = _hold(d["fwd_t"], d["fwd_q"], grid, 12)
    nn, nn_age = _hold(d["nn_t"], d["nn_q"], grid, 12)
    use_nn = nn_age < fwd_age
    d["cmd_q"] = np.where(use_nn[:, None], nn, fwd)
    d["cmd_src"] = np.where(use_nn, 1, np.where(np.isfinite(fwd_age), 0, -1)).astype(np.int8)
    d["cmd_age"] = np.minimum(fwd_age, nn_age)
    d["kp_cmd"] = _hold(d["kp_t"], d["kp"], grid, 12)[0]
    d["kd_cmd"] = _hold(d["kd_t"], d["kd"], grid, 12)[0]
    d["cmd_vel"] = _hold(d["cmdvel_t"], d["cmdvel"], grid, 3)[0]
    after_estop = np.zeros(len(grid), bool)
    if len(d["estop_t"]):
        after_estop = grid >= d["estop_t"][0]
    d["valid"] = (d["imu_age"] < 0.05) & (d["cmd_age"] < 0.1) & ~after_estop
    for k in [k for k in d if k.endswith("_t")]:
        d[k] = d[k] - t0

    urdf = raw["/robot_description"]
    out = session / "dataset.npz"
    np.savez_compressed(out, joints=np.array(JOINTS), urdf=np.array(urdf[-1][1].data if urdf else ""),
                        meta=np.array((session / "meta.json").read_text()), **d)
    return out


def info(session: Path) -> None:
    meta = json.loads((session / "meta.json").read_text())
    print(f"{session.name}: {meta['note']!r}, {meta.get('duration_s', float('nan')):.1f} с, git {meta['git_commit'][:8]}")
    if meta["extra"]:
        print("режим:", json.dumps(meta["extra"], ensure_ascii=False))
    raw = read_bag(session)
    for topic, msgs in raw.items():
        if len(msgs) > 1:
            t = np.array([x for x, _ in msgs])
            gap = np.diff(t).max()
            print(f"  {topic:42s} {len(msgs):7d} сообщ. {len(msgs) / (t[-1] - t[0]):7.1f} Гц, макс. пропуск {gap * 1000:.0f} мс")
        else:
            print(f"  {topic:42s} {len(msgs):7d} сообщ.")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--note", required=True, help="что делаем: режим, скорость, покрытие, подставка/пол")
    r.add_argument("--duration", type=float, default=0.0, help="секунд, 0 = до Ctrl+C")
    for name in ("export", "info"):
        sub.add_parser(name).add_argument("session", type=Path)
    a = p.parse_args()

    if a.cmd == "record":
        rec = Recorder()
        session = rec.start(a.note, {"source": "cli"})
        print("пишу в", session, "— Ctrl+C для остановки")
        t_end = time.monotonic() + a.duration if a.duration > 0 else float("inf")
        try:
            while time.monotonic() < t_end and rec.proc.poll() is None:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        print("готово:", rec.stop())
    elif a.cmd == "export":
        print("готово:", export(a.session.expanduser()))
    else:
        info(a.session.expanduser())


if __name__ == "__main__":
    main()
