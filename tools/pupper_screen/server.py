#!/usr/bin/env python3
"""Веб-экран Pupper: загрузка фото/видео/YouTube и показ на HDMI.

Запуск как сервис:
  ./install_service.sh
  # или вручную
  ./run.sh

Открой http://<ip-pi>:8090 с телефона/ноутбука.
На экране робота Chromium откроет /view в fullscreen.
"""

from __future__ import annotations

import cgi
import json
import mimetypes
import os
import re
import signal
import subprocess
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

HOST = "0.0.0.0"
PORT = 8090
ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
MEDIA = ROOT / "media"
LIBRARY = MEDIA / "library.json"
CHROMIUM_PROFILE = Path("/tmp/pupper-screen-chromium")
VIEW_URL = f"http://127.0.0.1:{PORT}/view"

IMAGE_EXT = {".jpg", ".jpeg", ".png", ".gif", ".webp", ".bmp"}
VIDEO_EXT = {".mp4", ".webm", ".mov", ".mkv", ".m4v", ".ogv"}
YT_RE = re.compile(
    r"(?:youtube\.com/watch\?v=|youtu\.be/|youtube\.com/embed/|"
    r"youtube\.com/shorts/|youtube\.com/live/)([A-Za-z0-9_-]{11})"
)

_chromium_proc: subprocess.Popen | None = None


def _load_library() -> dict:
    if not LIBRARY.exists():
        return {"items": [], "current": None}
    return json.loads(LIBRARY.read_text(encoding="utf-8"))


def _save_library(data: dict) -> None:
    MEDIA.mkdir(parents=True, exist_ok=True)
    LIBRARY.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _youtube_id(url: str) -> str | None:
    m = YT_RE.search(url.strip())
    return m.group(1) if m else None


def _safe_name(name: str) -> str:
    base = os.path.basename(name).replace(" ", "_")
    base = re.sub(r"[^A-Za-z0-9._\-а-яА-ЯёЁ]", "", base)
    return base or f"file_{uuid.uuid4().hex[:8]}"


def _find_item(data: dict, item_id: str) -> dict | None:
    for item in data["items"]:
        if item["id"] == item_id:
            return item
    return None


def _chromium_env() -> dict[str, str]:
    env = os.environ.copy()
    env["DISPLAY"] = ":0"
    env["XAUTHORITY"] = str(Path.home() / ".Xauthority")
    runtime = f"/run/user/{os.getuid()}"
    env["XDG_RUNTIME_DIR"] = runtime
    wayland = Path(runtime) / "wayland-0"
    if wayland.exists():
        env["WAYLAND_DISPLAY"] = "wayland-0"
    else:
        env.pop("WAYLAND_DISPLAY", None)
    return env


def _chromium_pids() -> list[str]:
    # Без ведущих "--": иначе pgrep воспринимает pattern как свои флаги.
    result = subprocess.run(
        ["pgrep", "-f", f"user-data-dir={CHROMIUM_PROFILE}"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return []
    return [p for p in result.stdout.split() if p]


def _chromium_running() -> bool:
    global _chromium_proc
    if _chromium_proc is not None and _chromium_proc.poll() is None:
        return True
    return bool(_chromium_pids())


def _youtube_watch_url(youtube_id: str) -> str:
    # Top-level YouTube page — iframe с localhost даёт "Video unavailable".
    return f"https://www.youtube.com/watch?v={youtube_id}"


def _display_url(item: dict | None) -> str:
    if item and item.get("type") == "youtube" and item.get("youtube_id"):
        return _youtube_watch_url(item["youtube_id"])
    return VIEW_URL


def start_chromium(url: str = VIEW_URL, *, force: bool = False) -> None:
    global _chromium_proc
    if _chromium_running():
        if not force:
            return
        stop_chromium()
        time.sleep(0.6)
    CHROMIUM_PROFILE.mkdir(parents=True, exist_ok=True)
    browser = "chromium-browser" if Path("/usr/bin/chromium-browser").exists() else "chromium"
    cmd = [
        browser,
        f"--user-data-dir={CHROMIUM_PROFILE}",
        "--kiosk",
        "--start-fullscreen",
        "--noerrdialogs",
        "--disable-infobars",
        "--disable-session-crashed-bubble",
        "--check-for-update-interval=31536000",
        "--autoplay-policy=no-user-gesture-required",
        url,
    ]
    log_path = Path("/tmp/pupper-screen-chromium.log")
    log_f = log_path.open("w", encoding="utf-8")
    _chromium_proc = subprocess.Popen(
        cmd,
        env=_chromium_env(),
        stdout=log_f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(1.0)
    if _chromium_proc.poll() is not None:
        log_f.close()
        log_tail = log_path.read_text(encoding="utf-8")[-800:]
        raise RuntimeError(
            f"Chromium сразу завершился (код {_chromium_proc.returncode}). "
            f"Лог: {log_path}\n{log_tail}"
        )


def stop_chromium() -> None:
    global _chromium_proc
    if _chromium_proc is not None and _chromium_proc.poll() is None:
        os.killpg(os.getpgid(_chromium_proc.pid), signal.SIGTERM)
        _chromium_proc.wait(timeout=5)
        _chromium_proc = None
    for pid in _chromium_pids():
        os.kill(int(pid), signal.SIGTERM)
    time.sleep(0.3)


def _json_response(handler: BaseHTTPRequestHandler, code: int, payload: dict) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def _read_json(handler: BaseHTTPRequestHandler) -> dict:
    length = int(handler.headers.get("Content-Length", "0"))
    raw = handler.rfile.read(length) if length else b"{}"
    return json.loads(raw.decode("utf-8"))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        print(f"[pupper-screen] {self.address_string()} {fmt % args}")

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self._file(STATIC / "control.html", "text/html; charset=utf-8")
            return
        if path == "/view":
            self._file(STATIC / "view.html", "text/html; charset=utf-8")
            return
        if path == "/api/media":
            data = _load_library()
            _json_response(self, 200, data)
            return
        if path == "/api/current":
            data = _load_library()
            item = _find_item(data, data["current"]) if data.get("current") else None
            _json_response(
                self,
                200,
                {
                    "current": data.get("current"),
                    "item": item,
                    "chromium": _chromium_running(),
                },
            )
            return
        if path == "/api/status":
            _json_response(self, 200, {"chromium": _chromium_running(), "port": PORT})
            return
        if path.startswith("/media/"):
            name = unquote(path[len("/media/") :])
            if "/" in name or ".." in name:
                _json_response(self, 400, {"error": "неверное имя файла"})
                return
            fpath = MEDIA / name
            if not fpath.is_file():
                _json_response(self, 404, {"error": "файл не найден"})
                return
            ctype = mimetypes.guess_type(str(fpath))[0] or "application/octet-stream"
            self._file(fpath, ctype)
            return
        _json_response(self, 404, {"error": "not found"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/upload":
            self._upload()
            return
        if path == "/api/youtube":
            body = _read_json(self)
            url = (body.get("url") or "").strip()
            vid = _youtube_id(url)
            if not vid:
                _json_response(self, 400, {"error": "не распознал YouTube-ссылку"})
                return
            data = _load_library()
            item = {
                "id": uuid.uuid4().hex,
                "type": "youtube",
                "name": body.get("name") or f"YouTube {vid}",
                "youtube_id": vid,
                "url": url,
                "added": time.time(),
            }
            data["items"].insert(0, item)
            _save_library(data)
            _json_response(self, 200, {"item": item})
            return
        if path == "/api/show":
            body = _read_json(self)
            item_id = body.get("id")
            data = _load_library()
            item = _find_item(data, item_id) if item_id else None
            if item is None:
                _json_response(self, 404, {"error": "элемент не найден"})
                return
            data["current"] = item_id
            _save_library(data)
            start_chromium(_display_url(item), force=True)
            _json_response(self, 200, {"ok": True, "item": item, "chromium": True})
            return
        if path == "/api/stop":
            data = _load_library()
            data["current"] = None
            _save_library(data)
            stop_chromium()
            _json_response(self, 200, {"ok": True})
            return
        if path == "/api/open_screen":
            data = _load_library()
            item = _find_item(data, data["current"]) if data.get("current") else None
            start_chromium(_display_url(item), force=True)
            _json_response(self, 200, {"ok": True, "chromium": True})
            return
        _json_response(self, 404, {"error": "not found"})

    def do_DELETE(self) -> None:
        path = urlparse(self.path).path
        if not path.startswith("/api/media/"):
            _json_response(self, 404, {"error": "not found"})
            return
        item_id = path[len("/api/media/") :]
        data = _load_library()
        item = _find_item(data, item_id)
        if item is None:
            _json_response(self, 404, {"error": "элемент не найден"})
            return
        if item["type"] in ("image", "video") and item.get("file"):
            fpath = MEDIA / item["file"]
            if fpath.is_file():
                fpath.unlink()
        data["items"] = [i for i in data["items"] if i["id"] != item_id]
        if data.get("current") == item_id:
            data["current"] = None
        _save_library(data)
        _json_response(self, 200, {"ok": True})

    def _upload(self) -> None:
        form = cgi.FieldStorage(
            fp=self.rfile,
            headers=self.headers,
            environ={
                "REQUEST_METHOD": "POST",
                "CONTENT_TYPE": self.headers.get("Content-Type", ""),
                "CONTENT_LENGTH": self.headers.get("Content-Length", "0"),
            },
        )
        if "file" not in form:
            _json_response(self, 400, {"error": "нет поля file"})
            return
        field = form["file"]
        if isinstance(field, list):
            field = field[0]
        if not getattr(field, "filename", None):
            _json_response(self, 400, {"error": "пустой файл"})
            return
        original = field.filename
        ext = Path(original).suffix.lower()
        if ext in IMAGE_EXT:
            kind = "image"
        elif ext in VIDEO_EXT:
            kind = "video"
        else:
            _json_response(
                self,
                400,
                {
                    "error": f"формат {ext or '(без расширения)'} не поддержан",
                    "hint": "фото: jpg/png/gif/webp; видео: mp4/webm/mov/mkv",
                },
            )
            return
        MEDIA.mkdir(parents=True, exist_ok=True)
        safe = _safe_name(original)
        stored = f"{uuid.uuid4().hex[:10]}_{safe}"
        dest = MEDIA / stored
        data_bytes = field.file.read()
        if not data_bytes:
            _json_response(self, 400, {"error": "файл пустой"})
            return
        dest.write_bytes(data_bytes)
        data = _load_library()
        item = {
            "id": uuid.uuid4().hex,
            "type": kind,
            "name": original,
            "file": stored,
            "added": time.time(),
        }
        data["items"].insert(0, item)
        _save_library(data)
        _json_response(self, 200, {"item": item})

    def _file(self, path: Path, content_type: str) -> None:
        raw = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)


def main() -> None:
    MEDIA.mkdir(parents=True, exist_ok=True)
    if not LIBRARY.exists():
        _save_library({"items": [], "current": None})
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Pupper Screen: http://0.0.0.0:{PORT}")
    print(f"Управление: открой http://<ip-pi>:{PORT}/")
    print(f"Экран робота: {VIEW_URL}")
    print("Остановка: Ctrl+C")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nСтоп.")
    finally:
        stop_chromium()
        server.server_close()


if __name__ == "__main__":
    main()
