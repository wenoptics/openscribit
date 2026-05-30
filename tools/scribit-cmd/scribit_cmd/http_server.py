from __future__ import annotations

import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from .gcode import Cache
from .settings import SETTINGS, SharedSettings


class DynamicGcodeStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._gcode: dict[str, str] = {}

    def set(self, cmd: str, gcode: str) -> None:
        with self._lock:
            self._gcode[cmd] = gcode

    def get(self, cmd: str) -> str | None:
        with self._lock:
            return self._gcode.get(cmd)


CACHE = Cache()
DYNAMIC_GCODE = DynamicGcodeStore()

_log = logging.getLogger("scribit_cmd.http_server")


def send_response_body(handler: BaseHTTPRequestHandler, code: int, body: bytes, ctype: str) -> None:
    handler.send_response(code)
    handler.send_header("Content-Type", ctype)
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


class GcodeHandler(BaseHTTPRequestHandler):
    settings: SharedSettings = SETTINGS
    cache: Cache = CACHE
    dynamic_gcode: DynamicGcodeStore = DYNAMIC_GCODE

    def log_message(self, fmt: str, *args) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/health":
            return send_response_body(self, 200, b"ok\n", "text/plain; charset=utf-8")

        if path.startswith("/g/") and path.endswith(".gcode"):
            cmd = path.split("/")[-1].replace(".gcode", "")
            try:
                dynamic_gcode = self.dynamic_gcode.get(cmd)
                if dynamic_gcode is not None:
                    return send_response_body(
                        self,
                        200,
                        dynamic_gcode.encode("utf-8"),
                        "text/plain; charset=utf-8",
                    )

                gcode = self.cache.get_gcode(self.settings.key(), cmd)
                return send_response_body(self, 200, gcode.encode("utf-8"), "text/plain; charset=utf-8")
            except Exception as exc:
                return send_response_body(
                    self,
                    400,
                    f"bad request: {exc}\n".encode("utf-8"),
                    "text/plain; charset=utf-8",
                )

        return send_response_body(self, 404, b"not found\n", "text/plain; charset=utf-8")


class FileHandler(BaseHTTPRequestHandler):
    gcode_path: Path
    url_path: str
    downloaded: threading.Event

    def setup(self) -> None:
        super().setup()
        _log.debug("connect   %s:%s", *self.client_address)

    def log_message(self, fmt: str, *args) -> None:
        _log.debug(fmt, *args)

    def finish(self) -> None:
        super().finish()  # flushes wfile → kernel TCP buffer
        if getattr(self, "_file_served", False):
            self.downloaded.set()
        _log.debug("close     %s:%s", *self.client_address)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = unquote(parsed.path)

        if path == "/health":
            return send_response_body(self, 200, b"ok\n", "text/plain; charset=utf-8")

        if path != self.url_path:
            return send_response_body(self, 404, b"not found\n", "text/plain; charset=utf-8")

        try:
            body = self.gcode_path.read_bytes()
        except OSError as exc:
            return send_response_body(
                self,
                500,
                f"read failed: {exc}\n".encode("utf-8"),
                "text/plain; charset=utf-8",
            )

        self._file_served = True
        return send_response_body(self, 200, body, "text/plain; charset=utf-8")


def start_http_server(http_port: int, handler: type[BaseHTTPRequestHandler]) -> ThreadingHTTPServer:
    httpd = ThreadingHTTPServer(("0.0.0.0", http_port), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd

