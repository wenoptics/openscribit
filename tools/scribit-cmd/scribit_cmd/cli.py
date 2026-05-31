from __future__ import annotations

import json
import logging
import math
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Annotated
from urllib.parse import quote

import paho.mqtt.client as mqtt
import typer

from .app import App
from .http_server import FileHandler, start_http_server
from .mqtt_client import mqtt_pub
from .tui import run_curses

cli = typer.Typer(
    add_completion=False,
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Scribit command line tools. Serves GCODE over HTTP and commands the robot over MQTT.",
)


def validate_firmware_http_port(http_port: int) -> None:
    if http_port != 80:
        typer.secho(
            "ERROR: Your firmware rejects URLs with ':port'. Use --http-port 80 and run with sudo.",
            fg=typer.colors.RED,
            err=True,
        )
        raise typer.Exit(2)


@cli.command()
def interactive(
    robot_id: Annotated[
        str,
        typer.Option("--robot-id", help="Scribit robot id used in tin/<robot-id>/... MQTT topics."),
    ],
    mqtt_host: Annotated[
        str,
        typer.Option("--mqtt-host", help="MQTT broker host or IP address."),
    ],
    host_ip: Annotated[
        str,
        typer.Option("--host-ip", help="Unused by interactive manualMove mode; kept for compatibility."),
    ] = "127.0.0.1",
    mqtt_port: Annotated[int, typer.Option("--mqtt-port", min=1, max=65535, help="MQTT broker port.")] = 1883,
    mqtt_user: Annotated[
        str,
        typer.Option("--mqtt-user", help="MQTT username. Use an empty string to disable auth."),
    ] = "scribit",
    mqtt_pass: Annotated[
        str,
        typer.Option("--mqtt-pass", help="MQTT password. Use an empty string to disable auth."),
    ] = "scribit",
    http_port: Annotated[
        int,
        typer.Option(
            "--http-port",
            min=1,
            max=65535,
            help="Unused by interactive manualMove mode; kept for compatibility.",
        ),
    ] = 80,
    suffix: Annotated[
        str,
        typer.Option("--suffix", help="Unused by interactive manualMove mode; kept for compatibility."),
    ] = "G4 P0",
    step: Annotated[
        float,
        typer.Option("--step", min=0.001, help="Initial jog step in mm for cables and degrees for carousel."),
    ] = 2.0,
    feed: Annotated[int, typer.Option("--feed", min=1, help="Initial GCODE feed rate.")] = 900,
) -> None:
    app = App(
        robot_id=robot_id,
        mqtt_host=mqtt_host,
        mqtt_port=mqtt_port,
        mqtt_user=mqtt_user,
        mqtt_pass=mqtt_pass,
        host_ip=host_ip,
        http_port=http_port,
        suffix=suffix,
    )

    typer.echo(f"[scribit_jog_cli] MQTT broker {mqtt_host}:{mqtt_port}  robot_id={robot_id}  command=manualMove")

    try:
        run_curses(app, step0=step, feed0=feed)
    except KeyboardInterrupt:
        pass


@cli.command()
def draw(
    gcode_path: Annotated[
        Path,
        typer.Argument(
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
            resolve_path=True,
            help="Path to the GCODE file the robot should download and draw.",
        ),
    ],
    robot_id: Annotated[
        str,
        typer.Option("--robot-id", help="Scribit robot id used in tin/<robot-id>/... MQTT topics."),
    ],
    mqtt_host: Annotated[
        str,
        typer.Option("--mqtt-host", help="MQTT broker host or IP address."),
    ],
    host_ip: Annotated[
        str,
        typer.Option("--host-ip", help="This computer's IP address as reachable by the robot."),
    ],
    mqtt_port: Annotated[int, typer.Option("--mqtt-port", min=1, max=65535, help="MQTT broker port.")] = 1883,
    mqtt_user: Annotated[
        str,
        typer.Option("--mqtt-user", help="MQTT username. Use an empty string to disable auth."),
    ] = "scribit",
    mqtt_pass: Annotated[
        str,
        typer.Option("--mqtt-pass", help="MQTT password. Use an empty string to disable auth."),
    ] = "scribit",
    http_port: Annotated[
        int,
        typer.Option(
            "--http-port",
            min=1,
            max=65535,
            help="HTTP bind port. Current firmware requires 80 because URLs cannot include ':port'.",
        ),
    ] = 80,
    suffix: Annotated[
        str,
        typer.Option("--suffix", help="Required MQTT print payload suffix appended after the URL."),
    ] = "M18",
    wait_download: Annotated[
        float,
        typer.Option("--wait-download", min=0.0, help="Seconds to wait for the robot to request the GCODE file."),
    ] = 60.0,
    keep_alive: Annotated[
        float,
        typer.Option("--keep-alive", min=0.0, help="Seconds to keep serving after the first successful download."),
    ] = 5.0,
) -> None:
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s.%(msecs)03d %(levelname)-5s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    validate_firmware_http_port(http_port)

    filename = quote(gcode_path.name)
    url_path = f"/{filename}"
    url = f"http://{host_ip}{url_path}"
    payload = f"{url};{suffix}"
    downloaded = threading.Event()

    FileHandler.gcode_path = gcode_path
    FileHandler.url_path = f"/{gcode_path.name}"
    FileHandler.downloaded = downloaded
    httpd = start_http_server(http_port, FileHandler)

    # Make Scribit ready (leave BOOT, go IDLE)
    mqtt_pub(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, f"tin/{robot_id}/status", "{}")
    time.sleep(0.02)

    topic = f"tin/{robot_id}/print"
    typer.echo(f"[scribit_cmd] HTTP listening on 0.0.0.0:{http_port} (serving {gcode_path})")
    typer.echo(f"[scribit_cmd] MQTT broker {mqtt_host}:{mqtt_port}  topic={topic}")
    typer.echo(f"[scribit_cmd] print payload: {payload}")

    try:
        mqtt_pub(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, topic, payload)
        if wait_download > 0:
            if downloaded.wait(timeout=wait_download):
                typer.echo("[scribit_cmd] robot downloaded the GCODE file")
                if keep_alive > 0:
                    time.sleep(keep_alive)
            else:
                typer.secho(
                    f"[scribit_cmd] timed out waiting {wait_download:g}s for robot download; stopping HTTP server",
                    fg=typer.colors.YELLOW,
                    err=True,
                )
    finally:
        httpd.shutdown()


@cli.command()
def calibrate(
    left_mm: Annotated[float, typer.Option("--left", help="Left cable length in mm (nail centre to robot centre).")],
    right_mm: Annotated[float, typer.Option("--right", help="Right cable length in mm (nail centre to robot centre).")],
    robot_id: Annotated[str, typer.Option("--robot-id", help="Scribit robot id used in tin/<robot-id>/... MQTT topics.")],
    mqtt_host: Annotated[str, typer.Option("--mqtt-host", help="MQTT broker host or IP address.")],
    nail_spacing_mm: Annotated[float, typer.Option("--nail-spacing", help="Horizontal distance between the two nails in mm.")] = 1860.0,
    mqtt_port: Annotated[int, typer.Option("--mqtt-port", min=1, max=65535, help="MQTT broker port.")] = 1883,
    mqtt_user: Annotated[str, typer.Option("--mqtt-user", help="MQTT username.")] = "scribit",
    mqtt_pass: Annotated[str, typer.Option("--mqtt-pass", help="MQTT password.")] = "scribit",
) -> None:
    """Set robot position from measured cable lengths. No IMU or cloud service required.

    Measure the left and right cable lengths with a tape measure (nail centre to
    robot centre), then run this command. The robot's absolute wall position is
    computed from the polargraph geometry and injected via a G92 command.
    """
    D, L, R = nail_spacing_mm, left_mm, right_mm
    x = (L**2 - R**2 + D**2) / (2 * D)
    y = math.sqrt(max(0.0, L**2 - x**2))
    payload = f"G92 X{x:.2f} Y{y:.2f}"

    typer.echo(f"[scribit_cmd] Computed position: X={x:.1f} mm, Y={y:.1f} mm")
    typer.echo(f"[scribit_cmd] Sending manualMove payload: {payload}")
    mqtt_pub(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, f"tin/{robot_id}/manualMove", payload)
    typer.echo("[scribit_cmd] Done.")


@cli.command()
def autocal(
    wall_id: Annotated[
        int,
        typer.Option(
            "--wall-id",
            min=1,
            max=9,
            help=(
                "Scribit measuring-tape wall ID (1-9). "
                "A-tape (2-2.75 m walls): 1-4. B-tape (3-4 m walls): 5-9."
            ),
        ),
    ],
    robot_id: Annotated[
        str,
        typer.Option("--robot-id", help="Scribit robot id used in tin/<robot-id>/... MQTT topics."),
    ],
    mqtt_host: Annotated[
        str,
        typer.Option("--mqtt-host", help="MQTT broker host or IP address."),
    ],
    cal_service_url: Annotated[
        str,
        typer.Option(
            "--cal-service",
            help="Base URL of the running calibration service (e.g. http://192.168.1.10:9915).",
        ),
    ],
    send_on_stop: Annotated[
        str,
        typer.Option(
            "--send-on-stop",
            help=(
                "G-code command the robot sends to the SAMD21 after calibration completes "
                "(forwarded verbatim as the 'G1' portion of the calibration MQTT payload). "
                "Default positions the robot at the wall centre."
            ),
        ),
    ] = "G1 X200 Y200 F2000",
    mqtt_port: Annotated[int, typer.Option("--mqtt-port", min=1, max=65535, help="MQTT broker port.")] = 1883,
    mqtt_user: Annotated[
        str,
        typer.Option("--mqtt-user", help="MQTT username. Use an empty string to disable auth."),
    ] = "scribit",
    mqtt_pass: Annotated[
        str,
        typer.Option("--mqtt-pass", help="MQTT password. Use an empty string to disable auth."),
    ] = "scribit",
    timeout: Annotated[
        float,
        typer.Option("--timeout", min=1.0, help="Seconds to wait for calibration to complete."),
    ] = 600.0,
) -> None:
    """Run the IMU-based automatic wall calibration (autocal).

    The robot must be hanging freely at Point Zero (as marked by the Scribit
    measuring tape) and in IDLE state before running this command.

    Flow:
    1. Verify the calibration service is reachable.
    2. Subscribe to tout/<robot-id>/# to monitor firmware status.
    3. Publish the calibration command to tin/<robot-id>/calibration.
    4. The firmware downloads autocal.GCODE from the service, runs the IMU
       tilt sequence, POSTs the four readings to the service, and applies the
       returned G92 command to set its absolute wall position.
    5. Exit 0 on success, 1 on firmware-reported failure, 2 on timeout/error.
    """
    _log = lambda msg, **kw: typer.secho(f"[autocal] {msg}", **kw)  # noqa: E731

    # --- 1. Sanity-check calibration service ------------------------------
    health_url = cal_service_url.rstrip("/") + "/health"
    _log(f"Checking calibration service at {health_url} …")
    try:
        with urllib.request.urlopen(health_url, timeout=10) as resp:
            body = resp.read().decode()
        _log(f"  Service OK: {body.strip()}", fg=typer.colors.GREEN)
    except urllib.error.URLError as exc:
        _log(
            f"Calibration service unreachable: {exc.reason}",
            fg=typer.colors.RED,
            err=True,
        )
        raise typer.Exit(2)

    # --- 2. Derive MQTT topics -------------------------------------------
    # Firmware subscribes on tin/<robot-id>/... and publishes on tout/<robot-id>/...
    # robot_id is the 12-hex-char MAC, e.g. "aabbccddeeff"
    out_prefix = f"tout/{robot_id}/"
    in_calibration = f"tin/{robot_id}/calibration"
    in_status = f"tin/{robot_id}/status"

    # Calibration MQTT payload: "<send-on-stop>;<wall-id>"
    # parseCalibPyaload() in Scribit_mqtt.cpp reads:
    #   m_sendOnStop = substring from indexOf("G1") to indexOf(";")
    #   m_wallID     = substring after ";" converted to int
    cal_payload = f"{send_on_stop};{wall_id}"

    # --- 3. Set up MQTT subscriber ----------------------------------------
    done = threading.Event()
    result: dict[str, object] = {}  # filled by on_message

    def on_connect(client: mqtt.Client, _userdata: None, _flags: dict, rc: int, _props: object) -> None:
        if rc != 0:
            _log(f"MQTT subscriber connect failed (rc={rc})", fg=typer.colors.RED, err=True)
            result["error"] = f"MQTT connect rc={rc}"
            done.set()
            return
        client.subscribe(f"{out_prefix}#")
        _log(f"Subscribed to {out_prefix}#")

    def on_message(_client: mqtt.Client, _userdata: None, msg: mqtt.MQTTMessage) -> None:
        suffix = msg.topic.removeprefix(out_prefix)
        payload = msg.payload.decode(errors="replace")

        # Map verbose/noisy topics to colour-coded log lines
        if suffix == "idle":
            _log(f"  idle: {payload}")
        elif suffix == "calibrating":
            # {"status":0} = success, {"status":1} = failed
            _log(f"  calibrating: {payload}", fg=typer.colors.CYAN)
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                # empty {} published when entering calibration state — not terminal
                return
            status = data.get("status")
            if status == 0:
                _log("Calibration succeeded.", fg=typer.colors.GREEN)
                result["status"] = "ok"
                done.set()
            elif status == 1:
                _log("Firmware reported calibration FAILED.", fg=typer.colors.RED, err=True)
                result["status"] = "fail"
                done.set()
        elif suffix == "error":
            _log(f"  ERROR from robot: {payload}", fg=typer.colors.RED, err=True)
            # Don't set done — errors mid-flow may be recoverable (firmware retries)
        elif suffix in {"calibDebug", "calibEcho", "debug", "serialecho"}:
            _log(f"  [{suffix}] {payload}", fg=typer.colors.BRIGHT_BLACK)
        elif suffix == "success":
            _log(f"  success: {payload}", fg=typer.colors.GREEN)
        elif suffix in {"printing", "erasing", "boot"}:
            _log(f"  {suffix}: {payload}")
        else:
            _log(f"  {suffix}: {payload}", fg=typer.colors.BRIGHT_BLACK)

    sub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    sub.on_connect = on_connect  # type: ignore[assignment]
    sub.on_message = on_message  # type: ignore[assignment]
    if mqtt_user or mqtt_pass:
        sub.username_pw_set(mqtt_user, mqtt_pass)

    _log(f"Connecting subscriber to MQTT broker {mqtt_host}:{mqtt_port} …")
    try:
        sub.connect(mqtt_host, mqtt_port, keepalive=60)
    except OSError as exc:
        _log(f"Cannot connect to MQTT broker: {exc}", fg=typer.colors.RED, err=True)
        raise typer.Exit(2)

    sub.loop_start()

    try:
        # --- 4. Wake robot (BOOT → IDLE) then send calibration command -------
        _log(f"Sending status ping to {in_status}")
        mqtt_pub(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, in_status, "{}")
        time.sleep(0.1)

        _log(f"Publishing calibration command to {in_calibration}")
        _log(f"  payload: {cal_payload!r}")
        _log(f"  wall-id={wall_id}  send-on-stop={send_on_stop!r}")
        mqtt_pub(mqtt_host, mqtt_port, mqtt_user, mqtt_pass, in_calibration, cal_payload)

        _log(f"Waiting up to {timeout:g}s for calibration to complete …")
        completed = done.wait(timeout=timeout)
    finally:
        sub.loop_stop()
        sub.disconnect()

    if not completed:
        _log(
            f"Timed out after {timeout:g}s — calibration did not complete.",
            fg=typer.colors.YELLOW,
            err=True,
        )
        raise typer.Exit(2)

    if result.get("error"):
        _log(str(result["error"]), fg=typer.colors.RED, err=True)
        raise typer.Exit(2)

    if result.get("status") != "ok":
        raise typer.Exit(1)


def main() -> None:
    cli()
