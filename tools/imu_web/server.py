#!/usr/bin/env python3
"""HTTP/SSE dashboard with a single-owner DM-S2325 control worker."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parent
INDEX = ROOT / "index.html"
REQUIRED_FIELDS = {
    "sequence", "timestamp_us", "source", "healthy", "age_ms", "flags",
    "accel_mps2", "gyro_rad_s", "roll_rad", "pitch_rad", "yaw_rad",
    "temperature_c",
}
STREAM_TIMEOUT_S = 1.5
STOP = threading.Event()
COND = threading.Condition()
PROCESS_LOCK = threading.Lock()
ACTIVE_PROCESS: subprocess.Popen | None = None
VERSION = 0
LAST_PUBLISH_MONOTONIC = 0.0
STREAM_ERROR = "waiting for first sample"
CONTROL_QUEUE: queue.Queue = queue.Queue()
CONTROL_LOCK = threading.Lock()
CONTROL_STATUS = {
    "hardware": False,
    "direction_confirmed": False,
    "armed": False,
    "moving": False,
    "online_mask": 0,
    "enabled_mask": 0,
    "fault_mask": 0,
    "runtime_state": 0,
    "runtime_fault": 0,
    "target_vx_mps": 0.0,
    "target_vy_mps": 0.0,
    "target_wz_rad_s": 0.0,
    "measured_vx_mps": 0.0,
    "measured_vy_mps": 0.0,
    "measured_wz_rad_s": 0.0,
    "measured_speed_mps": 0.0,
    "peak_speed_mps": 0.0,
    "motor_speed_rad_s": [0.0, 0.0, 0.0, 0.0],
    "motor_torque_nm": [0.0, 0.0, 0.0, 0.0],
    "max_abs_torque_nm": 0.0,
    "peak_abs_torque_nm": 0.0,
    "feedback_age_ms": None,
    "speed_error": None,
    "_speed_timestamp_monotonic": 0.0,
    "error": None,
}
# Matches the C-board's 24 V no-load limit for DM-S2325 with 30 mm wheels.
MAX_LINEAR_MPS = 2.29
MAX_ANGULAR_RAD_S = 0.8
# The Pi owns the C-board heartbeat. Browser timing only decides how long a
# requested target remains valid; it must not directly pace the 100 ms board watchdog.
CONTROL_HEARTBEAT_S = 0.02
CONTROL_DEADMAN_S = 0.20


def initial_status() -> dict:
    return {
        "sequence": 0,
        "timestamp_us": 0,
        "source": "starting",
        "healthy": False,
        "age_ms": 0.0,
        "flags": 0,
        "accel_mps2": [0.0, 0.0, 9.80665],
        "gyro_rad_s": [0.0, 0.0, 0.0],
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
        "temperature_c": None,
        "stream_connected": False,
        "stream_error": STREAM_ERROR,
        "direction_flags": [
            "AXIS_MAPPING_UNVERIFIED",
            "ROBOT_DIRECTION_UNDEFINED",
            "DEFERRED_UNTIL_FINAL_ASSEMBLY",
        ],
    }


LATEST = initial_status()


def reset_runtime_state() -> None:
    global LATEST, VERSION, LAST_PUBLISH_MONOTONIC, STREAM_ERROR, ACTIVE_PROCESS
    STOP.clear()
    with COND:
        LATEST = initial_status()
        VERSION = 0
        LAST_PUBLISH_MONOTONIC = 0.0
        STREAM_ERROR = "waiting for first sample"
    with PROCESS_LOCK:
        ACTIVE_PROCESS = None
    with CONTROL_LOCK:
        CONTROL_STATUS.update({
            "hardware": False,
            "direction_confirmed": False,
            "armed": False,
            "moving": False,
            "online_mask": 0,
            "enabled_mask": 0,
            "fault_mask": 0,
            "runtime_state": 0,
            "runtime_fault": 0,
            "target_vx_mps": 0.0,
            "target_vy_mps": 0.0,
            "target_wz_rad_s": 0.0,
            "measured_vx_mps": 0.0,
            "measured_vy_mps": 0.0,
            "measured_wz_rad_s": 0.0,
            "measured_speed_mps": 0.0,
            "peak_speed_mps": 0.0,
            "motor_speed_rad_s": [0.0, 0.0, 0.0, 0.0],
            "motor_torque_nm": [0.0, 0.0, 0.0, 0.0],
            "max_abs_torque_nm": 0.0,
            "peak_abs_torque_nm": 0.0,
            "feedback_age_ms": None,
            "speed_error": None,
            "_speed_timestamp_monotonic": 0.0,
            "error": None,
        })
    while True:
        try:
            CONTROL_QUEUE.get_nowait()
        except queue.Empty:
            break


def _reject_constant(value: str):
    raise ValueError(f"non-finite JSON constant {value}")


def parse_json_line(line: str) -> dict:
    sample = json.loads(line, parse_constant=_reject_constant)
    if not isinstance(sample, dict):
        raise ValueError("sample is not an object")
    missing = REQUIRED_FIELDS - sample.keys()
    if missing:
        raise ValueError(f"missing fields: {sorted(missing)}")
    if sample["source"] not in ("hardware", "demo"):
        raise ValueError("source must be hardware or demo")
    if not isinstance(sample["accel_mps2"], list) or len(sample["accel_mps2"]) != 3:
        raise ValueError("accel_mps2 must contain three values")
    if not isinstance(sample["gyro_rad_s"], list) or len(sample["gyro_rad_s"]) != 3:
        raise ValueError("gyro_rad_s must contain three values")
    for value in (
        sample["age_ms"], sample["roll_rad"], sample["pitch_rad"], sample["yaw_rad"],
        *sample["accel_mps2"], *sample["gyro_rad_s"],
    ):
        if value is not None and (not isinstance(value, (int, float)) or not math.isfinite(value)):
            raise ValueError("sample contains a non-finite numeric value")
    return sample


def publish(sample: dict) -> None:
    global LATEST, VERSION, LAST_PUBLISH_MONOTONIC, STREAM_ERROR
    clean = dict(sample)
    clean["stream_connected"] = True
    clean["stream_error"] = None
    with COND:
        LATEST = clean
        VERSION += 1
        LAST_PUBLISH_MONOTONIC = time.monotonic()
        STREAM_ERROR = ""
        COND.notify_all()


def mark_stream_error(message: str) -> None:
    global LATEST, VERSION, STREAM_ERROR
    with COND:
        LATEST = dict(LATEST)
        LATEST["healthy"] = False
        LATEST["stream_connected"] = False
        LATEST["stream_error"] = message
        STREAM_ERROR = message
        VERSION += 1
        COND.notify_all()


def status_snapshot() -> dict:
    with COND:
        sample = dict(LATEST)
        last_publish = LAST_PUBLISH_MONOTONIC
    if last_publish <= 0.0 or time.monotonic() - last_publish > STREAM_TIMEOUT_S:
        sample["healthy"] = False
        sample["stream_connected"] = False
        sample["stream_error"] = STREAM_ERROR or "data timeout"
    return sample


def control_snapshot() -> dict:
    with CONTROL_LOCK:
        snapshot = dict(CONTROL_STATUS)
    timestamp = snapshot.pop("_speed_timestamp_monotonic", 0.0)
    snapshot["speed_age_ms"] = (
        max(0.0, (time.monotonic() - timestamp) * 1000.0) if timestamp else None
    )
    return snapshot


def _set_control_status(**values) -> None:
    with CONTROL_LOCK:
        CONTROL_STATUS.update(values)


def submit_control(action: str, payload: dict | None = None, timeout_s: float = 3.0) -> dict:
    if not control_snapshot()["hardware"]:
        return {"success": False, "error": "hardware control is not active"}
    request = {
        "action": action,
        "payload": payload or {},
        "done": threading.Event(),
        "result": None,
    }
    CONTROL_QUEUE.put(request)
    if not request["done"].wait(timeout_s):
        return {"success": False, "error": "hardware worker timeout"}
    return request["result"]


def _finite_limited(value, limit: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or abs(value) > limit:
        raise ValueError(f"{name} must be finite and within +/-{limit}")
    return value


def _telemetry_dict(status) -> dict:
    estimate = status.velocity_estimate
    motor_torque_nm = [float(motor.torque_nm) for motor in status.motors]
    return {
        "online_mask": status.online_mask,
        "enabled_mask": status.enabled_mask,
        "fault_mask": status.fault_mask,
        "runtime_state": status.runtime_state,
        "runtime_fault": status.runtime_fault,
        "measured_vx_mps": estimate.vx_mps,
        "measured_vy_mps": estimate.vy_mps,
        "measured_wz_rad_s": estimate.wz_rad_s,
        "measured_speed_mps": estimate.speed_mps,
        "motor_speed_rad_s": list(estimate.motor_rad_s),
        "motor_torque_nm": motor_torque_nm,
        "max_abs_torque_nm": max(abs(value) for value in motor_torque_nm),
        "feedback_age_ms": max(motor.feedback_age_ms for motor in status.motors),
        "speed_error": None,
        "_speed_timestamp_monotonic": time.monotonic(),
    }


def _update_encoder_telemetry(status) -> None:
    values = _telemetry_dict(status)
    values["peak_speed_mps"] = max(
        control_snapshot().get("peak_speed_mps", 0.0), values["measured_speed_mps"])
    values["peak_abs_torque_nm"] = max(
        control_snapshot().get("peak_abs_torque_nm", 0.0),
        values["max_abs_torque_nm"],
    )
    _set_control_status(**values)


def hardware_source(port: str, direction_confirmed: bool) -> None:
    """Own CDC for its whole lifetime; HTTP threads communicate only by queue."""
    source_root = ROOT.parent.parent / "src"
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))
    from utils.DM2325Bits import DM2325Bits, IMUJsonRateTracker, build_imu_web_record

    bits = DM2325Bits(port=port, exclusive=True)
    tracker = IMUJsonRateTracker()
    deadline = None
    next_control = None
    next_encoder = None
    target_twist = (0.0, 0.0, 0.0)
    next_imu = 0.0
    next_status = 0.0
    owner_client_id = None
    try:
        bits.OpenPort()
        disabled = bits.RefreshDisabledTelemetry()
        _set_control_status(
            hardware=True,
            direction_confirmed=bool(direction_confirmed),
            armed=False,
            moving=False,
            error=None,
            **_telemetry_dict(disabled),
        )
        while not STOP.is_set():
            now = time.monotonic()
            try:
                request = CONTROL_QUEUE.get(timeout=0.01)
            except queue.Empty:
                request = None

            if request is not None:
                action = request["action"]
                payload = request["payload"]
                try:
                    client_id = payload.get("client_id")
                    if not isinstance(client_id, str) or not (1 <= len(client_id) <= 128):
                        raise ValueError("valid client_id is required")
                    if action == "arm":
                        if not direction_confirmed:
                            raise RuntimeError("wheel direction is not confirmed")
                        if owner_client_id is not None and owner_client_id != client_id:
                            raise RuntimeError("chassis is controlled by another browser")
                        bits.Arm()
                        owner_client_id = client_id
                        target_twist = (0.0, 0.0, 0.0)
                        bits.SetChassisTwist(*target_twist)
                        now = time.monotonic()
                        deadline = now + CONTROL_DEADMAN_S
                        next_control = now + CONTROL_HEARTBEAT_S
                        next_encoder = now + 0.04
                        result = {"success": True, "armed": True}
                        _set_control_status(
                            armed=True, moving=False, enabled_mask=0x0F,
                            runtime_state=3, runtime_fault=0, error=None,
                            target_vx_mps=0.0, target_vy_mps=0.0,
                            target_wz_rad_s=0.0, peak_speed_mps=0.0,
                            peak_abs_torque_nm=0.0,
                        )
                    elif action == "move":
                        vx = _finite_limited(payload.get("vx", 0.0), MAX_LINEAR_MPS, "vx")
                        vy = _finite_limited(payload.get("vy", 0.0), MAX_LINEAR_MPS, "vy")
                        wz = _finite_limited(payload.get("wz", 0.0), MAX_ANGULAR_RAD_S, "wz")
                        if not control_snapshot()["armed"]:
                            if any(abs(value) > 1e-6 for value in (vx, vy, wz)):
                                raise RuntimeError("chassis is not armed")
                            # A zero heartbeat may already be in flight when a
                            # concurrent Disable completes. It is a safe no-op.
                            result = {"success": True, "vx": vx, "vy": vy, "wz": wz}
                            _set_control_status(moving=False, error=None)
                            request["result"] = result
                            request["done"].set()
                            continue
                        if owner_client_id != client_id:
                            raise RuntimeError("control lease belongs to another browser")
                        target_twist = (vx, vy, wz)
                        deadline = time.monotonic() + CONTROL_DEADMAN_S
                        moving = any(abs(value) > 1e-6 for value in (vx, vy, wz))
                        _set_control_status(
                            moving=moving, error=None,
                            target_vx_mps=vx, target_vy_mps=vy, target_wz_rad_s=wz,
                        )
                        result = {"success": True, "vx": vx, "vy": vy, "wz": wz}
                    elif action == "stop":
                        if not control_snapshot()["armed"]:
                            _set_control_status(moving=False, error=None)
                            request["result"] = {"success": True}
                            request["done"].set()
                            continue
                        if owner_client_id != client_id:
                            raise RuntimeError("control lease belongs to another browser")
                        target_twist = (0.0, 0.0, 0.0)
                        bits.SetChassisTwist(*target_twist)
                        now = time.monotonic()
                        deadline = now + CONTROL_DEADMAN_S
                        next_control = now + CONTROL_HEARTBEAT_S
                        _set_control_status(
                            moving=False, error=None,
                            target_vx_mps=0.0, target_vy_mps=0.0, target_wz_rad_s=0.0,
                        )
                        result = {"success": True}
                    elif action == "disable":
                        target_twist = (0.0, 0.0, 0.0)
                        was_armed = control_snapshot()["armed"]
                        if was_armed:
                            bits.stop()
                            bits.disable()
                        disabled = bits.RefreshDisabledTelemetry()
                        deadline = None
                        next_control = None
                        next_encoder = None
                        owner_client_id = None
                        _set_control_status(
                            armed=False, moving=False, error=None,
                            target_vx_mps=0.0, target_vy_mps=0.0, target_wz_rad_s=0.0,
                            **_telemetry_dict(disabled),
                        )
                        result = {"success": True, "armed": False}
                    else:
                        raise ValueError(f"unknown control action {action}")
                except Exception as exc:
                    result = {"success": False, "error": str(exc)}
                request["result"] = result
                request["done"].set()

            now = time.monotonic()
            if deadline is not None and now >= deadline:
                # A lost browser, Wi-Fi link, or JS timer must leave all motors Disabled.
                try:
                    bits.stop()
                    bits.disable()
                    disabled = bits.RefreshDisabledTelemetry()
                    _set_control_status(
                        armed=False, moving=False, error="deadman timeout: Disabled",
                        **_telemetry_dict(disabled),
                    )
                except Exception as exc:
                    _set_control_status(armed=False, moving=False, error=f"deadman stop failed: {exc}")
                deadline = None
                next_control = None
                next_encoder = None
                owner_client_id = None
                target_twist = (0.0, 0.0, 0.0)
                _set_control_status(
                    target_vx_mps=0.0, target_vy_mps=0.0, target_wz_rad_s=0.0,
                )
            elif next_control is not None and now >= next_control:
                # Keep the C-board watchdog independent from browser and Wi-Fi
                # scheduling. The browser deadline above still bounds how long
                # any non-zero target may survive after the client disappears.
                bits.SetChassisTwist(*target_twist)
                next_control = now + CONTROL_HEARTBEAT_S

            armed = control_snapshot()["armed"]
            now = time.monotonic()
            if armed and next_encoder is not None and now >= next_encoder:
                try:
                    # One bounded status exchange follows a fresh 20 ms control
                    # frame. A timeout cannot consume the C-board's 100 ms lease.
                    status = bits.GetChassisTelemetry(timeout=0.035)
                    _update_encoder_telemetry(status)
                except Exception as exc:
                    _set_control_status(speed_error=str(exc))
                next_encoder = time.monotonic() + 0.05
            if not armed and now >= next_imu:
                try:
                    publish(build_imu_web_record(bits.GetIMUState(), tracker))
                except Exception as exc:
                    mark_stream_error(str(exc))
                next_imu = now + 0.04
            if not armed and now >= next_status:
                try:
                    # While Disabled, explicit read-only refresh keeps all four
                    # feedback ages meaningful without enabling the motors.
                    status = bits.RefreshDisabledTelemetry()
                    _update_encoder_telemetry(status)
                except Exception as exc:
                    _set_control_status(error=str(exc))
                next_status = now + 0.2
    except Exception as exc:
        mark_stream_error(str(exc))
        _set_control_status(hardware=False, armed=False, moving=False, error=str(exc))
    finally:
        try:
            bits.shutdown()
        except Exception as exc:
            _set_control_status(error=f"shutdown failed: {exc}")
        _set_control_status(hardware=False, armed=False, moving=False)


def demo_source() -> None:
    sequence = 0
    started = time.monotonic()
    while not STOP.is_set():
        elapsed = time.monotonic() - started
        sequence += 1
        burst = 1.0 if int(elapsed) % 8 in (4, 5) else 0.0
        publish({
            "sequence": sequence,
            "timestamp_us": time.monotonic_ns() // 1000,
            "source": "demo",
            "healthy": True,
            "age_ms": 0.0,
            "flags": 0,
            "accel_mps2": [
                0.03 * math.sin(elapsed * 2.3) + burst * 1.2 * math.sin(elapsed * 17),
                0.04 * math.sin(elapsed * 1.7) + burst * 0.9 * math.sin(elapsed * 13),
                9.80665 + 0.04 * math.sin(elapsed * 1.2)
                + burst * 1.5 * math.sin(elapsed * 19),
            ],
            "gyro_rad_s": [
                0.01 * math.sin(elapsed) + burst * 0.8 * math.sin(elapsed * 15),
                0.01 * math.sin(elapsed * 0.8) + burst * 0.6 * math.sin(elapsed * 11),
                0.01 * math.sin(elapsed * 0.6) + burst * 0.5 * math.sin(elapsed * 9),
            ],
            "roll_rad": 0.03 * math.sin(elapsed * 0.5),
            "pitch_rad": 0.02 * math.sin(elapsed * 0.4),
            "yaw_rad": (elapsed * 0.03) % (2 * math.pi),
            "temperature_c": 33.0,
            "direction_flags": [
                "AXIS_MAPPING_UNVERIFIED", "ROBOT_DIRECTION_UNDEFINED",
                "DEFERRED_UNTIL_FINAL_ASSEMBLY",
            ],
        })
        STOP.wait(1 / 46)


def terminate_process(proc: subprocess.Popen | None, timeout_s: float = 2.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    proc.wait(timeout=timeout_s)


def stop_active_process() -> None:
    with PROCESS_LOCK:
        proc = ACTIVE_PROCESS
    terminate_process(proc)


def command_source(command: str, max_restarts: int = 3, restart_delay_s: float = 2.0) -> None:
    global ACTIVE_PROCESS
    attempts = 0
    while not STOP.is_set() and attempts <= max_restarts:
        proc = None
        try:
            with PROCESS_LOCK:
                if ACTIVE_PROCESS is not None and ACTIVE_PROCESS.poll() is None:
                    raise RuntimeError("a capture subprocess is already running")
                proc = subprocess.Popen(
                    ["/bin/sh", "-lc", command],
                    cwd=str(ROOT.parent.parent),
                    stdout=subprocess.PIPE,
                    stderr=sys.stderr,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
                ACTIVE_PROCESS = proc
            assert proc.stdout is not None
            for line in proc.stdout:
                if STOP.is_set():
                    break
                try:
                    publish(parse_json_line(line))
                except (json.JSONDecodeError, ValueError, TypeError) as exc:
                    print(f"[imu-web] discarded source line: {exc}", file=sys.stderr, flush=True)
            if STOP.is_set():
                break
            return_code = proc.wait(timeout=1.0)
            attempts += 1
            mark_stream_error(
                f"capture subprocess exited rc={return_code}; restart {attempts}/{max_restarts}")
        except Exception as exc:
            attempts += 1
            mark_stream_error(f"capture subprocess error: {exc}; restart {attempts}/{max_restarts}")
            print(f"[imu-web] source error: {exc}", file=sys.stderr, flush=True)
        finally:
            terminate_process(proc)
            with PROCESS_LOCK:
                if ACTIVE_PROCESS is proc:
                    ACTIVE_PROCESS = None
        if not STOP.is_set() and attempts <= max_restarts:
            STOP.wait(restart_delay_s)
    if not STOP.is_set():
        mark_stream_error("capture restart limit reached")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        sys.stderr.write("[http] " + fmt % args + "\n")

    def do_GET(self):
        if self.path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Cache-Control", "public, max-age=86400")
            self.end_headers()
            return
        if self.path in ("/", "/index.html"):
            body = INDEX.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/status":
            body = json.dumps(
                status_snapshot(), ensure_ascii=False, allow_nan=False
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/control/status":
            self._send_json(200, control_snapshot())
            return
        if self.path == "/api/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            last_version = -1
            try:
                while not STOP.is_set():
                    with COND:
                        if VERSION == last_version:
                            COND.wait(timeout=1.0)
                        version = VERSION
                    sample = status_snapshot()
                    if version != last_version:
                        data = json.dumps(
                            sample, ensure_ascii=False, separators=(",", ":"), allow_nan=False
                        )
                        self.wfile.write(f"data: {data}\n\n".encode())
                        self.wfile.flush()
                        last_version = version
                    else:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        self.send_error(404)

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path not in {
            "/api/control/arm", "/api/control/move", "/api/control/stop",
            "/api/control/disable",
        }:
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                raise ValueError("request body too large")
            payload = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(payload, dict):
                raise ValueError("JSON body must be an object")
            action = self.path.rsplit("/", 1)[-1]
            result = submit_control(action, payload)
            self._send_json(200 if result.get("success") else 409, result)
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json(400, {"success": False, "error": str(exc)})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--cmd")
    mode.add_argument("--demo", action="store_true")
    mode.add_argument("--hardware", action="store_true")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument(
        "--direction-confirmed",
        action="store_true",
        help="allow Arm only after FL/FR/RL/RR motor signs are physically verified",
    )
    parser.add_argument("--max-restarts", type=int, default=3)
    parser.add_argument("--restart-delay", type=float, default=2.0)
    args = parser.parse_args()
    if args.max_restarts < 0:
        parser.error("--max-restarts must be non-negative")
    if not math.isfinite(args.restart_delay) or args.restart_delay < 2.0:
        parser.error("--restart-delay must be finite and at least 2 seconds")

    reset_runtime_state()
    if args.hardware:
        source_target = hardware_source
        source_args = (args.device, args.direction_confirmed)
    elif args.cmd:
        source_target = command_source
        source_args = (args.cmd, args.max_restarts, args.restart_delay)
    else:
        source_target = demo_source
        source_args = ()
    source_thread = threading.Thread(
        target=source_target,
        args=source_args,
        daemon=False,
        name="imu-web-source",
    )
    source_thread.start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)

    def request_stop(_signum, _frame):
        STOP.set()
        stop_active_process()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(f"[imu-web] http://127.0.0.1:{args.port}", file=sys.stderr, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        request_stop(signal.SIGINT, None)
    finally:
        STOP.set()
        stop_active_process()
        server.server_close()
        source_thread.join(timeout=5.0)
        if source_thread.is_alive():
            print("[imu-web] source thread did not stop cleanly", file=sys.stderr, flush=True)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
