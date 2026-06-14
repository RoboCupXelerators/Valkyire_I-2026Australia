"""Small UDP JSON client for Unity scene control.

The matching Unity endpoint lives in ``XelRuntimeControlUi`` and is intended
for repeatable simulator tests: place the ball, place robots, toggle rules,
and read a compact scene snapshot without touching the in-game UI.
"""

from __future__ import annotations

import json
import os
import socket
import time
from typing import Any


class XeleratorSceneControl:
    def __init__(
        self,
        host: str | None = None,
        port: int | None = None,
        *,
        timeout_s: float = 1.0,
    ) -> None:
        self.host = host or os.environ.get("XEL_SCENE_CONTROL_HOST", "127.0.0.1")
        self.port = int(port if port is not None else os.environ.get("XEL_SCENE_CONTROL_PORT", "20100"))
        self.timeout_s = float(timeout_s)

    def request(self, cmd: str, **payload: Any) -> dict[str, Any]:
        message = {"cmd": cmd}
        message.update(payload)
        data = json.dumps(message, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        attempts = max(1, int(os.environ.get("XEL_SCENE_CONTROL_RETRIES", "4")))
        last_error: OSError | None = None
        for attempt in range(attempts):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.settimeout(self.timeout_s)
                    sock.sendto(data, (self.host, self.port))
                    raw, _ = sock.recvfrom(65535)
                return json.loads(raw.decode("utf-8", errors="replace"))
            except (ConnectionResetError, socket.timeout, OSError) as exc:
                last_error = exc
                if attempt + 1 >= attempts:
                    break
                time.sleep(0.15 * (attempt + 1))
        if last_error is not None:
            raise last_error
        raise TimeoutError(f"scene-control request timed out: {cmd}")

    def ping(self) -> dict[str, Any]:
        return self.request("ping")

    def status(self) -> dict[str, Any]:
        return self.request("status")

    def set_ball(
        self,
        x_cm: float,
        y_cm: float,
        *,
        vx_cm_s: float = 0.0,
        vy_cm_s: float = 0.0,
        reset_velocity: bool = True,
    ) -> dict[str, Any]:
        return self.request(
            "set_ball",
            x_cm=float(x_cm),
            y_cm=float(y_cm),
            vx_cm_s=float(vx_cm_s),
            vy_cm_s=float(vy_cm_s),
            reset_velocity=bool(reset_velocity),
        )

    def set_robot(
        self,
        robot_id: int,
        x_cm: float,
        y_cm: float,
        yaw_deg: float = 0.0,
        *,
        active: bool = True,
    ) -> dict[str, Any]:
        return self.request(
            "set_robot",
            robot_id=int(robot_id),
            x_cm=float(x_cm),
            y_cm=float(y_cm),
            yaw_deg=float(yaw_deg),
            active=bool(active),
        )

    def set_rules(
        self,
        *,
        penalties: bool,
        lack: bool,
        pushing: bool,
    ) -> dict[str, Any]:
        return self.request(
            "set_rules",
            penalties=bool(penalties),
            lack=bool(lack),
            pushing=bool(pushing),
        )

    def set_ball_physics(
        self,
        *,
        drag: float | None = None,
        angular_drag: float | None = None,
        ball_friction: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        if drag is not None:
            payload["drag"] = float(drag)
        if angular_drag is not None:
            payload["angular_drag"] = float(angular_drag)
        if ball_friction is not None:
            payload["ball_friction"] = float(ball_friction)
        return self.request("set_ball_physics", **payload)

    def set_robot_tuning(
        self,
        *,
        robot_id: int = 0,
        linear_scale: float | None = None,
        lateral_scale: float | None = None,
        angular_scale: float | None = None,
        linear_accel: float | None = None,
        linear_brake_accel: float | None = None,
        angular_accel: float | None = None,
        angular_brake_accel: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {"robot_id": int(robot_id)}
        for key, value in (
            ("linear_scale", linear_scale),
            ("lateral_scale", lateral_scale),
            ("angular_scale", angular_scale),
            ("linear_accel", linear_accel),
            ("linear_brake_accel", linear_brake_accel),
            ("angular_accel", angular_accel),
            ("angular_brake_accel", angular_brake_accel),
        ):
            if value is not None:
                payload[key] = float(value)
        return self.request("set_robot_tuning", **payload)

    def set_dribble(self, robot_id: int, enabled: bool, *, strength: float = 0.7) -> dict[str, Any]:
        return self.request(
            "set_dribble",
            robot_id=int(robot_id),
            active=bool(enabled),
            dribble=bool(enabled),
            strength=float(strength),
        )

    def kick(self, robot_id: int, *, strength: float = 1.0) -> dict[str, Any]:
        return self.request("kick", robot_id=int(robot_id), strength=float(strength))

    def reset_score(self) -> dict[str, Any]:
        return self.request("reset_score")

    def kickoff_ally(self) -> dict[str, Any]:
        return self.request("kickoff_ally")

    def kickoff_enemy(self) -> dict[str, Any]:
        return self.request("kickoff_enemy")


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Send one Unity scene-control command.")
    parser.add_argument("cmd", nargs="?", default="status")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--x", type=float, default=0.0, help="x_forward in cm")
    parser.add_argument("--y", type=float, default=0.0, help="y_left in cm")
    parser.add_argument("--yaw", type=float, default=0.0, help="software yaw in degrees")
    parser.add_argument("--vx", type=float, default=0.0, help="ball x_forward velocity in cm/s")
    parser.add_argument("--vy", type=float, default=0.0, help="ball y_left velocity in cm/s")
    parser.add_argument("--linear-scale", type=float, default=None)
    parser.add_argument("--lateral-scale", type=float, default=None)
    parser.add_argument("--angular-scale", type=float, default=None)
    parser.add_argument("--strength", type=float, default=1.0)
    parser.add_argument("--no-reset-velocity", action="store_true")
    parser.add_argument("--rules-off", action="store_true")
    args = parser.parse_args()

    ctl = XeleratorSceneControl(args.host, args.port)
    if args.cmd == "set_ball":
        response = ctl.set_ball(args.x, args.y, vx_cm_s=args.vx, vy_cm_s=args.vy, reset_velocity=not args.no_reset_velocity)
    elif args.cmd == "set_robot":
        response = ctl.set_robot(args.robot_id, args.x, args.y, args.yaw)
    elif args.cmd == "set_robot_tuning":
        response = ctl.set_robot_tuning(
            robot_id=args.robot_id,
            linear_scale=args.linear_scale,
            lateral_scale=args.lateral_scale,
            angular_scale=args.angular_scale,
        )
    elif args.cmd == "set_dribble":
        response = ctl.set_dribble(args.robot_id, True, strength=args.strength)
    elif args.cmd == "dribble_off":
        response = ctl.set_dribble(args.robot_id, False, strength=0.0)
    elif args.cmd == "kick":
        response = ctl.kick(args.robot_id, strength=args.strength)
    elif args.cmd == "reset_score":
        response = ctl.reset_score()
    elif args.cmd == "kickoff_ally":
        response = ctl.kickoff_ally()
    elif args.cmd == "kickoff_enemy":
        response = ctl.kickoff_enemy()
    elif args.cmd == "rules_off" or args.rules_off:
        response = ctl.set_rules(penalties=False, lack=False, pushing=False)
    elif args.cmd == "ping":
        response = ctl.ping()
    else:
        response = ctl.status()

    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0 if response.get("ok", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
