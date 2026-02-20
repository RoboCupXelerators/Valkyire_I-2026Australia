"""XeleratorBits

用于控制 Unity 模拟器（SoccerSimulatorForXels）中的 Xelerator 机器人。

设计目标：
- 形态尽量贴近 Software/src/utils/IceLoongBits.py
- 提供类似底盘/外设的控制入口：
  - SetWheelSpeed(w1,w2,w3,w4)  # 四轮轮速模式
  - SetTwist(vx, vy, wz)        # 机体速度模式（仿真更自然）
  - VirtualPorts(port, status)  # port=3 kick, port=4 dribble
  - GetYaw() / GetPose() / HasBall()

通讯：UDP（跨机器）。Unity 端会监听 cmd_port，并按命令包里的 status_port
将状态回传到“命令发送方 IP + status_port”。

协议版本：v1（定长二进制，little-endian）
CommandPacket (44 bytes):
  <BBHBBH  + 9f
  version:u8, type:u8(1), seq:u16, robot_id:u8, flags:u8, status_port:u16,
  vx,vy,wz,w1,w2,w3,w4,kick_strength,dribble_strength : float32

StatusPacket (32 bytes):
  <BBHBBH + 6f
  version:u8, type:u8(2), seq:u16, robot_id:u8, flags:u8(bit0=has_ball), reserved:u16,
  x,y,yaw,vx,vy,wz : float32

坐标系建议（Unity 默认）：
- Unity: X 右, Z 前, Y 上
- Software 输出: x_forward = Z, y_left = -X
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple


_CMD_STRUCT = struct.Struct("<BBHBBH9f")
_STATUS_STRUCT = struct.Struct("<BBHBBH6f")


class _CmdFlags:
    USE_WHEELS = 1 << 0
    KICK = 1 << 1
    DRIBBLE_ON = 1 << 2


class _StatusFlags:
    HAS_BALL = 1 << 0


@dataclass
class XelStatus:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    has_ball: bool = False
    seq: int = 0
    last_update_ts: float = 0.0


class XeleratorBits:
    """Unity 模拟器通讯封装类（UDP）。

    与 IceLoongBits 一样采用后台线程周期发送指令。
    """

    def __init__(
        self,
        unity_ip: str,
        *,
        cmd_port: int = 20001,
        status_port: int = 20002,
        robot_id: int = 1,
        send_hz: float = 50.0,
        recv_timeout: float = 0.2,
    ):
        self.unity_addr = (unity_ip, int(cmd_port))
        self.status_port = int(status_port)
        self.robot_id = int(robot_id) & 0xFF
        self.send_period = 1.0 / float(send_hz)
        self.recv_timeout = float(recv_timeout)

        self.logger = logging.getLogger(self.__class__.__name__)
        if not self.logger.handlers:
            self.logger.addHandler(logging.NullHandler())

        # --- desired command state ---
        self._use_wheels = False
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._wheels = (0.0, 0.0, 0.0, 0.0)

        self._kick_pending = False
        self._kick_strength = 1.0
        self._dribble_on = False
        self._dribble_strength = 0.0

        self._seq = 0
        self._lock = threading.Lock()

        # --- status ---
        self._status = XelStatus()

        # UDP sockets
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # status socket MUST bind for receiving
        self._status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._status_sock.bind(("0.0.0.0", self.status_port))
        self._status_sock.settimeout(self.recv_timeout)

        self._stop_evt = threading.Event()

        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread.start()
        self._recv_thread.start()

    # ------------------------- public api (IceLoong-like) -------------------------
    def Close(self) -> None:
        self._stop_evt.set()
        try:
            self._cmd_sock.close()
        except Exception:
            pass
        try:
            self._status_sock.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.Close()

    def SetWheelSpeed(self, w1: float, w2: float, w3: float, w4: float) -> None:
        """四轮轮速控制（仿真端会做简化的 mecanum/omni 换算）。"""
        with self._lock:
            self._use_wheels = True
            self._wheels = (float(w1), float(w2), float(w3), float(w4))

    def SetTwist(self, vx: float, vy: float, wz: float) -> None:
        """机体速度控制：vx(前+), vy(左+), wz(逆时针+，单位 rad/s)。"""
        with self._lock:
            self._use_wheels = False
            self._vx = float(vx)
            self._vy = float(vy)
            self._wz = float(wz)

    def VirtualPorts(self, port: int, status):
        """兼容上层外设接口。

        - port=3: Kick（status 为 bool 或数值力度）
        - port=4: Dribble（status 为 bool 或 0~1 float）
        """
        if int(port) == 3:
            # Kick: True triggers with default strength; number triggers with that strength
            strength = 1.0
            if isinstance(status, (int, float)):
                strength = float(status)
            self.Kick(strength=strength)
            return

        if int(port) == 4:
            if isinstance(status, bool):
                self.SetDribble(status, strength=self._dribble_strength)
            else:
                # treat as strength
                s = float(status)
                self.SetDribble(s > 1e-6, strength=s)
            return

    def Kick(self, *, strength: float = 1.0) -> None:
        with self._lock:
            self._kick_pending = True
            self._kick_strength = float(strength)

    def SetDribble(self, on: bool, *, strength: float = 1.0) -> None:
        with self._lock:
            self._dribble_on = bool(on)
            self._dribble_strength = float(strength)

    def GetYaw(self) -> float:
        return float(self._status.yaw)

    def GetPose(self) -> Tuple[float, float, float]:
        """(x_forward, y_left, yaw_deg)."""
        st = self._status
        return float(st.x), float(st.y), float(st.yaw)

    def HasBall(self) -> bool:
        return bool(self._status.has_ball)

    def GetStatus(self) -> XelStatus:
        return self._status

    # ------------------------- internals -------------------------
    def _build_cmd_packet(self) -> bytes:
        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFF
            seq = self._seq

            flags = 0
            if self._use_wheels:
                flags |= _CmdFlags.USE_WHEELS
            if self._dribble_on:
                flags |= _CmdFlags.DRIBBLE_ON
            if self._kick_pending:
                flags |= _CmdFlags.KICK

            vx, vy, wz = self._vx, self._vy, self._wz
            w1, w2, w3, w4 = self._wheels
            kick_strength = self._kick_strength
            dribble_strength = self._dribble_strength

            # kick is edge-triggered: send once then clear
            self._kick_pending = False

        return _CMD_STRUCT.pack(
            1,  # version
            1,  # type=command
            seq,
            self.robot_id,
            flags,
            self.status_port,
            vx,
            vy,
            wz,
            w1,
            w2,
            w3,
            w4,
            kick_strength,
            dribble_strength,
        )

    def _send_loop(self) -> None:
        next_t = time.time()
        while not self._stop_evt.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
            next_t = next_t + self.send_period
            try:
                pkt = self._build_cmd_packet()
                self._cmd_sock.sendto(pkt, self.unity_addr)
            except OSError:
                # socket closed
                break
            except Exception as e:
                self.logger.debug("send failed: %s", e)

    def _recv_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                data, _addr = self._status_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.logger.debug("recv failed: %s", e)
                continue

            if len(data) < _STATUS_STRUCT.size:
                continue
            try:
                (
                    ver,
                    typ,
                    seq,
                    rid,
                    flags,
                    _reserved,
                    x,
                    y,
                    yaw,
                    vx,
                    vy,
                    wz,
                ) = _STATUS_STRUCT.unpack_from(data)
            except Exception:
                continue
            if ver != 1 or typ != 2:
                continue
            if (rid & 0xFF) != self.robot_id:
                # ignore other robots
                continue

            self._status = XelStatus(
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                vx=float(vx),
                vy=float(vy),
                wz=float(wz),
                has_ball=bool(flags & _StatusFlags.HAS_BALL),
                seq=int(seq),
                last_update_ts=time.time(),
            )
