"""Team communication protocol wrapper for LunaPre."""

from __future__ import annotations

from ReasonData import logger, Settings

from . import runtime
from .strategy_tools import BALL_NOT_FOUND

cfg = runtime.cfg

class Communication:
    """蓝牙/Wi-Fi 队内通信封装。

    协议参考 `蓝牙数据协议.csv`：
    - 帧头: 0x23
    - 帧尾: 0x42
    - 字段顺序:
        [帧头, 识别ID, 角色, 状态, x+, y+, x-, y-, 备用, 帧尾]

    说明：
    - 这里保留了一个通用实现，既可以用于串口 BLE，也可以用于 Wi-Fi
      方案中的二次封装。
    - 对于坐标字段，默认采用“正负拆分”的方式：
        signed = positive - negative
      其中每个分量都会被限制到 0~0xFF。
    """

    FRAME_HEAD = 0x23
    FRAME_TAIL = 0x42
    FRAME_LENGTH = 10

    # 角色编码（对齐蓝牙数据协议.csv）
    ROLE_OFFENCE_YELLOW = 0x01
    ROLE_DEFENCE_YELLOW = 0x00
    ROLE_DEFENCE_BLUE = 0x02
    ROLE_OFFENCE_BLUE = 0x03
    ROLE_BAN = 0x04

    # 状态编码
    STATUS_CATCH_BALL = 0x00
    STATUS_CHASING = 0x01
    STATUS_AVOIDING = 0xEE
    STATUS_NO_BALL = 0xFF

    def __init__(self, *, bits=None, peer_id: int | None = None):
        """Pass simulator Bits and peer_id to carry LunaPre frames through Unity."""
        if (bits is None) != (peer_id is None):
            raise ValueError("bits and peer_id must be provided together")
        if peer_id is not None and (not isinstance(peer_id, int) or not 1 <= peer_id <= 255):
            raise ValueError("peer_id must be from 1 to 255")
        self._bits = bits
        self._peer_id = peer_id
        if bits is not None:
            self.transimission = None
        elif cfg.Transimission.Method == "WIFI":
            from utils.ReasonBeacon import MisakaNetwork
            self.transimission = MisakaNetwork()
        elif cfg.Transimission.Method == "BLE":
            import serial
            self.transimission = serial.Serial(getattr(Settings.Ports, "BLEPort", "/dev/ble"), 115200, timeout=0.1)
        else:
            raise RuntimeError("None Transmission Method Selected!")

        self.BallFlag = [0, 0]
        self.PeerPosition = list(BALL_NOT_FOUND)
        self.FullLog = cfg.Debug.FullLog
        self.RobotID = int(getattr(cfg.RoboInfo, "number", 1))
        self.Role = self._default_role_from_config()
        self.Status = self.STATUS_NO_BALL
        self.LastFrame = None
        self.LastPeerFrame = None

    def _default_role_from_config(self) -> int:
        robo_type = str(getattr(cfg.RoboInfo, "type", "OP")).upper()
        color = str(getattr(cfg.RoboInfo, "Color", getattr(cfg.RoboInfo, "color", "Y"))).upper()

        if robo_type in {"DP", "DEFENCE", "DEFENSE"}:
            return self.ROLE_DEFENCE_BLUE if color.startswith("B") else self.ROLE_DEFENCE_YELLOW
        return self.ROLE_OFFENCE_BLUE if color.startswith("B") else self.ROLE_OFFENCE_YELLOW

    @staticmethod
    def _clamp_byte(value) -> int:
        try:
            value = int(round(float(value)))
        except Exception:
            value = 0
        return max(0, min(0xFF, value))

    @staticmethod
    def _split_signed_xy(x, y) -> tuple[int, int, int, int]:
        """将带符号坐标拆成正负两路字节。"""
        try:
            x = float(x)
        except Exception:
            x = 0.0
        try:
            y = float(y)
        except Exception:
            y = 0.0

        x_pos = Communication._clamp_byte(max(x, 0.0))
        y_pos = Communication._clamp_byte(max(y, 0.0))
        x_neg = Communication._clamp_byte(max(-x, 0.0))
        y_neg = Communication._clamp_byte(max(-y, 0.0))
        return x_pos, y_pos, x_neg, y_neg

    @staticmethod
    def _merge_signed_xy(x_pos, y_pos, x_neg, y_neg) -> list[int]:
        return [int(x_pos) - int(x_neg), int(y_pos) - int(y_neg)]

    def BuildFrame(self, identity_id: int | None = None, role: int | None = None, status: int | None = None,
                   x_pos: int = 0, y_pos: int = 0, x_neg: int = 0, y_neg: int = 0, reserve: int = 0) -> bytes:
        """构造一帧通信报文。"""
        frame = bytes([
            self.FRAME_HEAD,
            self._clamp_byte(self.RobotID if identity_id is None else identity_id),
            self._clamp_byte(self.Role if role is None else role),
            self._clamp_byte(self.Status if status is None else status),
            self._clamp_byte(x_pos),
            self._clamp_byte(y_pos),
            self._clamp_byte(x_neg),
            self._clamp_byte(y_neg),
            self._clamp_byte(reserve),
            self.FRAME_TAIL,
        ])
        self.LastFrame = frame
        return frame

    def ParseFrame(self, payload: bytes | bytearray | list[int]):
        """解析协议帧，返回字典；非法帧返回 None。"""
        if payload is None:
            return None
        if isinstance(payload, list):
            payload = bytes(payload)
        if not isinstance(payload, (bytes, bytearray)):
            return None
        if len(payload) != self.FRAME_LENGTH:
            return None
        if payload[0] != self.FRAME_HEAD or payload[-1] != self.FRAME_TAIL:
            return None

        identity_id = int(payload[1])
        role = int(payload[2])
        status = int(payload[3])
        x_pos, y_pos, x_neg, y_neg, reserve = map(int, payload[4:9])
        signed_xy = self._merge_signed_xy(x_pos, y_pos, x_neg, y_neg)
        data = {
            "id": identity_id,
            "role": role,
            "status": status,
            "x_pos": x_pos,
            "y_pos": y_pos,
            "x_neg": x_neg,
            "y_neg": y_neg,
            "reserve": reserve,
            "signed_xy": signed_xy,
        }
        self.LastPeerFrame = data
        self.PeerPosition = signed_xy
        self.BallFlag = [status, reserve]
        return data

    def UpdateState(self, *, role: int | None = None, status: int | None = None,
                    ball_flag: list[int] | None = None, peer_position: list[int] | None = None):
        """更新本机待发送状态。"""
        if role is not None:
            self.Role = self._clamp_byte(role)
        if status is not None:
            self.Status = self._clamp_byte(status)
        if ball_flag is not None and len(ball_flag) >= 2:
            self.BallFlag = [self._clamp_byte(ball_flag[0]), self._clamp_byte(ball_flag[1])]
        if peer_position is not None and len(peer_position) >= 2:
            self.PeerPosition = [int(peer_position[0]), int(peer_position[1])]

    def PackSelfStatus(self, *, identity_id: int | None = None, role: int | None = None,
                       status: int | None = None, position: list[int] | tuple[int, int] | None = None,
                       reserve: int = 0) -> bytes:
        """根据当前状态和位置打包一帧。"""
        if position is None:
            position = self.PeerPosition
        if isinstance(position, (list, tuple)) and len(position) >= 2:
            x_pos, y_pos, x_neg, y_neg = self._split_signed_xy(position[0], position[1])
        else:
            x_pos, y_pos, x_neg, y_neg = 0, 0, 0, 0
        return self.BuildFrame(
            identity_id=identity_id,
            role=role,
            status=status,
            x_pos=x_pos,
            y_pos=y_pos,
            x_neg=x_neg,
            y_neg=y_neg,
            reserve=reserve,
        )

    def Send(self, payload: bytes | bytearray | list[int] | None = None) -> bool:
        """发送原始帧或当前状态帧。"""
        if payload is None:
            payload = self.PackSelfStatus()

        try:
            if self._bits is not None:
                self._bits.SendMessage(self._peer_id, bytes(payload))
            elif hasattr(self.transimission, "send") and callable(getattr(self.transimission, "send")):
                self.transimission.send(payload)
            elif hasattr(self.transimission, "sendall") and callable(getattr(self.transimission, "sendall")):
                self.transimission.sendall(payload)
            elif hasattr(self.transimission, "write") and callable(getattr(self.transimission, "write")):
                self.transimission.write(payload)
            else:
                raise AttributeError("transimission object has no send/write interface")
            return True
        except Exception as e:
            if self.FullLog:
                logger.warning(f"Communication send failed: {e}")
            return False

    def Receive(self):
        """尝试接收一帧并解析。"""
        try:
            raw = None
            if self._bits is not None:
                message = self._bits.ReceiveMessage()
                if message is None or message[0] != self._peer_id:
                    return None
                raw = message[1]
            elif hasattr(self.transimission, "recv") and callable(getattr(self.transimission, "recv")):
                raw = self.transimission.recv(self.FRAME_LENGTH)
            elif hasattr(self.transimission, "read") and callable(getattr(self.transimission, "read")):
                raw = self.transimission.read(self.FRAME_LENGTH)
            elif hasattr(self.transimission, "receive") and callable(getattr(self.transimission, "receive")):
                raw = self.transimission.receive()
            else:
                raise AttributeError("transimission object has no recv/read/receive interface")

            if not raw:
                return None

            if isinstance(raw, tuple):
                # 某些实现会返回 (addr, data)
                raw = raw[-1]

            if isinstance(raw, list):
                raw = bytes(raw)

            if isinstance(raw, str):
                raw = raw.encode("latin1", errors="ignore")

            # 兼容一次读到多帧的场景，尽量从中截取第一帧合法数据。
            if isinstance(raw, (bytes, bytearray)) and len(raw) >= self.FRAME_LENGTH:
                for idx in range(0, len(raw) - self.FRAME_LENGTH + 1):
                    chunk = raw[idx: idx + self.FRAME_LENGTH]
                    parsed = self.ParseFrame(chunk)
                    if parsed is not None:
                        return parsed
            return self.ParseFrame(raw)
        except Exception as e:
            if self.FullLog:
                logger.warning(f"Communication receive failed: {e}")
            return None

    def SendStatus(self, *, status: int | None = None, role: int | None = None,
                   position: list[int] | tuple[int, int] | None = None, reserve: int = 0) -> bool:
        """发送当前机器人状态。"""
        payload = self.PackSelfStatus(role=role, status=status, position=position, reserve=reserve)
        return self.Send(payload)

    def SendBallFlag(self, ball_flag: list[int] | None = None, *, reserve: int = 0) -> bool:
        """便捷接口：发送球状态（兼容旧变量命名）。"""
        if ball_flag is not None and len(ball_flag) >= 2:
            self.BallFlag = [self._clamp_byte(ball_flag[0]), self._clamp_byte(ball_flag[1])]
        return self.SendStatus(status=self.Status, position=self.PeerPosition, reserve=reserve)

    def Close(self):
        """关闭通信句柄。"""
        try:
            if hasattr(self.transimission, "close") and callable(getattr(self.transimission, "close")):
                self.transimission.close()
        except Exception as e:
            if self.FullLog:
                logger.warning(f"Communication close failed: {e}")

    # Backward-compatible aliases
    def SendMessage(self, *args, **kwargs):
        return self.Send(*args, **kwargs)

    def RecvMessage(self):
        return self.Receive()

    def SetBallFlag(self, ball_flag):
        self.BallFlag = [self._clamp_byte(ball_flag[0]), self._clamp_byte(ball_flag[1])]
        return self.BallFlag

    def SetPeerPosition(self, peer_position):
        self.PeerPosition = [int(peer_position[0]), int(peer_position[1])]
        return self.PeerPosition
