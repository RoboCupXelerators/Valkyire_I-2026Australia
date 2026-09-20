import logging
import math
from typing import Optional, List
from enum import IntEnum
import struct
import time
import threading

try:
    import serial
    from serial.serialutil import SerialException
except Exception:  # pragma: no cover - import error handled at runtime
    serial = None
    SerialException = Exception


class Commands(IntEnum):
    HAND_SHAKE             = 0x01
    SET_ROOM_SIZE          = 0x02
    GET_POSE               = 0x03
    SET_WHEELS_SPEED       = 0x04
    KICK                   = 0x05
    SET_DRIBBLE_SPEED      = 0x06
    GET_REFEREE_UNLOCK_PIN = 0x07
    MORSE_PLAY             = 0x08
    LINK_SEND              = 0x09
    LINK_GET               = 0x0A
    GET_DRIBBLE_CURRENT    = 0x0B
    GET_BATTERY_VOLTAGE    = 0x0C


COMMAND_RESPONSE_LENGTHS = {
    Commands.HAND_SHAKE:             1,
    Commands.SET_ROOM_SIZE:          1,
    Commands.GET_POSE:               1 + 6 * 4,
    Commands.SET_WHEELS_SPEED:       1,
    Commands.KICK:                   1,
    Commands.SET_DRIBBLE_SPEED:      1,
    Commands.GET_REFEREE_UNLOCK_PIN: 1 + 1,
    Commands.MORSE_PLAY:             1,
    Commands.LINK_SEND:              1,
    Commands.LINK_GET:               None,
    Commands.GET_DRIBBLE_CURRENT:    1 + 4,
    Commands.GET_BATTERY_VOLTAGE:    1 + 4,
}

MAX_DRIBBLE_INPUT = 1.0
MIN_KICK_POWER_LEVEL = 1
MAX_KICK_POWER_LEVEL = 4
DEFAULT_KICK_POWER_LEVEL = 4
KICK_POWER_LEVEL_ERROR = "kick power level must be an integer from 1 to 4"
ROOM_SIZE_ERROR = (
    "room size must be positive finite millimetres in horizontal, vertical order"
)
MM_PER_CM = 10.0
MAX_LINK_PAYLOAD = 224
LINK_GET_HEADER_LENGTH = 2
LINK_PAYLOAD_HEADER_LENGTH = 1
LINK_SEND_HEADER_LENGTH = 3
LINK_PAYLOAD_FLAG_POSE = 0x01
LINK_POSE_LENGTH = 6 * 4
MAX_LINK_SEND_PAYLOAD = MAX_LINK_PAYLOAD - LINK_SEND_HEADER_LENGTH
MAX_LINK_POSE_PAYLOAD = MAX_LINK_PAYLOAD - LINK_PAYLOAD_HEADER_LENGTH - LINK_POSE_LENGTH
LINK_SEND_ARGUMENT_ERROR = (
    "LINK_SEND requires one bytes-like payload and an optional include_pose flag"
)


class IceLoongBits:
    """串口通信封装类，用于与运动控制器交互。
    方法命名使用大驼峰（PascalCase）。
    
    支持串口异常自动恢复：
    - 当串口 I/O 错误发生时，自动关闭并重新打开串口
    - 后台命令线程会在异常后继续运行
    - 提供线程监控和重启机制
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, timeout: float = 0.05, testing = False):
        """初始化串口参数（不自动打开）。

        Args:
            port: 串口设备路径，例如 "/dev/ttyUSB0" 或 "COM3"。
            baudrate: 波特率，例如 115200。
            timeout: 读超时（秒）。
            testing: 测试模式，不启动后台线程。
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.CommandLine = {
            'HAND_SHAKE': True,
            'GET_POSE': True,
            'SET_WHEELS_SPEED': (0,0,0,0),
            'SET_DRIBBLE_SPEED': 0.0,
            'GET_REFEREE_UNLOCK_PIN': True,
            'GET_DRIBBLE_CURRENT': True,
            'GET_BATTERY_VOLTAGE': True,
            'MORSE_PLAY': None,
            'LINK_SEND': None,
        }

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.position_mm = None
        self.pose = None
        self.referee_key = False
        self.dribble_current = None
        self.battery_voltage = None
        self.link_data = None
        self.peer_pose = None
        self.peer_position_mm = None
        self.kick_power_level = DEFAULT_KICK_POWER_LEVEL
        self.motor_speed_k = 1.3
        # 串口句柄（运行时为 serial.Serial 实例）
        self.ser = None
        self._io_lock = threading.RLock()
        self._command_lock = threading.Lock()
        self._reconnect_lock = threading.Lock()
        self._consecutive_errors = 0
        self._max_consecutive_errors = 5
        self._reconnect_delay = 0.3  # 重连间隔秒数
        self._thread_restart_count = 0
        self._lifecycle_lock = threading.RLock()
        self._closed = False

        # 命令循环节拍与降频轮询（2026-09-19 实机诊断后引入）：
        # - command_period_s: 设计控制周期 50Hz；
        # - 慢变量（盘球电流/电池电压）每 slow_poll_interval_s 随 GET 组查一次；
        # - LINK_GET 当前无消费者，按 link_poll_interval_s 降频。
        self.command_period_s = 0.02
        self.slow_poll_interval_s = 1.0
        self.link_poll_interval_s = 0.5
        self._last_slow_poll_t = 0.0
        self._last_link_poll_t = 0.0
        
        self.logger = logging.getLogger(self.__class__.__name__)
        if not self.logger.handlers:
            self.logger.addHandler(logging.NullHandler())

        self._stop_thread = threading.Event()
        if not testing:
            self.SendUnitedCommandThread = threading.Thread(
                target=self._SendUnitedCommandLoop, 
                daemon=True,
                name="IceLoongBits-CommandThread"
            )
            self.SendUnitedCommandThread.start()

    def _SendUnitedCommandLoop(self):
        """后台线程主循环，包含异常恢复机制。"""
        while not self._stop_thread.is_set():
            cycle_start = time.monotonic()
            try:
                self._SendUnitedCommandOnce()
                # 成功执行后重置错误计数
                self._consecutive_errors = 0
            except SerialException as e:
                self._consecutive_errors += 1
                self.logger.warning(
                    f"Serial I/O error in command loop (attempt {self._consecutive_errors}/"
                    f"{self._max_consecutive_errors}): {e}"
                )
                # 尝试串口重连
                self._HandleSerialError(e)
            except Exception as e:
                self.logger.exception(f"IceLoongBits command loop failed: {e}")
            # 自适应节拍：事务耗时不与固定 sleep 叠加。实测固件对每条命令
            # 的应答约 11-20ms，若每周期固定再睡 20ms，会把 50Hz 设计节拍
            # 拖到 ~9Hz（2026-09-19 实机测量）。
            elapsed = time.monotonic() - cycle_start
            self._stop_thread.wait(max(0.0, self.command_period_s - elapsed))

    def _HandleSerialError(self, error: Exception) -> bool:
        """处理串口错误，尝试重连。
        
        Args:
            error: 捕获到的异常
            
        Returns:
            bool: 重连是否成功
        """
        with self._reconnect_lock:
            if self._closed or self._stop_thread.is_set():
                return False
            self.logger.info(f"Attempting serial reconnection due to error: {error}")
            
            # 先关闭现有连接
            self._ForceClosePort()
            
            # 等待一段时间后重连
            if self._stop_thread.wait(self._reconnect_delay):
                return False
            
            try:
                self.OpenPort()
                self.logger.info(f"Serial port reconnected successfully: {self.port}")
                return True
            except Exception as e:
                self.logger.error(f"Failed to reconnect serial port: {e}")
                return False

    def _ForceClosePort(self) -> None:
        """强制关闭串口，忽略所有错误。"""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            finally:
                self.ser = None

    def IsThreadAlive(self) -> bool:
        """检查后台命令线程是否存活。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        return thread is not None and thread.is_alive()

    def RestartThreadIfNeeded(self) -> bool:
        """如果后台线程已死，尝试重启。
        
        Returns:
            bool: 线程是否存活（重启成功或本来就存活）
        """
        with self._lifecycle_lock:
            if self._closed:
                return False
            if self.IsThreadAlive():
                return True
            
        self.logger.warning("Background command thread is dead, attempting restart...")
        
        # 清理旧线程状态
        if hasattr(self, "_stop_thread"):
            self._stop_thread.set()
        
        # 创建新线程
        with self._lifecycle_lock:
            if self._closed:
                return False
            self._stop_thread = threading.Event()
            self.SendUnitedCommandThread = threading.Thread(
                target=self._SendUnitedCommandLoop,
                daemon=True,
                name=f"IceLoongBits-CommandThread-Restart-{self._thread_restart_count}"
            )
            self._thread_restart_count += 1
            self.SendUnitedCommandThread.start()
        
        # 给线程一点时间启动
        time.sleep(0.1)
        
        if self.IsThreadAlive():
            self.logger.info("Background command thread restarted successfully")
            return True
        else:
            self.logger.error("Failed to restart background command thread")
            return False

    def _CommandSnapshot(self):
        with self._command_lock:
            snapshot = {
                "HAND_SHAKE": self.CommandLine["HAND_SHAKE"],
                "GET_POSE": self.CommandLine["GET_POSE"],
                "GET_REFEREE_UNLOCK_PIN": self.CommandLine["GET_REFEREE_UNLOCK_PIN"],
                "GET_DRIBBLE_CURRENT": self.CommandLine["GET_DRIBBLE_CURRENT"],
                "GET_BATTERY_VOLTAGE": self.CommandLine["GET_BATTERY_VOLTAGE"],
                "SET_WHEELS_SPEED": tuple(self.CommandLine["SET_WHEELS_SPEED"]),
                "SET_DRIBBLE_SPEED": self.CommandLine["SET_DRIBBLE_SPEED"],
                "MORSE_PLAY": self.CommandLine["MORSE_PLAY"],
                "LINK_SEND": self.CommandLine["LINK_SEND"],
            }
            self.CommandLine["MORSE_PLAY"] = None
            self.CommandLine["LINK_SEND"] = None
        return snapshot

    def _SendUnitedCommandOnce(self):
        snapshot = self._CommandSnapshot()

        Speed1, Speed2, Speed3, Speed4 = snapshot["SET_WHEELS_SPEED"]
        Dribble = snapshot["SET_DRIBBLE_SPEED"]
        # 控制输出与高频状态查询共用一个固件复合帧。这样每个 20 ms 周期
        # 只承担一次写/读往返，同时仍逐条校验固件应答。
        command_list = [
            (Commands.SET_WHEELS_SPEED, Speed1, Speed2, Speed3, Speed4),
            (Commands.SET_DRIBBLE_SPEED, Dribble),
        ]

        if snapshot["HAND_SHAKE"]:
            command_list.append((Commands.HAND_SHAKE,))
        if snapshot["GET_POSE"]:
            command_list.append((Commands.GET_POSE,))
        if snapshot["GET_REFEREE_UNLOCK_PIN"]:
            command_list.append((Commands.GET_REFEREE_UNLOCK_PIN,))

        now = time.monotonic()
        if now - self._last_slow_poll_t >= self.slow_poll_interval_s:
            self._last_slow_poll_t = now
            if snapshot["GET_DRIBBLE_CURRENT"]:
                command_list.append((Commands.GET_DRIBBLE_CURRENT,))
            if snapshot["GET_BATTERY_VOLTAGE"]:
                command_list.append((Commands.GET_BATTERY_VOLTAGE,))

        response = self.ExecuteCommands(command_list)
        self._ApplyResponses(response)

        if snapshot["MORSE_PLAY"] is not None:
            self.ExecuteCommands([(Commands.MORSE_PLAY, str(snapshot["MORSE_PLAY"]))])

        if snapshot["LINK_SEND"] is not None:
            payload, include_pose = self._ParseLinkSend(snapshot["LINK_SEND"])
            self.ExecuteCommands([(Commands.LINK_SEND, payload, include_pose)])

        # LINK_GET 目前没有策略层消费者；按 link_poll_interval_s 降频，
        # 避免每周期一次 ~50ms 超时等待。有队友通信需求时再调低间隔。
        if now - self._last_link_poll_t >= self.link_poll_interval_s:
            self._last_link_poll_t = now
            response = self.ExecuteCommands([(Commands.LINK_GET,)])
            self._ApplyResponses(response)

    def _ApplyResponses(self, response):
        for i in response:
            match i[0] ^ 0x80:
                case 0x00:
                    continue
                case Commands.HAND_SHAKE:
                    pass
                case Commands.SET_ROOM_SIZE:
                    pass
                case Commands.GET_POSE:
                    _, yaw, pitch, roll, x_mm, y_mm, z_mm = i
                    self.yaw = yaw % 360
                    self.pitch = pitch
                    self.roll = roll
                    if all(math.isfinite(value) for value in (x_mm, y_mm, z_mm)):
                        self.position_mm = (x_mm, y_mm, z_mm)
                    else:
                        self.position_mm = None
                    if self.position_mm is not None and math.isfinite(yaw):
                        self.pose = (x_mm / MM_PER_CM, y_mm / MM_PER_CM, self.yaw)
                    else:
                        self.pose = None
                case Commands.SET_WHEELS_SPEED:
                    pass
                case Commands.KICK:
                    pass
                case Commands.SET_DRIBBLE_SPEED:
                    pass
                case Commands.MORSE_PLAY:
                    pass
                case Commands.GET_REFEREE_UNLOCK_PIN:
                    _, referee_key = i
                    self.referee_key = bool(referee_key)
                case Commands.GET_DRIBBLE_CURRENT:
                    _, dribble_current = i
                    self.dribble_current = dribble_current
                case Commands.GET_BATTERY_VOLTAGE:
                    _, battery_voltage = i
                    self.battery_voltage = battery_voltage
                case Commands.LINK_GET:
                    self._ApplyLinkPayload(i[1])

    def OpenPort(self) -> None:
        """打开串口（如果尚未打开）。"""
        if self._closed:
            raise RuntimeError("IceLoongBits is closed")
        if serial is None:
            raise RuntimeError("pyserial 未安装，请运行: pip install pyserial")
        if self.ser and getattr(self.ser, "is_open", False):
            return
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def ClosePort(self) -> None:
        """关闭串口并释放句柄。"""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                self.logger.exception("关闭串口失败")
            finally:
                self.ser = None

    def _ReadUntilTimeout(self) -> bytes:
        """辅助：读取直到超时（按字节读取）。"""
        buf = bytearray()
        while True:
            b = self.ser.read(1)
            if not b:
                break
            buf.extend(b)
        return bytes(buf)

    def SendHex(self, data: bytes, read_response: bool = False, response_length: int = 0, response_timeout: Optional[float] = None) -> bytes:
        """发送原始字节命令到运动控制器。

        Args:
            data: 需要发送的原始字节数据。
            read_response: 是否在发送后立即读取响应。
            response_length: 如果大于 0，按长度读取；否则读取直到超时。
            response_timeout: 单次读取的临时超时（秒），若 None 则使用初始化的 timeout。

        Returns:
            bytes: 读取到的响应（如果 read_response=False 返回 b''）。
            
        Raises:
            SerialException: 当串口 I/O 错误时抛出，调用者应处理重连逻辑。
        """
        with self._io_lock:
            if serial is None:
                raise RuntimeError("pyserial 未安装，请运行: pip install pyserial")
            if not self.ser or not getattr(self.ser, "is_open", False):
                self.OpenPort()

            # 清空输入缓冲，避免读到旧数据
            try:
                self.ser.reset_input_buffer()
            except Exception:
                # 有些 serial 实现可能没有该方法
                pass

            try:
                self.ser.write(data)
                self.ser.flush()
            except SerialException:
                # 串口写入错误，重新抛出让上层处理
                raise
            except OSError as e:
                # 转换 OSError 为 SerialException 统一处理
                raise SerialException(f"Serial write failed: {e}") from e

            if not read_response:
                return b""

            # 暂时调整 timeout（若提供），发送后立即读取
            old_timeout = getattr(self.ser, "timeout", None)
            if response_timeout is not None:
                self.ser.timeout = response_timeout

            try:
                if response_length > 0:
                    resp = self.ser.read(response_length)
                else:
                    # 读取直到超时
                    resp = self._ReadUntilTimeout()
                return resp
            except SerialException:
                raise
            except OSError as e:
                raise SerialException(f"Serial read failed: {e}") from e
            finally:
                if response_timeout is not None:
                    self.ser.timeout = old_timeout

    def _SendVariableHex(self, data: bytes, response_timeout: float = 0.05) -> bytes:
        """发送命令并读取 ``response-id + u16 length + payload`` 响应。"""
        with self._io_lock:
            if serial is None:
                raise RuntimeError("pyserial 未安装，请运行: pip install pyserial")
            if not self.ser or not getattr(self.ser, "is_open", False):
                self.OpenPort()

            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass

            try:
                self.ser.write(data)
                self.ser.flush()
            except SerialException:
                raise
            except OSError as e:
                raise SerialException(f"Serial write failed: {e}") from e

            old_timeout = getattr(self.ser, "timeout", None)
            self.ser.timeout = response_timeout
            try:
                header = self.ser.read(LINK_GET_HEADER_LENGTH)
                if len(header) != LINK_GET_HEADER_LENGTH:
                    return b""
                payload_len = header[1]
                if header[0] != (Commands.LINK_GET | 0x80) or payload_len > MAX_LINK_PAYLOAD:
                    return b""
                payload = self.ser.read(payload_len)
                if len(payload) != payload_len:
                    return b""
                return header + payload
            except SerialException:
                raise
            except OSError as e:
                raise SerialException(f"Serial read failed: {e}") from e
            finally:
                self.ser.timeout = old_timeout

    def _ExecuteLinkGet(self) -> list[tuple]:
        try:
            response = self._SendVariableHex(bytes([Commands.LINK_GET]), 0.05)
        except SerialException as e:
            self.logger.warning(f"LINK_GET failed due to serial error: {e}")
            self._HandleSerialError(e)
            return []
        except Exception as e:
            self.logger.exception(f"LINK_GET failed with unexpected error: {e}")
            return []

        if len(response) < LINK_GET_HEADER_LENGTH or response[0] != (Commands.LINK_GET | 0x80):
            return []
        payload_len = response[1]
        if len(response) != LINK_GET_HEADER_LENGTH + payload_len:
            return []
        return [(response[0], bytes(response[2:]))]

    def __enter__(self):
        self.OpenPort()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def ExecuteCommands(self, commands: List[tuple]) -> list[tuple]:
        """执行单条或多条命令
        
        Args:
            commands: 命令列表
            
        Returns:
            list[tuple]: 响应数据列表，失败时返回空列表
            
        Note:
            此方法会捕获串口异常并尝试重连，不会向调用者抛出异常。
        """
        if any(cmd[0] == Commands.LINK_GET for cmd in commands):
            if len(commands) != 1:
                raise ValueError("LINK_GET must be sent as a standalone command")
            return self._ExecuteLinkGet()

        if len(commands) > 1:
            tx_bytes, resp_len = bytes([0x00]), 1
        else:
            tx_bytes, resp_len = bytes(), 0

        for cmd in commands:
            match cmd[0]:
                case Commands.HAND_SHAKE:
                    tx_bytes += bytes([cmd[0]])
                case Commands.SET_ROOM_SIZE:
                    if len(cmd) != 3:
                        raise ValueError(ROOM_SIZE_ERROR)
                    horizontal_mm = self._ValidateRoomDimensionMm(cmd[1])
                    vertical_mm = self._ValidateRoomDimensionMm(cmd[2])
                    # Wire order is fixed: horizontal first, vertical second.
                    tx_bytes += bytes([cmd[0]]) + struct.pack(
                        '<ff', horizontal_mm, vertical_mm
                    )
                case Commands.GET_POSE:
                    tx_bytes += bytes([cmd[0]])
                case Commands.SET_WHEELS_SPEED:
                    tx_bytes += bytes([cmd[0]]) + struct.pack('<ffff', cmd[1], cmd[2], cmd[3], cmd[4])
                case Commands.KICK:
                    if len(cmd) != 2:
                        raise ValueError(KICK_POWER_LEVEL_ERROR)
                    tx_bytes += bytes([cmd[0], self._ValidateKickPowerLevel(cmd[1])])
                case Commands.SET_DRIBBLE_SPEED:
                    tx_bytes += bytes([cmd[0]]) + struct.pack(
                        '<f', self._NormalizeDribbleInput(cmd[1])
                    )
                case Commands.GET_REFEREE_UNLOCK_PIN:
                    tx_bytes += bytes([cmd[0]])
                case Commands.GET_DRIBBLE_CURRENT:
                    tx_bytes += bytes([cmd[0]])
                case Commands.GET_BATTERY_VOLTAGE:
                    tx_bytes += bytes([cmd[0]])
                case Commands.MORSE_PLAY:
                    msg = str(cmd[1]) if len(cmd) > 1 else ""
                    raw = msg.encode('ascii', errors='replace')[:23]
                    if len(raw) < 23:
                        raw += b'\x00'
                    length = len(raw)
                    tx_bytes += bytes([cmd[0], length]) + raw
                case Commands.LINK_SEND:
                    if len(cmd) not in (2, 3):
                        raise ValueError(LINK_SEND_ARGUMENT_ERROR)
                    include_pose = bool(cmd[2]) if len(cmd) == 3 else False
                    payload = self._NormalizeLinkPayload(cmd[1], include_pose)
                    tx_bytes += bytes([
                        cmd[0], LINK_PAYLOAD_FLAG_POSE if include_pose else 0,
                        len(payload)
                    ]) + payload
                case _:
                    continue
            response_length = COMMAND_RESPONSE_LENGTHS.get(cmd[0], 0)
            if response_length is not None:
                resp_len += response_length

        if len(commands) > 1:
            tx_bytes += bytes([0])
            resp_len += 1

        # 发送帧，返回接收帧（带异常处理）
        try:
            resp = self.SendHex(tx_bytes, True, resp_len, 0.05)
        except SerialException as e:
            self.logger.warning(f"ExecuteCommands failed due to serial error: {e}")
            self._HandleSerialError(e)
            return []
        except Exception as e:
            self.logger.exception(f"ExecuteCommands failed with unexpected error: {e}")
            return []

        rx_data = []
        if resp:
            # 短读容忍：应答超时会得到不完整字节流（实测约 10% 的周期）。
            # 逐记录按 id 解析，尾部被截断的记录整体丢弃，之前已对齐的
            # 记录（尤其是 GET_POSE）仍然生效；未知 id 立即停止。
            i = 0
            n = len(resp)
            while i < n:
                record_id = resp[i] ^ 0x80
                record_length = COMMAND_RESPONSE_LENGTHS.get(record_id, 1)
                if (
                    record_length is not None
                    and record_id != 0x00
                    and i + record_length > n
                ):
                    break
                match resp[i] ^ 0x80:
                    case 0x00:
                        i += 1
                        continue
                    case Commands.HAND_SHAKE:
                        rx_data.append((resp[i],))
                    case Commands.SET_ROOM_SIZE:
                        rx_data.append((resp[i],))
                    case Commands.GET_POSE:
                        rx_data.append((resp[i], *struct.unpack('<ffffff', resp[i+1:i+25])))
                    case Commands.SET_WHEELS_SPEED:
                        rx_data.append((resp[i],))
                    case Commands.KICK:
                        rx_data.append((resp[i],))
                    case Commands.SET_DRIBBLE_SPEED:
                        rx_data.append((resp[i],))
                    case Commands.GET_REFEREE_UNLOCK_PIN:
                        rx_data.append((resp[i], resp[i+1]))
                    case Commands.GET_DRIBBLE_CURRENT:
                        rx_data.append((resp[i], *struct.unpack('<f', resp[i+1:i+5])))
                    case Commands.GET_BATTERY_VOLTAGE:
                        rx_data.append((resp[i], *struct.unpack('<f', resp[i+1:i+5])))
                    case Commands.MORSE_PLAY:
                        rx_data.append((resp[i],))
                    case Commands.LINK_SEND:
                        rx_data.append((resp[i],))
                    case _:
                        break
                response_length = COMMAND_RESPONSE_LENGTHS.get(resp[i] ^ 0x80, 1)
                i += response_length if response_length is not None else 1
        return rx_data

    def GetYaw(self) -> float:
        """获取当前偏航角（度）。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            try:
                self._RefreshPose()
            except Exception as e:
                self.logger.debug(f"Refresh yaw failed: {e}")
        return self.yaw

    def GetPose(self):
        """返回融合场地位姿 ``(x_cm, y_cm, yaw_deg)``，无效时返回 ``None``。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            try:
                self._RefreshPose()
            except Exception as e:
                self.logger.debug(f"Refresh pose failed: {e}")
        return self.pose

    def _RefreshPose(self):
        response = self.ExecuteCommands([(Commands.GET_POSE,)])
        self._ApplyResponses(response)
        return self.pose
    
    def VirtualPorts(self, port, status):
        """设置虚拟端口状态（带异常保护）。
        
        此方法会捕获所有串口异常，确保不会阻塞调用者。
        """
        try:
            port = int(port)
        except Exception:
            pass

        try:
            match port:
                case 3:
                    if not status:
                        self.Kick()
                case 4:
                    self._SetDribbleSpeedNow(status)
        except SerialException as e:
            self.logger.warning(f"VirtualPorts operation failed due to serial error: {e}")
            self._HandleSerialError(e)
        except Exception as e:
            self.logger.exception(f"VirtualPorts operation failed: {e}")

    @staticmethod
    def _NormalizeDribbleInput(value):
        try:
            speed = float(value)
        except Exception:
            return 0.0
        if speed < 0.0:
            return 0.0
        if speed > MAX_DRIBBLE_INPUT:
            return MAX_DRIBBLE_INPUT
        return speed

    @staticmethod
    def _ValidateKickPowerLevel(value):
        if isinstance(value, bool):
            raise ValueError(KICK_POWER_LEVEL_ERROR)
        try:
            numeric_level = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(KICK_POWER_LEVEL_ERROR) from exc
        if not math.isfinite(numeric_level) or not numeric_level.is_integer():
            raise ValueError(KICK_POWER_LEVEL_ERROR)
        level = int(numeric_level)
        if not MIN_KICK_POWER_LEVEL <= level <= MAX_KICK_POWER_LEVEL:
            raise ValueError(KICK_POWER_LEVEL_ERROR)
        return level

    @staticmethod
    def _ValidateRoomDimensionMm(value):
        if isinstance(value, bool):
            raise ValueError(ROOM_SIZE_ERROR)
        try:
            dimension_mm = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(ROOM_SIZE_ERROR) from exc
        if not math.isfinite(dimension_mm) or dimension_mm <= 0:
            raise ValueError(ROOM_SIZE_ERROR)
        return dimension_mm

    def SetRoomSize(self, horizontal_mm, vertical_mm):
        """设置场地尺寸。

        重要：协议顺序固定为 ``horizontal_mm, vertical_mm``，即水平在前、
        竖直在后。两者传反会导致下位机定位算法无法正常工作。
        """
        horizontal_mm = self._ValidateRoomDimensionMm(horizontal_mm)
        vertical_mm = self._ValidateRoomDimensionMm(vertical_mm)
        response = self.ExecuteCommands([
            (Commands.SET_ROOM_SIZE, horizontal_mm, vertical_mm),
        ])
        self._ApplyResponses(response)
        return response

    def SetKickPowerLevel(self, power_level):
        """设置后续 ``Kick()`` 和虚拟端口弹射使用的默认力度档位。"""
        level = self._ValidateKickPowerLevel(power_level)
        with self._command_lock:
            self.kick_power_level = level
        return level

    def Kick(self, power_level=None):
        """立即按 1 到 4 档力度弹射；省略参数时使用已设置的默认档位。"""
        if power_level is None:
            with self._command_lock:
                power_level = self.kick_power_level
        return self._KickNow(power_level)

    def _KickNow(self, power_level):
        level = self._ValidateKickPowerLevel(power_level)
        with self._command_lock:
            Dribble = self.CommandLine["SET_DRIBBLE_SPEED"]
        response = self.ExecuteCommands([
            (Commands.KICK, level),
            (Commands.SET_DRIBBLE_SPEED, Dribble),
        ])
        self._ApplyResponses(response)
        return response

    def _SetDribbleSpeedNow(self, value):
        speed = self._NormalizeDribbleInput(value)
        with self._command_lock:
            self.CommandLine["SET_DRIBBLE_SPEED"] = speed

        response = self.ExecuteCommands([(Commands.SET_DRIBBLE_SPEED, speed)])
        self._ApplyResponses(response)
        return response

    @staticmethod
    def _NormalizeLinkPayload(value, include_pose: bool = False) -> bytes:
        if isinstance(value, memoryview):
            value = value.tobytes()
        elif isinstance(value, bytearray):
            value = bytes(value)
        elif isinstance(value, bytes):
            pass
        else:
            raise ValueError("LINK payload must be bytes-like")
        limit = MAX_LINK_POSE_PAYLOAD if include_pose else MAX_LINK_SEND_PAYLOAD
        minimum = 0 if include_pose else 1
        if len(value) < minimum or len(value) > limit:
            raise ValueError(
                f"LINK payload length must be from {minimum} to {limit} bytes"
            )
        return value

    @staticmethod
    def _ParseLinkSend(pending):
        if isinstance(pending, tuple):
            payload, include_pose = pending
            return payload, bool(include_pose)
        return pending, False

    def SendLinkData(self, payload, include_pose: bool = False) -> bool:
        """保存一份待发送的最新链路 Payload，include_pose 为真时附带本机位姿并允许空 Payload。"""
        payload = self._NormalizeLinkPayload(payload, include_pose)
        with self._command_lock:
            self.CommandLine["LINK_SEND"] = (payload, bool(include_pose))
        return True

    def GetLinkData(self):
        """返回最近一次从另一台机器收到的 Payload，对方只发了位姿或尚未收到时返回 ``None``。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            response = self.ExecuteCommands([(Commands.LINK_GET,)])
            self._ApplyResponses(response)
        return self.link_data

    def GetPeerPose(self):
        """返回对方机器人位姿 ``(x_cm, y_cm, yaw_deg)``，最近一条消息未附带位姿时返回 ``None``。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            response = self.ExecuteCommands([(Commands.LINK_GET,)])
            self._ApplyResponses(response)
        return self.peer_pose

    def GetPeerYaw(self):
        """返回对方机器人偏航角（度），最近一条消息未附带位姿时返回 ``None``。"""
        pose = self.GetPeerPose()
        return None if pose is None else pose[2]

    def _ApplyLinkPayload(self, raw) -> None:
        raw = bytes(raw)
        self.link_data = None
        self.peer_pose = None
        self.peer_position_mm = None
        if not raw:
            return
        offset = LINK_PAYLOAD_HEADER_LENGTH
        if raw[0] & LINK_PAYLOAD_FLAG_POSE:
            if len(raw) < offset + LINK_POSE_LENGTH:
                return
            yaw, pitch, roll, x_mm, y_mm, z_mm = struct.unpack_from(
                '<ffffff', raw, offset
            )
            offset += LINK_POSE_LENGTH
            if all(math.isfinite(v) for v in (yaw, pitch, roll, x_mm, y_mm, z_mm)):
                self.peer_pose = (x_mm / MM_PER_CM, y_mm / MM_PER_CM, yaw % 360)
                self.peer_position_mm = (x_mm, y_mm, z_mm)
        self.link_data = raw[offset:] or None

    def SetWheelSpeed(self, Speed1, Speed2, Speed3, Speed4):
        """设置轮子速度"""
        with self._command_lock:
            self.CommandLine['SET_WHEELS_SPEED'] = (-Speed1*self.motor_speed_k,-Speed2*self.motor_speed_k,Speed3*self.motor_speed_k,Speed4*self.motor_speed_k)

    def MorsePlay(self, Msg):
        """摩斯电码播放"""
        with self._command_lock:
            self.CommandLine['MORSE_PLAY'] = Msg

    def GetRefereeUnlockPin(self):
        """获取裁判解锁状态"""
        with self._command_lock:
            return self.referee_key

    def _RefreshDribbleCurrent(self):
        response = self.ExecuteCommands([(Commands.GET_DRIBBLE_CURRENT,)])
        self._ApplyResponses(response)
        return self.dribble_current

    def GetDribbleCurrent(self):
        """获取盘球电流（安培），尚未读到有效值时返回 ``None``。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            try:
                self._RefreshDribbleCurrent()
            except Exception as e:
                self.logger.debug(f"Refresh dribble current failed: {e}")
        return self.dribble_current

    def _RefreshBatteryVoltage(self):
        response = self.ExecuteCommands([(Commands.GET_BATTERY_VOLTAGE,)])
        self._ApplyResponses(response)
        return self.battery_voltage

    def GetBatteryVoltage(self):
        """获取电池电压（伏特），尚未读到有效值时返回 ``None``。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            try:
                self._RefreshBatteryVoltage()
            except Exception as e:
                self.logger.debug(f"Refresh battery voltage failed: {e}")
        return self.battery_voltage

    def close(self, join_timeout: float = 2.0):
        """幂等停止后台线程并释放串口；关闭后禁止隐式重连。"""
        with self._lifecycle_lock:
            self._closed = True
            self._stop_thread.set()
            thread = getattr(self, "SendUnitedCommandThread", None)

        if thread is not None and thread is not threading.current_thread() and thread.is_alive():
            thread.join(timeout=max(0.0, float(join_timeout)))
        self.ClosePort()

    Close = close
    Shutdown = close
