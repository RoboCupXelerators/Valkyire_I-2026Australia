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
    GET_POSE               = 0x02
    SET_WHEELS_SPEED       = 0x03
    KICK                   = 0x04
    SET_DRIBBLE_SPEED      = 0x05
    GET_REFEREE_UNLOCK_PIN = 0x06
    MORSE_PLAY             = 0x07


COMMAND_RESPONSE_LENGTHS = {
    Commands.HAND_SHAKE:             1,
    Commands.GET_POSE:               1 + 6 * 4,
    Commands.SET_WHEELS_SPEED:       1,
    Commands.KICK:                   1,
    Commands.SET_DRIBBLE_SPEED:      1,
    Commands.GET_REFEREE_UNLOCK_PIN: 1 + 1,
    Commands.MORSE_PLAY:             1,
}

MAX_DRIBBLE_INPUT = 1.0
MIN_KICK_POWER_LEVEL = 1
MAX_KICK_POWER_LEVEL = 4
DEFAULT_KICK_POWER_LEVEL = 4
KICK_POWER_LEVEL_ERROR = "kick power level must be an integer from 1 to 4"
MM_PER_CM = 10.0


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
            'SET_DRIBBLE_SPEED': [0,0],
            'KICK': False,
            'GET_REFEREE_UNLOCK_PIN': True,
            'MORSE_PLAY': None
        }

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.position_mm = None
        self.pose = None
        self.referee_key = False
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
            try:
                self._SendUnitedCommandOnce()
                # 成功执行后重置错误计数
                self._consecutive_errors = 0
            except SerialException as e:
                self._consecutive_errors += 1
                self.logger.warning(
                    "Serial I/O error in command loop (attempt %d/%d): %s", 
                    self._consecutive_errors, self._max_consecutive_errors, e
                )
                # 尝试串口重连
                self._HandleSerialError(e)
            except Exception as e:
                self.logger.exception("IceLoongBits command loop failed: %s", e)
            time.sleep(0.02)

    def _HandleSerialError(self, error: Exception) -> bool:
        """处理串口错误，尝试重连。
        
        Args:
            error: 捕获到的异常
            
        Returns:
            bool: 重连是否成功
        """
        with self._reconnect_lock:
            self.logger.info("Attempting serial reconnection due to error: %s", error)
            
            # 先关闭现有连接
            self._ForceClosePort()
            
            # 等待一段时间后重连
            time.sleep(self._reconnect_delay)
            
            try:
                self.OpenPort()
                self.logger.info("Serial port reconnected successfully: %s", self.port)
                return True
            except Exception as e:
                self.logger.error("Failed to reconnect serial port: %s", e)
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
        if self.IsThreadAlive():
            return True
            
        self.logger.warning("Background command thread is dead, attempting restart...")
        
        # 清理旧线程状态
        if hasattr(self, "_stop_thread"):
            self._stop_thread.set()
        
        # 创建新线程
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
                "SET_WHEELS_SPEED": tuple(self.CommandLine["SET_WHEELS_SPEED"]),
                "SET_DRIBBLE_SPEED": tuple(self.CommandLine["SET_DRIBBLE_SPEED"]),
                "KICK": self.CommandLine["KICK"],
                "MORSE_PLAY": self.CommandLine["MORSE_PLAY"],
            }
            self.CommandLine["KICK"] = False
            self.CommandLine["MORSE_PLAY"] = None
        return snapshot

    def _SendUnitedCommandOnce(self):
        snapshot = self._CommandSnapshot()

        Speed1, Speed2, Speed3, Speed4 = snapshot["SET_WHEELS_SPEED"]
        self.ExecuteCommands([(Commands.SET_WHEELS_SPEED, Speed1, Speed2, Speed3, Speed4)])

        Dribble1, Dribble2 = snapshot["SET_DRIBBLE_SPEED"]
        self.ExecuteCommands([(Commands.SET_DRIBBLE_SPEED, Dribble1, Dribble2)])

        if snapshot["MORSE_PLAY"] is not None:
            self.ExecuteCommands([(Commands.MORSE_PLAY, str(snapshot["MORSE_PLAY"]))])

        _CommandList = []
        if snapshot["HAND_SHAKE"]:
            _CommandList.append((Commands.HAND_SHAKE,))
        if snapshot["GET_POSE"]:
            _CommandList.append((Commands.GET_POSE,))
        if snapshot["KICK"]:
            _CommandList.append((Commands.KICK,))
        if snapshot["GET_REFEREE_UNLOCK_PIN"]:
            _CommandList.append((Commands.GET_REFEREE_UNLOCK_PIN,))

        response = self.ExecuteCommands(_CommandList)
        self._ApplyResponses(response)

    def _ApplyResponses(self, response):
        for i in response:
            match i[0] ^ 0x80:
                case 0x00:
                    continue
                case Commands.HAND_SHAKE:
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

    def OpenPort(self) -> None:
        """打开串口（如果尚未打开）。"""
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
            except Exception as e:
                import subprocess
                a = subprocess.run(['sudo fuser -k /dev/iceloong'], shell=True, capture_output=True, text=True)
                self.logger.exception("关闭串口失败: %s", a.returncode)
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

    def __enter__(self):
        self.OpenPort()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.ClosePort()

    def ExecuteCommands(self, commands: List[tuple]) -> list[tuple]:
        """执行单条或多条命令
        
        Args:
            commands: 命令列表
            
        Returns:
            list[tuple]: 响应数据列表，失败时返回空列表
            
        Note:
            此方法会捕获串口异常并尝试重连，不会向调用者抛出异常。
        """
        if len(commands) > 1:
            tx_bytes, resp_len = bytes([0x00]), 1
        else:
            tx_bytes, resp_len = bytes(), 0

        for cmd in commands:
            match cmd[0]:
                case Commands.HAND_SHAKE:
                    tx_bytes += bytes([cmd[0]])
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
                        '<ff',
                        self._NormalizeDribbleInput(cmd[1]),
                        self._NormalizeDribbleInput(cmd[2]),
                    )
                case Commands.GET_REFEREE_UNLOCK_PIN:
                    tx_bytes += bytes([cmd[0]])
                case Commands.MORSE_PLAY:
                    msg = str(cmd[1]) if len(cmd) > 1 else ""
                    raw = msg.encode('ascii', errors='replace')[:23]
                    if len(raw) < 23:
                        raw += b'\x00'
                    length = len(raw)
                    tx_bytes += bytes([cmd[0], length]) + raw
                case _:
                    continue
            resp_len += COMMAND_RESPONSE_LENGTHS.get(cmd[0], 0)

        if len(commands) > 1:
            tx_bytes += bytes([0])
            resp_len += 1

        # 发送帧，返回接收帧（带异常处理）
        try:
            resp = self.SendHex(tx_bytes, True, resp_len, 0.05)
        except SerialException as e:
            self.logger.warning("ExecuteCommands failed due to serial error: %s", e)
            self._HandleSerialError(e)
            return []
        except Exception as e:
            self.logger.exception("ExecuteCommands failed with unexpected error: %s", e)
            return []

        rx_data = []
        if resp and len(resp) == resp_len:
            i = 0
            while i < resp_len:
                match resp[i] ^ 0x80:
                    case 0x00:
                        i += 1
                        continue
                    case Commands.HAND_SHAKE:
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
                    case Commands.MORSE_PLAY:
                        rx_data.append((resp[i],))
                    case _:
                        break
                i += COMMAND_RESPONSE_LENGTHS.get(resp[i] ^ 0x80, 1)
        return rx_data

    def GetYaw(self) -> float:
        """获取当前偏航角（度）。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            try:
                self._RefreshPose()
            except Exception as e:
                self.logger.debug("Refresh yaw failed: %s", e)
        return self.yaw

    def GetPose(self):
        """返回融合场地位姿 ``(x_cm, y_cm, yaw_deg)``，无效时返回 ``None``。"""
        thread = getattr(self, "SendUnitedCommandThread", None)
        if thread is None or not thread.is_alive():
            try:
                self._RefreshPose()
            except Exception as e:
                self.logger.debug("Refresh pose failed: %s", e)
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
                    self._SetDribbleSpeedNow(left=status)
                case 5:
                    self._SetDribbleSpeedNow(right=status)
        except SerialException as e:
            self.logger.warning("VirtualPorts operation failed due to serial error: %s", e)
            self._HandleSerialError(e)
        except Exception as e:
            self.logger.exception("VirtualPorts operation failed: %s", e)

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

    def SetKickPowerLevel(self, power_level):
        """设置后续 ``Kick()`` 和虚拟端口弹射使用的默认力度档位。"""
        level = self._ValidateKickPowerLevel(power_level)
        with self._command_lock:
            self.kick_power_level = level
        return level

    def Kick(self, power_level=None):
        """立即按 1 到 4 档力度弹射；省略参数时使用默认档位。"""
        if power_level is None:
            with self._command_lock:
                power_level = self.kick_power_level
        return self._KickNow(power_level)

    def _KickNow(self, power_level):
        level = self._ValidateKickPowerLevel(power_level)
        with self._command_lock:
            Dribble1, Dribble2 = self.CommandLine["SET_DRIBBLE_SPEED"]
        response = self.ExecuteCommands([
            (Commands.KICK, level),
            (Commands.SET_DRIBBLE_SPEED, Dribble1, Dribble2),
        ])
        self._ApplyResponses(response)
        return response

    def _SetDribbleSpeedNow(self, left=None, right=None):
        with self._command_lock:
            Dribble1, Dribble2 = self.CommandLine["SET_DRIBBLE_SPEED"]
            if left is not None:
                Dribble1 = self._NormalizeDribbleInput(left)
            if right is not None:
                Dribble2 = self._NormalizeDribbleInput(right)
            self.CommandLine["SET_DRIBBLE_SPEED"] = [Dribble1, Dribble2]

        response = self.ExecuteCommands([(Commands.SET_DRIBBLE_SPEED, Dribble1, Dribble2)])
        self._ApplyResponses(response)
        return response

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

    def Shutdown(self):
        """安全关闭串口和后台线程。"""
        self._stop_thread.set()
        self.ClosePort()
