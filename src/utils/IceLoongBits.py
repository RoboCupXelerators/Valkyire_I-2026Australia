# IceLoongBits.py

import logging
from typing import Optional, List, Tuple
from enum import IntEnum
import struct

class Commands(IntEnum):
    HAND_SHAKE        = 0x01
    GET_BMI088_DATA   = 0x02
    SET_WHEELS_SPEED  = 0x03
    KICK              = 0x04

COMMAND_RESPONSE_LENGTHS = {
    Commands.HAND_SHAKE:       1 + 1, 
    Commands.GET_BMI088_DATA:  1 + 3 * 4 + 4, 
    Commands.SET_WHEELS_SPEED: 1, 
    Commands.KICK:             1,
}

try:
    import serial
except Exception:  # pragma: no cover - import error handled at runtime
    serial = None

class IceLoongBits:
    """串口通信封装类，用于与 F407（STM32）类设备交互。
    方法命名使用大驼峰（PascalCase）。
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, timeout: float = 0.02):
        """初始化串口参数（不自动打开）。

        Args:
            port: 串口设备路径，例如 "/dev/ttyUSB0" 或 "COM3"。
            baudrate: 波特率，例如 115200。
            timeout: 读超时（秒）。
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        # 串口句柄（运行时为 serial.Serial 实例），这里不在类型注解中引用 serial 变量以避免 lint 问题
        self.ser = None
        self.logger = logging.getLogger(self.__class__.__name__)
        if not self.logger.handlers:
            # 避免多次添加 handler
            self.logger.addHandler(logging.NullHandler())

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
                self.logger.exception("关闭串口失败: %s", e)
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
        """发送原始字节命令到 F407。

        Args:
            data: 需要发送的原始字节数据。
            read_response: 是否在发送后立即读取响应。
            response_length: 如果大于 0，按长度读取；否则读取直到超时。
            response_timeout: 单次读取的临时超时（秒），若 None 则使用初始化的 timeout。

        Returns:
            bytes: 读取到的响应（如果 read_response=False 返回 b''）。
        """
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

        self.ser.write(data)
        self.ser.flush()

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
        finally:
            if response_timeout is not None:
                self.ser.timeout = old_timeout

    def __enter__(self):
        self.OpenPort()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.ClosePort()

    def ExecuteCommands(self, commands: List[tuple]) -> tuple:
        """执行单条或多条命令"""
        
        if len(commands) > 1:
            tx_bytes, resp_len = bytes([0x00]), 1
        else:
            tx_bytes, resp_len = bytes(), 0

        for cmd in commands:
            match cmd[0]:
                case Commands.HAND_SHAKE:
                    tx_bytes += bytes([cmd[0]])
                case Commands.GET_BMI088_DATA:
                    tx_bytes += bytes([cmd[0]])
                case Commands.SET_WHEELS_SPEED:
                    tx_bytes += bytes([cmd[0]]) + struct.pack('<ffff', cmd[1], cmd[2], cmd[3], cmd[4])
                case Commands.KICK:
                    tx_bytes += bytes([cmd[0]])
                case _:
                    continue
            resp_len += COMMAND_RESPONSE_LENGTHS.get(cmd[0], 0)

        if len(commands) > 1:
            tx_bytes += bytes([0])
            resp_len += 1

        # 发送帧，返回接收帧
        resp = self.SendHex(tx_bytes, True, resp_len, 0.01)

        rx_data = []
        if resp and len(resp) == resp_len:
            i = 0
            while i < resp_len:
                match resp[i] ^ 0x80:
                    case 0x00:
                        i += 1
                        continue
                    case Commands.HAND_SHAKE:
                        rx_data.append((resp[i], resp[i+1]))
                    case Commands.GET_BMI088_DATA:
                        rx_data.append((resp[i], *struct.unpack('<fffi', resp[i+1:i+17])))
                    case Commands.SET_WHEELS_SPEED:
                        rx_data.append((resp[i],))
                    case Commands.KICK:
                        rx_data.append((resp[i],))
                    case _:
                        continue
                i += COMMAND_RESPONSE_LENGTHS.get(resp[i] ^ 0x80, 1)
        return rx_data
