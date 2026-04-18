# IceLoongBits.py

import logging
from typing import Optional, List, Tuple
from enum import IntEnum
import struct
import time
import threading

class Commands(IntEnum):
    HAND_SHAKE        = 0x01
    GET_BMI088_DATA   = 0x02
    SET_WHEELS_SPEED  = 0x03
    KICK              = 0x04
    SET_DRIBBLE_PWM   = 0x05
    SET_RYB_LIGHTS    = 0x06
    GET_RUN_KEY_DELTA = 0x07

COMMAND_RESPONSE_LENGTHS = {
    Commands.HAND_SHAKE:        1, 
    Commands.GET_BMI088_DATA:   1 + 3 * 4, 
    Commands.SET_WHEELS_SPEED:  1, 
    Commands.KICK:              1, 
    Commands.SET_DRIBBLE_PWM:   1, 
    Commands.SET_RYB_LIGHTS:    1, 
    Commands.GET_RUN_KEY_DELTA: 1 + 1, 
}

try:
    import serial
except Exception:  # pragma: no cover - import error handled at runtime
    serial = None

class IceLoongBits:
    """串口通信封装类，用于与 F407（STM32）类设备交互。
    方法命名使用大驼峰（PascalCase）。
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, timeout: float = 0.05, testing = False):
        """初始化串口参数（不自动打开）。

        Args:
            port: 串口设备路径，例如 "/dev/ttyUSB0" 或 "COM3"。
            baudrate: 波特率，例如 115200。
            timeout: 读超时（秒）。
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.CommandLine = {
            'HAND_SHAKE': True,
            'GET_BMI088_DATA': True,
            'SET_WHEELS_SPEED': (0,0,0,0),
            'SET_DRIBBLE_PWM': 0,
            'KICK': 0,
        }

        self.yaw = 0.0
        # 串口句柄（运行时为 serial.Serial 实例），这里不在类型注解中引用 serial 变量以避免 lint 问题
        self.ser = None
        self.logger = logging.getLogger(self.__class__.__name__)
        if not self.logger.handlers:
            # 避免多次添加 handler
            self.logger.addHandler(logging.NullHandler())

        if not testing:
            self.SendUnitedCommandThread = threading.Thread(target=self.SendUnitedCommand, daemon=True)
            self.SendUnitedCommandThread.daemon = True
            self.SendUnitedCommandThread.start()

    def SendUnitedCommand(self):
        while (1):
            _CommandList = []
            for i in self.CommandLine:
                match i:
                    case 'HAND_SHAKE':
                        if self.CommandLine[i]:
                            _CommandList.append((Commands.HAND_SHAKE,))
                    case 'GET_BMI088_DATA':
                        if self.CommandLine[i]:
                            _CommandList.append((Commands.GET_BMI088_DATA,))
                    case 'SET_WHEELS_SPEED':
                        Speed1,Speed2,Speed3,Speed4 = self.CommandLine[i]
                        self.ExecuteCommands([(Commands.SET_WHEELS_SPEED, Speed1,Speed2,Speed3,Speed4)])
                    case 'SET_DRIBBLE_PWM':
                        Speed = self.CommandLine[i]
                        self.ExecuteCommands([(Commands.SET_DRIBBLE_PWM, Speed)])
                    case 'KICK':
                        if self.CommandLine[i]:
                            _CommandList.append((Commands.KICK,))
                            self.CommandLine['KICK'] = False
            response = self.ExecuteCommands(_CommandList)
            for i in response:
                match i[0] ^ 0x80:
                    case 0x00:
                        continue
                    case Commands.HAND_SHAKE:
                        pass
                    case Commands.GET_BMI088_DATA:
                        _, self.yaw, pitch, roll = i
                    case Commands.SET_WHEELS_SPEED:
                        pass
                    case Commands.KICK:
                        pass
                    case Commands.SET_DRIBBLE_PWM:
                        pass
            time.sleep(0.02)

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
                case Commands.SET_DRIBBLE_PWM:
                    tx_bytes += bytes([cmd[0]]) + struct.pack('<f', cmd[1])
                case Commands.SET_RYB_LIGHTS:
                    tx_bytes += bytes([cmd[0]]) + bytes([((cmd[1] & 1) << 0) | ((cmd[2] & 1) << 1) | ((cmd[3] & 1) << 2)])
                case Commands.GET_RUN_KEY_DELTA:
                    tx_bytes += bytes([cmd[0]])
                case _:
                    continue
            resp_len += COMMAND_RESPONSE_LENGTHS.get(cmd[0], 0)

        if len(commands) > 1:
            tx_bytes += bytes([0])
            resp_len += 1

        # 发送帧，返回接收帧
        resp = self.SendHex(tx_bytes, True, resp_len, 0.05)

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
                    case Commands.GET_BMI088_DATA:
                        rx_data.append((resp[i], *struct.unpack('<fff', resp[i+1:i+13])))
                    case Commands.SET_WHEELS_SPEED:
                        rx_data.append((resp[i],))
                    case Commands.KICK:
                        rx_data.append((resp[i],))
                    case Commands.SET_DRIBBLE_PWM:
                        rx_data.append((resp[i],))
                    case Commands.SET_RYB_LIGHTS:
                        rx_data.append((resp[i],))
                    case Commands.GET_RUN_KEY_DELTA:
                        rx_data.append((resp[i], resp[i+1]))
                    case _:
                        break
                i += COMMAND_RESPONSE_LENGTHS.get(resp[i] ^ 0x80, 1)
        return rx_data

    def GetYaw(self) -> float:
        """获取当前偏航角（度）。"""
        return self.yaw
    
    def VirtualPorts(self,port,status):
        """设置虚拟端口状态"""
        match port:
            case 3:
                if not status:
                    self.CommandLine['KICK'] = True
                else:
                    self.CommandLine['KICK'] = False
            case 4:
                self.CommandLine['SET_DRIBBLE_PWM'] = status

    def SetWheelSpeed(self,Speed1,Speed2,Speed3,Speed4):
        """设置轮子速度"""
        self.CommandLine['SET_WHEELS_SPEED'] = (Speed1,Speed2,-Speed3,-Speed4)

    def GetKey(self):
        """获取按键状态"""
        # TODO
        return 0
