# test_bot.py

import time
import logging
from IceLoongBits import IceLoongBits, Commands

def main():
    # 根据实际设备修改端口（Linux通常为/dev/ttyUSB0，Windows为COM3等）
    PORT = "/dev/ttyTHS1"
    # PORT = "/dev/iceloong"
    # PORT = "/dev/ttyUSB0"
    
    with IceLoongBits(port=PORT,testing=True) as bot:

        # 测试1：单命令 - 握手
        print("\n=== 测试1: 单命令 (HAND_SHAKE) ===")
        resp1 = bot.ExecuteCommands([(Commands.HAND_SHAKE,)])
        print("响应:", resp1)
        time.sleep(1)

        # 测试2：复合命令 - 握手 + 获取位姿 + 弹射
        print("\n=== 测试2: 复合命令 (HAND_SHAKE + GET_POSE) ===")
        resp2 = bot.ExecuteCommands([
            (Commands.HAND_SHAKE,),
            (Commands.GET_POSE,),
            (Commands.GET_REFEREE_UNLOCK_PIN,), 
            # (Commands.KICK, 4),
            # (Commands.SET_WHEELS_SPEED, 100, 100, 100, 100), 
            # (Commands.MORSE_PLAY, "Hello")
        ])
        print("响应:", resp2)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logging.exception("测试失败: %s", e)
