# test_bot.py

import time
import logging
from IceLoongBits import IceLoongBits, Commands

def main():
    # 根据实际设备修改端口（Linux通常为/dev/ttyUSB0，Windows为COM3等）
    PORT = "/dev/ttyTHS1"
    # PORT = "/dev/iceloong"
    
    with IceLoongBits(port=PORT,testing=True) as bot:

        a = 0
        # 测试5：盘球降速
        while False:
            if a % 100:
                resp4 = bot.ExecuteCommands([
                    (Commands.HAND_SHAKE,), 
                    (Commands.SET_WHEELS_SPEED, 1000, 1000, 1000, 1000), 
                    (Commands.GET_BMI088_DATA,), 
                    (Commands.GET_REFEREE_UNLOCK_PIN,), 
                    (Commands.GET_RUN_KEY_DELTA,), 
                    (Commands.SET_DRIBBLE_SPEED, 0.0, 0), 
                ])
            else:
                resp4 = bot.ExecuteCommands([
                    (Commands.HAND_SHAKE,), 
                    (Commands.SET_WHEELS_SPEED, 1000, 1000, 1000, 1000), 
                    (Commands.GET_BMI088_DATA,), 
                    (Commands.GET_REFEREE_UNLOCK_PIN,), 
                    (Commands.GET_RUN_KEY_DELTA,), 
                    (Commands.SET_DRIBBLE_SPEED, 0.0, 0), 
                    (Commands.KICK,), 
                ])
            print("响应:", resp4)
            for i in resp4:
                match i[0] ^ 0x80:
                    case 0x00:
                        continue
                    case Commands.HAND_SHAKE:
                        pass
                    case Commands.GET_BMI088_DATA:
                        _, yaw, pitch, roll = i
                        yaw = yaw % 360
                    case Commands.SET_WHEELS_SPEED:
                        pass
                    case Commands.KICK:
                        pass
                    case Commands.SET_DRIBBLE_SPEED:
                        pass
                    case Commands.GET_RUN_KEY_DELTA:
                        _ , key = i
                        if key == 1:
                            print(i)
            time.sleep(0.01)
            a += 1

        # 测试1：单命令 - 握手
        print("\n=== 测试1: 单命令 (HAND_SHAKE) ===")
        resp1 = bot.ExecuteCommands([(Commands.HAND_SHAKE,)])
        print("响应:", resp1)
        time.sleep(1)

        # 测试2：复合命令 - 握手 + 获取传感器数据 + 弹射
        print("\n=== 测试2: 复合命令 (HAND_SHAKE + GET_BMI088_DATA) ===")
        resp2 = bot.ExecuteCommands([
            (Commands.HAND_SHAKE,),
            (Commands.GET_BMI088_DATA,), 
            (Commands.GET_REFEREE_UNLOCK_PIN,), 
            (Commands.GET_RUN_KEY_DELTA,),
            (Commands.MORSE_PLAY, "")
        ])
        print("响应:", resp2)
        time.sleep(1)

        # 测试3：带参数命令 - 设置轮速
        print("\n=== 测试3: 带参数命令 (SET_WHEELS_SPEED) ===")
        resp3 = bot.ExecuteCommands([
            # (Commands.SET_WHEELS_SPEED, -1000, -1000, -1000, -1000)  # 四轮速度
            (Commands.SET_WHEELS_SPEED, 0, 0, 0, 0), 
        ])
        print("响应:", resp3)
        time.sleep(1)

        # 测试4：混合复合命令
        print("\n=== 测试4: 混合复合命令 ===")
        resp4 = bot.ExecuteCommands([
            (Commands.HAND_SHAKE,), 
            (Commands.GET_BMI088_DATA,), 
            (Commands.SET_DRIBBLE_SPEED, 0.55, 0), 
            (Commands.GET_REFEREE_UNLOCK_PIN,), 
            (Commands.GET_RUN_KEY_DELTA,)
        ])
        print("响应:", resp4)
        time.sleep(5)

        print("\n=== 测试5：盘球降速 ===")
        resp4 = bot.ExecuteCommands([
            (Commands.KICK,), 
            (Commands.SET_DRIBBLE_SPEED, 0.0, 0)
        ])
        print("响应:", resp4)
        time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logging.exception("测试失败: %s", e)
