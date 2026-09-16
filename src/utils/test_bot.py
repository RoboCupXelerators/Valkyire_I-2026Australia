"""Small hardware smoke test for the IceLoong serial protocol."""

import argparse
import logging
import os
import time

from IceLoongBits import IceLoongBits, Commands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=os.environ.get("ICELOONG_PORT", "/dev/ttyTHS1"),
        help="STM32 host UART (default: ICELOONG_PORT or /dev/ttyTHS1)",
    )
    parser.add_argument(
        "--send-hex",
        default="",
        help="optional LINK_SEND payload, encoded as hexadecimal",
    )
    args = parser.parse_args()

    with IceLoongBits(port=args.port, testing=True) as bot:

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

        if args.send_hex:
            payload = bytes.fromhex(args.send_hex)
            print(f"\n=== 测试3: LINK_SEND ({len(payload)} bytes) ===")
            response = bot.ExecuteCommands([(Commands.LINK_SEND, payload)])
            print("响应:", response)

        print("\n=== 测试4: LINK_GET (variable payload) ===")
        response = bot.ExecuteCommands([(Commands.LINK_GET,)])
        print("响应:", response)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logging.exception("测试失败: %s", e)
