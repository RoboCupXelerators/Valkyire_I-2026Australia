#!/usr/bin/env python3
"""调用测试：球坐标 + 盘球电流 融合持球检测（BallStatus / BallLoadInfo）。

用法：
    cd ~/Software/src
    python3 test_ballstatus.py --dribble 0.3       # 测试期间开转盘球 power=0.3，退出自动关
    python3 test_ballstatus.py --dribble 1.0       # 全速盘球
    python3 test_ballstatus.py                     # 不动盘球，只观察（球由别处驱动）
    python3 test_ballstatus.py --rows 25           # 窗口里保留多少行历史
    python3 test_ballstatus.py --plain             # 不刷新，逐行滚动（重定向日志时用）

看什么：
    - status  : BallStatus() 融合后的持球状态（lost / free / held）
    - loaded  : 电流是否判定滚轮受载（BallLoadInfo）
    - margin  : current - baseline，超过 DRIBBLE_LOAD_MARGIN(0.08A) 就是受载
    - baseline: 自适应空转基线，会随下发功率漂移
    - 顶部那行会显示下发的盘球功率和当前 Permission

盘球要转需要裁判放行；无裁判真机调试加 ROBOCUPX_DEBUG=1：
    ROBOCUPX_DEBUG=1 python3 test_ballstatus.py --dribble 0.5
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque

from LunaPre import UnitedPosition, Bits

HEADER = ("t_rel   ball(x,y)            status   cur    smooth  base    margin  thr    loaded")
LINE_W = max(len(HEADER), 72)


def fmt_current(value):
    return "None" if value is None else f"{value:.3f}"


def dribble_title(peripheral, power):
    if peripheral is None:
        return "盘球 未控制（不传 --dribble；球由别处驱动）"
    try:
        perm = peripheral.Permission()
    except Exception:
        perm = "?"
    return f"盘球 power={power:g}   Permission={perm}"


def format_row(t, ball, status, info, cur_fallback):
    cur = info["current"] if info["current"] is not None else cur_fallback
    if ball and list(ball) != [0xFFF, 0xFFF]:
        ball_s = f"({ball[0]:6.2f},{ball[1]:6.2f})"
    else:
        ball_s = "      none       "
    return (f"{t:5.2f}  {ball_s}  {status:6s}  "
            f"{fmt_current(cur):>6s}  {fmt_current(info.get('smoothed')):>6s}  "
            f"{fmt_current(info['baseline']):>6s}  "
            f"{fmt_current(info['margin']):>6s}  {fmt_current(info.get('threshold')):>6s}  "
            f"{info['loaded']}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=0,
                        help="打印时长(秒)，<=0 表示一直打到 Ctrl-C")
    parser.add_argument("--period", type=float, default=0.05, help="采样周期(秒)")
    parser.add_argument("--rows", type=int, default=15, help="固定窗口保留的历史行数")
    parser.add_argument("--dribble", type=float, default=None, metavar="POWER",
                        help="测试期间持续下发盘球功率(0~1)，退出自动关；不传则不动盘球")
    parser.add_argument("--plain", action="store_true",
                        help="不刷新，逐行滚动输出（重定向到文件时自动使用）")
    args = parser.parse_args()

    peripheral = None
    if args.dribble is not None:
        from LunaPre import peripheral
        try:
            peripheral.Dribble(float(args.dribble))
        except Exception as error:
            print(f"[warn] 开盘球失败: {error}")

    live = (not args.plain) and sys.stdout.isatty()
    history = deque(maxlen=max(1, args.rows))

    if live:
        sys.stdout.write("\033[2J")  # 清一次屏
    else:
        print(dribble_title(peripheral, args.dribble))
        print(HEADER)

    t0 = time.time()
    warned = False
    try:
        while True:
            t = time.time() - t0
            if args.seconds > 0 and t >= args.seconds:
                break

            if peripheral is not None:
                # 每帧下发一次；chassis 缓存了状态，值没变不会重复发帧。
                peripheral.Dribble(float(args.dribble))
                if not warned and not peripheral.Permission():
                    print("[warn] 裁判未放行 (Permission=False)，盘球不会转。"
                          "无裁判真机调试请加 ROBOCUPX_DEBUG=1。", file=sys.stderr)
                    warned = True

            ball = UnitedPosition.RobotBallPos()
            status = UnitedPosition.BallStatus(ball)
            info = UnitedPosition.BallLoadInfo()
            row = format_row(t, ball, status, info, Bits.GetDribbleCurrent())

            if live:
                history.append(row)
                fixed = [dribble_title(peripheral, args.dribble), HEADER]
                block = fixed + list(history)
                block += [""] * (len(fixed) + max(1, args.rows) - len(block))
                width = max(LINE_W, max(len(x) for x in block))
                # 光标回左上角，重画固定块，再清掉块后面的残影
                sys.stdout.write("\033[H" + "\n".join(x.ljust(width) for x in block) + "\033[J")
                sys.stdout.flush()
            else:
                print(row)

            time.sleep(max(0.0, args.period))
    except KeyboardInterrupt:
        if not live:
            print("\nCtrl-C 停止")
    finally:
        if peripheral is not None:
            try:
                peripheral.Dribble(False)
                if not live:
                    print("盘球已关闭")
            except Exception as error:
                print(f"[warn] 关闭盘球失败: {error}")
        if live:
            sys.stdout.write("\n")
            sys.stdout.flush()
        UnitedPosition.Stop()


if __name__ == "__main__":
    main()
