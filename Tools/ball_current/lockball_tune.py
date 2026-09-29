#!/usr/bin/env python3
"""近处追球调试台：实时跑 LockBallSlip，打印球距 / 下发速度 / 盘球功率 / 咬合 / 保持态。

用法（任意目录，需要带依赖的解释器）：
    python3 ~/Software/Tools/ball_current/lockball_tune.py
    python3 ~/Software/Tools/ball_current/lockball_tune.py --rate 30

把球放在机器人前方不同距离，观察各列：
    d       球到机体的距离(cm)
    vx/vy/w 本帧下发的底盘速度 (m/s, m/s, deg/s)
    power   本帧下发的盘球功率
    load    电流是否判定滚轮受载
    hold    是否在“抓到后保持提升”transition 中
    ret     LockBallSlip() 返回值
Ctrl-C 停止（会停车并关盘球）。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

# --- 定位 src、加 path、切到 src（同其它工具）---
_SRC = next((p / "src" for p in Path(__file__).resolve().parents
             if (p / "src" / "LunaPre").is_dir()), None)
if _SRC is None:
    _SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
os.chdir(_SRC)

import BasicFuc as B
from LunaPre import UnitedPosition

# 截获底盘/盘球输出，方便看“实际下发了多少”
_cmd = {"x": 0.0, "y": 0.0, "w": 0.0, "power": None}

_real_move = B.chassis.RobotMoveVector
_real_dribble = B.peripheral.Dribble


def _spy_move(sx, sy, sw, *a, **k):
    _cmd["x"], _cmd["y"], _cmd["w"] = float(sx), float(sy), float(sw)
    return _real_move(sx, sy, sw, *a, **k)


def _spy_dribble(power):
    _cmd["power"] = power
    return _real_dribble(power)


B.chassis.RobotMoveVector = _spy_move
B.peripheral.Dribble = _spy_dribble


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=30.0, help="循环频率 Hz")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="运行时长(秒)，<=0 表示一直跑到 Ctrl-C")
    args = parser.parse_args()
    period = 1.0 / max(args.rate, 1e-6)

    print("t     d(cm)  ball(x,y)          vx     vy     w      power  cur    dev    idx    load  hold",
          flush=True)
    t0 = time.time()
    try:
        while True:
            if args.seconds > 0 and time.time() - t0 >= args.seconds:
                break
            B.lockball.LockBallSlip()
            info = UnitedPosition.BallLoadInfo()
            ball = UnitedPosition.RobotBallPos()
            if ball and list(ball) != [0xFFF, 0xFFF]:
                d = math.hypot(float(ball[0]), float(ball[1]))
                bs = f"({ball[0]:6.2f},{ball[1]:6.2f})"
            else:
                d, bs = float("nan"), "      none       "
            power = _cmd["power"]
            power_s = "  -  " if power is None else f"{float(power):.2f} "
            cur = info.get("current")
            dev = info.get("margin")
            idx = info.get("index")
            cur_s = "  -  " if cur is None else f"{float(cur):.3f}"
            dev_s = "  -   " if dev is None else f"{float(dev):+.3f}"
            idx_s = "  -  " if idx is None else f"{float(idx):+.2f}"
            print(f"{time.time()-t0:5.2f} {d:6.1f}  {bs}  "
                  f"{_cmd['x']:5.2f}  {_cmd['y']:5.2f}  {_cmd['w']:5.1f}  "
                  f"{power_s} {cur_s} {dev_s} {idx_s}  {str(info['loaded'])[:4]:5s} "
                  f"{str(getattr(B.lockball, '_lockball_holding', False))[:4]:5s}",
                  flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl-C 停止")
    finally:
        try:
            B.chassis.stop()
        except Exception:
            pass
        try:
            B.peripheral.Dribble(False)
        except Exception:
            pass


if __name__ == "__main__":
    main()
