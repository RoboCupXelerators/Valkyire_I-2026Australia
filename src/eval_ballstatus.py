#!/usr/bin/env python3
"""回放评估：用录制 CSV 检验 BallStatus 的电流受载检测（自适应 margin）。

真值用球距离近似：near = d<8.5cm（应该在咬球），far = d>=12cm（应该空转）。
重点看每个功率档下 TPR(near) / FPR(far) / 无球帧误报率，以及自适应 threshold。
"""
from __future__ import annotations

import csv
import glob
import math
from collections import Counter
from pathlib import Path

from LunaPre.positions import BALL_NOT_FOUND, Positions


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, max(0, int(p * (len(v) - 1))))] if v else float("nan")


def eval_file(path, require_load=False):
    import LunaPre.positions as P
    P.BALL_STATUS_REQUIRE_LOAD = require_load
    pos = Positions()
    near = {"n": 0, "loaded": 0}
    far = {"n": 0, "loaded": 0}
    none = {"n": 0, "loaded": 0}
    ths = []
    loaded_series = []
    rows = list(csv.DictReader(open(path)))
    cmd = rows[0].get("dribble_cmd") or "?"
    try:
        cmd_val = float(cmd)
    except (TypeError, ValueError):
        cmd_val = None
    for r in rows:
        if not r["dribble_current"]:
            continue
        cur = float(r["dribble_current"])
        now = float(r["t_rel"])
        ball = ([float(r["ball_x"]), float(r["ball_y"])] if r["ball_x"]
                else list(BALL_NOT_FOUND))
        pos.BallStatus(ball, current=cur, now=now, dribble_cmd=cmd_val)
        info = pos.BallLoadInfo()
        ths.append(info["threshold"])
        loaded_series.append(info["loaded"])
        if r["ball_x"]:
            d = math.hypot(float(r["ball_x"]), float(r["ball_y"]))
            bucket = near if d < 8.5 else (far if d >= 12 else None)
        else:
            bucket = none
        if bucket is not None:
            bucket["n"] += 1
            bucket["loaded"] += info["loaded"]
    tpr = near["loaded"] / near["n"] if near["n"] else float("nan")
    fpr = far["loaded"] / far["n"] if far["n"] else float("nan")
    fnone = none["loaded"] / none["n"] if none["n"] else float("nan")
    ba = (tpr + (1 - fpr)) / 2
    return cmd, tpr, fpr, fnone, ba, ths


def main():
    csvs = sorted(Path("Records/ball_current").glob("*.csv"))
    print("power  TPR(near)  FPR(far)  FPR(no-ball)  bal_acc   threshold(med)  noise")
    for path in csvs:
        cmd, tpr, fpr, fnone, ba, ths = eval_file(path)
        print(f"{str(cmd):>5}   {tpr:6.2f}     {fpr:6.2f}    {fnone:6.2f}       {ba:.3f}     "
              f"{q(ths, 0.5):.3f}")


if __name__ == "__main__":
    main()
