#!/usr/bin/env python3
"""盘球空转校准：按功率测出空转电流基线 baseline 和受载阈值 margin，写成一个
持久化文件，运行时按当前下发功率查表即用——**不需要每次开机重新校准**。

两种模式
--------
1. 离线（推荐，安全）：用已录的 CSV 反推。要求录制时球在远处/无球（确定空转）。
       python3 calibrate_dribble.py --from-csv
2. 在线：各功率逐个空转实测（会真的转盘球，务必让球离开滚轮）。
       python3 calibrate_dribble.py --powers 0.2 0.3 0.5 0.8 --seconds 2

输出：<src>/ReasonData/data/dribble_calib.json
margin 计算： margin = clamp(K * sigma, FLOOR, CAP_FRAC * baseline)
  sigma 用稳健标准差 MAD*1.4826（抗尖峰），只用确定空转帧。
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import statistics as st
import time
from datetime import datetime
from pathlib import Path

CALIB_REL = "ReasonData/data/dribble_calib.json"
FREESPIN_MIN_CM = 12.0
K = 2.5
FLOOR = 0.010
CAP_FRAC = 0.9


def mad_sigma(values):
    if len(values) < 3:
        return 0.0
    med = st.median(values)
    mad = st.median([abs(v - med) for v in values])
    return 1.4826 * mad


def make_point(cmd, baseline, sigma, n):
    margin = max(FLOOR, K * sigma)
    margin = min(margin, CAP_FRAC * baseline) if baseline > 0 else margin
    return {"cmd": round(float(cmd), 4), "baseline": round(float(baseline), 5),
            "sigma": round(float(sigma), 5), "margin": round(float(margin), 5), "n": int(n)}


def calibrate_from_csv(csvs, calib_path):
    groups = {}
    for path in csvs:
        rows = list(csv.DictReader(open(path)))
        if not rows:
            continue
        cmd = rows[0].get("dribble_cmd") or ""
        try:
            cmd = float(cmd)
        except ValueError:
            print(f"跳过（无 dribble_cmd）: {path.name}")
            continue
        bucket = groups.setdefault(cmd, [])
        far = []
        allc = []
        for r in rows:
            if not r.get("dribble_current"):
                continue
            c = float(r["dribble_current"])
            allc.append(c)
            if r.get("ball_x"):
                d = math.hypot(float(r["ball_x"]), float(r["ball_y"]))
                if d >= FREESPIN_MIN_CM:
                    far.append(c)
        # 有明确“远处”帧就用它，否则（比如校准录制里球根本没被看到）用全部帧
        bucket.extend(far if far else allc)
        print(f"  {path.name}: cmd={cmd:g} far={len(far)} all={len(allc)}")

    points = []
    for cmd in sorted(groups):
        vals = groups[cmd]
        if len(vals) < 5:
            continue
        med = st.median(vals)
        sigma = mad_sigma(vals)
        points.append(make_point(cmd, med, sigma, len(vals)))
        print(f"cmd={cmd:g}: baseline={med:.4f} sigma={sigma:.4f} "
              f"margin={points[-1]['margin']:.4f} n={len(vals)}")
    return points


def calibrate_live(powers, seconds, period=0.01):
    import statistics as _st
    from LunaPre import peripheral, Bits  # 需要硬件/裁判放行
    points = []
    for power in powers:
        print(f"[live] power={power} 空转 {seconds}s ...")
        peripheral.Dribble(power)
        time.sleep(0.5)  # 起转
        samples = []
        t_end = time.time() + seconds
        while time.time() < t_end:
            cur = Bits.GetDribbleCurrent()
            if cur is not None:
                samples.append(float(cur))
            time.sleep(period)
        peripheral.Dribble(False)
        if len(samples) < 5:
            print(f"  power={power}: 样本不足，跳过")
            continue
        med = _st.median(samples)
        sigma = mad_sigma(samples)
        points.append(make_point(power, med, sigma, len(samples)))
        print(f"  power={power}: baseline={med:.4f} sigma={sigma:.4f} "
              f"margin={points[-1]['margin']:.4f} n={len(samples)}")
    return points


def main():
    parser = argparse.ArgumentParser(description="盘球空转校准")
    parser.add_argument("--from-csv", action="store_true",
                        help="用 Records/ball_current/*.csv 反推（球需在远处/无球）")
    parser.add_argument("--csv-dir", default="Records/ball_current")
    parser.add_argument("--powers", nargs="*", type=float, default=[],
                        help="在线模式要测的功率，例如 0.2 0.3 0.5 0.8")
    parser.add_argument("--seconds", type=float, default=2.0, help="每档空转采样时长")
    parser.add_argument("--out", default=None, help="校准文件路径（默认 src/ReasonData/data/）")
    args = parser.parse_args()

    src_root = Path(__file__).resolve().parent
    out_path = Path(args.out) if args.out else (src_root / CALIB_REL)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if args.from_csv:
        csvs = sorted(Path(args.csv_dir).glob("*.csv"))
        if not csvs:
            print(f"没找到 CSV: {args.csv_dir}")
            return 1
        points = calibrate_from_csv(csvs, out_path)
        source = "from-csv"
    elif args.powers:
        points = calibrate_live(args.powers, args.seconds)
        source = "live"
    else:
        parser.error("要么 --from-csv，要么 --powers ...")

    if not points:
        print("没有生成任何校准点")
        return 1

    doc = {
        "version": 1,
        "generated": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "source": source,
        "unit": "A",
        "k": K, "floor": FLOOR, "cap_frac": CAP_FRAC,
        "points": points,
    }
    out_path.write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[done] 写入 {out_path}（{len(points)} 个功率点）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
