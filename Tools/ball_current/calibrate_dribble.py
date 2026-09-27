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
import os
import statistics as st
import sys
import time
from datetime import datetime
from pathlib import Path

# --- 让工具放在 ~/Software/Tools/ 下也能直接跑：定位 src、加 path、切到 src ---
_SRC = next((p / "src" for p in Path(__file__).resolve().parents
             if (p / "src" / "LunaPre").is_dir()), None)
if _SRC is None:
    _SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
os.chdir(_SRC)

CALIB_REL = "ReasonData/data/dribble_calib.json"
FREESPIN_MIN_CM = 12.0
NEAR_MAX_CM = 8.5          # d < 此值算“咬合”帧，用来量真实咬球增量
GRIP_MARGIN_RATIO = 0.35   # margin = RATIO * (咬合中位 - 空转中位)，越小越易触发
K = 2.5                    # 没有咬合样本时的兜底：margin = K * sigma_空转
FLOOR = 0.010
CAP_FRAC = 0.9


def mad_sigma(values):
    if len(values) < 3:
        return 0.0
    med = st.median(values)
    mad = st.median([abs(v - med) for v in values])
    return 1.4826 * mad


def make_point(cmd, baseline, sigma, n, margin=None):
    if margin is None:
        margin = max(FLOOR, K * sigma)
    margin = max(FLOOR, float(margin))
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
        g = groups.setdefault(cmd, {"far": [], "near": [], "all": []})
        for r in rows:
            if not r.get("dribble_current"):
                continue
            c = float(r["dribble_current"])
            g["all"].append(c)
            if r.get("ball_x"):
                d = math.hypot(float(r["ball_x"]), float(r["ball_y"]))
                if d >= FREESPIN_MIN_CM:
                    g["far"].append(c)
                elif d < NEAR_MAX_CM:
                    g["near"].append(c)
        print(f"  {path.name}: cmd={cmd:g} far={len(g['far'])} near={len(g['near'])} "
              f"all={len(g['all'])}")

    points = []
    for cmd in sorted(groups):
        g = groups[cmd]
        base_vals = g["far"] if len(g["far"]) >= 5 else g["all"]
        if len(base_vals) < 5:
            continue
        baseline = st.median(base_vals)
        sigma = mad_sigma(base_vals)
        margin = None
        tag = "noise"
        if len(g["near"]) >= 20:
            # 用录制里真实的“咬合 - 空转”增量定 margin，比只看空转噪声准得多
            delta = st.median(g["near"]) - baseline
            margin = max(FLOOR, GRIP_MARGIN_RATIO * delta)
            tag = f"grip_delta={delta:.4f}"
        pts = make_point(cmd, baseline, sigma, len(base_vals), margin=margin)
        points.append(pts)
        print(f"cmd={cmd:g}: baseline={baseline:.4f} sigma={sigma:.4f} "
              f"margin={pts['margin']:.4f} ({tag}) n={len(base_vals)}")
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


def calibrate_grip_live(powers, seconds, out_path, period=0.01):
    """测“咬合电流”：球压在滚轮上，配合已有空转基线算真实咬球增量 -> margin。"""
    from LunaPre import peripheral, Bits
    existing = {}
    if out_path.exists():
        try:
            old = json.loads(out_path.read_text(encoding="utf-8"))
            for p in old.get("points", []):
                if p.get("cmd") is not None:
                    existing[round(float(p["cmd"]), 4)] = p
        except Exception:
            pass
    points = []
    for power in powers:
        print(f"[grip] power={power}：把球压在滚轮上并保持 {seconds}s ...")
        peripheral.Dribble(power)
        time.sleep(0.5)
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
        grip = st.median(samples)
        key = round(float(power), 4)
        base = existing.get(key, {}).get("baseline")
        sigma = existing.get(key, {}).get("sigma", 0.0)
        if base is not None:
            margin = max(FLOOR, GRIP_MARGIN_RATIO * (grip - base))
            margin = min(margin, CAP_FRAC * base) if base > 0 else margin
        else:
            margin = None  # 没基线就先按噪声兜底
        pts = make_point(power, base if base is not None else grip, sigma,
                         len(samples), margin=margin)
        pts["grip"] = round(float(grip), 5)
        points.append(pts)
        print(f"  power={power}: grip={grip:.4f} baseline="
              f"{'None' if base is None else f'{base:.4f}'} margin={pts['margin']:.4f}")
    return points


def main():
    parser = argparse.ArgumentParser(description="盘球空转/咬合校准")
    parser.add_argument("--from-csv", action="store_true",
                        help="用 Records/ball_current/*.csv 反推（球需在远处/无球）")
    parser.add_argument("--grip", action="store_true",
                        help="测咬合电流（球压在滚轮上），配合已有空转基线算 margin")
    parser.add_argument("--csv-dir", default="Records/ball_current")
    parser.add_argument("--powers", nargs="*", type=float, default=[],
                        help="在线模式要测的功率，例如 0.2 0.3 0.5 0.8")
    parser.add_argument("--seconds", type=float, default=2.0, help="每档采样时长")
    parser.add_argument("--out", default=None, help="校准文件路径（默认 src/ReasonData/data/）")
    args = parser.parse_args()

    src_root = _SRC
    out_path = Path(args.out) if args.out else (src_root / CALIB_REL)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if args.from_csv:
        csvs = sorted(Path(args.csv_dir).glob("*.csv"))
        if not csvs:
            print(f"没找到 CSV: {args.csv_dir}")
            return 1
        points = calibrate_from_csv(csvs, out_path)
        source = "from-csv"
    elif args.grip and args.powers:
        points = calibrate_grip_live(args.powers, args.seconds, out_path)
        source = "grip"
    elif args.powers:
        points = calibrate_live(args.powers, args.seconds)
        source = "live"
    else:
        parser.error("要么 --from-csv，要么 --powers ...，咬合用 --grip --powers ...")

    if not points:
        print("没有生成任何校准点")
        return 1

    # 合并已有校准点（按 cmd 覆盖）：只更新本次测到的功率，其它保留，
    # 避免“只补一个 0.2 档”却把 0.3/0.5/0.8 冲掉。
    merged = {}
    if out_path.exists():
        try:
            old = json.loads(out_path.read_text(encoding="utf-8"))
            for p in old.get("points", []):
                if p.get("cmd") is not None:
                    merged[round(float(p["cmd"]), 4)] = p
        except Exception as error:
            print(f"[warn] 读取旧校准失败，将只写本次结果: {error}")
    for p in points:
        merged[round(float(p["cmd"]), 4)] = p
    points = sorted(merged.values(), key=lambda p: float(p["cmd"]))

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
