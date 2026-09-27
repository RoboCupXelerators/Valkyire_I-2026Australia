#!/usr/bin/env python3
"""离线回放验证：把录制 CSV 喂给 BallStatus 融合检测，和 vision 标签对比。

说明：vision 的 ball_status 只是“弱标签”（窗口/可见性），不是物理持球真值，
所以这里主要看两件事：
  1) 融合后和 vision 一致率；
  2) 有多少 vision=lost 的帧被电流“救回”成 held（跨遮挡）。

用法：
    cd ~/Software/src
    python3 validate_ballstatus.py                       # 默认 relaxed 模式
    python3 validate_ballstatus.py --require-load        # 保守模式：窗口内没受载判 free
    python3 validate_ballstatus.py Records/ball_current/ball_current_xxx.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import Counter
from pathlib import Path

# --- 让工具放在 ~/Software/Tools/ 下也能直接跑：定位 src、加 path、切到 src ---
_SRC = next((p / "src" for p in Path(__file__).resolve().parents
             if (p / "src" / "LunaPre").is_dir()), None)
if _SRC is None:
    _SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
os.chdir(_SRC)

import LunaPre.positions as P
from LunaPre.positions import BALL_NOT_FOUND, Positions

CSV_DIR = "Records/ball_current"


def load_rows(path: Path):
    with path.open("r", newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def replay(path: Path, require_load: bool):
    P.BALL_STATUS_REQUIRE_LOAD = require_load
    pos = Positions()
    rows = load_rows(path)
    n = agree = bridge = 0
    conf = Counter()
    examples = []
    for r in rows:
        if not r.get("t_rel"):
            continue
        ball = ([float(r["ball_x"]), float(r["ball_y"])] if r.get("ball_x")
                else list(BALL_NOT_FOUND))
        cur = float(r["dribble_current"]) if r.get("dribble_current") else None
        now = float(r["t_rel"])
        det = pos.BallStatus(ball, current=cur, now=now)
        vis = r.get("ball_status", "")
        n += 1
        agree += det == vis
        conf[(vis, det)] += 1
        if vis == "lost" and det == "held":
            bridge += 1
            if len(examples) < 5:
                examples.append((float(r["t_rel"]), cur, ball))
    return {"n": n, "agree": agree, "bridge": bridge, "conf": conf, "examples": examples}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csvs", nargs="*", type=Path)
    parser.add_argument("--require-load", action="store_true")
    args = parser.parse_args()

    csvs = args.csvs or sorted(Path(CSV_DIR).glob("*.csv"))
    if not csvs:
        print("没有 CSV")
        return 1

    print(f"模式: BALL_STATUS_REQUIRE_LOAD={args.require_load}\n")
    print("file                        n     一致率   lost->held(电流救回)")
    for path in csvs:
        res = replay(path, args.require_load)
        rate = 100 * res["agree"] / max(1, res["n"])
        print(f"{path.name[-26:]:28s} {res['n']:5d}  {rate:5.1f}%   {res['bridge']:5d}")

    # 汇总混淆矩阵（用最后一个文件做示例）
    res = replay(csvs[0], args.require_load)
    print(f"\n混淆矩阵示例 {csvs[0].name}  (行=vision, 列=fusion)")
    cols = ("held", "free", "lost")
    print("vision\\fusion " + "".join(f"{c:>7s}" for c in cols))
    for vis in cols:
        print(f"{vis:>13s} " + "".join(f"{res['conf'][(vis, c)]:7d}" for c in cols))
    if res["examples"]:
        print("\n电流救回的样例 (t_rel, current, ball):")
        for t, cur, ball in res["examples"]:
            print(f"  t={t:6.2f}s cur={cur} ball={ball}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
