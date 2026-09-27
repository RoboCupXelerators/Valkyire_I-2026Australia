#!/usr/bin/env python3
"""盘球电流 x 球坐标 持有检测数据分析。

目标：回答“拿到球的时候，怎么结合电流和球坐标判断持球”。

分析口径
--------
- 球在机身坐标系下的距离 ``d = hypot(ball_x, ball_y)``（cm）。
- 把每一帧分成三类：
  - ``near``：d < 9cm（CatchVal=6.5，容差后大致就是接球窗口）
  - ``far`` ：d >= 12cm
  - ``none``：这一帧没有有效球坐标（丢球）
- 用电流区分 ``near`` 与 ``far+none``，算 AUC 和最佳阈值。
- 同时给出“自适应基线”方案：每次运行用电流低分位当基线，
  超过基线一定裕量就判为滚轮受载（球在咬合）。

用法
----
    python3 analyze_ball_current.py                       # 分析 Records/ball_current/*.csv
    python3 analyze_ball_current.py a.csv b.csv --out dir
"""

from __future__ import annotations

import argparse
import base64
import csv
import glob
import html
import math
import os
import statistics as st
import sys
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

NEAR_CM = 9.0
FAR_CM = 12.0
CSV_GLOB = "Records/ball_current/*.csv"


# ==================== 基础统计 ====================

def load_rows(path: Path):
    with path.open("r", newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def percentile(values, p):
    vals = sorted(v for v in values if v is not None)
    if not vals:
        return float("nan")
    idx = min(len(vals) - 1, max(0, int(round(p * (len(vals) - 1)))))
    return vals[idx]


def auc(pos, neg):
    import bisect
    pos = [v for v in pos if v is not None]
    neg = [v for v in neg if v is not None]
    if not pos or not neg:
        return float("nan")
    s = sorted(neg)
    total = 0.0
    for x in pos:
        lo = bisect.bisect_left(s, x)
        hi = bisect.bisect_right(s, x)
        total += lo + 0.5 * (hi - lo)
    return total / (len(pos) * len(neg))


def best_threshold(pos, neg):
    """在 pos/neg 上扫阈值，最大化 balanced accuracy，返回 (t, tpr, fpr)。"""
    pos = [v for v in pos if v is not None]
    neg = [v for v in neg if v is not None]
    if not pos or not neg:
        return None
    cands = sorted(set(pos + neg))
    best = None
    for t in cands:
        tpr = sum(1 for v in pos if v >= t) / len(pos)
        fpr = sum(1 for v in neg if v >= t) / len(neg)
        score = tpr - fpr
        if best is None or score > best[0]:
            best = (score, t, tpr, fpr)
    return best[1], best[2], best[3]


def frame_class(row):
    """返回 (kind, distance)。kind in near/mid/far/none。"""
    if not row.get("ball_x"):
        return "none", None
    try:
        d = math.hypot(float(row["ball_x"]), float(row["ball_y"]))
    except (ValueError, TypeError):
        return "none", None
    if d < NEAR_CM:
        return "near", d
    if d < FAR_CM:
        return "mid", d
    return "far", d


def analyze_file(path: Path):
    rows = load_rows(path)
    buckets = {"near": [], "mid": [], "far": [], "none": []}
    for r in rows:
        if r.get("dribble_current", "") == "":
            continue
        c = float(r["dribble_current"])
        kind, _ = frame_class(r)
        buckets[kind].append(c)
    allc = buckets["near"] + buckets["mid"] + buckets["far"] + buckets["none"]
    baseline = percentile(allc, 0.10)
    pos = buckets["near"]
    neg = buckets["far"] + buckets["none"]
    th = best_threshold(pos, neg)
    return {
        "path": path,
        "n": len(rows),
        "baseline": baseline,
        "buckets": buckets,
        "auc": auc(pos, neg),
        "threshold": th,
        "near": pos,
        "neg": neg,
    }


def evaluate_margin(infos, margins=(0.05, 0.07, 0.09, 0.12, 0.15)):
    """自适应阈值: current - baseline_file > MARGIN，在 near vs far/none 上的表现。"""
    print("\n[检测评估] 判据 current - baseline_file > MARGIN")
    print("  margin   TPR(near)   FPR(far/none)   balanced_acc")
    for m in margins:
        tp = fn = fp = tn = 0
        for info in infos:
            b = info["baseline"]
            for v in info["near"]:
                if v - b > m:
                    tp += 1
                else:
                    fn += 1
            for v in info["neg"]:
                if v - b > m:
                    fp += 1
                else:
                    tn += 1
        tpr = tp / (tp + fn) if tp + fn else float("nan")
        fpr = fp / (fp + tn) if fp + tn else float("nan")
        print(f"  {m:.2f}     {tpr:.3f}        {fpr:.3f}           {(tpr + (1 - fpr)) / 2:.3f}")


def evaluate_fusion(infos, margin=0.09):
    """电流相对 vision 标签的增量信息：

    - vision=lost 但 球在 near 且电流受载 -> vision 漏判的持球（电流能救回来）
    - vision=held 但 电流不受载       -> 只是球在窗口里、滚轮没咬住
    """
    print(f"\n[融合评估] margin={margin:.2f}A（loaded = current - baseline > margin）")
    counts = {s: {"n": 0, "loaded": 0} for s in ("held", "free", "lost")}
    # 用 +-0.15s 让电流事件和 vision 标签在同一时间窗内匹配（电流有上升/衰减延迟）
    for info in infos:
        b = info["baseline"]
        rows = load_rows(info["path"])
        t = [float(r["t_rel"]) for r in rows]
        loaded = [r.get("dribble_current") != "" and float(r["dribble_current"]) - b > margin
                  for r in rows]
        for i, r in enumerate(rows):
            s = r.get("ball_status", "")
            if s not in counts:
                continue
            counts[s]["n"] += 1
            if loaded[i]:
                counts[s]["loaded"] += 1

    for s in ("held", "free", "lost"):
        n = counts[s]["n"]
        ld = counts[s]["loaded"]
        print(f"  vision={s:4s}: 共{n:5d} 帧, 电流受载 {ld:5d} ({100*ld/max(1,n):5.1f}%)")

    print("  解读：")
    print("   - vision=held 中“受载”比例 = 真正被滚轮咬住的比例；其余只是球进了窗口。")
    print("   - vision=lost/free 中“受载”比例 = 电流能补回的持球（vision 漏判/误判）。")


# ==================== 绘图 ====================

def _font():
    try:
        import matplotlib.font_manager as fm
        names = {f.name for f in fm.fontManager.ttflist}
        for c in ("Noto Sans CJK JP", "Noto Sans CJK SC", "Droid Sans Fallback"):
            if c in names:
                return c
    except Exception:
        pass
    return None


def render(infos, out_png: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["axes.formatter.useoffset"] = False
    plt.rcParams["axes.unicode_minus"] = False
    font = _font()
    if font:
        plt.rcParams["font.family"] = font

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    colors = {"near": "#2ca02c", "mid": "#ff7f0e", "far": "#1f77b4", "none": "#999999"}

    # (a) 距离 vs 电流 散点
    ax = axes[0][0]
    for info in infos:
        rows = load_rows(info["path"])
        for kind in ("near", "mid", "far"):
            xs, ys = [], []
            for r in rows:
                if r.get("dribble_current", "") == "":
                    continue
                k, d = frame_class(r)
                if k == kind:
                    xs.append(d)
                    ys.append(float(r["dribble_current"]))
            ax.scatter(xs, ys, s=5, alpha=0.35, color=colors[kind],
                       label=kind if info is infos[0] else None)
    ax.axvline(NEAR_CM, color="k", linestyle="--", linewidth=0.8)
    ax.axvline(FAR_CM, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("球距离 hypot(x,y) (cm)")
    ax.set_ylabel("盘球电流 (A)")
    ax.set_title("(a) 球距离 vs 电流：越靠近接球窗口电流越高")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # (b) 电流分布：near vs far/none
    ax = axes[0][1]
    for info in infos:
        ax.hist(info["near"], bins=40, range=(0.2, 1.2), histtype="step",
                color=colors["near"], alpha=0.6, label="near" if info is infos[0] else None)
        ax.hist(info["neg"], bins=40, range=(0.2, 1.2), histtype="step",
                color=colors["none"], alpha=0.6, label="far/none" if info is infos[0] else None)
    ax.set_xlabel("盘球电流 (A)")
    ax.set_ylabel("帧数")
    ax.set_title("(b) 电流分布：near(绿) vs far/none(灰)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (c) ROC
    ax = axes[1][0]
    import bisect
    pos = [v for info in infos for v in info["near"]]
    neg = [v for info in infos for v in info["neg"]]
    if pos and neg:
        cands = sorted(set(pos + neg))
        sneg = sorted(neg)
        tprs, fprs = [], []
        for t in cands:
            tprs.append(sum(1 for v in pos if v >= t) / len(pos))
            fprs.append(sum(1 for v in neg if v >= t) / len(neg))
        ax.plot(fprs, tprs, color="#333")
        ax.plot([0, 1], [0, 1], ":", color="gray")
        pooled = auc(pos, neg)
        ax.set_title(f"(c) ROC   pooled AUC={pooled:.3f}")
    ax.set_xlabel("误报率 FPR")
    ax.set_ylabel("召回率 TPR")
    ax.grid(True, alpha=0.3)

    # (d) 每个文件的 AUC / 基线 / 阈值
    ax = axes[1][1]
    ax.axis("off")
    lines = ["file                      base   AUC   best_t  TPR  FPR"]
    for info in infos:
        th = info["threshold"]
        if th:
            lines.append(f"{info['path'].name[-18:]:22s} {info['baseline']:.3f} "
                         f"{info['auc']:.3f}  {th[0]:.3f}  {th[1]:.2f} {th[2]:.2f}")
        else:
            lines.append(f"{info['path'].name[-18:]:22s} {info['baseline']:.3f} "
                         f"{info['auc']:.3f}   -     -    -")
    ax.text(0, 1, "\n".join(lines), family="monospace", fontsize=9, va="top")

    fig.suptitle("盘球电流 x 球坐标：持球/接触检测分析", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def render_timeline(path: Path, out_png: Path, baseline: float, threshold: float):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["axes.formatter.useoffset"] = False
    plt.rcParams["axes.unicode_minus"] = False
    font = _font()
    if font:
        plt.rcParams["font.family"] = font

    rows = load_rows(path)
    t = [float(r["t_rel"]) for r in rows]
    cur = [float(r["dribble_current"]) if r.get("dribble_current") else float("nan") for r in rows]
    dist = []
    for r in rows:
        _, d = frame_class(r)
        dist.append(d if d is not None else float("nan"))
    status = [r.get("ball_status", "") for r in rows]

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(15, 7), sharex=True)
    a1.plot(t, cur, color="#d62728", linewidth=0.8)
    a1.axhline(baseline, color="gray", linestyle=":", label=f"baseline={baseline:.3f}")
    a1.axhline(threshold, color="green", linestyle="--", label=f"detect_t={threshold:.3f}")
    a1.set_ylabel("盘球电流 (A)")
    a1.legend(fontsize=8, loc="upper right")
    a1.grid(True, alpha=0.3)

    a2.plot(t, dist, color="#1f77b4", linewidth=0.8)
    a2.axhline(NEAR_CM, color="green", linestyle="--", linewidth=0.8)
    a2.axhline(FAR_CM, color="gray", linestyle="--", linewidth=0.8)
    a2.set_ylabel("球距离 (cm)")
    a2.set_xlabel("时间 t (s)")
    a2.grid(True, alpha=0.3)

    # 用背景色标出 vision 的 held 段
    held = [s == "held" for s in status]
    start = None
    for i, h in enumerate(held + [False]):
        if h and start is None:
            start = t[i]
        elif not h and start is not None:
            for ax in (a1, a2):
                ax.axvspan(start, t[i - 1] if i - 1 >= 0 else start, color="green", alpha=0.08)
            start = None

    fig.suptitle(f"时间线：{path.name}（绿色阴影=vision 判定 held）", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def write_report(out_html, pngs, infos, timeline_file):
    imgs = "\n".join(
        f'<figure><img src="data:image/png;base64,'
        f'{base64.b64encode(p.read_bytes()).decode("ascii")}"/>'
        f"<figcaption>{html.escape(c)}</figcaption></figure>"
        for p, c in pngs
    )
    print_lines = []
    for info in infos:
        th = info["threshold"]
        print_lines.append(
            f"{info['path'].name}: baseline={info['baseline']:.3f}A, AUC={info['auc']:.3f}"
            + (f", best_t={th[0]:.3f}, TPR={th[1]:.2f}, FPR={th[2]:.2f}" if th else ""))
    body = "\n".join(f"<li>{html.escape(s)}</li>" for s in print_lines)
    out_html.write_text(f"""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>盘球电流 x 球坐标 分析</title>
<style>body{{font-family:-apple-system,"Noto Sans CJK SC",sans-serif;margin:24px;background:#fafafa}}
img{{max-width:100%;border:1px solid #ddd;border-radius:6px}} code{{background:#eee;padding:1px 5px}}</style>
</head><body>
<h1>盘球电流 x 球坐标 持球/接触检测分析</h1>
<p>生成时间 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}；near=d&lt;{NEAR_CM}cm，far=d&gt;={FAR_CM}cm</p>
<ul>{body}</ul>
{imgs}
</body></html>""", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description="盘球电流 x 球坐标 持球检测分析")
    parser.add_argument("csvs", nargs="*", type=Path, help="要分析的 CSV；缺省用 Records/ball_current/*.csv")
    parser.add_argument("--out", type=Path, default=Path("Records/analysis"))
    args = parser.parse_args(argv)

    root = _SRC
    csvs = args.csvs or sorted(Path(root).glob(CSV_GLOB))
    if not csvs:
        print("没有找到 CSV")
        return 1

    out_dir = args.out
    if not out_dir.is_absolute():
        out_dir = root / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    infos = [analyze_file(p) for p in csvs]
    print(f"分析 {len(infos)} 个文件")
    for info in infos:
        th = info["threshold"]
        line = (f"{info['path'].name}: base={info['baseline']:.3f}A AUC={info['auc']:.3f}")
        if th:
            line += f" best_t={th[0]:.3f} TPR={th[1]:.2f} FPR={th[2]:.2f}"
        print("  " + line)

    evaluate_margin(infos)
    evaluate_fusion(infos)

    png1 = out_dir / "analysis_dist_roc.png"
    render(infos, png1)

    # 选数据最丰富、AUC 最高的文件画时间线
    rich = max(infos, key=lambda i: (i["n"], i["auc"] if i["auc"] == i["auc"] else 0))
    th = rich["threshold"]
    th_t = th[0] if th else rich["baseline"] + 0.12
    png2 = out_dir / "analysis_timeline.png"
    render_timeline(rich["path"], png2, rich["baseline"], th_t)

    out_html = out_dir / "analysis.html"
    write_report(out_html, [(png1, "距离-电流 / 分布 / ROC"), (png2, f"时间线示例：{rich['path'].name}")],
                 infos, rich["path"])
    print(f"[done] {out_html}")
    print(f"[done] {png1}")
    print(f"[done] {png2}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
