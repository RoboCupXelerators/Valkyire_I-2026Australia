#!/usr/bin/env python3
"""球坐标 / 盘球电流采集与可视化工具。

功能
----
1. 运行时按固定频率采集：
   - 球相对机身坐标 ``UnitedPosition.RobotBallPos()`` -> ``[x, y]``（cm）
   - 盘球电流 ``Bits.GetDribbleCurrent()`` -> 安培（A）
   - 机器人场地位姿 ``UnitedPosition.FieldRobotPose()`` -> ``[x, y, yaw]``
   - 抱球状态 ``UnitedPosition.BallStatus()`` -> ``lost`` / ``free`` / ``held``
2. 全程写入 CSV（采集中途 Ctrl-C 也不丢数据）。
3. 采集结束后自动生成静态折线图（PNG）和一个自包含的 HTML 报告。

图的特点
--------
- x 轴为时间 t(s)，x / y 轴都自动伸缩，y 轴**不从 0 开始**，贴合数据真实区间。
- 球坐标 y 轴刻度标注到小数点后 2 位，电流 y 轴刻度标注到小数点后 3 位，
  配合自动缩放，能清楚看到 0.01 / 0.001 级别的变化。
- 丢球时段（哨兵值）会断开折线，不会画成一条假直线。

用法
----
    cd ~/Software/src
    python3 ball_current_viz.py --duration 30          # 采集 30 秒后出图
    python3 ball_current_viz.py --duration 0           # 一直采集，Ctrl-C 停止并出图
    python3 ball_current_viz.py --rate 50              # 50 Hz
    python3 ball_current_viz.py --dribble 0.3          # 采集期间盘球开转(0.3)，结束自动关
    python3 ball_current_viz.py --from-csv Records/ball_current/xxx.csv   # 离线出图
    python3 ball_current_viz.py --open                 # 采集完成后在 :0 屏幕上打开 HTML

输出默认放在 ``src/Records/ball_current/`` 下。

如果要在**已经运行策略**的进程里采集（避免两个进程抢串口 / 视觉），
用可嵌入的 ``BallCurrentRecorder``，在策略主循环里逐帧 ``sample()``：

    from ball_current_viz import BallCurrentRecorder
    rec = BallCurrentRecorder("Records/ball_current/run.csv")
    # 每帧: rec.sample(UnitedPosition, Bits)
    # 结束: rec.close(); rec.render()
"""

from __future__ import annotations

import argparse
import base64
import csv
import html
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# ==================== 配置区 ====================

DEFAULT_DURATION_S = 30.0
DEFAULT_RATE_HZ = 50.0
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "Records" / "ball_current"

# 与仓库一致的哨兵值
BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]
POSE_NOT_FOUND = [0xFFF, 0xFFF, 0xFFF]

# 刻度精度：球坐标 2 位小数，电流 3 位小数
BALL_DECIMALS = 2
CURRENT_DECIMALS = 3

BALL_UNIT = "cm"
CURRENT_UNIT = "A"

CSV_FIELDS = [
    "t_wall", "t_rel",
    "ball_x", "ball_y", "ball_status",
    "dribble_current", "dribble_cmd",
    "robot_x", "robot_y", "robot_yaw",
]


# ==================== 工具函数 ====================

def _finite(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _valid_ball(ball):
    """把哨兵值 / 非法坐标折叠成 None。"""
    if ball is None:
        return None
    try:
        x = float(ball[0])
        y = float(ball[1])
    except (TypeError, IndexError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    if x >= float(BALL_HELD[0]) or y >= float(BALL_HELD[1]):
        return None
    return x, y


def _valid_pose(pose):
    if pose is None or len(pose) < 3:
        return None
    try:
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    except (TypeError, IndexError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None
    if x >= 0xFFF or y >= 0xFFF:
        return None
    return x, y, yaw


def _pick_cjk_font():
    """返回一个可用的中文字体名，找不到就返回 None。"""
    try:
        import matplotlib.font_manager as fm
    except Exception:
        return None
    names = {f.name for f in fm.fontManager.ttflist}
    for candidate in ("Noto Sans CJK JP", "Noto Sans CJK SC", "Droid Sans Fallback",
                      "WenQuanYi Micro Hei", "SimHei"):
        if candidate in names:
            return candidate
    return None


def _robot_number():
    """从 config.json 读取本机编号，用于图表标题。"""
    path = Path(__file__).resolve().parent / "ReasonData" / "data" / "config.json"
    try:
        cfg = json.loads(path.read_text(encoding="utf-8"))
        return cfg.get("RoboInfo", {}).get("number", "?")
    except Exception:
        return "?"


# ==================== 采集 ====================

def _read_ball(UnitedPosition):
    try:
        ball = UnitedPosition.RobotBallPos()
    except Exception:
        return None, None, "error"
    valid = _valid_ball(ball)
    try:
        status = UnitedPosition.BallStatus(ball)
    except Exception:
        status = "unknown"
    if valid is None:
        return None, None, status
    return valid[0], valid[1], status


def _read_current(Bits):
    try:
        cur = Bits.GetDribbleCurrent()
    except Exception:
        return None
    return float(cur) if _finite(cur) else None


def _read_pose(UnitedPosition):
    try:
        pose = UnitedPosition.FieldRobotPose()
    except Exception:
        return None, None, None
    valid = _valid_pose(pose)
    if valid is None:
        return None, None, None
    return valid


class BallCurrentRecorder:
    """采集器：可独立运行，也可嵌进已有策略的循环里逐帧调用。

    嵌进策略时不要新开进程（否则会和正在跑的进程抢串口 / 视觉），
    直接在策略主循环里调用 :meth:`sample` 即可::

        from ball_current_viz import BallCurrentRecorder
        rec = BallCurrentRecorder("Records/ball_current/manual.csv")
        # 每帧：
        rec.sample(UnitedPosition, Bits)
        # 结束时：
        rec.close()
        rec.render()
    """

    def __init__(self, csv_path, *, UnitedPosition=None, Bits=None):
        self.csv_path = Path(csv_path).expanduser()
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._fh, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self.start_time = time.time()
        self.rows: list[dict] = []
        self._UnitedPosition = UnitedPosition
        self._Bits = Bits
        self._dribble_cmd = None
        self._closed = False

    def _sources(self):
        if self._UnitedPosition is None or self._Bits is None:
            from LunaPre import UnitedPosition, Bits  # 用到时才 import，离线出图不需要硬件
            self._UnitedPosition = UnitedPosition
            self._Bits = Bits
        return self._UnitedPosition, self._Bits

    def set_dribble_cmd(self, power):
        """记录当前下发的盘球功率（分析电流时要按功率分档看基线）。"""
        self._dribble_cmd = None if power is None else float(power)

    def sample(self, UnitedPosition=None, Bits=None, dribble_cmd=None):
        """采一帧，写入 CSV 并返回该行。

        可以显式传入当前的 ``UnitedPosition`` / ``Bits``（策略里推荐这样），
        也可以不传，由采集器自己 import。``dribble_cmd`` 是这一帧下发的盘球
        功率（True/False/0~1 的 float），不传就沿用上次 set_dribble_cmd 的值。
        """
        if UnitedPosition is not None:
            self._UnitedPosition = UnitedPosition
        if Bits is not None:
            self._Bits = Bits
        if dribble_cmd is not None:
            self.set_dribble_cmd(dribble_cmd)
        UnitedPosition, Bits = self._sources()
        t_wall = time.time()
        bx, by, status = _read_ball(UnitedPosition)
        cur = _read_current(Bits)
        rx, ry, ryaw = _read_pose(UnitedPosition)
        row = {
            "t_wall": round(t_wall, 6),
            "t_rel": round(t_wall - self.start_time, 6),
            "ball_x": round(bx, 5) if bx is not None else "",
            "ball_y": round(by, 5) if by is not None else "",
            "ball_status": status,
            "dribble_current": round(cur, 6) if cur is not None else "",
            "dribble_cmd": "" if self._dribble_cmd is None else round(self._dribble_cmd, 3),
            "robot_x": round(rx, 4) if rx is not None else "",
            "robot_y": round(ry, 4) if ry is not None else "",
            "robot_yaw": round(ryaw, 4) if ryaw is not None else "",
        }
        self._writer.writerow(row)
        self.rows.append(row)
        return row

    def flush(self):
        self._fh.flush()

    def close(self):
        if not self._closed:
            self._fh.flush()
            self._fh.close()
            self._closed = True

    def render(self, out_dir=None, tag=None):
        """出图 + HTML，返回 (html_path, [png_paths])。"""
        out_dir = Path(out_dir).expanduser() if out_dir else self.csv_path.parent
        out_dir.mkdir(parents=True, exist_ok=True)
        return build_report(self.rows, out_dir, tag or self.csv_path.stem, self.csv_path)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def collect(duration_s: float, rate_hz: float, csv_path: Path, dribble_power=None):
    """独立采集：定时循环调用 recorder.sample()，期间实时写盘。

    ``dribble_power`` 给定（如 0.3 / 1.0）时，采集期间持续开转盘球，
    结束时自动关掉。全程在本进程主循环里完成，不新开线程 / 进程。
    """
    rec = BallCurrentRecorder(csv_path)
    t0 = rec.start_time
    period = 1.0 / max(rate_hz, 1e-6)
    next_t = t0
    last_report = t0

    peripheral = None
    if dribble_power is not None:
        from LunaPre import peripheral  # 借同一个进程的 IO 出口，避免抢硬件
        rec.set_dribble_cmd(dribble_power)
        if not peripheral.Permission():
            print("[warn] 裁判未放行 (Permission=False)，盘球不会转。"
                  "真机无裁判调试时请用 ROBOCUPX_DEBUG=1 运行。")

    print(f"[collect] 开始采集: rate={rate_hz:g}Hz, "
          f"duration={'Ctrl-C' if duration_s <= 0 else f'{duration_s:g}s'}"
          + (f", 盘球 power={dribble_power:g}" if dribble_power is not None else ""))
    try:
        while True:
            now = time.time()
            if duration_s > 0 and now - t0 >= duration_s:
                break
            if peripheral is not None:
                # 每帧下发一次；chassis 缓存了状态，值没变不会重复发帧。
                peripheral.Dribble(float(dribble_power))
            rec.sample()
            if now - last_report >= 1.0:
                rec.flush()
                last_report = now
                print(f"\r[collect] {now - t0:6.1f}s / {len(rec.rows)} 帧", end="", flush=True)
            next_t += period
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.time()  # 落后了就重置节拍，避免追帧风暴
    except KeyboardInterrupt:
        print("\n[collect] 收到 Ctrl-C，停止采集")
    finally:
        rec.close()
        if peripheral is not None:
            try:
                peripheral.Dribble(False)
                print("[collect] 盘球已关闭")
            except Exception as error:
                print(f"[warn] 关闭盘球失败: {error}")

    print(f"\r[collect] 完成: {len(rec.rows)} 帧, {time.time() - t0:.1f}s -> {csv_path}")
    return rec.rows


def read_csv(csv_path: Path):
    rows = []
    with csv_path.open("r", newline="", encoding="utf-8") as fh:
        for raw in csv.DictReader(fh):
            rows.append(raw)
    return rows


def _rows_to_arrays(rows):
    """把 CSV 行转成浮点数组，空串 / 非法值 -> None。"""
    def col(name):
        out = []
        for r in rows:
            v = r.get(name, "")
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                out.append(None)
        return out

    t = col("t_rel")
    if any(v is None for v in t):  # 兜底：没有 t_rel 就用行号
        t = [float(i) for i in range(len(rows))]
    data = {
        "t": t,
        "ball_x": col("ball_x"),
        "ball_y": col("ball_y"),
        "dribble_current": col("dribble_current"),
        "robot_x": col("robot_x"),
        "robot_y": col("robot_y"),
        "ball_status": [r.get("ball_status", "") for r in rows],
    }
    return data


# ==================== 绘图 ====================

def _auto_ylim(values, decimals):
    """贴合数据区间自动缩放（不从 0 开始），只留很小边距让曲线铺满视窗。

    边距取“数据区间的 10%”和“1/4 个最小显示刻度”里的较大者：
    - 数据有波动时，用 10% 边距，曲线不会被压扁；
    - 数据几乎不变时，用 1/4 刻度兜底，平线也不会贴边。
    """
    vals = [float(v) for v in values if _finite(v)]
    if not vals:
        return None
    lo, hi = min(vals), max(vals)
    step = 10.0 ** (-decimals)
    pad = max((hi - lo) * 0.10, step * 0.25)
    return lo - pad, hi + pad


def _tick_step(span, decimals, target=6):
    """选一个步长：既是 1/2/2.5/5 x 10^k 这种好读的数字，
    又不会小于小数点后 decimals 位，避免刻度标签出现重复。"""
    base = 10.0 ** (-decimals)
    if span <= 0 or not math.isfinite(span):
        return base
    units = (span / target) / base
    mag = 10.0 ** math.floor(math.log10(units)) if units > 0 else 1.0
    for nice in (1.0, 2.0, 2.5, 5.0, 10.0):
        if nice * mag >= units:
            return max(nice * mag * base, base)
    return max(10.0 * mag * base, base)


def _apply_yaxis(ax, values, decimals, ylabel=None):
    """设置 y 轴范围 + 精度受控的刻度，保证 0.01 / 0.001 级变化可读。"""
    import matplotlib.ticker as mticker

    lim = _auto_ylim(values, decimals)
    if lim:
        ax.set_ylim(*lim)
        lo, hi = lim
    else:
        lo, hi = ax.get_ylim()
    ax.yaxis.set_major_locator(mticker.MultipleLocator(_tick_step(hi - lo, decimals)))
    ax.yaxis.set_major_formatter(mticker.FormatStrFormatter(f"%.{decimals}f"))
    if ylabel:
        ax.set_ylabel(ylabel)


def _series(ax, t, values, color, label, decimals, ylabel):
    xs = [ti for ti, v in zip(t, values) if _finite(v)]
    ys = [float(v) for v in values if _finite(v)]
    if not xs:
        ax.set_ylabel(ylabel)
        ax.text(0.5, 0.5, "无有效数据", ha="center", va="center",
                transform=ax.transAxes, color="gray")
        return

    ax.plot(xs, ys, color=color, linewidth=1.0, label=label)
    _apply_yaxis(ax, ys, decimals, ylabel)

    # 最小 / 最大参考线与数值标注
    vmin, vmax = min(ys), max(ys)
    ax.axhline(vmin, color=color, linewidth=0.6, linestyle=":", alpha=0.6)
    ax.axhline(vmax, color=color, linewidth=0.6, linestyle=":", alpha=0.6)
    ax.annotate(f"min {vmin:.{decimals}f}", xy=(xs[0], vmin), xytext=(3, 3),
                textcoords="offset points", fontsize=7, color=color)
    ax.annotate(f"max {vmax:.{decimals}f}", xy=(xs[0], vmax), xytext=(3, 3),
                textcoords="offset points", fontsize=7, color=color)

    ax.grid(True, linestyle="--", alpha=0.35)
    ax.text(0.995, 0.03, f"Δ={vmax - vmin:.{decimals}f}", transform=ax.transAxes,
            ha="right", va="bottom", fontsize=7, color=color)


def render_timeseries(data, out_png: Path, title: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["axes.formatter.useoffset"] = False
    plt.rcParams["axes.unicode_minus"] = False
    font = _pick_cjk_font()
    if font:
        plt.rcParams["font.family"] = font

    t = data["t"]
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    _series(axes[0], t, data["ball_x"], "#1f77b4", "Ball X",
            BALL_DECIMALS, f"球 X ({BALL_UNIT})")
    _series(axes[1], t, data["ball_y"], "#ff7f0e", "Ball Y",
            BALL_DECIMALS, f"球 Y ({BALL_UNIT})")
    _series(axes[2], t, data["dribble_current"], "#d62728", "Dribble Current",
            CURRENT_DECIMALS, f"盘球电流 ({CURRENT_UNIT})")

    axes[-1].set_xlabel("时间 t (s)")
    fig.suptitle(title, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def render_overlay(data, out_png: Path, title: str):
    """球 X / 球 Y（左轴）与 盘球电流（右轴）同时间轴叠加，直接看关系。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["axes.formatter.useoffset"] = False
    plt.rcParams["axes.unicode_minus"] = False
    font = _pick_cjk_font()
    if font:
        plt.rcParams["font.family"] = font

    t = data["t"]
    fig, ax1 = plt.subplots(figsize=(14, 6))
    ax2 = ax1.twinx()

    for values, color, label in (
        (data["ball_x"], "#1f77b4", "Ball X"),
        (data["ball_y"], "#ff7f0e", "Ball Y"),
    ):
        xs = [ti for ti, v in zip(t, values) if _finite(v)]
        ys = [float(v) for v in values if _finite(v)]
        if xs:
            ax1.plot(xs, ys, color=color, linewidth=1.0, label=label)

    cx = [ti for ti, v in zip(t, data["dribble_current"]) if _finite(v)]
    cy = [float(v) for v in data["dribble_current"] if _finite(v)]
    if cx:
        ax2.plot(cx, cy, color="#d62728", linewidth=1.0, label="Dribble Current")

    ax1.set_xlabel("时间 t (s)")
    _apply_yaxis(ax1, [v for v in data["ball_x"] + data["ball_y"]],
                 BALL_DECIMALS, f"球坐标 ({BALL_UNIT})")
    _apply_yaxis(ax2, data["dribble_current"],
                 CURRENT_DECIMALS, f"盘球电流 ({CURRENT_UNIT})")
    ax1.grid(True, linestyle="--", alpha=0.35)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=9)
    fig.suptitle(title, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def render_trajectory(data, out_png: Path, title: str):
    """球在机身坐标系里的运动轨迹（x-y），按盘球电流着色。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["axes.formatter.useoffset"] = False
    plt.rcParams["axes.unicode_minus"] = False
    font = _pick_cjk_font()
    if font:
        plt.rcParams["font.family"] = font

    xs, ys, cs = [], [], []
    for x, y, c in zip(data["ball_x"], data["ball_y"], data["dribble_current"]):
        if _finite(x) and _finite(y):
            xs.append(float(x))
            ys.append(float(y))
            cs.append(float(c) if _finite(c) else float("nan"))

    fig, ax = plt.subplots(figsize=(8, 8))
    if xs:
        sc = ax.scatter(xs, ys, c=cs, cmap="turbo", s=8)
        fig.colorbar(sc, ax=ax, label=f"盘球电流 ({CURRENT_UNIT})")
        ax.plot(xs, ys, color="gray", linewidth=0.4, alpha=0.4)
    else:
        ax.text(0.5, 0.5, "无有效球坐标", ha="center", va="center",
                transform=ax.transAxes, color="gray")
    ax.axhline(0, color="k", linewidth=0.5)
    ax.axvline(0, color="k", linewidth=0.5)
    ax.set_xlabel(f"球 X ({BALL_UNIT})")
    ax.set_ylabel(f"球 Y ({BALL_UNIT})")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


# ==================== 报告 ====================

def _stats(values, decimals):
    vals = [float(v) for v in values if _finite(v)]
    if not vals:
        return None
    return {
        "n": len(vals),
        "min": min(vals),
        "max": max(vals),
        "mean": sum(vals) / len(vals),
        "last": vals[-1],
        "range": max(vals) - min(vals),
        "decimals": decimals,
    }


def _fmt_stat(s):
    if not s:
        return "—"
    d = s["decimals"]
    return (f"{s['min']:.{d}f} ~ {s['max']:.{d}f}"
            f" (Δ={s['range']:.{d}f}, mean={s['mean']:.{d}f}, last={s['last']:.{d}f})")


def _embed_png(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")


def write_html(out_html: Path, csv_path: Path, pngs, stats, n_rows, duration_s):
    img_blocks = "\n".join(
        f'<figure><img src="data:image/png;base64,{_embed_png(p)}" alt="{p.name}"/>'
        f"<figcaption>{html.escape(caption)}</figcaption></figure>"
        for p, caption in pngs
    )
    rows = "\n".join(
        f"<tr><td>{html.escape(name)}</td><td>{html.escape(_fmt_stat(stats[name]))}</td></tr>"
        for name in ("ball_x", "ball_y", "dribble_current")
    )
    doc = f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>球坐标 / 盘球电流 可视化</title>
<style>
  body {{ font-family: -apple-system, "Noto Sans CJK SC", sans-serif; margin: 24px; background:#fafafa; color:#222; }}
  h1 {{ font-size: 20px; }}
  .meta {{ color:#666; font-size: 13px; margin-bottom: 16px; }}
  figure {{ margin: 0 0 28px; }}
  img {{ max-width: 100%; border:1px solid #ddd; border-radius:6px; background:#fff; }}
  figcaption {{ font-size: 13px; color:#555; margin-top:6px; }}
  table {{ border-collapse: collapse; font-size: 13px; background:#fff; }}
  td {{ border:1px solid #e0e0e0; padding:6px 12px; }}
  code {{ background:#eee; padding:1px 5px; border-radius:4px; }}
</style></head>
<body>
<h1>球坐标 / 盘球电流 采集可视化</h1>
<div class="meta">
  样本数 <b>{n_rows}</b> · 时长 <b>{duration_s:.2f}s</b> ·
  数据文件 <code>{html.escape(str(csv_path))}</code> ·
  生成时间 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
</div>
<table><tr><td>通道</td><td>最小 ~ 最大 (均值 / 末值)</td></tr>
{rows}
</table>
{img_blocks}
</body></html>
"""
    out_html.write_text(doc, encoding="utf-8")


# ==================== 主流程 ====================

def build_report(rows, out_dir: Path, tag: str, csv_path: Path):
    data = _rows_to_arrays(rows)
    t = data["t"]
    duration_s = (t[-1] - t[0]) if len(t) >= 2 else 0.0

    title = f"球坐标 / 盘球电流  (Robot {_robot_number()}, {tag})"

    png_ts = out_dir / f"ball_current_{tag}_timeseries.png"
    png_ov = out_dir / f"ball_current_{tag}_overlay.png"
    png_tr = out_dir / f"ball_current_{tag}_trajectory.png"

    render_timeseries(data, png_ts, title)
    render_overlay(data, png_ov, title)
    render_trajectory(data, png_tr, title)

    stats = {
        "ball_x": _stats(data["ball_x"], BALL_DECIMALS),
        "ball_y": _stats(data["ball_y"], BALL_DECIMALS),
        "dribble_current": _stats(data["dribble_current"], CURRENT_DECIMALS),
    }

    out_html = out_dir / f"ball_current_{tag}.html"
    write_html(
        out_html, csv_path,
        [(png_ts, "1) 时间序列：球 X / 球 Y / 盘球电流（y 轴自动缩放，不从 0 开始）"),
         (png_ov, "2) 叠加对比：球坐标与盘球电流在同一时间轴上"),
         (png_tr, "3) 机身坐标系下球的运动轨迹（颜色=盘球电流）")],
        stats, len(rows), duration_s,
    )
    return out_html, [png_ts, png_ov, png_tr]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="采集球坐标 / 盘球电流并生成折线图与 HTML 报告")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S,
                        help="采集时长(秒)，<=0 表示一直采集到 Ctrl-C（默认 %g）" % DEFAULT_DURATION_S)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                        help="采样频率 Hz（默认 %g）" % DEFAULT_RATE_HZ)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT_DIR,
                        help="输出目录（默认 %s）" % DEFAULT_OUT_DIR)
    parser.add_argument("--tag", default=None, help="文件名标签（默认时间戳）")
    parser.add_argument("--dribble", type=float, default=None, metavar="POWER",
                        help="采集期间持续开转盘球（0~1，例如 0.3 或 1.0），结束自动关闭；"
                             "不传则不动盘球")
    parser.add_argument("--from-csv", type=Path, default=None,
                        help="跳过采集，直接根据已有 CSV 出图")
    parser.add_argument("--open", action="store_true",
                        help="生成后用 xdg-open 在 :0 屏幕上打开 HTML")
    args = parser.parse_args(argv)

    tag = args.tag or datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out.expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.from_csv:
        csv_path = args.from_csv.expanduser()
        rows = read_csv(csv_path)
        print(f"[render] 读取 {csv_path} ({len(rows)} 行)")
    else:
        csv_path = out_dir / f"ball_current_{tag}.csv"
        rows = collect(args.duration, args.rate, csv_path, dribble_power=args.dribble)

    out_html, pngs = build_report(rows, out_dir, tag, csv_path)

    print(f"[done] CSV : {csv_path}")
    for p in pngs:
        print(f"[done] PNG : {p}")
    print(f"[done] HTML: {out_html}")

    if args.open:
        env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
        try:
            subprocess.Popen(["xdg-open", str(out_html)], env=env,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[done] 已尝试在 :0 打开 HTML")
        except Exception as e:
            print(f"[warn] 打开 HTML 失败: {e}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
