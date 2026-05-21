"""Compare real robot and simulator motion-response CSV files.

Inputs:
  --real Records/friction/friction_real_sampling_20260519_190045.csv
  --sim  Records/sim_motion/sim_motion_sampling_YYYYMMDD_HHMMSS.csv

The comparison uses medians and filters absurd outliers from localization
jumps, because the existing real collection contains occasional pose spikes.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


@dataclass
class MotionStats:
    axis: str
    level: float
    sign: str
    median_rate: float
    sample_count: int


def _rate_field(axis: str) -> str:
    if axis == "x":
        return "est_vx_cm_s"
    if axis == "y":
        return "est_vy_cm_s"
    if axis == "w":
        return "est_wz_deg_s"
    raise ValueError(axis)


def _command_sign(row: dict, axis: str) -> str:
    field = {"x": "cmd_vx", "y": "cmd_vy", "w": "cmd_wz"}[axis]
    return "+" if float(row.get(field, 0.0)) >= 0 else "-"


def _inlier_abs_rate(axis: str, value: float) -> bool:
    value = abs(value)
    if axis == "w":
        return 0.05 <= value <= 250.0
    return 0.05 <= value <= 250.0


def _robust_median(values: list[float]) -> float | None:
    if not values:
        return None
    med = statistics.median(values)
    deviations = [abs(v - med) for v in values]
    mad = statistics.median(deviations) if deviations else 0.0
    if mad <= 1e-9:
        return med
    filtered = [v for v in values if abs(v - med) <= 4.0 * mad]
    return statistics.median(filtered or values)


def load_action_stats(path: Path) -> dict[tuple[str, float, str], MotionStats]:
    grouped: dict[tuple[str, float, str], list[float]] = defaultdict(list)
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            phase = str(row.get("phase", ""))
            if "coast" in phase:
                continue
            axis = str(row.get("axis", "")).strip()
            if axis not in {"x", "y", "w"}:
                continue
            try:
                level = float(row.get("level", 0.0))
                value = float(row.get(_rate_field(axis), 0.0))
            except Exception:
                continue
            if not _inlier_abs_rate(axis, value):
                continue
            sign = _command_sign(row, axis)
            grouped[(axis, level, sign)].append(abs(value))

    stats = {}
    for key, values in grouped.items():
        median = _robust_median(values)
        if median is None:
            continue
        axis, level, sign = key
        stats[key] = MotionStats(axis, level, sign, median, len(values))
    return stats


def compare(real: dict, sim: dict) -> list[dict]:
    rows = []
    for key in sorted(real):
        if key not in sim:
            continue
        r = real[key]
        s = sim[key]
        ratio = s.median_rate / r.median_rate if r.median_rate else 0.0
        rows.append(
            {
                "axis": r.axis,
                "level": r.level,
                "sign": r.sign,
                "real_median_rate": round(r.median_rate, 4),
                "sim_median_rate": round(s.median_rate, 4),
                "sim_over_real": round(ratio, 4),
                "real_n": r.sample_count,
                "sim_n": s.sample_count,
            }
        )
    return rows


def recommend(rows: list[dict], current_linear: float, current_angular: float) -> dict:
    by_axis: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        ratio = float(row["sim_over_real"])
        if 0.05 <= ratio <= 20.0:
            by_axis[str(row["axis"])].append(ratio)

    rec = {}
    for axis, ratios in by_axis.items():
        med_ratio = statistics.median(ratios)
        if axis == "w":
            rec[axis] = {
                "current": current_angular,
                "median_sim_over_real": med_ratio,
                "suggested": current_angular / med_ratio if med_ratio else current_angular,
            }
        else:
            rec[axis] = {
                "current": current_linear,
                "median_sim_over_real": med_ratio,
                "suggested": current_linear / med_ratio if med_ratio else current_linear,
            }
    if "x" in rec and "y" in rec:
        rec["linear_combined"] = {
            "current": current_linear,
            "suggested": statistics.median([rec["x"]["suggested"], rec["y"]["suggested"]]),
        }
    return rec


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--real", type=Path, default=Path("Records/friction/friction_real_sampling_20260519_190045.csv"))
    parser.add_argument("--sim", type=Path)
    parser.add_argument("--out", type=Path, default=Path("Records/sim_motion/motion_compare_latest.json"))
    parser.add_argument("--current-linear", type=float, default=0.48)
    parser.add_argument("--current-angular", type=float, default=1.36)
    args = parser.parse_args()

    real = load_action_stats(args.real)
    if not args.sim:
        payload = {
            "real_file": str(args.real),
            "real_stats": [vars(item) for item in real.values()],
            "note": "Pass --sim to compare against simulator sampling.",
        }
    else:
        sim = load_action_stats(args.sim)
        rows = compare(real, sim)
        payload = {
            "real_file": str(args.real),
            "sim_file": str(args.sim),
            "comparisons": rows,
            "recommendations": recommend(rows, args.current_linear, args.current_angular),
        }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
