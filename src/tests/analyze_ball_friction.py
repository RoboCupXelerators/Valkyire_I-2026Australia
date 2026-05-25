"""Analyze ball rolling friction CSV collected from real-field trials."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


def _float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        raw = row.get(key, "")
        if raw is None or str(raw).strip() == "":
            return default
        return float(raw)
    except Exception:
        return default


def _visible(row: dict[str, str]) -> bool:
    raw = str(row.get("visible", "1")).strip().lower()
    return raw not in {"0", "false", "no", "bad"}


def _median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * q)))
    return ordered[idx]


def _linear_slope(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 2 or len(xs) != len(ys):
        return None
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    denom = sum((x - mean_x) ** 2 for x in xs)
    if denom <= 1e-9:
        return None
    return sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom


def _path_length(points: list[dict[str, Any]], until_t: float | None = None) -> float:
    length = 0.0
    prev = None
    for row in points:
        if until_t is not None and row["t_s"] > until_t:
            break
        if prev is not None:
            length += math.hypot(row["x_cm"] - prev["x_cm"], row["y_cm"] - prev["y_cm"])
        prev = row
    return length


def load_trials(path: Path) -> dict[str, list[dict[str, Any]]]:
    trials: dict[str, list[dict[str, Any]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not _visible(row):
                continue
            trial_id = str(row.get("trial_id") or "trial").strip()
            trials[trial_id].append(
                {
                    "trial_id": trial_id,
                    "session_id": row.get("session_id", ""),
                    "mode": row.get("mode", ""),
                    "strength": row.get("strength", ""),
                    "t_s": _float(row, "t_s"),
                    "x_cm": _float(row, "x_cm"),
                    "y_cm": _float(row, "y_cm"),
                    "notes": row.get("notes", ""),
                }
            )
    for rows in trials.values():
        rows.sort(key=lambda item: item["t_s"])
        if rows:
            t0 = rows[0]["t_s"]
            for row in rows:
                row["t_s"] -= t0
    return trials


def analyze_trial(points: list[dict[str, Any]], *, stop_speed_cm_s: float) -> dict[str, Any] | None:
    if len(points) < 4:
        return None

    speeds: list[tuple[float, float]] = []
    for a, b in zip(points, points[1:]):
        dt = b["t_s"] - a["t_s"]
        if dt <= 1e-4 or dt > 0.2:
            continue
        speed = math.hypot(b["x_cm"] - a["x_cm"], b["y_cm"] - a["y_cm"]) / dt
        speeds.append(((a["t_s"] + b["t_s"]) * 0.5, speed))

    if len(speeds) < 3:
        return None

    first_window = [speed for t, speed in speeds if t <= 0.35]
    initial_speed = _percentile(first_window or [speed for _, speed in speeds[:5]], 0.75)
    if initial_speed is None:
        return None

    last_moving_t = 0.0
    for t, speed in speeds:
        if speed > stop_speed_cm_s:
            last_moving_t = t
    stop_time = last_moving_t if last_moving_t > 0.0 else speeds[-1][0]
    stop_distance = _path_length(points, stop_time)

    fit_min = max(stop_speed_cm_s * 1.5, initial_speed * 0.15)
    fit_max = max(fit_min + 1.0, initial_speed * 0.95)
    fit = [(t, s) for t, s in speeds if fit_min <= s <= fit_max]
    if len(fit) < 3:
        fit = speeds

    slope = _linear_slope([t for t, _ in fit], [s for _, s in fit])
    decel_cm_s2 = slope if slope is not None else 0.0
    mu_eff = abs(decel_cm_s2 / 100.0) / 9.81

    first = points[0]
    return {
        "trial_id": first["trial_id"],
        "session_id": first["session_id"],
        "mode": first["mode"],
        "strength": first["strength"],
        "samples": len(points),
        "initial_speed_cm_s": round(initial_speed, 4),
        "stop_time_s": round(stop_time, 4),
        "stop_distance_cm": round(stop_distance, 4),
        "decel_cm_s2": round(decel_cm_s2, 4),
        "mu_eff": round(mu_eff, 6),
        "notes": first["notes"],
    }


def summarize(trials: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for trial in trials:
        grouped[(str(trial["mode"]), str(trial["strength"]))].append(trial)

    out = []
    for (mode, strength), rows in sorted(grouped.items()):
        out.append(
            {
                "mode": mode,
                "strength": strength,
                "n": len(rows),
                "initial_speed_cm_s_median": _median([r["initial_speed_cm_s"] for r in rows]),
                "stop_time_s_median": _median([r["stop_time_s"] for r in rows]),
                "stop_distance_cm_median": _median([r["stop_distance_cm"] for r in rows]),
                "decel_cm_s2_median": _median([r["decel_cm_s2"] for r in rows]),
                "mu_eff_median": _median([r["mu_eff"] for r in rows]),
            }
        )
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("Records/ball_friction/ball_friction_summary_latest.json"))
    parser.add_argument("--stop-speed-cm-s", type=float, default=2.0)
    args = parser.parse_args()

    trials = load_trials(args.csv)
    analyzed = []
    skipped = []
    for trial_id, points in trials.items():
        result = analyze_trial(points, stop_speed_cm_s=args.stop_speed_cm_s)
        if result is None:
            skipped.append(trial_id)
        else:
            analyzed.append(result)

    payload = {
        "source_csv": str(args.csv),
        "trial_count": len(trials),
        "usable_trial_count": len(analyzed),
        "skipped_trials": skipped,
        "trials": analyzed,
        "summary": summarize(analyzed),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
