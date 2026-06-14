"""Static training-result exports for MLP trainers.

The trainers already write TensorBoard event files. This module adds YOLO-like
artifacts (`results.csv` and `results.png`) that are easy to inspect after a run.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Iterable


PREFERRED_COLUMNS = [
    "epoch",
    "step",
    "train_loss",
    "val_loss",
    "best_val_loss",
    "train_continuous_loss",
    "val_continuous_loss",
    "train_discrete_loss",
    "val_discrete_loss",
    "total_loss",
    "policy_loss",
    "value_loss",
    "entropy",
    "mean_reward",
    "mean_length",
    "lr",
]


def _as_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _ordered_fieldnames(rows: list[dict[str, Any]]) -> list[str]:
    fields = set()
    for row in rows:
        fields.update(row.keys())
    ordered = [name for name in PREFERRED_COLUMNS if name in fields]
    ordered.extend(sorted(fields - set(ordered)))
    return ordered


def write_results_csv(rows: Iterable[dict[str, Any]], path: str | Path) -> Path:
    """Write training rows to CSV and return the written path."""
    rows = list(rows)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    if not rows:
        path.write_text("", encoding="utf-8")
        return path

    fieldnames = _ordered_fieldnames(rows)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in fieldnames})
    return path


def read_results_csv(path: str | Path) -> list[dict[str, float]]:
    """Read a training CSV written by write_results_csv."""
    path = Path(path)
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        rows = []
        for row in csv.DictReader(f):
            parsed = {}
            for key, value in row.items():
                number = _as_float(value)
                if number is not None:
                    parsed[key] = number
            if parsed:
                rows.append(parsed)
        return rows


def plot_results(rows: Iterable[dict[str, Any]], path: str | Path, title: str = "MLP Training Results") -> Path | None:
    """Plot YOLO-like training curves.

    Returns the output path when plotting succeeded. Missing matplotlib is not
    fatal, because training should still finish on headless machines.
    """
    rows = list(rows)
    if not rows:
        return None

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - depends on local env
        print(f"[plots] matplotlib unavailable, skipped results.png: {exc}")
        return None

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    x_key = "epoch" if "epoch" in rows[0] else "step"
    xs = [_as_float(row.get(x_key)) for row in rows]
    xs = [idx + 1 if x is None else x for idx, x in enumerate(xs)]

    groups = [
        ("Total loss", ["train_loss", "val_loss", "best_val_loss", "total_loss"]),
        ("Continuous loss", ["train_continuous_loss", "val_continuous_loss"]),
        ("Discrete loss", ["train_discrete_loss", "val_discrete_loss"]),
        ("RL / misc", ["policy_loss", "value_loss", "entropy", "mean_reward", "mean_length", "lr"]),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), dpi=140)
    axes = axes.ravel()
    plotted_any = False

    for ax, (name, keys) in zip(axes, groups):
        plotted = False
        for key in keys:
            ys = [_as_float(row.get(key)) for row in rows]
            if all(y is None for y in ys):
                continue
            clean_x = [x for x, y in zip(xs, ys) if y is not None]
            clean_y = [y for y in ys if y is not None]
            ax.plot(clean_x, clean_y, marker="o", markersize=2.5, linewidth=1.4, label=key)
            plotted = True
            plotted_any = True
        ax.set_title(name)
        ax.set_xlabel(x_key)
        ax.grid(True, alpha=0.25)
        if plotted:
            ax.legend(fontsize=8)
        else:
            ax.text(0.5, 0.5, "no data", ha="center", va="center", transform=ax.transAxes, alpha=0.55)

    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(path)
    plt.close(fig)

    return path if plotted_any else None


def export_results(log_path: str | Path, rows: Iterable[dict[str, Any]], title: str) -> tuple[Path, Path | None]:
    """Write `results.csv` and `results.png` under a training log directory."""
    log_path = Path(log_path)
    rows = list(rows)
    csv_path = write_results_csv(rows, log_path / "results.csv")
    png_path = plot_results(rows, log_path / "results.png", title=title)
    return csv_path, png_path


def rows_from_tensorboard(log_path: str | Path) -> list[dict[str, float]]:
    """Best-effort conversion from TensorBoard scalars to epoch rows."""
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except Exception as exc:  # pragma: no cover - depends on local env
        raise RuntimeError(f"tensorboard is unavailable: {exc}") from exc

    accumulator = EventAccumulator(str(log_path))
    accumulator.Reload()

    tags = set(accumulator.Tags().get("scalars", []))
    epoch_tags = {
        "epoch/train_loss": "train_loss",
        "epoch/val_loss": "val_loss",
        "epoch/train_continuous_loss": "train_continuous_loss",
        "epoch/val_continuous_loss": "val_continuous_loss",
        "epoch/train_discrete_loss": "train_discrete_loss",
        "epoch/val_discrete_loss": "val_discrete_loss",
    }

    rows_by_epoch: dict[int, dict[str, float]] = {}
    for tb_tag, column in epoch_tags.items():
        if tb_tag not in tags:
            continue
        for event in accumulator.Scalars(tb_tag):
            epoch = int(event.step) + 1
            rows_by_epoch.setdefault(epoch, {"epoch": float(epoch)})[column] = float(event.value)

    rows = [rows_by_epoch[key] for key in sorted(rows_by_epoch)]
    best = float("inf")
    for row in rows:
        val_loss = row.get("val_loss")
        if val_loss is not None:
            best = min(best, val_loss)
            row["best_val_loss"] = best
    return rows
