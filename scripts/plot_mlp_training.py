"""Export YOLO-like MLP training plots from a log directory."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from mlp.training_plots import export_results, read_results_csv, rows_from_tensorboard  # noqa: E402


def _latest_log_dir(root: Path) -> Path:
    candidates = [
        path
        for path in root.rglob("*")
        if path.is_dir()
        and (
            list(path.glob("events.out.tfevents.*"))
            or (path / "results.csv").exists()
        )
    ]
    if not candidates:
        raise FileNotFoundError(f"No training log directories found under {root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create results.csv/results.png for an MLP training run")
    parser.add_argument("--log-dir", default="", help="training log directory; defaults to latest under logs/mlp")
    parser.add_argument("--logs-root", default="logs/mlp", help="root used when --log-dir is omitted")
    parser.add_argument("--title", default="MLP Training Results")
    args = parser.parse_args()

    log_dir = Path(args.log_dir) if args.log_dir else _latest_log_dir(Path(args.logs_root))
    if not log_dir.exists():
        raise FileNotFoundError(log_dir)

    rows = read_results_csv(log_dir / "results.csv")
    if not rows:
        rows = rows_from_tensorboard(log_dir)
    if not rows:
        raise RuntimeError(f"No scalar rows found in {log_dir}")

    csv_path, png_path = export_results(log_dir, rows, title=args.title)
    print(f"rows={len(rows)}")
    print(f"csv={csv_path}")
    print(f"png={png_path}")


if __name__ == "__main__":
    main()
