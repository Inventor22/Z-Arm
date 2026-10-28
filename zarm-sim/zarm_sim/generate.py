"""Write (or check) the generated robot descriptions in ``zarm-sim/models/``.

    python -m zarm_sim.generate          # rewrite models/
    python -m zarm_sim.generate --check  # exit 1 if models/ is out of date
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Optional, Sequence

from .mjcf import mjcf_text
from .model import RobotModel, build_model
from .urdf import urdf_text

MODELS_DIR = Path(__file__).resolve().parents[1] / "models"
URDF_NAME = "zarm_2442.urdf"
MJCF_NAME = "zarm_2442.mjcf.xml"


def render_all(model: Optional[RobotModel] = None) -> Dict[str, bytes]:
    """File name -> exact bytes (UTF-8, LF line endings) of every generated file."""
    model = model if model is not None else build_model()
    return {
        URDF_NAME: urdf_text(model).encode("utf-8"),
        MJCF_NAME: mjcf_text(model).encode("utf-8"),
    }


def stale_files(directory: Path = MODELS_DIR) -> Dict[str, str]:
    """Generated files that are missing or differ from what the generator writes now, with the reason."""
    problems = {}
    for name, data in render_all().items():
        path = directory / name
        if not path.is_file():
            problems[name] = "missing"
        elif path.read_bytes() != data:
            problems[name] = "differs from the generator's output"
    return problems


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Generate the Z-Arm URDF and MJCF from zarm_sim/dimensions.py.")
    parser.add_argument("--check", action="store_true", help="don't write; exit 1 if a file is out of date")
    parser.add_argument("--out", type=Path, default=MODELS_DIR, help=f"output folder (default {MODELS_DIR})")
    args = parser.parse_args(argv)
    if args.check:
        problems = stale_files(args.out)
        for name, why in problems.items():
            print(f"{args.out / name}: {why}; run python -m zarm_sim.generate", file=sys.stderr)
        if not problems:
            print("Generated models are up to date.")
        return 1 if problems else 0
    args.out.mkdir(parents=True, exist_ok=True)
    for name, data in render_all().items():
        path = args.out / name
        if path.is_file() and path.read_bytes() == data:
            print(f"unchanged {path}")
            continue
        path.write_bytes(data)
        print(f"wrote     {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
