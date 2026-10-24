"""Shared helpers for the zarm-sim tests (stdlib unittest; run from zarm-sim/)."""

from __future__ import annotations

import importlib.util
import shutil
from pathlib import Path

from zarm_sim.sdk import import_zarm

zarm = import_zarm()  # the sibling zarm-sdk checkout

SIM_ROOT = Path(__file__).resolve().parents[1]
HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None
NODE = shutil.which("node")
