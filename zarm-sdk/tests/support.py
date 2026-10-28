"""Shared helpers for the test suite."""

from __future__ import annotations

import os
import unittest
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

from zarm import FakeBackend, ManualClock, ZArm, ZArmConfig

REPO_ROOT = Path(__file__).resolve().parents[2]
_LEGACY_ROOTS = (REPO_ROOT, REPO_ROOT / "legacy" / "csharp")
_CREATED: List[ZArm] = []


def legacy_path(*parts: str) -> Optional[Path]:
    """A path inside the legacy C# projects, wherever they currently live (repo root or legacy/csharp)."""
    for root in _LEGACY_ROOTS:
        candidate = root.joinpath(*parts)
        if candidate.exists():
            return candidate
    return None


def is_real_binary(path: Path) -> bool:
    """False for a missing file or a Git LFS pointer."""
    if not path.is_file():
        return False
    with open(path, "rb") as handle:
        return handle.read(2) == b"MZ"


def x64_vendor_dir() -> Optional[Path]:
    override = os.environ.get("ZARM_TEST_VENDOR_DIR")
    directory = Path(override) if override else legacy_path("Z-Arm Sliders", "Z-Arm Sliders", "Lib")
    if directory is None or not is_real_binary(directory / "small_scara_interface.dll"):
        return None
    return directory


def make_arm(
    cards: Iterable[int] = (59,),
    config: Optional[ZArmConfig] = None,
    **fake_options: object,
) -> Tuple[ZArm, FakeBackend, ManualClock]:
    clock = ManualClock()
    fake = FakeBackend(tuple(cards), clock=clock, **fake_options)  # type: ignore[arg-type]
    arm = ZArm(fake, config if config is not None else ZArmConfig(card_id=59), clock=clock, sleep=clock.sleep)
    _CREATED.append(arm)
    return arm, fake, clock


def ready_arm(**kwargs: object) -> Tuple[ZArm, FakeBackend, ManualClock]:
    arm, fake, clock = make_arm(**kwargs)  # type: ignore[arg-type]
    arm.connect()
    arm.initialize()
    return arm, fake, clock


class ArmTestCase(unittest.TestCase):
    """Closes every arm created through make_arm/ready_arm after each test."""

    def tearDown(self) -> None:
        while _CREATED:
            _CREATED.pop().close()
