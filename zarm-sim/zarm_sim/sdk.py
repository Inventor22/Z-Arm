"""zarm-sdk for the simulator: found next to zarm-sim, and only ever its simulated backend.

The SDK lives in the sibling folder ``zarm-sdk/``. Its folder is put first on
``sys.path``, so the simulator always uses the SDK from the same checkout (an
installed copy is used only if that folder is missing).
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType

SDK_DIR = Path(__file__).resolve().parents[2] / "zarm-sdk"


class NotSimulatedError(RuntimeError):
    """The simulator was given an arm that isn't on zarm-sdk's simulated backend."""


def import_zarm() -> ModuleType:
    if "zarm" not in sys.modules and (SDK_DIR / "zarm" / "__init__.py").is_file():
        sdk = str(SDK_DIR)
        if sdk not in sys.path:
            sys.path.insert(0, sdk)
    import zarm

    return zarm


def require_simulated(arm: object) -> None:
    """Raise NotSimulatedError unless ``arm`` is a zarm.ZArm on exactly zarm.fake.FakeBackend.

    Never accepts the native backend (Hitbot's DLL) or anything derived from FakeBackend.
    """
    zarm = import_zarm()
    from zarm.fake import FakeBackend

    if not isinstance(arm, zarm.ZArm):
        raise NotSimulatedError(f"expected a zarm.ZArm, got {type(arm).__name__}")
    backend = arm.backend
    if type(backend) is not FakeBackend:
        raise NotSimulatedError(
            f"the simulator view only drives zarm's simulated backend (FakeBackend), not {type(backend).__name__}"
        )
