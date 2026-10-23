"""ctypes backend over Hitbot's ``small_scara_interface.dll`` (x64 build, plain C exports).

The vendor package must be in one directory, because the DLL looks for its
helpers next to itself (it builds paths from its own ``GetModuleFileNameA``):

* ``small_scara_interface.dll``: the API this module binds
* ``share.dll``: shared memory between the DLL and the server, loaded by ``net_port_initial``
* ``server.exe``: the TCP server the arm connects to, started by ``net_port_initial``

These binaries have no published license; keep them outside the repository and
point :class:`NativeBackend` at their directory.

Every binding in :data:`FUNCTIONS` was checked by static disassembly against the
x64 set listed in :data:`VERIFIED_BUNDLES`. ``docs/native-api.md`` has the
details and says which meanings are verified and which are inferred.
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import struct
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple, Union

from .backend import Backend, EncoderParam, ScaraParam
from .errors import LibraryError, NotConnectedError
from .pe import PeFormatError, read_pe_info
from .types import require_c_float, require_c_int, require_positive_c_float

DLL_NAME = "small_scara_interface.dll"
REQUIRED_FILES = (DLL_NAME, "share.dll", "server.exe")


@dataclass(frozen=True)
class VendorBundle:
    """A set of vendor files known to work together, identified by SHA-256."""

    description: str
    sha256: Dict[str, str]


# The DLL, share.dll and server.exe share memory layouts, so they are verified as a set.
VERIFIED_BUNDLES: Tuple[VendorBundle, ...] = (
    VendorBundle(
        "x64 set in this repository's legacy folders "
        "('Z-Arm Sliders/Z-Arm Sliders/Lib' and 'ZArmUnityAdapter/ZArmUnityAdapter/Lib')",
        {
            DLL_NAME: "c8844f3c35ad581f8f0776f7e135aa988ba45e1b51ad7eaae16776aaa0d82abe",
            "share.dll": "a9fb4b1d152bf1cb7c3e0b595b39e98d36c690530fadf31dc20c0b1d707645c2",
            "server.exe": "92b12ece6b0349dfa44c1b434dcdeb0ca5d58718e6f4af61422b5cd30d7ae081",
        },
    ),
)

# The DLL keeps its own directory in a 256-byte ANSI buffer (GetModuleFileNameA).
MAX_DLL_PATH_CHARS = 255

_F = ctypes.POINTER(ctypes.c_float)
_B = ctypes.POINTER(ctypes.c_bool)
_I = ctypes.POINTER(ctypes.c_int)


@dataclass(frozen=True)
class NativeFunction:
    """One C export with its ctypes signature and how it was established."""

    name: str
    restype: Any
    argtypes: Tuple[Any, ...]
    params: Tuple[str, ...]
    types: str
    semantics: str
    evidence: str
    sdk_uses: bool = True


def _fn(
    name: str,
    restype: Any,
    params: Iterable[Tuple[str, Any]],
    semantics: str,
    evidence: str,
    sdk_uses: bool = True,
) -> NativeFunction:
    pairs = tuple(params)
    return NativeFunction(
        name=name,
        restype=restype,
        argtypes=tuple(kind for _, kind in pairs),
        params=tuple(label for label, _ in pairs),
        types="verified",
        semantics=semantics,
        evidence=evidence,
        sdk_uses=sdk_uses,
    )


_CARD = ("card", ctypes.c_int)
_POSE_OUT = (("x_mm", _F), ("y_mm", _F), ("z_mm", _F), ("angle1_deg", _F), ("angle2_deg", _F), ("r_deg", _F))
_MOVE_CODES = (
    "Wrapper returns 102 if is_collision(), 103 if any joint state is 0, 105 if any is 7 or 8; "
    "ControlBean then returns 101 for NaN, 0 while moving, the card id (garbage) if roughly is outside [0, 1], "
    "2 if speed <= 0, 3 if not initialized, 6 if servo off, 11 if the mobile terminal is in control, "
    "4 if out of range, else 1."
)

FUNCTIONS: Tuple[NativeFunction, ...] = (
    _fn(
        "net_port_initial",
        ctypes.c_int,
        (),
        "verified",
        "Alias of net_port_initial_auto. LoadLibraryA(<dll dir>\\share.dll); if no server is running, "
        "ShellExecuteA(<dll dir>\\server.exe) and poll up to 3 s. Returns 1, or -1 after a modal message box.",
    ),
    _fn(
        "close_server",
        None,
        (),
        "verified",
        "Sets the shutdown flag in shared memory; dereferences memory created by net_port_initial.",
    ),
    _fn(
        "card_number_is_connect",
        ctypes.c_int,
        (_CARD,),
        "verified",
        "1 connected, 0 not, 2 card outside 0..255 (unsigned compare). Needs net_port_initial first.",
    ),
    _fn("robot_is_connect", ctypes.c_int, (_CARD,), "verified", "Identical code to card_number_is_connect.", False),
    _fn(
        "robot_initial",
        ctypes.c_int,
        (_CARD, ("generation", ctypes.c_int), ("z_travel_mm", ctypes.c_int)),
        "corroborated",
        "Wrapper converts z_travel int->float (cvtdq2ps) for ControlBean::initial(int,float), so a float "
        "argument would be misread. 0/2/11/12/105 traced (105 shows a modal message box); 1 accepted; "
        "3 (encoder error) per vendor header. Generation and z_travel may be replaced by machine-reported values.",
    ),
    _fn(
        "robot_unlock_position",
        ctypes.c_int,
        (_CARD,),
        "verified",
        "Tail call to ControlBean::unlock_position(); 1 if connected (sets two shared-memory flags), else 0.",
    ),
    _fn(
        "robot_set_arm_length",
        None,
        (_CARD, ("l1_mm", ctypes.c_float), ("l2_mm", ctypes.c_float)),
        "inferred",
        "Tail call to ControlBean::set_arm_length(float,float). Units/meaning per vendor header (l1 must be 200).",
    ),
    _fn(
        "robot_get_scara_param",
        None,
        (_CARD,) + _POSE_OUT
        + (("communicate_success", _B), ("initial_finish", _B), ("move_flag", _B)),
        "corroborated",
        "Forwards to ControlBean::get_scara_param(float*x6, bool*x4) with a dummy for servo_off_flag. "
        "x/y/z/angle1/angle2/r order confirmed by the DLL's own movel_z, movel_xy and movel_xyz_by_offset; "
        "move_flag read by robot_is_stop; initial_finish is the flag whose absence makes moves return 3; "
        "matches both community wrappers.",
    ),
    _fn(
        "robot_get_encoder_param",
        None,
        (_CARD,) + _POSE_OUT,
        "inferred",
        "Tail call to ControlBean::get_scara_real_coor(float*x6). Output order per community wrappers.",
    ),
    _fn(
        "robot_get_joint_state",
        ctypes.c_int,
        (_CARD, ("joint", ctypes.c_int)),
        "corroborated",
        "Returns 0..7 (traced); 2 bad joint; joint 4 always 1. On some faults shows a modal message box "
        "('communication disorder' / 'missing-axis alarm') and returns 7.",
    ),
    _fn("robot_is_collision", ctypes.c_bool, (_CARD,), "verified", "Tail call to bool ControlBean::is_collision()."),
    _fn(
        "robot_is_stop",
        ctypes.c_int,
        (_CARD,),
        "verified",
        "Calls get_scara_param and returns 1 when move_flag is 0.",
    ),
    _fn(
        "robot_wait_stop",
        ctypes.c_bool,
        (_CARD,),
        "verified",
        "UNBOUNDED: loops Sleep(10) until the arm stops, with no timeout. The SDK never calls it.",
        False,
    ),
    _fn(
        "robot_movej_angle",
        ctypes.c_int,
        (
            _CARD,
            ("angle1_deg", ctypes.c_float),
            ("angle2_deg", ctypes.c_float),
            ("z_mm", ctypes.c_float),
            ("r_deg", ctypes.c_float),
            ("speed", ctypes.c_float),
            ("roughly", ctypes.c_float),
        ),
        "corroborated",
        "Forwards to ControlBean::movej_angle(float x6). Order confirmed by robot_movel_z, which passes "
        "(angle1, angle2, z, r, speed, 0). " + _MOVE_CODES + " Speed units per vendor header (inferred).",
    ),
    _fn(
        "robot_movej_xyz",
        ctypes.c_int,
        (
            _CARD,
            ("x_mm", ctypes.c_float),
            ("y_mm", ctypes.c_float),
            ("z_mm", ctypes.c_float),
            ("r_deg", ctypes.c_float),
            ("speed", ctypes.c_float),
            ("roughly", ctypes.c_float),
        ),
        "corroborated",
        "Forwards to ControlBean::movej_xyz(float x6); robot_movej_xy passes (x, y, z, r, speed, 0). "
        + _MOVE_CODES,
    ),
    _fn(
        "robot_movej_xyz_lr",
        ctypes.c_int,
        (
            _CARD,
            ("x_mm", ctypes.c_float),
            ("y_mm", ctypes.c_float),
            ("z_mm", ctypes.c_float),
            ("r_deg", ctypes.c_float),
            ("speed", ctypes.c_float),
            ("roughly", ctypes.c_float),
            ("lr", ctypes.c_int),
        ),
        "inferred",
        "Forwards to ControlBean::movej_xyz_lr(float x6, int). lr goes to the vendor's IK routine; returns 7 "
        "when the target is unreachable in that configuration. Which lr value means which hand is unknown.",
    ),
    _fn(
        "robot_movel_xyz",
        ctypes.c_int,
        (
            _CARD,
            ("x_mm", ctypes.c_float),
            ("y_mm", ctypes.c_float),
            ("z_mm", ctypes.c_float),
            ("r_deg", ctypes.c_float),
            ("speed_mm_s", ctypes.c_float),
        ),
        "corroborated",
        "Forwards to ControlBean::movel_xyz(float x5); robot_movel_xy passes (x, y, z, r, speed). "
        "Same pre-checks as the movej wrappers. Speed in mm/s per vendor header (inferred).",
    ),
    _fn(
        "robot_movel_xyz_by_offset",
        ctypes.c_int,
        (
            _CARD,
            ("dx_mm", ctypes.c_float),
            ("dy_mm", ctypes.c_float),
            ("dz_mm", ctypes.c_float),
            ("dr_deg", ctypes.c_float),
            ("speed_mm_s", ctypes.c_float),
        ),
        "verified",
        "Adds the offsets to get_scara_param's x/y/z/r and calls ControlBean::movel_xyz.",
        False,
    ),
    _fn(
        "robot_movej_xy",
        ctypes.c_int,
        (_CARD, ("x_mm", ctypes.c_float), ("y_mm", ctypes.c_float), ("speed", ctypes.c_float)),
        "verified",
        "movej_xyz to (x, y, current z, current r) with roughly 0.",
        False,
    ),
    _fn(
        "robot_movel_xy",
        ctypes.c_int,
        (_CARD, ("x_mm", ctypes.c_float), ("y_mm", ctypes.c_float), ("speed_mm_s", ctypes.c_float)),
        "verified",
        "movel_xyz to (x, y, current z, current r).",
        False,
    ),
    _fn(
        "robot_movel_z",
        ctypes.c_int,
        (_CARD, ("z_mm", ctypes.c_float), ("speed", ctypes.c_float)),
        "verified",
        "Despite the name: movej_angle(current angle1, current angle2, z, current r, speed, 0).",
        False,
    ),
    _fn(
        "robot_movej_agnle12",
        ctypes.c_int,
        (_CARD, ("angle1_deg", ctypes.c_float), ("angle2_deg", ctypes.c_float), ("speed", ctypes.c_float)),
        "verified",
        "Vendor typo for 'angle12': movej_angle(angle1, angle2, current z, current r, speed, 0).",
        False,
    ),
    _fn(
        "robot_change_attitude",
        ctypes.c_int,
        (_CARD, ("speed", ctypes.c_float)),
        "corroborated",
        "Tail call to ControlBean::change_attitude(float), no wrapper pre-checks. Returns 0/1/2/3/4/6/11/101 "
        "(traced); switches hand configuration per vendor header.",
    ),
    _fn("robot_stop_move", None, (_CARD,), "verified", "Tail call to void ControlBean::stop_move()."),
    _fn(
        "robot_emergency_stop",
        None,
        (_CARD,),
        "inferred",
        "Tail call to void ControlBean::emergency_stop(), which starts a detached thread and returns at once. "
        "What the thread does to the servos is unverified.",
    ),
    _fn("robot_pause_move", ctypes.c_int, (_CARD,), "verified", "ControlBean::pause_move(): 1 OK, 0 failure."),
    _fn("robot_resume_move", ctypes.c_int, (_CARD,), "verified", "ControlBean::resume_move(): 1 OK, 0 failure."),
    _fn(
        "robot_joint_home",
        ctypes.c_int,
        (_CARD, ("joint", ctypes.c_int)),
        "corroborated",
        "ControlBean::joint_home(int) clears the initialized flag on entry, before any check; 0 not connected, "
        "2 joint outside 1..4, 3 refused by a status flag (traced); 1 = started (inferred). Moves the joint.",
    ),
    _fn(
        "robot_set_drag_teach",
        ctypes.c_bool,
        (_CARD, ("enable", ctypes.c_bool)),
        "inferred",
        "Tail call to bool ControlBean::set_drag_teach(bool).",
    ),
    _fn(
        "robot_get_drag_teach",
        ctypes.c_int,
        (_CARD,),
        "verified",
        "Returns 0/1 from bool ControlBean::get_drag_teach().",
    ),
    _fn(
        "robot_set_cooperation_fun_state",
        ctypes.c_bool,
        (_CARD, ("enable", ctypes.c_bool)),
        "inferred",
        "Tail call to bool ControlBean::set_cooperation_fun_state(bool); meaning per legacy C# (CooperationMode).",
    ),
    _fn(
        "robot_get_cooperation_fun_state",
        ctypes.c_bool,
        (_CARD,),
        "inferred",
        "Tail call to bool ControlBean::get_cooperation_fun_state().",
    ),
    _fn(
        "robot_get_digital_io_in",
        ctypes.c_int,
        (_CARD, ("io", ctypes.c_int)),
        "corroborated",
        "3 not connected, -1 invalid io, else NOT(controller bit) as 0/1 (traced). Electrical meaning unverified.",
    ),
    _fn(
        "robot_get_digital_io_out",
        ctypes.c_int,
        (_CARD, ("io", ctypes.c_int)),
        "corroborated",
        "3 not connected, -1 invalid io, else 0/1 (traced).",
    ),
    _fn(
        "robot_set_digital_io_out",
        ctypes.c_bool,
        (_CARD, ("io", ctypes.c_int), ("value", ctypes.c_bool)),
        "inferred",
        "Tail call to bool ControlBean::set_digital_out(int, bool).",
    ),
    _fn(
        "robot_set_efg_state",
        ctypes.c_int,
        (_CARD, ("efg_type", ctypes.c_int), ("distance", ctypes.c_float)),
        "unknown",
        "Tail call to int ControlBean::set_efg_state(int, float); electric gripper, semantics unknown.",
        False,
    ),
    _fn(
        "robot_get_efg_state",
        ctypes.c_int,
        (_CARD, ("efg_type", _I), ("distance", _F)),
        "unknown",
        "Tail call to int ControlBean::get_efg_state(int*, float*); electric gripper, semantics unknown.",
        False,
    ),
)

UNBOUND_EXPORTS: Dict[str, str] = {
    "net_port_initial_auto": "same function as net_port_initial",
    "net_port_initial_1": "internal: takes the shared-memory pointer that net_port_initial obtains",
    "set_card_number_for_muti_control": "internal: writes a ControlBean field and returns the ControlBean pointer",
}
_E_PREFIX_REASON = "e_* variants are stdcall copies for other languages; on x64 they are the same code as robot_*"

FUNCTIONS_BY_NAME: Dict[str, NativeFunction] = {function.name: function for function in FUNCTIONS}


def unbound_reason(export: str) -> Optional[str]:
    if export in UNBOUND_EXPORTS:
        return UNBOUND_EXPORTS[export]
    if export.startswith("e_"):
        return _E_PREFIX_REASON
    return None


def file_sha256(path: Union[str, "os.PathLike[str]"]) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class LibraryReport:
    """Static preflight results for a vendor directory. Nothing is loaded or executed."""

    library_dir: Path
    dll_path: Path
    files_present: Dict[str, bool]
    machine: Optional[str]
    sha256: Dict[str, str]
    verified_build: bool
    exports: Tuple[str, ...]
    missing_exports: Tuple[str, ...]
    problems: Tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.problems

    def raise_for_problems(self) -> None:
        if self.problems:
            raise LibraryError(
                f"vendor library at {self.library_dir} failed preflight:\n  - " + "\n  - ".join(self.problems)
            )


def inspect_library_dir(
    library_dir: Union[str, "os.PathLike[str]"],
    *,
    check_runtime: bool = True,
    allow_unverified_build: bool = False,
) -> LibraryReport:
    """Check a vendor directory without loading anything.

    ``check_runtime`` adds the checks for this interpreter (Windows, 64-bit).
    """
    directory = Path(library_dir).resolve()
    dll_path = directory / DLL_NAME
    problems = []
    if not directory.is_dir():
        problems.append(f"{directory} is not a directory")
    files_present = {name: (directory / name).is_file() for name in REQUIRED_FILES}
    for name, present in files_present.items():
        if not present:
            problems.append(
                f"{name} is missing; net_port_initial needs {', '.join(REQUIRED_FILES)} in the same directory"
            )

    machine: Optional[str] = None
    exports: Tuple[str, ...] = ()
    missing: Tuple[str, ...] = ()
    hashes = {name: file_sha256(directory / name) for name, present in files_present.items() if present}
    bundle = next((candidate for candidate in VERIFIED_BUNDLES if candidate.sha256 == hashes), None)
    if files_present.get(DLL_NAME):
        try:
            info = read_pe_info(dll_path)
        except (OSError, PeFormatError) as exc:
            problems.append(f"cannot read {DLL_NAME}: {exc}")
        else:
            machine = info.machine
            exports = info.exports
            if info.machine != "x64":
                problems.append(
                    f"{DLL_NAME} is an {info.machine} build; only the x64 C API has verified signatures"
                )
            available = set(info.exports)
            missing = tuple(f.name for f in FUNCTIONS if f.sdk_uses and f.name not in available)
            if missing:
                problems.append(f"{DLL_NAME} lacks required exports: {', '.join(missing)}")
        path_text = str(dll_path)
        if len(path_text) > MAX_DLL_PATH_CHARS:
            problems.append(
                f"the DLL path is {len(path_text)} characters; the DLL stores it in a 256-byte buffer"
            )
        if not path_text.isascii():
            problems.append("the DLL path must be ASCII; the DLL resolves its directory with ANSI APIs")
    if all(files_present.values()) and bundle is None and not allow_unverified_build:
        known = VERIFIED_BUNDLES[0].sha256
        differing = [name for name in REQUIRED_FILES if hashes.get(name) != known.get(name)]
        problems.append(
            "these files are not the verified set (sha256 differs for " + ", ".join(differing) + "); "
            "pass allow_unverified_build=True only after checking docs/native-api.md against them"
        )

    if check_runtime:
        if os.name != "nt":
            problems.append("the vendor library is a Windows DLL; this is not Windows")
        if struct.calcsize("P") != 8:
            problems.append("a 64-bit Python is required for the x64 DLL")

    return LibraryReport(
        library_dir=directory,
        dll_path=dll_path,
        files_present=files_present,
        machine=machine,
        sha256=hashes,
        verified_build=bundle is not None,
        exports=exports,
        missing_exports=missing,
        problems=tuple(problems),
    )


class NativeLibrary:
    """The loaded DLL with every available function in :data:`FUNCTIONS` bound.

    Loading runs only the DLL's initialization (it records its own directory).
    Binding resolves addresses; no vendor function is called.
    """

    def __init__(self, dll_path: Path, handle: Any, functions: Dict[str, Any]):
        self.dll_path = dll_path
        self.handle = handle
        self.functions = functions

    def __getattr__(self, name: str) -> Any:
        functions = self.__dict__.get("functions", {})
        if name in functions:
            return functions[name]
        raise AttributeError(f"{name} is not bound (not in FUNCTIONS, or missing from this build)")


_LOAD_LOCK = threading.Lock()
_LOADED: Optional[NativeLibrary] = None


def load_library(dll_path: Union[str, "os.PathLike[str]"]) -> NativeLibrary:
    """Load the DLL once per process and bind signatures. Loading a second, different copy is refused."""
    global _LOADED
    path = Path(dll_path).resolve()
    with _LOAD_LOCK:
        if _LOADED is not None:
            if os.path.normcase(str(_LOADED.dll_path)) != os.path.normcase(str(path)):
                raise LibraryError(
                    f"{DLL_NAME} is already loaded from {_LOADED.dll_path}; the library keeps process-wide "
                    "state, so a second copy cannot be used in the same process"
                )
            return _LOADED
        if os.name != "nt":
            raise LibraryError("the vendor library is a Windows DLL")
        if struct.calcsize("P") != 8:
            raise LibraryError("a 64-bit Python is required for the x64 DLL")
        info = read_pe_info(path)
        if info.machine != "x64":
            raise LibraryError(f"{path} is an {info.machine} build; expected x64")
        try:
            handle = ctypes.CDLL(str(path))
        except OSError as exc:
            raise LibraryError(f"cannot load {path}: {exc}") from exc
        available = set(info.exports)
        functions = {}
        for spec in FUNCTIONS:
            if spec.name not in available:
                continue
            function = getattr(handle, spec.name)
            function.argtypes = list(spec.argtypes)
            function.restype = spec.restype
            functions[spec.name] = function
        _LOADED = NativeLibrary(path, handle, functions)
        return _LOADED


_SERVER_STARTED = False
_ACTIVE_BACKEND: Optional["NativeBackend"] = None
_STATE_LOCK = threading.Lock()


def _check_card(card: int) -> int:
    """Card ids index a 256-entry table that the DLL does not bounds-check."""
    return require_c_int("card", card, 0, 255)


def _c_floats(names: Tuple[str, ...], values: Tuple[float, ...]) -> Tuple[float, ...]:
    """Values that stay finite as C floats (ctypes turns 1e300 into inf and passes NaN through)."""
    return tuple(require_c_float(name, value) for name, value in zip(names, values))


class NativeBackend(Backend):
    """:class:`Backend` over the vendor DLL. Only one instance may be active per process.

    Calls are serialized with a lock, and the server/closed checks happen under
    the same lock as the call they protect. ``stop_move`` and ``emergency_stop``
    use a separate lock so a stop is never queued behind a slow call (the legacy
    C# app also called the library from several threads). Every argument is
    range-checked before ctypes can narrow it.

    Known vendor behaviour the caller cannot avoid: ``net_port_initial``,
    ``robot_initial`` and ``robot_get_joint_state`` (which every move wrapper
    calls) can show a modal message box that blocks the calling thread until
    it is dismissed.
    """

    def __init__(
        self,
        library_dir: Union[str, "os.PathLike[str]"],
        *,
        allow_unverified_build: bool = False,
    ) -> None:
        super().__init__()
        report = inspect_library_dir(library_dir, allow_unverified_build=allow_unverified_build)
        report.raise_for_problems()
        global _ACTIVE_BACKEND
        with _STATE_LOCK:
            if _ACTIVE_BACKEND is not None:
                raise LibraryError("a NativeBackend is already active in this process; close it first")
            self._lib = load_library(report.dll_path)
            _ACTIVE_BACKEND = self
        self.report = report
        self._call_lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._closed = False

    @property
    def library(self) -> NativeLibrary:
        return self._lib

    def close(self) -> None:
        """Wait for calls in progress, then free the process-wide slot. The DLL stays loaded.

        Refused while a ZArm still holds a client lease on this backend.
        """
        global _ACTIVE_BACKEND
        if self.has_clients():
            raise LibraryError("this NativeBackend is still in use by a ZArm; close that first")
        with self._call_lock, self._stop_lock:
            self._closed = True
        with _STATE_LOCK:
            if _ACTIVE_BACKEND is self:
                _ACTIVE_BACKEND = None

    def _require_ready(self) -> None:
        """Call with _call_lock or _stop_lock held."""
        if self._closed:
            raise LibraryError("this NativeBackend is closed")
        if not _SERVER_STARTED:
            raise NotConnectedError(
                "start_server() must succeed first; the DLL dereferences shared memory created by net_port_initial"
            )

    def _robot(self, name: str, card: int, *args: Any) -> Any:
        card = _check_card(card)
        with self._call_lock:
            self._require_ready()
            return self._lib.functions[name](card, *args)

    def _robot_stop(self, name: str, card: int) -> None:
        card = _check_card(card)
        with self._stop_lock:
            self._require_ready()
            self._lib.functions[name](card)

    # Server and discovery

    def start_server(self) -> int:
        global _SERVER_STARTED
        with self._call_lock:
            if self._closed:
                raise LibraryError("this NativeBackend is closed")
            if _SERVER_STARTED:
                return 1
            result = int(self._lib.net_port_initial())
            if result == 1:
                with self._stop_lock:
                    _SERVER_STARTED = True
            return result

    def close_server(self) -> None:
        global _SERVER_STARTED
        with self._call_lock, self._stop_lock:
            if self._closed or not _SERVER_STARTED:
                return
            self._lib.close_server()
            _SERVER_STARTED = False

    def card_connect_status(self, card: int) -> int:
        card = require_c_int("card", card)
        with self._call_lock:
            self._require_ready()
            return int(self._lib.card_number_is_connect(card))

    # Setup

    def initial(self, card: int, generation: int, z_travel_mm: int) -> int:
        generation = require_c_int("generation", generation)
        z_travel_mm = require_c_int("z_travel_mm", z_travel_mm)
        return int(self._robot("robot_initial", card, generation, z_travel_mm))

    def unlock_position(self, card: int) -> int:
        return int(self._robot("robot_unlock_position", card))

    def set_arm_length(self, card: int, l1_mm: float, l2_mm: float) -> None:
        lengths = (require_positive_c_float("l1_mm", l1_mm), require_positive_c_float("l2_mm", l2_mm))
        self._robot("robot_set_arm_length", card, *lengths)

    # State

    def get_scara_param(self, card: int) -> ScaraParam:
        floats = [ctypes.c_float() for _ in range(6)]
        flags = [ctypes.c_bool() for _ in range(3)]
        pointers = [ctypes.byref(value) for value in floats] + [ctypes.byref(flag) for flag in flags]
        self._robot("robot_get_scara_param", card, *pointers)
        x, y, z, angle1, angle2, r = (value.value for value in floats)
        communicate, initialized, moving = (flag.value for flag in flags)
        return ScaraParam(x, y, z, angle1, angle2, r, communicate, initialized, moving)

    def get_encoder_param(self, card: int) -> EncoderParam:
        floats = [ctypes.c_float() for _ in range(6)]
        self._robot("robot_get_encoder_param", card, *[ctypes.byref(value) for value in floats])
        return EncoderParam(*(value.value for value in floats))

    def get_joint_state(self, card: int, joint: int) -> int:
        return int(self._robot("robot_get_joint_state", card, require_c_int("joint", joint)))

    def is_collision(self, card: int) -> bool:
        return bool(self._robot("robot_is_collision", card))

    def is_stop(self, card: int) -> bool:
        return int(self._robot("robot_is_stop", card)) == 1

    # Motion

    def movej_angle(
        self, card: int, angle1_deg: float, angle2_deg: float, z_mm: float, r_deg: float, speed: float, roughly: float
    ) -> int:
        args = _c_floats(
            ("angle1_deg", "angle2_deg", "z_mm", "r_deg", "speed", "roughly"),
            (angle1_deg, angle2_deg, z_mm, r_deg, speed, roughly),
        )
        return int(self._robot("robot_movej_angle", card, *args))

    def movej_xyz(
        self, card: int, x_mm: float, y_mm: float, z_mm: float, r_deg: float, speed: float, roughly: float
    ) -> int:
        args = _c_floats(
            ("x_mm", "y_mm", "z_mm", "r_deg", "speed", "roughly"), (x_mm, y_mm, z_mm, r_deg, speed, roughly)
        )
        return int(self._robot("robot_movej_xyz", card, *args))

    def movej_xyz_lr(
        self,
        card: int,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        r_deg: float,
        speed: float,
        roughly: float,
        lr: int,
    ) -> int:
        args = _c_floats(
            ("x_mm", "y_mm", "z_mm", "r_deg", "speed", "roughly"), (x_mm, y_mm, z_mm, r_deg, speed, roughly)
        )
        return int(self._robot("robot_movej_xyz_lr", card, *args, require_c_int("lr", lr)))

    def movel_xyz(self, card: int, x_mm: float, y_mm: float, z_mm: float, r_deg: float, speed_mm_s: float) -> int:
        args = _c_floats(("x_mm", "y_mm", "z_mm", "r_deg", "speed_mm_s"), (x_mm, y_mm, z_mm, r_deg, speed_mm_s))
        return int(self._robot("robot_movel_xyz", card, *args))

    def change_attitude(self, card: int, speed: float) -> int:
        return int(self._robot("robot_change_attitude", card, *_c_floats(("speed",), (speed,))))

    def joint_home(self, card: int, joint: int) -> int:
        return int(self._robot("robot_joint_home", card, require_c_int("joint", joint)))

    def stop_move(self, card: int) -> None:
        self._robot_stop("robot_stop_move", card)

    def emergency_stop(self, card: int) -> None:
        self._robot_stop("robot_emergency_stop", card)

    def pause_move(self, card: int) -> int:
        return int(self._robot("robot_pause_move", card))

    def resume_move(self, card: int) -> int:
        return int(self._robot("robot_resume_move", card))

    # Modes and I/O

    def set_drag_teach(self, card: int, enable: bool) -> bool:
        return bool(self._robot("robot_set_drag_teach", card, bool(enable)))

    def get_drag_teach(self, card: int) -> bool:
        return int(self._robot("robot_get_drag_teach", card)) != 0

    def set_cooperation_fun_state(self, card: int, enable: bool) -> bool:
        return bool(self._robot("robot_set_cooperation_fun_state", card, bool(enable)))

    def get_cooperation_fun_state(self, card: int) -> bool:
        return bool(self._robot("robot_get_cooperation_fun_state", card))

    def get_digital_io_in(self, card: int, io: int) -> int:
        return int(self._robot("robot_get_digital_io_in", card, require_c_int("io", io)))

    def get_digital_io_out(self, card: int, io: int) -> int:
        return int(self._robot("robot_get_digital_io_out", card, require_c_int("io", io)))

    def set_digital_io_out(self, card: int, io: int, value: bool) -> bool:
        return bool(self._robot("robot_set_digital_io_out", card, require_c_int("io", io), bool(value)))


def describe_python() -> str:
    return f"Python {sys.version.split()[0]} ({struct.calcsize('P') * 8}-bit, {sys.platform})"
