"""Checks of the ctypes binding layer.

Nothing here calls a vendor function or touches the arm. Static checks parse
the DLL as bytes. The Windows-only checks load the DLL (its initialization only
records its own directory) and resolve function addresses.
"""

import ctypes
import inspect
import os
import struct
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from zarm import Backend, FakeBackend, InvalidArgumentError, LibraryError, NotConnectedError
from zarm.native import (
    FUNCTIONS,
    FUNCTIONS_BY_NAME,
    REQUIRED_FILES,
    VERIFIED_BUNDLES,
    NativeBackend,
    VendorBundle,
    file_sha256,
    inspect_library_dir,
    load_library,
    unbound_reason,
)
from zarm.pe import PeFormatError, read_pe_info

from .support import is_real_binary, legacy_path, x64_vendor_dir

VENDOR_DIR = x64_vendor_dir()
NEEDS_DLL = unittest.skipIf(VENDOR_DIR is None, "legacy x64 vendor DLL not found (or only a Git LFS pointer)")
ON_WINDOWS_X64 = os.name == "nt" and struct.calcsize("P") == 8


class SignatureTableTests(unittest.TestCase):
    def test_table_is_consistent(self):
        self.assertEqual(len(FUNCTIONS), len(FUNCTIONS_BY_NAME), "duplicate names")
        for function in FUNCTIONS:
            with self.subTest(function=function.name):
                self.assertEqual(len(function.params), len(function.argtypes))
                self.assertEqual(function.types, "verified")
                self.assertIn(function.semantics, ("verified", "corroborated", "inferred", "unknown"))
                self.assertTrue(function.evidence)
                if function.name.startswith("robot_"):
                    self.assertEqual(function.params[0], "card")
                    self.assertIs(function.argtypes[0], ctypes.c_int)

    def test_findings_that_differ_from_community_wrappers(self):
        # robot_initial takes z_travel as an int in this build (the wrapper converts it to float).
        self.assertEqual(FUNCTIONS_BY_NAME["robot_initial"].argtypes, (ctypes.c_int,) * 3)
        # robot_get_scara_param has no servo_off_flag output in the C API.
        scara = FUNCTIONS_BY_NAME["robot_get_scara_param"]
        self.assertEqual(len(scara.argtypes), 10)
        self.assertEqual(scara.params[-3:], ("communicate_success", "initial_finish", "move_flag"))
        # bool returns must be read as bool: only AL is defined.
        self.assertIs(FUNCTIONS_BY_NAME["robot_is_collision"].restype, ctypes.c_bool)

    def test_unbounded_wait_is_never_used(self):
        self.assertFalse(FUNCTIONS_BY_NAME["robot_wait_stop"].sdk_uses)
        self.assertNotIn("wait_stop", inspect.getsource(NativeBackend))

    def test_backends_implement_the_seam(self):
        self.assertEqual(NativeBackend.__abstractmethods__, frozenset())
        self.assertEqual(FakeBackend.__abstractmethods__, frozenset())
        self.assertTrue(Backend.__abstractmethods__)

    def test_docs_cover_every_binding(self):
        docs = (Path(__file__).resolve().parents[1] / "docs" / "native-api.md").read_text(encoding="utf-8")
        for function in FUNCTIONS:
            with self.subTest(function=function.name):
                self.assertIn(f"| `{function.name}` |", docs)
        for bundle in VERIFIED_BUNDLES:
            for digest in bundle.sha256.values():
                self.assertIn(digest, docs)


@NEEDS_DLL
class StaticDllTests(unittest.TestCase):
    def test_pe_facts(self):
        info = read_pe_info(VENDOR_DIR / "small_scara_interface.dll")
        self.assertEqual(info.machine, "x64")
        self.assertTrue(info.is_dll)
        self.assertFalse(info.is_dotnet)
        self.assertEqual(len(info.exports), 184)

    def test_build_is_the_verified_one(self):
        expected = VERIFIED_BUNDLES[0].sha256
        for name, digest in expected.items():
            with self.subTest(file=name):
                self.assertEqual(file_sha256(VENDOR_DIR / name), digest)

    def test_companion_files_are_part_of_the_verified_set(self):
        known = VERIFIED_BUNDLES[0].sha256
        altered = VendorBundle("test", dict(known, **{"share.dll": "0" * 64}))
        with mock.patch("zarm.native.VERIFIED_BUNDLES", (altered,)):
            report = inspect_library_dir(VENDOR_DIR, check_runtime=False)
        self.assertFalse(report.verified_build)
        self.assertTrue(any("share.dll" in problem and "server.exe" not in problem for problem in report.problems))
        with mock.patch("zarm.native.VERIFIED_BUNDLES", (altered,)):
            relaxed = inspect_library_dir(VENDOR_DIR, check_runtime=False, allow_unverified_build=True)
        self.assertEqual(relaxed.problems, ())

    def test_every_plain_c_export_is_accounted_for(self):
        exports = read_pe_info(VENDOR_DIR / "small_scara_interface.dll").exports
        plain = {name for name in exports if not name.startswith(("?", "Java_"))}
        self.assertEqual(len(plain), 68)
        bound = set(FUNCTIONS_BY_NAME)
        self.assertLessEqual(bound, plain)
        for name in sorted(plain - bound):
            with self.subTest(export=name):
                self.assertIsNotNone(unbound_reason(name))

    def test_preflight(self):
        report = inspect_library_dir(VENDOR_DIR, check_runtime=False)
        self.assertEqual(report.machine, "x64")
        self.assertTrue(report.verified_build)
        self.assertEqual(report.missing_exports, ())
        for name in REQUIRED_FILES:
            if not (VENDOR_DIR / name).is_file():
                self.assertTrue(any(name in problem for problem in report.problems))
        if all((VENDOR_DIR / name).is_file() for name in REQUIRED_FILES):
            self.assertEqual(report.problems, ())

    def test_preflight_rejects_the_x86_build(self):
        x86 = legacy_path("Z-Arm Sliders", "Z-Arm Sliders", "Lib_Bak", "new")
        if x86 is None or not is_real_binary(x86 / "small_scara_interface.dll"):
            self.skipTest("x86 build not available")
        report = inspect_library_dir(x86, check_runtime=False)
        self.assertEqual(report.machine, "x86")
        self.assertFalse(report.verified_build)
        self.assertTrue(any("x86" in problem for problem in report.problems))

    def test_control_bean_assembly_is_dotnet(self):
        info = read_pe_info(VENDOR_DIR / "ClassLibrary_ControlBean.dll")
        self.assertTrue(info.is_dotnet)
        self.assertEqual(info.exports, ())


class PreflightTests(unittest.TestCase):
    def test_empty_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            report = inspect_library_dir(directory, check_runtime=False)
            self.assertFalse(report.ok)
            self.assertEqual(sum("is missing" in problem for problem in report.problems), 3)
            with self.assertRaises(LibraryError):
                report.raise_for_problems()
            with self.assertRaises(LibraryError):
                NativeBackend(directory)

    def test_not_a_pe_file(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in REQUIRED_FILES:
                Path(directory, name).write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
            report = inspect_library_dir(directory, check_runtime=False)
            self.assertTrue(any("cannot read" in problem for problem in report.problems))
            with self.assertRaises(PeFormatError):
                read_pe_info(Path(directory, "share.dll"))

    def test_truncated_pe(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory, "broken.dll")
            path.write_bytes(b"MZ" + b"\0" * 58 + struct.pack("<I", 4096))
            with self.assertRaises(PeFormatError):
                read_pe_info(path)


@NEEDS_DLL
@unittest.skipUnless(ON_WINDOWS_X64, "loading the DLL needs 64-bit Python on Windows")
class LoadedDllTests(unittest.TestCase):
    def test_binding_and_guards(self):
        library = load_library(VENDOR_DIR / "small_scara_interface.dll")
        self.assertEqual(set(library.functions), set(FUNCTIONS_BY_NAME))
        for function in FUNCTIONS:
            bound = library.functions[function.name]
            self.assertEqual(tuple(bound.argtypes), function.argtypes)
            self.assertIs(bound.restype, function.restype)

        backend = NativeBackend(VENDOR_DIR)
        try:
            with self.assertRaises(LibraryError):
                NativeBackend(VENDOR_DIR)  # one per process
            # Every robot call is refused until start_server() succeeds, without reaching the DLL.
            for call in (
                lambda: backend.get_scara_param(59),
                lambda: backend.stop_move(59),
                lambda: backend.card_connect_status(59),
            ):
                with self.assertRaises(NotConnectedError):
                    call()
            # Values that ctypes would narrow never reach the DLL.
            for call in (
                lambda: backend.movej_angle(59, 1e300, 0.0, 0.0, 0.0, 10.0, 0.0),
                lambda: backend.movej_angle(59, float("nan"), 0.0, 0.0, 0.0, 10.0, 0.0),
                lambda: backend.get_digital_io_in(59, 2**32),
                lambda: backend.movej_xyz_lr(59, 250.0, 0.0, 0.0, 0.0, 10.0, 0.0, 2**31),
                lambda: backend.stop_move(256),
            ):
                with self.assertRaises(InvalidArgumentError):
                    call()
        finally:
            backend.close()
        with self.assertRaises(LibraryError):
            backend.get_scara_param(59)
        backend = NativeBackend(VENDOR_DIR)
        backend.close()

        other_copy = legacy_path("ZArmUnityAdapter", "ZArmUnityAdapter", "Lib", "small_scara_interface.dll")
        if other_copy is not None and is_real_binary(other_copy):
            with self.assertRaises(LibraryError):
                load_library(other_copy)


class CardGuardTests(unittest.TestCase):
    def test_card_range_is_checked_before_the_dll(self):
        from zarm.native import _check_card

        for bad in (-1, 256, True, 1.5):
            with self.subTest(card=bad), self.assertRaises(InvalidArgumentError):
                _check_card(bad)
        _check_card(0)
        _check_card(255)


if __name__ == "__main__":
    unittest.main()
