import unittest

from zarm import (
    AmbiguousArmError,
    ArmBusyError,
    CollisionError,
    CommandError,
    HomeResult,
    InitializationError,
    InitResult,
    JointFaultError,
    JointState,
    MoveResult,
    NotInitializedError,
    UnreachableError,
    ZArmConnectionError,
    ZArmError,
    describe,
)
from zarm.codes import DESCRIPTIONS, coerce
from zarm.errors import error_for_move_code


class CodeTests(unittest.TestCase):
    def test_coerce_known_and_unknown(self):
        self.assertIs(coerce(MoveResult, 102), MoveResult.COLLISION)
        self.assertIs(coerce(JointState, 7), JointState.FAULT)
        unknown = coerce(MoveResult, 999)
        self.assertEqual(unknown, 999)
        self.assertNotIsInstance(unknown, MoveResult)

    def test_describe(self):
        self.assertIn("NaN", describe(MoveResult.INVALID_PARAMETER))
        self.assertIn("999", describe(999))

    def test_descriptions_do_not_collide_across_enums(self):
        # IntEnum members hash like ints; a single dict would mix up InitResult 3 and HomeResult 3.
        self.assertIn("encoder", describe(InitResult.ENCODER_ERROR))
        self.assertIn("refused", describe(HomeResult.REFUSED))
        self.assertIn("communication", describe(InitResult.NOT_CONNECTED))
        self.assertIn("not initialized", describe(MoveResult.NOT_INITIALIZED))
        for enum_cls, table in DESCRIPTIONS.items():
            with self.subTest(enum=enum_cls.__name__):
                self.assertEqual(set(table), {member.value for member in enum_cls})

    def test_move_codes_match_the_dll(self):
        # Values read out of small_scara_interface.dll (see docs/native-api.md).
        self.assertEqual(
            {code.name: code.value for code in MoveResult},
            {
                "BUSY": 0,
                "OK": 1,
                "INVALID_SPEED": 2,
                "NOT_INITIALIZED": 3,
                "UNREACHABLE": 4,
                "SERVO_OFF": 6,
                "HAND_CONFIGURATION_UNREACHABLE": 7,
                "MOBILE_TERMINAL_IN_CONTROL": 11,
                "INVALID_PARAMETER": 101,
                "COLLISION": 102,
                "JOINT_NEEDS_REINITIALIZATION": 103,
                "JOINT_FAULT": 105,
            },
        )
        self.assertEqual(InitResult.Z_TRAVEL_ERROR, 12)


class ErrorTests(unittest.TestCase):
    def test_move_code_mapping(self):
        expected = {
            0: ArmBusyError,
            3: NotInitializedError,
            4: UnreachableError,
            7: UnreachableError,
            102: CollisionError,
            103: JointFaultError,
            105: JointFaultError,
        }
        for code, cls in expected.items():
            with self.subTest(code=code):
                error = error_for_move_code("movej_angle", code)
                self.assertIs(type(error), cls)
                self.assertEqual(error.code, code)
                self.assertEqual(error.command, "movej_angle")
                self.assertIn(f"vendor code {code}", str(error))

    def test_unmapped_codes_use_the_base_class(self):
        for code in (2, 6, 11, 101, 999):
            with self.subTest(code=code):
                self.assertIs(type(error_for_move_code("movel_xyz", code)), CommandError)

    def test_hierarchy(self):
        self.assertTrue(issubclass(CollisionError, CommandError))
        self.assertTrue(issubclass(CommandError, ZArmError))
        self.assertTrue(issubclass(AmbiguousArmError, ZArmConnectionError))

    def test_initialization_error(self):
        error = InitializationError(12, attempts=1)
        self.assertIs(error.code, InitResult.Z_TRAVEL_ERROR)
        self.assertIn("z_travel", str(error))
        self.assertEqual(InitializationError(77, attempts=4).code, 77)

    def test_ambiguous_arm_error_lists_cards(self):
        error = AmbiguousArmError([3, 59])
        self.assertEqual(error.card_ids, (3, 59))
        self.assertIn("card_id", str(error))


if __name__ == "__main__":
    unittest.main()
