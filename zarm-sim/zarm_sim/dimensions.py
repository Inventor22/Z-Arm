"""Every dimension of the Z-Arm model, with where it comes from.

Lengths are in millimetres, angles in degrees and masses in kilograms, the units
of the sources. :mod:`zarm_sim.model` converts them to SI.

The model is a Hitbot Z-Arm 2442 (assumed; the arm is in storage) with a 310 mm
Z stroke. Positions are given in the robot frame: origin at the centre of the
base plate's underside (the mounting surface), +Z up, +X from the column toward
the J1 axis (the direction the arm points at J1 = J2 = 0), +Y to the left.

Sources, cited by tag in each field's ``source``:

``SPEC``
    Hitbot's Z-Arm 2442E product page,
    https://www.hitbotrobot.com/product/robotic-arm-z-arm-2442e/
``DRAWING``
    Hitbot's motion-range drawing, 2442-Size-Diagram.zip from the download
    centre (one PDF; "Z-Arm 2442", V1.1.0, updated 2020-01-18, A3 at 1:5).
``STEP``
    Hitbot's CAD, 2442-3D-Diagram.zip ("Z-Arm 2442 V1.0.0.stp", CATIA V5,
    2020-08-14, mm), measured in the robot frame. Its arm sits at J1 = J2 = 0
    with the flange face 82.5 mm above the mounting surface.
    ``scripts/fetch_cad_meshes.py`` repeats the measurements and checks them.
``HITBOT_SIM``
    The community URDF ``urdf/Z-Arm_10042C0.urdf`` in jjh1214/hitbot_sim (MIT),
    a SolidWorks export of the 1 m column variant of the same XX42 arm.
``LEGACY``
    This repository's legacy C# "Z-Arm Sliders" app (z_travel 310, slider
    ranges) and the defaults zarm-sdk took from it.
``ASSUMED``
    Not documented anywhere we found; chosen and explained here.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Dict, List, Tuple


def _dim(value: float, source: str) -> float:
    return field(default=value, metadata={"source": source})  # type: ignore[return-value]


@dataclass(frozen=True)
class ZArmDimensions:
    # This arm's configuration
    z_stroke_mm: float = _dim(
        310.0,
        "LEGACY: z_travel 310 and the J3 slider range -310..0, which zarm-sdk uses as "
        "ZArmConfig.z_travel_mm and its J3 soft limits. SPEC sells 110/210/310/410/510 mm strokes.",
    )
    reference_stroke_mm: float = _dim(210.0, "SPEC: standard Z-axis stroke 210 mm.")
    reference_height_mm: float = _dim(
        596.0,
        "SPEC: height 596 mm (with the 210 mm stroke); DRAWING: overall height 596. "
        "The STEP's column is 593 mm tall.",
    )

    # Base and column (fixed)
    base_plate_size_mm: float = _dim(200.0, "SPEC: foundation 200 x 200 x 10 mm; DRAWING; STEP.")
    base_plate_thickness_mm: float = _dim(10.0, "SPEC; DRAWING (10); STEP.")
    mount_hole_spacing_mm: float = _dim(
        160.0, "SPEC: fixing holes 160 x 160 mm, 4 x M8; DRAWING: 4 x 8.5 through, 14 counterbore 5 deep."
    )
    mount_counterbore_diameter_mm: float = _dim(14.0, "DRAWING: 4 x 14 counterbore, 5 deep.")
    column_depth_mm: float = _dim(
        128.0,
        "STEP: front and back faces at x = +/-64 mm, centred on the base. DRAWING agrees: J1 axis to "
        "column back face 197 = 133 + 64.",
    )
    column_width_mm: float = _dim(
        148.0, "STEP: the sides are R140 arcs, 148 mm apart at their widest (modelled as a box); DRAWING front view."
    )
    column_cap_height_mm: float = _dim(15.0, "STEP: top cap from 578 to 593 mm.")
    j1_axis_offset_mm: float = _dim(
        133.0,
        "STEP: J1 axis 133.000 mm from the base centre, square to the base edges; DRAWING: 197 - 64; "
        "HITBOT_SIM: shoulder joint origin x = 0.133 m.",
    )

    # Arms (move with J3 and the revolute joints)
    j1_arm_length_mm: float = _dim(220.0, "SPEC: J1 arm 220 mm; DRAWING (220); STEP (220.000 between axes).")
    j2_arm_length_mm: float = _dim(200.0, "SPEC: J2 arm 200 mm; DRAWING (200); STEP (200.000 between axes).")
    j1_arm_end_radius_mm: float = _dim(53.0, "STEP: R53 round ends about the J1 and elbow axes (106 mm wide).")
    j1_arm_thickness_mm: float = _dim(73.0, "STEP: 141.4 to 214.4 mm at the STEP's height; DRAWING: about 72.")
    j2_arm_elbow_radius_mm: float = _dim(53.0, "STEP: R53 round end about the elbow axis.")
    j2_arm_tool_radius_mm: float = _dim(47.5, "STEP: R47.5 round end about the J4 axis; DRAWING view A-A: about 95 wide.")
    j2_arm_thickness_mm: float = _dim(50.0, "STEP: 89.4 to 139.4 mm at the STEP's height; DRAWING: about 45 plus covers.")
    arm_gap_mm: float = _dim(
        2.0,
        "STEP: 2 mm between the J2 arm's top and the J1 arm's underside. ASSUMED: the same gap under the J1 "
        "housing (the STEP's housing is recessed around the J1 arm's hub; DRAWING shows a gap of about 2.5).",
    )
    flange_diameter_mm: float = _dim(
        84.0, "STEP: 84 mm boss on the J4 axis. DRAWING view A-A gives the tool interface (72, 55, 40 H7)."
    )
    flange_protrusion_mm: float = _dim(6.9, "STEP: flange face at 82.5 mm, J2 arm underside at 89.4 mm.")
    housing_width_mm: float = _dim(100.0, "STEP: J1 housing y = +/-50 mm with an R50 nose about J1; DRAWING front view 101.")
    housing_top_above_flange_mm: float = _dim(
        195.9, "STEP: housing top at 278.4 mm with the flange face at 82.5 mm; DRAWING: about 195.6."
    )
    flange_face_at_bottom_mm: float = _dim(
        64.0,
        "DRAWING: 64 +/- 1 mm from the mounting surface to the flange face at the lowest position. ASSUMED: "
        "that is the bottom of the 310 mm stroke too (the drawing's two positions are 240 mm apart on a "
        "210 mm-stroke arm, so the rated stroke may sit up to 30 mm higher).",
    )

    # Joint ranges of the hardware (the SDK's soft limits are its own)
    j1_range_deg: float = _dim(90.0, "SPEC: 1st axis +/-90 deg (LEGACY sliders and zarm-sdk agree).")
    j2_range_deg: float = _dim(164.0, "SPEC: 2nd axis +/-164 deg (LEGACY sliders and zarm-sdk agree).")
    j4_range_deg: float = _dim(1080.0, "SPEC: R axis +/-1080 deg (zarm-sdk's soft limit is +/-180).")

    # Masses
    total_mass_kg: float = _dim(19.0, "SPEC: weight 19 kg (210 mm stroke; the extra 100 mm of column is ignored).")
    carriage_mass_kg: float = _dim(0.99, "HITBOT_SIM: link1 (the Z carriage) 0.994 kg.")
    j1_arm_mass_kg: float = _dim(5.86, "HITBOT_SIM: link2 (the J1 arm with its motors) 5.858 kg.")
    j2_arm_mass_kg: float = _dim(3.43, "HITBOT_SIM: link3 (the J2 arm with the R drive) 3.433 kg.")
    flange_mass_kg: float = _dim(0.12, "HITBOT_SIM: link4 (the R flange) 0.121 kg.")

    # Drive limits for simulators. Hitbot publishes none of these per joint.
    j1_j2_velocity_rad_s: float = _dim(
        2.9, "ASSUMED: SPEC's 1220 mm/s linear speed over the 420 mm reach (an upper bound, not a rating)."
    )
    j3_velocity_m_s: float = _dim(0.5, "ASSUMED placeholder.")
    j4_velocity_rad_s: float = _dim(6.28, "ASSUMED placeholder (one turn per second).")
    j1_j2_effort_nm: float = _dim(30.0, "ASSUMED placeholder (HITBOT_SIM uses 30 for its revolute joints).")
    j3_effort_n: float = _dim(300.0, "ASSUMED placeholder: about twice the weight of the arms plus a 3 kg payload.")
    j4_effort_nm: float = _dim(5.0, "ASSUMED placeholder.")

    # Derived values

    @property
    def overall_height_mm(self) -> float:
        """ASSUMED: the column grows one-for-one with the stroke option (596 mm for 210 mm, so 696 for 310)."""
        return self.reference_height_mm + (self.z_stroke_mm - self.reference_stroke_mm)

    @property
    def flange_face_at_top_mm(self) -> float:
        """Flange face height at the top of travel (J3 = 0): the bottom of travel plus the stroke."""
        return self.flange_face_at_bottom_mm + self.z_stroke_mm

    @property
    def base_mass_kg(self) -> float:
        """The spec's total less the moving links."""
        return self.total_mass_kg - self.carriage_mass_kg - self.j1_arm_mass_kg - self.j2_arm_mass_kg - self.flange_mass_kg

    def stack_mm(self) -> Dict[str, Tuple[float, float]]:
        """(bottom, top) of each moving body above the flange face, from the STEP's vertical stack."""
        flange = (0.0, self.flange_protrusion_mm)
        j2_arm = (flange[1], flange[1] + self.j2_arm_thickness_mm)
        j1_arm = (j2_arm[1] + self.arm_gap_mm, j2_arm[1] + self.arm_gap_mm + self.j1_arm_thickness_mm)
        housing = (j1_arm[1] + self.arm_gap_mm, self.housing_top_above_flange_mm)
        return {"flange": flange, "j2_arm": j2_arm, "j1_arm": j1_arm, "housing": housing}

    def validate(self) -> None:
        for spec in fields(self):
            value = getattr(self, spec.name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not value > 0:
                raise ValueError(f"{spec.name} must be a positive number, got {value!r}")
        bottom, top = self.stack_mm()["housing"]
        if top <= bottom:
            raise ValueError("the J1 housing has no height: housing_top_above_flange_mm is too small")
        if self.j1_axis_offset_mm - self.j1_arm_end_radius_mm < self.column_depth_mm / 2:
            raise ValueError("the J1 arm's round end would cut into the column")
        if self.overall_height_mm <= self.flange_face_at_top_mm + self.housing_top_above_flange_mm:
            raise ValueError("the carriage would rise above the column at the top of travel")


def dimension_table(dims: ZArmDimensions = ZArmDimensions()) -> List[Tuple[str, float, str]]:
    """(field name, value, source) for every dimension, in declaration order."""
    return [(spec.name, getattr(dims, spec.name), spec.metadata["source"]) for spec in fields(dims)]
