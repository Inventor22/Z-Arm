"""MuJoCo MJCF writer for :class:`zarm_sim.model.RobotModel`, reusable in a larger scene.

:func:`arm_mjcf` turns the model into the pieces of one arm, every name prefixed
and the base placed at a given pose: its materials, its body subtree, actuators,
sensors and home keyframe values. :func:`scene` assembles any number of such
pieces into a ``<mujoco>`` document, so a cell with several arms only needs a
different prefix and base pose per arm. ``models/zarm_2442.mjcf.xml`` is
``scene([arm_mjcf(model)])``.

Fixed child frames without geometry (``tool0``, ``sdk_origin``) become sites.
The joint and actuator dynamics below are placeholders: Hitbot publishes no
servo data. Moving bodies use ``gravcomp`` (MuJoCo 3), standing in for the
controller's gravity compensation, so the position servos only handle inertia.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .model import Box, Cylinder, Joint, Link, Part, RobotModel
from .xmlfmt import num, to_text, vec

Quat = Tuple[float, float, float, float]

# Placeholder servo settings per joint: position gain, joint damping and armature (reflected rotor inertia).
SERVOS: Dict[str, Dict[str, float]] = {
    "j3_vertical": {"kp": 20000.0, "damping": 800.0, "armature": 0.5},
    "j1_shoulder": {"kp": 200.0, "damping": 15.0, "armature": 0.01},
    "j2_elbow": {"kp": 100.0, "damping": 5.0, "armature": 0.01},
    "j4_wrist": {"kp": 5.0, "damping": 0.1, "armature": 0.001},
}


def yaw_quat(yaw_rad: float) -> Quat:
    """MuJoCo quaternion (w, x, y, z) for a rotation about +Z."""
    return (math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0))


@dataclass
class ArmMjcf:
    """One arm's MJCF pieces. Names are all prefixed; joints are listed in qpos order."""

    prefix: str
    materials: List[ET.Element]
    body: ET.Element
    actuators: List[ET.Element]
    sensors: List[ET.Element]
    joints: Tuple[str, ...]
    home_qpos: Tuple[float, ...]
    home_ctrl: Tuple[float, ...]
    excludes: List[ET.Element] = field(default_factory=list)
    sites: Dict[str, str] = field(default_factory=dict)  # model frame name -> prefixed site name


def _geom(part: Part, prefix: str) -> ET.Element:
    shape = part.shape
    if isinstance(shape, Box):
        attrs = {"type": "box", "size": vec(s / 2.0 for s in shape.size)}
    elif isinstance(shape, Cylinder):
        attrs = {"type": "cylinder", "size": vec((shape.radius, shape.length / 2.0))}
    else:  # pragma: no cover - the model only builds boxes and cylinders
        raise TypeError(f"unsupported shape {shape!r}")
    element = ET.Element("geom", name=prefix + part.name, **attrs)
    element.set("pos", vec(part.xyz))
    element.set("material", prefix + part.material)
    if not part.collision:
        element.set("contype", "0")
        element.set("conaffinity", "0")
        element.set("group", "2")
    return element


def _is_frame(model: RobotModel, link: Link) -> bool:
    has_children = any(joint.parent == link.name for joint in model.joints)
    return not link.parts and link.inertial is None and not has_children


def _body(model: RobotModel, link: Link, joint: Optional[Joint], prefix: str, sites: Dict[str, str]) -> ET.Element:
    body = ET.Element("body", name=prefix + link.name)
    if joint is not None:
        body.set("pos", vec(joint.xyz))
        if joint.kind != "fixed":
            body.set("gravcomp", "1")
            servo = SERVOS[joint.name]
            ET.SubElement(body, "joint", name=prefix + joint.name, type="slide" if joint.kind == "prismatic" else "hinge",
                          axis=vec(joint.axis), range=vec((joint.lower, joint.upper)), damping=num(servo["damping"]),
                          armature=num(servo["armature"]))
    if link.inertial is not None:
        ixx, ixy, ixz, iyy, iyz, izz = link.inertial.inertia
        ET.SubElement(body, "inertial", pos=vec(link.inertial.com), mass=num(link.inertial.mass),
                      fullinertia=vec((ixx, iyy, izz, ixy, ixz, iyz)))
    for part in link.parts:
        body.append(_geom(part, prefix))
    for child_joint in model.joints:
        if child_joint.parent != link.name:
            continue
        child = model.link(child_joint.child)
        if child_joint.kind == "fixed" and _is_frame(model, child):
            sites[child.name] = prefix + child.name
            ET.SubElement(body, "site", name=prefix + child.name, pos=vec(child_joint.xyz), size="0.004",
                          rgba="1 0.45 0.1 1")
        else:
            body.append(_body(model, child, child_joint, prefix, sites))
    return body


def arm_mjcf(
    model: RobotModel,
    prefix: str = "zarm_",
    pos: Sequence[float] = (0.0, 0.0, 0.0),
    quat: Quat = (1.0, 0.0, 0.0, 0.0),
) -> ArmMjcf:
    """The arm's MJCF pieces, with every name prefixed and the base underside at ``pos``/``quat`` in the world."""
    if not prefix or not prefix.replace("_", "").isalnum():
        raise ValueError("prefix must be a non-empty identifier such as 'zarm_' or 'zarm2_'")
    roots = [link for link in model.links if all(joint.child != link.name for joint in model.joints)]
    if len(roots) != 1:
        raise ValueError(f"the model must have exactly one root link, found {[link.name for link in roots]}")
    sites: Dict[str, str] = {}
    body = _body(model, roots[0], None, prefix, sites)
    body.set("pos", vec(pos))
    body.set("quat", vec(quat))
    materials = [ET.Element("material", name=prefix + material.name, rgba=vec(material.rgba))
                 for material in model.materials]
    movable = model.movable_joints()
    actuators = []
    for joint in model.sdk_joints():
        servo = SERVOS[joint.name]
        actuators.append(ET.Element("position", name=prefix + joint.name, joint=prefix + joint.name,
                                    kp=num(servo["kp"]), ctrlrange=vec((joint.lower, joint.upper)),
                                    forcerange=vec((-joint.effort, joint.effort))))
    sensors = [ET.Element("jointpos", name=f"{prefix}{joint.name}_pos", joint=prefix + joint.name)
               for joint in model.sdk_joints()]
    if "tool0" in sites and "sdk_origin" in sites:
        frame = {"objtype": "site", "objname": sites["tool0"], "reftype": "site", "refname": sites["sdk_origin"]}
        sensors.append(ET.Element("framepos", name=prefix + "tool0_pos", **frame))
        sensors.append(ET.Element("framequat", name=prefix + "tool0_quat", **frame))
    # MuJoCo doesn't filter parent-child contacts when the parent is welded to the world, as the base is. The
    # carriage rides in the column's slot (its joint is the guide), so their touching shapes must not collide.
    excludes = [ET.Element("exclude", body1=prefix + joint.parent, body2=prefix + joint.child)
                for joint in movable if joint.parent == roots[0].name]
    return ArmMjcf(
        prefix=prefix,
        materials=materials,
        body=body,
        actuators=actuators,
        sensors=sensors,
        joints=tuple(prefix + joint.name for joint in movable),
        home_qpos=tuple(0.0 for _ in movable),
        home_ctrl=tuple(0.0 for _ in actuators),
        excludes=excludes,
        sites=sites,
    )


def scene(arms: Sequence[ArmMjcf], name: str, floor_half_size_m: float = 1.0) -> ET.Element:
    """A ``<mujoco>`` document with a floor, a light and the given arms (in order)."""
    root = ET.Element("mujoco", model=name)
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3", specular="0 0 0")
    ET.SubElement(visual, "global", azimuth="-130", elevation="-25")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="floor_grid", type="2d", builtin="checker", rgb1="0.25 0.26 0.28",
                  rgb2="0.21 0.22 0.24", mark="edge", markrgb="0.36 0.38 0.41", width="200", height="200")
    ET.SubElement(asset, "material", name="floor_grid", texture="floor_grid", texrepeat="5 5", texuniform="true")
    for arm in arms:
        asset.extend(arm.materials)
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="key", pos="0.6 -0.6 1.6", dir="-0.35 0.35 -1", directional="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size=vec((floor_half_size_m, floor_half_size_m, 0.05)),
                  material="floor_grid")
    for arm in arms:
        world.append(arm.body)
    contact = ET.SubElement(root, "contact")
    for arm in arms:
        contact.extend(arm.excludes)
    actuator = ET.SubElement(root, "actuator")
    sensor = ET.SubElement(root, "sensor")
    for arm in arms:
        actuator.extend(arm.actuators)
        sensor.extend(arm.sensors)
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", name="home", qpos=vec(q for arm in arms for q in arm.home_qpos),
                  ctrl=vec(c for arm in arms for c in arm.home_ctrl))
    return root


def mjcf_text(model: RobotModel, prefix: str = "zarm_") -> str:
    d = model.dimensions
    comments = [
        "Generated by `python -m zarm_sim.generate` from zarm-sim/zarm_sim/dimensions.py; do not edit by hand.",
        f"Hitbot Z-Arm 2442 (assumed) with a {num(d.z_stroke_mm)} mm Z stroke, standing on a floor. SI units. "
        "Dimension sources and assumptions are in zarm-sim/README.md; servo settings are placeholders.",
        "qpos order: j3_vertical (m, 0 at the top), j1_shoulder, j2_elbow, j4_wrist (rad). Actuators are position "
        "servos in zarm-sdk order: J1, J2, J3, J4.",
        "To view it in MuJoCo's own viewer, see zarm-sim/README.md.",
    ]
    return to_text(scene([arm_mjcf(model, prefix)], model.name), comments)
