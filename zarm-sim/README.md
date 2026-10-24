# zarm-sim

A robot model of the Hitbot Z-Arm (URDF and MuJoCo MJCF, generated from one parametric source) and a
**simulator view**: a web page on this PC that shows the arm in 3D, driven live by
[zarm-sdk](../zarm-sdk/README.md)'s simulated backend, with jog and move controls.

> **Status.** Everything here runs against zarm-sdk's simulator (`FakeBackend`). Nothing has driven the real
> arm, which is in storage. The arm is assumed to be a Z-Arm 2442 with a 310 mm Z stroke; the dimensions come from
> Hitbot's published drawing and CAD, and the ones that had to be assumed are listed [below](#assumptions).

## See it: the simulator view

```powershell
cd zarm-sim
py -3.12 -m zarm_sim.server            # then open http://127.0.0.1:8770
py -3.12 -m zarm_sim.server --open     # ...or let it open your browser
```

The first start downloads three.js 0.186.1 (which draws the 3D view) into `zarm-sim/vendor/three/`, exactly as
umbratek-code's jog panel does: from jsDelivr or unpkg, checked against pinned SHA-256 hashes, and served from the
checked copy only. After that the view works offline. `--port` picks another port (umbratek-code's jog panel uses
8765). Python 3.9 or later, standard library only; the SDK is found in the sibling `zarm-sdk/` folder.

What's on the page:

* **SIMULATED**, always: a placard in the sticky top bar, a mark on the 3D view, and the page title.
* **The monitor**: the arm built from `models/zarm_2442.urdf`, posed about 30 times a second from the SDK's state.
  The floor carries a millimetre grid (10 and 100 mm lines, labelled in the SDK's own x and y), the SDK's +x and +y
  axes at the J1 axis, the **reach envelope** for the SDK's soft limits (outlined) and Hitbot's **blind zone**
  (hatched red: positions the joint limits allow only with the J2 arm inside the column). While a move runs, an
  amber shadow arm and a ring with a drop line show its goal. Coloured rings and a rail name J1 to J4 in the same
  colours as the joint strips. Drag to orbit, right-drag to pan, scroll to zoom.
* **The bridge**: link lamp, the state window (READY, MOVING, NEEDS INIT, NO LINK), Initialize, E-STOP and STOP.
  Esc presses STOP from anywhere on the page.
* **Joints**: per-joint jog keys with step and speed detents, the position (J1, J2, J4 in degrees, J3 in mm), the
  goal while moving, and a meter across each joint's soft-limit range.
* **Pose**: x, y, z, r from the SDK, the tool's yaw (J1 + J2 + r), and a check that the model's forward
  kinematics agree with the SDK's pose.
* **Move to joints**, **Move to pose** (joint-interpolated, with hand selection: the current hand, or lr +1, -1 or 0
  through `movej_xyz_lr`), **Move linear**, and **Change hand**.
* **Messages**: every command's result; the SDK's refusals word for word, such as
  `SoftLimitError: target is outside the soft limits: j1_deg=95 is outside [-90, 90] deg`.

How it behaves, following umbratek-code's jog panel:

* **One owner.** The server owns the one `ZArm.simulated(ZArmConfig(arm_lengths_mm=(220.0, 200.0)))`; the page
  never talks to the SDK. The server refuses any arm whose backend isn't exactly zarm's `FakeBackend`
  (`zarm_sim.sdk.require_simulated`), and never imports the native backend or Hitbot's DLL.
* **Exact steps.** Each jog click moves one step from the last *commanded* goal, not from the lagging measured
  position, so quick clicks add up exactly. The simulated controller, like the real one, refuses a move while
  another runs, so clicks during a step are queued and sent as one move when it ends; the goal may run at most 3
  steps ahead. Holding a key repeats.
* **Stop** works at any time. **E-STOP** calls `emergency_stop()`, which latches: the state window shows NEEDS
  INIT and the arm won't move until you press Initialize.
* A move under way stops if the page goes quiet for 3 s (a closed tab, say).
* It listens on 127.0.0.1 only, checks the Host header, and every POST must carry a token embedded in the page
  when the server starts, so other web pages can't drive it.

Without WebGL, or if three.js couldn't be downloaded, the page says so in the 3D view's place; the readouts and
controls still work.

## The model

`models/zarm_2442.urdf` and `models/zarm_2442.mjcf.xml` are generated from `zarm_sim/dimensions.py` (every number
and its source) and `zarm_sim/model.py` (links, joints, shapes). Don't edit them by hand:

```powershell
cd zarm-sim
py -3.12 -m zarm_sim.generate          # rewrite models/
py -3.12 -m zarm_sim.generate --check  # exit 1 if models/ is out of date
```

Output is deterministic (fixed number formatting and ordering, LF line endings pinned by `.gitattributes`), and a
test regenerates both files and compares them byte for byte.

Frames, all with +Z up:

| Frame | Where |
|---|---|
| `base_link` | Centre of the base plate's underside (the mounting surface); +X from the column toward the arm |
| `carriage` | J3 slides it along the column axis; its origin is at the flange face's height |
| `j1_arm`, `j2_arm`, `flange` | On the J1, elbow and J4 axes, at the flange face's height |
| `tool0` | Centre of the flange face |
| `sdk_origin` | The frame zarm-sdk's Cartesian poses are assumed to be in: on the J1 axis, at the flange face's height at the top of travel (374 mm above the mounting surface) |

Joints and the SDK (the same conventions as zarm-sdk's `kinematics.py`):

| SDK | Model joint | Value | Range in the model (hardware) | SDK soft limits |
|---|---|---|---|---|
| J1 `j1_deg` | `j1_shoulder`, revolute | radians(J1) | ±90° | ±90° |
| J2 `j2_deg` | `j2_elbow`, revolute | radians(J2) | ±164° | ±164° |
| J3 `j3_mm` | `j3_vertical`, prismatic | J3 / 1000 m | -0.31 to 0 m | -310 to 0 mm |
| J4 `j4_deg` (the vendor's r) | `j4_wrist`, revolute | radians(J4) | ±1080° | ±180° |

J1 = J2 = 0 points the arm along +X; positive angles turn counter-clockwise seen from above. J3 is 0 at the top of
travel and negative going down. r is J4 relative to the J2 arm, so the tool's yaw is J1 + J2 + r. With these,
`tool0` in `sdk_origin` is exactly `(forward_xy(J1, J2, 220, 200), J3)` with yaw J1 + J2 + J4, which the tests check
for random joints. `zarm_sim.model.sdk_to_model` and `model_to_sdk` convert between the two.

The shapes are boxes and cylinders, in both visual and collision geometry: the base plate, the column (a box; the
real sides are slightly bowed R140 arcs) and its cap, the J1 housing on the carriage, the two arms (a box between
two round ends; the J2 arm tapers from R53 to R47.5, so its box takes the mean width) and the flange. Two kinds of
visual-only part: the four mounting counterbores, and an amber tab on the flange's +X edge that isn't on the arm
but makes J4 visible. Masses are the spec's 19 kg split by link (from hitbot_sim), spread over each link's shapes at
uniform density to give its inertia; treat them as estimates.

### Dimensions and their sources

Tags: **SPEC** Hitbot's [Z-Arm 2442E product page](https://www.hitbotrobot.com/product/robotic-arm-z-arm-2442e/).
**DRAWING** Hitbot's motion-range drawing, `2442-Size-Diagram.zip` in the download centre (one PDF, "Z-Arm 2442",
V1.1.0, updated 2020-01-18, A3 at 1:5). **STEP** Hitbot's CAD, `2442-3D-Diagram.zip` ("Z-Arm 2442 V1.0.0.stp",
CATIA V5, 2020-08-14, mm), measured in the robot frame: its arm sits at J1 = J2 = 0 with the flange face 82.5 mm up;
`scripts/fetch_cad_meshes.py` repeats the key measurements. **HITBOT_SIM** the MIT-licensed community URDF
`urdf/Z-Arm_10042C0.urdf` in [jjh1214/hitbot_sim](https://github.com/jjh1214/hitbot_sim), the 1 m column variant of
the same arm family. **LEGACY** this repository's old C# "Z-Arm Sliders" app and the zarm-sdk defaults taken from it.
**ASSUMED** chosen here, for the reason given.

| Field | Value | Source |
|---|---|---|
| `z_stroke_mm` | 310 mm | LEGACY: z_travel 310 and the J3 slider range -310..0, which zarm-sdk uses as ZArmConfig.z_travel_mm and its J3 soft limits. SPEC sells 110/210/310/410/510 mm strokes. |
| `reference_stroke_mm` | 210 mm | SPEC: standard Z-axis stroke 210 mm. |
| `reference_height_mm` | 596 mm | SPEC: height 596 mm (with the 210 mm stroke); DRAWING: overall height 596. The STEP's column is 593 mm tall. |
| `base_plate_size_mm` | 200 mm | SPEC: foundation 200 x 200 x 10 mm; DRAWING; STEP. |
| `base_plate_thickness_mm` | 10 mm | SPEC; DRAWING (10); STEP. |
| `mount_hole_spacing_mm` | 160 mm | SPEC: fixing holes 160 x 160 mm, 4 x M8; DRAWING: 4 x 8.5 through, 14 counterbore 5 deep. |
| `mount_counterbore_diameter_mm` | 14 mm | DRAWING: 4 x 14 counterbore, 5 deep. |
| `column_depth_mm` | 128 mm | STEP: front and back faces at x = +/-64 mm, centred on the base. DRAWING agrees: J1 axis to column back face 197 = 133 + 64. |
| `column_width_mm` | 148 mm | STEP: the sides are R140 arcs, 148 mm apart at their widest (modelled as a box); DRAWING front view. |
| `column_cap_height_mm` | 15 mm | STEP: top cap from 578 to 593 mm. |
| `j1_axis_offset_mm` | 133 mm | STEP: J1 axis 133.000 mm from the base centre, square to the base edges; DRAWING: 197 - 64; HITBOT_SIM: shoulder joint origin x = 0.133 m. |
| `j1_arm_length_mm` | 220 mm | SPEC: J1 arm 220 mm; DRAWING (220); STEP (220.000 between axes). |
| `j2_arm_length_mm` | 200 mm | SPEC: J2 arm 200 mm; DRAWING (200); STEP (200.000 between axes). |
| `j1_arm_end_radius_mm` | 53 mm | STEP: R53 round ends about the J1 and elbow axes (106 mm wide). |
| `j1_arm_thickness_mm` | 73 mm | STEP: 141.4 to 214.4 mm at the STEP's height; DRAWING: about 72. |
| `j2_arm_elbow_radius_mm` | 53 mm | STEP: R53 round end about the elbow axis. |
| `j2_arm_tool_radius_mm` | 47.5 mm | STEP: R47.5 round end about the J4 axis; DRAWING view A-A: about 95 wide. |
| `j2_arm_thickness_mm` | 50 mm | STEP: 89.4 to 139.4 mm at the STEP's height; DRAWING: about 45 plus covers. |
| `arm_gap_mm` | 2 mm | STEP: 2 mm between the J2 arm's top and the J1 arm's underside. ASSUMED: the same gap under the J1 housing (the STEP's housing is recessed around the J1 arm's hub; DRAWING shows a gap of about 2.5). |
| `flange_diameter_mm` | 84 mm | STEP: 84 mm boss on the J4 axis. DRAWING view A-A gives the tool interface (72, 55, 40 H7). |
| `flange_protrusion_mm` | 6.9 mm | STEP: flange face at 82.5 mm, J2 arm underside at 89.4 mm. |
| `housing_width_mm` | 100 mm | STEP: J1 housing y = +/-50 mm with an R50 nose about J1; DRAWING front view 101. |
| `housing_top_above_flange_mm` | 195.9 mm | STEP: housing top at 278.4 mm with the flange face at 82.5 mm; DRAWING: about 195.6. |
| `flange_face_at_bottom_mm` | 64 mm | DRAWING: 64 +/- 1 mm from the mounting surface to the flange face at the lowest position. ASSUMED: that is the bottom of the 310 mm stroke too (the drawing's two positions are 240 mm apart on a 210 mm-stroke arm, so the rated stroke may sit up to 30 mm higher). |
| `j1_range_deg` | 90 deg | SPEC: 1st axis +/-90 deg (LEGACY sliders and zarm-sdk agree). |
| `j2_range_deg` | 164 deg | SPEC: 2nd axis +/-164 deg (LEGACY sliders and zarm-sdk agree). |
| `j4_range_deg` | 1080 deg | SPEC: R axis +/-1080 deg (zarm-sdk's soft limit is +/-180). |
| `total_mass_kg` | 19 kg | SPEC: weight 19 kg (210 mm stroke; the extra 100 mm of column is ignored). |
| `carriage_mass_kg` | 0.99 kg | HITBOT_SIM: link1 (the Z carriage) 0.994 kg. |
| `j1_arm_mass_kg` | 5.86 kg | HITBOT_SIM: link2 (the J1 arm with its motors) 5.858 kg. |
| `j2_arm_mass_kg` | 3.43 kg | HITBOT_SIM: link3 (the J2 arm with the R drive) 3.433 kg. |
| `flange_mass_kg` | 0.12 kg | HITBOT_SIM: link4 (the R flange) 0.121 kg. |
| `j1_j2_velocity_rad_s` | 2.9 rad/s | ASSUMED: SPEC's 1220 mm/s linear speed over the 420 mm reach (an upper bound, not a rating). |
| `j3_velocity_m_s` | 0.5 m/s | ASSUMED placeholder. |
| `j4_velocity_rad_s` | 6.28 rad/s | ASSUMED placeholder (one turn per second). |
| `j1_j2_effort_nm` | 30 N m | ASSUMED placeholder (HITBOT_SIM uses 30 for its revolute joints). |
| `j3_effort_n` | 300 N | ASSUMED placeholder: about twice the weight of the arms plus a 3 kg payload. |
| `j4_effort_nm` | 5 N m | ASSUMED placeholder. |

Derived: overall height 696 mm (596 + 310 - 210); flange face 64 mm above the mounting surface at the bottom of
travel and 374 mm at the top; base and column 8.6 kg (19 kg less the moving links); the J1 housing 62 mm tall, from
2 mm above the J1 arm to 195.9 mm above the flange face (the drawing shows about 62).

Hitbot's drawing also gives, without being modelled: the base's R15 corners, two 8 mm H7 dowel holes 120 mm apart,
and the tool interface in view A-A (as read: a 40 mm H7 pilot, two 3 mm dowel holes 64 mm apart, 4 x M4 on
40 x 56 mm and 6 x M3 equally spaced). Check those against the arm before machining a tool.

### Assumptions

1. The arm is a **Z-Arm 2442** (220 and 200 mm arms) with a **310 mm** Z stroke, as the legacy code configures it.
2. The column is **596 + 100 = 696 mm** tall: Hitbot lists 596 mm for the 210 mm stroke and sells strokes in 100 mm
   steps; we assume the column grows by the same amount.
3. The flange face is **64 mm** above the mounting surface at the bottom of travel (J3 = -310) and **374 mm** at the
   top (J3 = 0). The drawing shows 64 ± 1 mm at its lowest position, but its two positions are 240 mm apart while the
   spec says 210, so the rated stroke may sit up to 30 mm higher on the real arm.
4. The SDK's Cartesian frame (`sdk_origin`) is on the J1 axis at the flange face's height at J3 = 0, so the SDK's
   z equals J3. The controller's own frame is unverified.
5. zarm-sdk's conventions: J1 = J2 = 0 along +X, counter-clockwise positive, r relative to the J2 arm, and the
   simulator's lr convention (lr > 0 gives J2 ≥ 0). See zarm-sdk's README, "Default assumptions".
6. J4 = 0 in the CAD is J4 = 0 on the arm (the flange looks round, so this only matters for the meshes' details).
7. Masses, inertias, velocity and effort limits, and the MuJoCo servo gains are estimates or placeholders.

## MuJoCo

The browser view needs only the standard library; MuJoCo is optional.

```powershell
py -3.12 -m pip install mujoco                                        # 3.x; tested with 3.17.0
py -3.12 -m mujoco.viewer --mjcf=zarm-sim/models/zarm_2442.mjcf.xml    # MuJoCo's own viewer
```

The file is a scene: a floor, a light, and the arm. qpos order is `j3_vertical` (m), `j1_shoulder`, `j2_elbow`,
`j4_wrist` (rad). Four position servos come in the SDK's order (J1, J2, J3, J4), with joint damping and armature,
and the moving bodies use `gravcomp` in place of the controller's gravity compensation; the gains are placeholders
tuned only so the servos settle. Sensors give each joint's position, and `tool0`'s position and orientation in
`sdk_origin`. The carriage-to-column contact is excluded (the J3 joint is the guide), so a contact means a real
collision, such as the J2 arm reaching into the column. The key `home` is all joints at 0.

For a cell with more arms, build the MJCF in Python rather than editing the file: `arm_mjcf` gives one arm's pieces
with every name prefixed and its base at a given pose, and `scene` assembles any number of them.

```python
from zarm_sim.mjcf import arm_mjcf, scene, yaw_quat
from zarm_sim.model import build_model
from zarm_sim.xmlfmt import to_text

model = build_model()
zarm = arm_mjcf(model, prefix="zarm_", pos=(0.6, 0.0, 0.75), quat=yaw_quat(3.14159))   # on a table, facing -X
text = to_text(scene([zarm], "cell"))   # or put zarm.body, .materials, .actuators, .sensors, .excludes in your own
```

Newton reads MJCF too, but `gravcomp` and the servo settings may need a Newton-specific pass.

To import `zarm_sim` from another project, install it editable (`py -3.12 -m pip install -e zarm-sim`): the
package reads `models/`, `vendor/` and `cache/` from this folder, so a regular install isn't supported.

## Optional: Hitbot's CAD as the view's meshes

Hitbot publishes the arm's CAD with no license, so neither it nor anything made from it is committed. To see the
real shapes on your own PC:

```powershell
uv run zarm-sim/scripts/fetch_cad_meshes.py      # needs uv; installs build123d (OpenCascade) in a throwaway env
```

The script downloads `2442-3D-Diagram.zip` into `zarm-sim/cache/cad/`, checks its size and SHA-256 (and the STEP's),
checks the CAD against the model (axes 220 and 200 mm apart, J1 133 mm from the base centre, square to the base),
splits it into the URDF's links, and writes one binary STL per link to `zarm-sim/cache/meshes/` with a manifest.
git ignores `cache/`. The CAD is the 593 mm-tall, 210 mm-stroke arm, so the script stretches the column's middle to
the model's 696 mm; that stretch is an approximation. Restart the server and the view draws the meshes, with a key
to switch back to the primitives; the URDF, the MJCF and the tests always use the primitives. If the model changes,
the view ignores the old meshes until you run the script again.

## Tests

```powershell
cd zarm-sim
py -3.12 -m unittest discover -s tests -t .
```

Standard-library `unittest`, like zarm-sdk's, and nothing touches hardware. They check that the generated files are
byte-identical to the generator's output (also across interpreters), that every dimension has a source and appears
above, that the URDF's forward kinematics match `zarm.kinematics.forward_xy` at 220/200 mm (and tool yaw and J3) for
random joints, the SDK-to-model joint mapping both ways, the MJCF's prefixed, composable pieces, the reach envelope
and blind zone against the SDK's inverse kinematics, three.js's checked download and the mesh cache's checks, and the
HTTP API on an ephemeral port (exact jogs, queueing, SDK refusals, Stop, E-stop and Initialize, the Host and token
checks, the page-quiet stop, and that only the simulated backend is accepted). The MuJoCo tests run when `mujoco` is
installed and skip otherwise; the JavaScript syntax check runs when `node` is on the PATH.

## Files

| Path | What |
|---|---|
| `zarm_sim/dimensions.py` | Every dimension, with its source |
| `zarm_sim/model.py` | Links, joints and shapes in SI units; inertia; SDK joint mapping |
| `zarm_sim/urdf.py`, `zarm_sim/mjcf.py`, `zarm_sim/xmlfmt.py` | The writers |
| `zarm_sim/generate.py` | `python -m zarm_sim.generate [--check]` |
| `zarm_sim/kinematics.py` | Forward kinematics over a URDF (standard library) |
| `zarm_sim/workspace.py` | Reach envelope and blind zone, in the SDK's frame |
| `zarm_sim/server.py` | The simulator view's server |
| `zarm_sim/sdk.py` | Finds zarm-sdk; refuses anything but its simulated backend |
| `zarm_sim/three_js.py`, `zarm_sim/meshes.py` | three.js's checked download; the optional mesh cache |
| `zarm_sim/web/` | The page: `index.html`, `style.css`, `app.js` (controls), `view.js` (3D), `urdf.js` |
| `models/` | The generated URDF and MJCF |
| `scripts/fetch_cad_meshes.py` | Opt-in: meshes from Hitbot's CAD |
| `tests/` | The tests |

The page follows umbratek-code's jog-panel design (its `DESIGN.md`): a charcoal desk where colour only reports
state, amber for motion, green for ready, red for stop and refusals; tape colours name the joints.

## Limitations

* **Simulator only.** The view drives `FakeBackend`, whose timing (each joint at constant speed, the slowest
  setting the duration), limits and refusals follow zarm-sdk's model of the controller, not the controller itself.
* **Dimensions.** Primitive shapes, with the assumptions above; the CAD meshes are optional and the column stretch
  in them is approximate. Not modelled: cables, the end effector, the connector panel, the column's bowed sides.
* **The blind zone** is a plan-view check of the J2 arm against the column, in the view only; the SDK doesn't refuse
  such moves, and a tool hanging below the flange isn't included.
* **Jogs** while a step is under way are sent as one move when it ends, so fast clicking moves in short bursts. The
  page-quiet stop also stops moves when a background tab's timers are throttled for over 3 s.
* **Dynamics** (masses, inertias, MuJoCo gains, limits) are estimates.
