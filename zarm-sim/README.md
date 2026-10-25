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
* **The monitor**: the arm, posed about 30 times a second from the SDK's state. It's drawn with Hitbot's CAD
  meshes once you've made them on this PC ([below](#optional-hitbots-cad-as-the-views-meshes)); until then it draws
  the model's own shapes from `models/zarm_2442.urdf`, and a note under the view gives the exact command that makes
  the meshes. The floor carries a millimetre grid (10 and 100 mm lines, labelled in the SDK's own x and y), the
  SDK's +x and +y axes at the J1 axis, the **reach envelope** for the SDK's soft limits (outlined) and Hitbot's
  **blind zone** (hatched red: positions the joint limits allow only with the J2 arm inside the column). An amber
  shadow arm and a ring with a drop line show a planned pose or a running move's goal. For a plan or a joint move,
  an amber curve shows the tool's path there. **Joints | Tool**, above the view, picks the grab handles (below).
  Joints has rings on J1, J2 and J4 and a double arrow on the carriage for J3, in the joint strips' colours. The
  double arrow rides a green rail on the column that spans the J1 housing's travel, with a stop at each end. Tool
  has a gizmo at the flange. The view opens framing the whole arm, base plate to column top, at whatever shape the
  panel is and wherever J3 is. Reset view frames it again. Drag anywhere else to orbit, right-drag to pan, scroll to
  zoom.
* **The bridge**: link lamp, the state window (READY, MOVING, NEEDS INIT, NO LINK), Initialize, E-STOP and STOP.
  Esc presses STOP from anywhere on the page, except during a drag, when it cancels the drag.
* **Joints**: per-joint jog keys with step and speed detents, the position (J1, J2, J4 in degrees, J3 in mm), the
  goal while moving, and a meter across each joint's soft-limit range.
* **Pose**: x, y, z, r from the SDK, the tool's yaw (J1 + J2 + r), and a check that the model's forward
  kinematics agree with the SDK's pose. Under each value is where a drag (→) or the plan (plan →) puts the tool.
* **Move to joints**, **Move to pose** (joint-interpolated, with hand selection: the current hand, or lr +1, -1 or 0
  through `movej_xyz_lr`), **Move linear**, and **Change hand**.
* **Messages**: every command's result; the SDK's refusals word for word, such as
  `SoftLimitError: target is outside the soft limits: j1_deg=95 is outside [-90, 90] deg`.

Planning a pose in the 3D view, as in umbratek-code's jog panel: **drag, then Move**.

* Point at a ring or the Z handle: it lights up, the pointer becomes a hand and a tag names the joint. Drag a ring
  around, or the Z handle up and down the column, to plan that joint (0.5° or 1 mm steps; hold Shift for 5° or
  10 mm). The shadow arm and the joint's row show the plan as you drag; the arm doesn't move.
* **Tool** mode is umbratek-code's end-effector manipulator, made for a SCARA. A gizmo at the flange (the SDK's
  pose point) stays the same size on screen. It has X (red), Y (green) and Z (blue) arrows along the SDK's axes and
  one ring that turns the tool's yaw about Z with the flange staying put. There's no roll or pitch, since a SCARA
  can't make them. Drag an arrow to slide the tool (1 mm steps, 10 with Shift) or the ring to turn it (0.5°, 5°
  with Shift).
  * The joints follow by closed-form inverse kinematics (`kinematics.js`, a port of `zarm.kinematics`). The arm
    keeps its current elbow (J2's sign) and holds the tool's yaw, so J4 = yaw − J1 − J2. Z is J3, and the ring
    turns J4 alone.
  * Out of reach (more than 420 mm from the J1 axis, or too close to it) or past a soft limit, the drag stops at
    the last pose it can reach and says why in red. The soft limits are J1's, J2's, J3's −310 to 0 mm, and J4's
    while it holds the yaw.
  * When a new plan leaves the gizmo at the screen's edge or under the plan bar, the view re-frames, unless you've
    moved the camera.
  * The page remembers the mode.
* Plan as many joints as you like, with rings and the gizmo mixed. A ring drag stages its joint and a gizmo drag
  every joint it changed; the plan bar under the view lists them. **Move** sends them all as one joint move (the
  joints you didn't plan stay where they are) at the speed detent, so the tool follows the drawn curve, not a
  straight line. **Clear**, or a joint's ×, drops them. The plan clears once the move starts.
* A drag stops at the SDK's soft limits and says so in red. A plan whose path would run the J2 arm into the column
  (the blind zone) turns red, shadow included, and Move refuses it. If the simulated arm is already in the blind
  zone, any plan that doesn't take it further in is allowed, so you can get out.
* Esc during a drag cancels it. STOP and E-STOP cancel a drag too; they leave a staged plan in place but drop one
  already sent, since the arm stops short of it. Mouse, touch and pen all work. The handles and the gizmo draw on
  top of the arm, so they can be grabbed through it.

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
real sides are slightly bowed R140 arcs, and its front slots aren't modelled) and its cap, the J1 housing on the
carriage, the two arms (a box between two round ends; the J2 arm tapers from R53 to R47.5, so its box takes the
mean width) and the flange. Two kinds of visual-only part: the four mounting counterbores, and an amber tab on the
flange's +X edge that isn't on the arm but makes J4 visible. Masses are the spec's 19 kg split by link (from
hitbot_sim), spread over each link's shapes at uniform density to give its inertia; treat them as estimates.

### Dimensions and their sources

Tags: **SPEC** Hitbot's [Z-Arm 2442E product page](https://www.hitbotrobot.com/product/robotic-arm-z-arm-2442e/).
**DRAWING** Hitbot's motion-range drawing, `2442-Size-Diagram.zip` in the download centre (one PDF, "Z-Arm 2442",
V1.1.0, updated 2020-01-18, A3 at 1:5). **STEP** Hitbot's CAD, `2442-3D-Diagram.zip` ("Z-Arm 2442 V1.0.0.stp",
CATIA V5, 2020-08-14, mm), measured in the robot frame: its arm sits at J1 = J2 = 0 with the flange face 82.5 mm up,
and it models the arm's outside only (no rails, belt or stops); `scripts/fetch_cad_meshes.py` repeats the key
measurements. **HITBOT_SIM** the MIT-licensed community URDF
`urdf/Z-Arm_10042C0.urdf` in [jjh1214/hitbot_sim](https://github.com/jjh1214/hitbot_sim), the 1 m column variant of
the same arm family. **LEGACY** this repository's old C# "Z-Arm Sliders" app and the zarm-sdk defaults taken from it.
**ASSUMED** chosen here, for the reason given.

| Field | Value | Source |
|---|---|---|
| `z_stroke_mm` | 310 mm | LEGACY: z_travel 310 and the J3 slider range -310..0, which zarm-sdk uses as ZArmConfig.z_travel_mm and its J3 soft limits. SPEC sells 110/210/310/410/510 mm strokes. The model's J3 range follows this one value, and so does the column's height once it's over 318.1 mm. |
| `cad_column_height_mm` | 593 mm | STEP: the column's top cap ends 593 mm above the mounting surface. DRAWING: the cap's top line measures 593.0, under a 3 mm raised lid. ASSUMED: a 310 mm stroke fits this stock column, which the STEP's slots allow (see cad_slot_bottom_mm), so the column isn't grown for it. |
| `cad_lid_height_mm` | 3 mm | DRAWING: a 3 mm raised lid on the cap, 596 mm overall; SPEC: height 596. Not in the STEP or the model. |
| `cad_slot_bottom_mm` | 172 mm | STEP: the column's front opening (88 mm wide, its 52 mm centre strip leaving a slot either side) is open from 172 mm, where the electronics bay's sheet-metal bracket closes it, up to the cap at 578 mm. No rails, belt or stops are modelled, so the slots bound the J1 housing's travel: 345 mm of motion, or up to a 318.1 mm stroke from the drawing's lowest position. |
| `cad_stroke_mm` | 240 mm | DRAWING: the configuration drawn: its lowest and highest positions are 240 mm apart (flange faces measured at 63.9 and 304.0 mm, J1 housing tops at 260.6 and 500.7 mm). SPEC lists 210 mm as the standard stroke. Information only, but for this: a stroke longer than the column holds stretches the part of the column the J1 housing sweeps over this travel. |
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
| `arm_gap_mm` | 2 mm | STEP: 2 mm between the J2 arm's top and the J1 arm's underside. |
| `flange_diameter_mm` | 84 mm | STEP: 84 mm boss on the J4 axis. DRAWING view A-A gives the tool interface (72, 55, 40 H7). |
| `flange_protrusion_mm` | 6.9 mm | STEP: flange face at 82.5 mm, J2 arm underside at 89.4 mm. |
| `housing_width_mm` | 100 mm | STEP: J1 housing y = +/-50 mm with an R50 nose about J1; DRAWING front view 101. |
| `housing_bottom_above_flange_mm` | 134.9 mm | STEP: the J1 housing's underside, where it meets the column, at 217.4 mm with the flange face at 82.5 mm (3 mm above the J1 arm; it dips lower only around the J1 hub). DRAWING: about 134.6. |
| `housing_top_above_flange_mm` | 195.9 mm | STEP: housing top at 278.4 mm with the flange face at 82.5 mm; DRAWING: about 196.7. |
| `flange_face_at_bottom_mm` | 64 mm | DRAWING: 64 +/- 1 mm from the mounting surface to the flange face at the lowest position (63.9 measured in both the side and front views). ASSUMED: the same with the 310 mm stroke, in the same column. |
| `j1_range_deg` | 90 deg | SPEC: 1st axis +/-90 deg (LEGACY sliders and zarm-sdk agree). |
| `j2_range_deg` | 164 deg | SPEC: 2nd axis +/-164 deg (LEGACY sliders and zarm-sdk agree). |
| `j4_range_deg` | 1080 deg | SPEC: R axis +/-1080 deg (zarm-sdk's soft limit is +/-180). |
| `total_mass_kg` | 19 kg | SPEC: weight 19 kg (the 596 mm arm; a grown column's extra metal is ignored). |
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

Derived: the column 593 mm tall to its cap (596 mm with the drawing's lid), Hitbot's column as drawn; flange face
64 mm above the mounting surface at the bottom of travel and 374 mm at the top; the J1 housing 61 mm tall, from
134.9 to 195.9 mm above the flange face, so over the 310 mm stroke it sweeps 198.9 to 569.9 mm of the column's slot
(open from 172 to 578 mm), leaving 26.9 mm of slot below it and 8.1 mm above; base and column 8.6 kg (19 kg less the
moving links). The slots hold strokes up to 318.1 mm. A longer stroke (`z_stroke_mm` 410, say) grows the column by
exactly the shortfall (91.9 mm), so the J1 housing just reaches the cap at J3 = 0.

Hitbot's drawing also gives, without being modelled: the base's R15 corners, two 8 mm H7 dowel holes 120 mm apart,
and the tool interface in view A-A (as read: a 40 mm H7 pilot, two 3 mm dowel holes 64 mm apart, 4 x M4 on
40 x 56 mm and 6 x M3 equally spaced). Check those against the arm before machining a tool.

### Assumptions

1. The arm is a **Z-Arm 2442** (220 and 200 mm arms) with a **310 mm** Z stroke, as the legacy code configures it.
2. The 310 mm stroke fits Hitbot's **stock column, 593 mm** to the cap (596 mm with its lid, the spec's height): the
   STEP's slots let the J1 housing travel 345 mm, enough for 318.1 mm from the drawing's lowest position. Hitbot's
   drawing shows that column with 240 mm between its lowest and highest positions, and the spec lists 210 mm as the
   standard stroke, so the real 310 mm arm may have a taller column. Measure it.
3. The flange face is **64 mm** above the mounting surface at the bottom of travel (J3 = -310), the drawing's 64 ± 1
   mm at its lowest position, and **374 mm** at the top (J3 = 0).
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
checks the CAD against the model (axes 220 and 200 mm apart, J1 133 mm from the base centre, square to the base; and
down the middle of the column's front slots, where they open and close, the column's top and the J1 housing's
underside and top), splits it into the URDF's links, and writes one binary STL per link to `zarm-sim/cache/meshes/`
with a manifest. git ignores `cache/`. The 310 mm stroke fits Hitbot's column as drawn, so the meshes are the CAD's
own shapes; the J1 housing's travel fills the slots but for 26.9 mm below and 8.1 mm above. For a stroke over
318.1 mm the script grows the column by the shortfall, stretching the plain middle of the column the J1 housing
sweeps over the drawing's 240 mm travel (198.9 to 499.9 mm) and moving the slot's end and the cap up unchanged; that
stretch is an approximation. Restart the server and the view draws the meshes. Without them, as on a fresh clone, or
with meshes made for an older model, the view draws the model's own shapes and says why, with this checkout's exact
command and a Copy key. The URDF, the MJCF and the tests always use those shapes: they're the collision geometry.

## Tests

```powershell
cd zarm-sim
py -3.12 -m unittest discover -s tests -t .
```

Standard-library `unittest`, like zarm-sdk's, and nothing touches hardware. They check that the generated files are
byte-identical to the generator's output (also across interpreters), that every dimension has a source and appears
above, that the URDF's forward kinematics match `zarm.kinematics.forward_xy` at 220/200 mm (and tool yaw and J3) for
random joints, the SDK-to-model joint mapping both ways, the MJCF's prefixed, composable pieces, the Z travel (the
J1 housing's sweep inside the column's slot with 26.9 mm to spare below and 8.1 mm above, in the URDF and, where
they've been made, in the CAD meshes; and for every stroke option, the column grown by exactly the shortfall), the
reach envelope and blind zone against the SDK's inverse kinematics, three.js's checked download and the mesh cache's
checks, and the
HTTP API on an ephemeral port (exact jogs, queueing, SDK refusals, Stop, E-stop and Initialize, the Host and token
checks, the page-quiet stop, and that only the simulated backend is accepted). The MuJoCo tests run when `mujoco` is
installed and skip otherwise. When `node` is on the PATH, the JavaScript is syntax-checked, the page's blind-zone
plan checks (`plan.js`) are held to `workspace.py`'s numbers, and the page's kinematics (`kinematics.js`) are held
to `zarm.kinematics`: forward and inverse kinematics, the tool pose, the gizmo's solver and where it stops, and the
sampled tool path.

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
| `zarm_sim/web/` | The page: `index.html`, `style.css`, `app.js` (controls, plan), `view.js` (3D, grab handles), `gizmo.js` (Tool mode's gizmo), `kinematics.js` (the SDK's kinematics, for the gizmo), `urdf.js`, `plan.js` (blind-zone checks) |
| `models/` | The generated URDF and MJCF |
| `scripts/fetch_cad_meshes.py` | Opt-in: meshes from Hitbot's CAD |
| `tests/` | The tests |

The page follows umbratek-code's jog-panel design (its `DESIGN.md`): a charcoal desk where colour only reports
state, amber for motion, green for ready, red for stop and refusals; tape colours name the joints.

## Limitations

* **Simulator only.** The view drives `FakeBackend`, whose timing (each joint at constant speed, the slowest
  setting the duration), limits and refusals follow zarm-sdk's model of the controller, not the controller itself.
* **Dimensions.** Primitive shapes, with the assumptions above; the CAD meshes are optional, and a column grown for
  a stroke over 318.1 mm is an approximation. Not modelled: cables, the end effector, the connector panel, the
  column's bowed sides, the cap's 3 mm lid, and what's inside the column (guide rails, belt, pulleys, end stops).
* **The blind zone** is a plan-view check of the J2 arm against the column, in the view only: Move refuses plans
  that cross it, but the jog keys, the move forms and the SDK don't check it, and a tool hanging below the flange
  isn't included.
* **Tool mode** plans joint moves, as umbratek-code's does, so the tool follows the drawn curve, not a straight line
  (Move linear makes straight lines). It keeps the elbow it starts on (Change hand swaps it), and has no roll or
  pitch.
* **Jogs** while a step is under way are sent as one move when it ends, so fast clicking moves in short bursts. The
  page-quiet stop also stops moves when a background tab's timers are throttled for over 3 s.
* **Dynamics** (masses, inertias, MuJoCo gains, limits) are estimates.
