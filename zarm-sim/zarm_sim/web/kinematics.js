// The Z-Arm's kinematics in the SDK's frame, for the 3D view's tool gizmo and the page's pose readouts: a port of
// zarm-sdk's zarm/kinematics.py (forward_xy, inverse_xy, elbow_of, tool_yaw_deg), which tests/test_web.py holds to
// the same numbers. x and y are mm from the J1 axis, z is J3 (0 at the top of travel), and the tool's yaw is
// J1 + J2 + J4 (J4 is the SDK's r, relative to the J2 arm).

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;
const MINUS = "\u2212";
const NAMES = {j1_deg: "J1", j2_deg: "J2", j3_mm: "J3", j4_deg: "J4"};

const number = (value) => String(value).replace("-", MINUS);

export function forwardXy(j1Deg, j2Deg, l1, l2) {
  const a1 = j1Deg * RAD;
  const a12 = (j1Deg + j2Deg) * RAD;
  return [l1 * Math.cos(a1) + l2 * Math.cos(a12), l1 * Math.sin(a1) + l2 * Math.sin(a12)];
}

function wrapDeg(angle) {
  let wrapped = (angle + 180) % 360;
  if (wrapped <= 0) wrapped += 360;
  return wrapped - 180;
}

// [J1, J2] in degrees for `elbow` +1 (J2 >= 0) or -1 (J2 <= 0), or null when (x, y) is out of the arms' reach.
export function inverseXy(x, y, l1, l2, elbow) {
  let cos2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  if (cos2 > 1 + 1e-9 || cos2 < -1 - 1e-9) return null;
  cos2 = Math.max(-1, Math.min(1, cos2));
  const sin2 = (elbow >= 0 ? 1 : -1) * Math.sqrt(Math.max(0, 1 - cos2 * cos2));
  const j2 = Math.atan2(sin2, cos2);
  const j1 = Math.atan2(y, x) - Math.atan2(l2 * sin2, l1 + l2 * cos2);
  return [wrapDeg(j1 * DEG), j2 * DEG];
}

export const elbowOf = (j2Deg) => (j2Deg < 0 ? -1 : 1);

// Where the joints (SDK values by field) put the tool: x, y, z in mm, r and yaw in degrees.
export function toolPose(joints, [l1, l2]) {
  const [x, y] = forwardXy(joints.j1_deg, joints.j2_deg, l1, l2);
  return {x_mm: x, y_mm: y, z_mm: joints.j3_mm, r_deg: joints.j4_deg, yaw_deg: joints.j1_deg + joints.j2_deg + joints.j4_deg};
}

function limitReason(field, [low, high], holdingYaw) {
  const unit = field === "j3_mm" ? " mm" : "\u00b0";
  return `${NAMES[field]} is at its limit (${number(low)} to ${number(high)}${unit})${holdingYaw ? " holding the tool's yaw" : ""}`;
}

// The joints that move the tool `amount` from where `base` (SDK joint values by field) puts it: mm along the SDK's
// "x", "y" or "z", or degrees of yaw about z ("yaw") with the flange staying put. The elbow keeps its side (J2's sign),
// and the tool's yaw holds while it slides, so J4 = yaw - J1 - J2. Every joint must stay within `limits`
// ({field: [low, high]}). Returns {ok: true, joints}, or {ok: false, field, reason}: the joint at its limit, or a null
// field when the pose is out of the arms' reach.
export function solveTool(base, axis, amount, lengths, limits) {
  let joints;
  if (axis === "z") {
    joints = {...base, j3_mm: base.j3_mm + amount};
  } else if (axis === "yaw") {
    joints = {...base, j4_deg: base.j4_deg + amount};
  } else if (amount === 0) {
    joints = {...base};
  } else {
    const [l1, l2] = lengths;
    const [x0, y0] = forwardXy(base.j1_deg, base.j2_deg, l1, l2);
    const x = x0 + (axis === "x" ? amount : 0);
    const y = y0 + (axis === "y" ? amount : 0);
    const solution = inverseXy(x, y, l1, l2, elbowOf(base.j2_deg));
    if (!solution) {
      const reason = Math.hypot(x, y) > l1 + l2 ? `Out of reach (${number(l1 + l2)} mm from the J1 axis at most)`
        : "Out of reach (too close to the J1 axis)";
      return {ok: false, field: null, reason};
    }
    const [j1, j2] = solution;
    joints = {j1_deg: j1, j2_deg: j2, j3_mm: base.j3_mm, j4_deg: base.j4_deg + (base.j1_deg + base.j2_deg) - (j1 + j2)};
  }
  for (const field of Object.keys(NAMES)) {
    const range = limits && limits[field];
    if (range && !(joints[field] >= range[0] && joints[field] <= range[1])) {
      return {ok: false, field, reason: limitReason(field, range, field === "j4_deg" && (axis === "x" || axis === "y"))};
    }
  }
  return {ok: true, joints};
}

// The tool's path on the straight joint-space move from `from` to `to` (SDK joint values), every joint turning in step
// as the SDK's joint moves do: [x, y, z] in mm, at least every `stepDeg` of J1 or J2 and every `stepMm` of J3. It's
// a curve, not the straight line a linear move would take.
export function toolPath(from, to, [l1, l2], {stepDeg = 1, stepMm = 2, maxSteps = 720} = {}) {
  const span = Math.max(Math.abs(to.j1_deg - from.j1_deg) / stepDeg, Math.abs(to.j2_deg - from.j2_deg) / stepDeg,
                        Math.abs(to.j3_mm - from.j3_mm) / stepMm);
  const steps = Math.min(maxSteps, Math.max(1, Math.ceil(span)));
  const points = [];
  for (let k = 0; k <= steps; k += 1) {
    const t = k / steps;
    const j1 = from.j1_deg + (to.j1_deg - from.j1_deg) * t;
    const j2 = from.j2_deg + (to.j2_deg - from.j2_deg) * t;
    const [x, y] = forwardXy(j1, j2, l1, l2);
    points.push([x, y, from.j3_mm + (to.j3_mm - from.j3_mm) * t]);
  }
  return points;
}
