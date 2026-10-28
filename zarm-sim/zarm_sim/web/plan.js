// Plan checks for the page and the 3D view: Hitbot's blind zone, where the J2 arm would hit the column, worked out
// in plan view exactly as zarm_sim/workspace.py does it (tests/test_web.py holds the two to the same numbers).
// `zone` comes from the server's page config: the column's rectangle and the J2 arm's shapes, in the SDK's frame (mm).

const RAD = Math.PI / 180;

function pointRectDistance(px, py, [x0, x1, y0, y1]) {
  const dx = Math.max(x0 - px, 0, px - x1);
  const dy = Math.max(y0 - py, 0, py - y1);
  return Math.hypot(dx, dy);
}

function pointSegmentDistance([px, py], [ax, ay], [bx, by]) {
  const dx = bx - ax;
  const dy = by - ay;
  const lengthSq = dx * dx + dy * dy;
  const t = lengthSq === 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lengthSq));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

function projections(points, [ax, ay]) {
  const values = points.map(([x, y]) => x * ax + y * ay);
  return [Math.min(...values), Math.max(...values)];
}

// Distance between a convex quadrilateral and an axis-aligned rectangle; 0 when they overlap.
function quadRectDistance(quad, rect) {
  const [x0, x1, y0, y1] = rect;
  const corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
  const axes = [[1, 0], [0, 1]];
  for (let i = 0; i < 2; i += 1) {
    const ex = quad[i + 1][0] - quad[i][0];
    const ey = quad[i + 1][1] - quad[i][1];
    const norm = Math.hypot(ex, ey);
    axes.push([-ey / norm, ex / norm]);
  }
  const separated = axes.some((axis) => {
    const [a0, a1] = projections(quad, axis);
    const [b0, b1] = projections(corners, axis);
    return a1 < b0 || b1 < a0;
  });
  if (!separated) return 0;
  let best = Math.min(...quad.map(([px, py]) => pointRectDistance(px, py, rect)));
  for (const corner of corners) {
    for (let i = 0; i < 4; i += 1) best = Math.min(best, pointSegmentDistance(corner, quad[i], quad[(i + 1) % 4]));
  }
  return best;
}

// Plan-view gap between the J2 arm and the column (mm); 0 or less means they overlap.
export function j2ClearanceMm(zone, j1Deg, j2Deg) {
  const a1 = j1Deg * RAD;
  const a12 = (j1Deg + j2Deg) * RAD;
  const ex = zone.l1Mm * Math.cos(a1);
  const ey = zone.l1Mm * Math.sin(a1);
  const ux = Math.cos(a12);
  const uy = Math.sin(a12);
  let best = Infinity;
  for (const [along, radius] of zone.j2CirclesMm) {
    best = Math.min(best, pointRectDistance(ex + along * ux, ey + along * uy, zone.columnRectMm) - radius);
  }
  const [start, end, half] = zone.j2BoxMm;
  const nx = -uy;
  const ny = ux;
  const quad = [
    [ex + start * ux + half * nx, ey + start * uy + half * ny],
    [ex + end * ux + half * nx, ey + end * uy + half * ny],
    [ex + end * ux - half * nx, ey + end * uy - half * ny],
    [ex + start * ux - half * nx, ey + start * uy - half * ny],
  ];
  return Math.min(best, quadRectDistance(quad, zone.columnRectMm));
}

// Whether the straight joint-space move from `from` to `to` (SDK joint values: the simulator, like the SDK's joint
// moves, turns every joint in step) keeps the J2 arm out of the column, checked at every degree of the larger turn.
// Only the simulator can start inside the blind zone (the SDK doesn't check it). From there every way out crosses the
// column, often deeper first, so only the end counts: it mustn't be further in. Returns {clear, reason}.
export function pathCheck(zone, from, to, stepDeg = 1) {
  if (!zone) return {clear: true, reason: null};
  const start = j2ClearanceMm(zone, from.j1_deg, from.j2_deg);
  if (start <= 0) {
    const end = j2ClearanceMm(zone, to.j1_deg, to.j2_deg);
    return end < start - 0.5
      ? {clear: false, reason: "the J2 arm would end up further into the column (Hitbot's blind zone)"}
      : {clear: true, reason: null};
  }
  const span = Math.max(Math.abs(to.j1_deg - from.j1_deg), Math.abs(to.j2_deg - from.j2_deg));
  const steps = Math.max(1, Math.ceil(span / stepDeg));
  for (let k = 1; k <= steps; k += 1) {
    const t = k / steps;
    const gap = j2ClearanceMm(zone, from.j1_deg + (to.j1_deg - from.j1_deg) * t, from.j2_deg + (to.j2_deg - from.j2_deg) * t);
    if (gap <= 0) {
      return {clear: false, reason: k === steps ? "the J2 arm would end up in the column (Hitbot's blind zone)"
        : "on the way, the J2 arm would hit the column (Hitbot's blind zone)"};
    }
  }
  return {clear: true, reason: null};
}
