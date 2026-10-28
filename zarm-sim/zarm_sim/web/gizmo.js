// The tool gizmo for the 3D view's Tool mode, after umbratek-code's (scripts/web/arm_view.js), cut to what a SCARA
// can do: arrows along the base frame's X, Y and Z (the SDK's x, y and z) and one ring, for yaw about Z; the arm
// can't roll or pitch the tool. Each axis wears its colour, X red, Y green and Z blue, as other 3D tools colour them,
// and a printed name, so colour is never the only cue. It's authored in pixels and scaled to keep that size on
// screen at any zoom, and drawn over everything with a Well Black rim, so it reads over the light arm and the dark
// floor alike. A dragged arrow lights amber, like a held key; a turned ring shows its turn as an amber wedge and arc.

const RAD = Math.PI / 180;
export const GIZMO = {
  shaft: [16, 74], shaftRadius: 1.5, head: [74, 94], headRadius: 5.5, ring: 58, tube: 1.4, rim: 1.7, label: 108,
  letterPx: 18, ringName: 88, ringNamePx: 13, far: 0.35, reach: 122,
};
const AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const NAME_SPOTS = [[1, 1], [-1, 1], [-1, -1], [1, -1]];
const ORDER = 30;  // over the arm, its shadow, the tool's path and the joint handles

function segmentGap(px, py, a, b) {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const t = Math.max(0, Math.min(1, ((px - a.x) * abx + (py - a.y) * aby) / Math.max(abx * abx + aby * aby, 1e-9)));
  return Math.hypot(a.x + abx * t - px, a.y + aby * t - py);
}

// colors: {axes: [x, y, z], well, amber, lit} as hex numbers.
export function createGizmo(THREE, colors) {
  const group = new THREE.Group();
  group.visible = false;
  const flat = (color, opacity) => new THREE.MeshBasicMaterial({
    color, transparent: true, opacity, depthTest: false, depthWrite: false, fog: false,
  });
  const yAxis = new THREE.Vector3(0, 1, 0);

  // A printed name that faces the camera: bold, in `fill`, with a 2px Well Black outline, `px` tall on screen.
  function letter(text, px, fill) {
    const scale = 4;  // canvas pixels per screen pixel, crisp on 2x screens
    const font = `700 ${px * scale}px 'Segoe UI Variable Text', 'Segoe UI', system-ui, sans-serif`;
    const canvas = document.createElement("canvas");
    let pen = canvas.getContext("2d");
    pen.font = font;
    canvas.width = Math.ceil(pen.measureText(text).width + px * scale * 0.7);
    canvas.height = Math.ceil(px * scale * 1.5);
    pen = canvas.getContext("2d");  // resizing resets the context's state
    pen.font = font;
    pen.textAlign = "center";
    pen.textBaseline = "middle";
    pen.lineJoin = "round";
    pen.lineWidth = 4 * scale;
    pen.strokeStyle = `#${colors.well.toString(16).padStart(6, "0")}`;
    pen.strokeText(text, canvas.width / 2, canvas.height / 2);
    pen.fillStyle = fill;
    pen.fillText(text, canvas.width / 2, canvas.height / 2);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: texture, transparent: true, depthTest: false, depthWrite: false, fog: false,
    }));
    sprite.scale.set(canvas.width / scale, canvas.height / scale, 1);
    sprite.renderOrder = ORDER + 3;
    return sprite;
  }

  const handles = [];
  AXES.forEach((direction, k) => {
    const axis = new THREE.Vector3(...direction);
    const name = "XYZ"[k];
    const tone = colors.axes[k];
    // An arrow: a shaft and a head along +axis, each over a slightly larger rim.
    const arrow = new THREE.Group();
    arrow.quaternion.setFromUnitVectors(yAxis, axis);
    const core = flat(tone, 1);
    const rim = flat(colors.well, 0.9);
    const band = flat(tone, 0.25);
    band.visible = false;  // the hover highlight
    const [s0, s1] = GIZMO.shaft;
    const [h0, h1] = GIZMO.head;
    const shaft = (r) => new THREE.CylinderGeometry(r, r, s1 - s0, 12).translate(0, (s0 + s1) / 2, 0);
    const head = (r, grow) => new THREE.ConeGeometry(r, h1 - h0 + 2 * grow, 24).translate(0, (h0 + h1) / 2, 0);
    for (const [geometry, material, order] of [
      [shaft(GIZMO.shaftRadius + GIZMO.rim), rim, ORDER], [head(GIZMO.headRadius + GIZMO.rim, GIZMO.rim), rim, ORDER],
      [shaft(GIZMO.shaftRadius), core, ORDER + 1], [head(GIZMO.headRadius, 0), core, ORDER + 1], [shaft(6), band, ORDER - 1],
    ]) {
      const mesh = new THREE.Mesh(geometry, material);
      mesh.renderOrder = order;
      arrow.add(mesh);
    }
    const label = letter(name, GIZMO.letterPx, `#${tone.toString(16).padStart(6, "0")}`);
    label.position.copy(axis).multiplyScalar(GIZMO.label);
    group.add(arrow, label);
    handles.push({kind: "gizmo", type: "move", axis: k, name, tone, core, rim, band, label});
  });

  // The yaw ring, about Z in the XY plane (u = X, v = Y, so positive angles turn counter-clockwise seen from above).
  // It's drawn as two halves, so the half away from the camera can be dimmed, which gives it a near and far side.
  {
    const tone = colors.axes[2];
    const u = new THREE.Vector3(1, 0, 0);
    const v = new THREE.Vector3(0, 1, 0);
    const ring = new THREE.Group();
    const core = flat(tone, 1);
    const rim = flat(colors.well, 0.9);
    const farCore = flat(tone, GIZMO.far);
    const farRim = flat(colors.well, 0.9 * GIZMO.far);
    const band = flat(tone, 0.25);
    band.visible = false;
    const half = (tube) => new THREE.TorusGeometry(GIZMO.ring, tube, 8, 80, Math.PI);
    const near = new THREE.Group();
    const far = new THREE.Group();
    for (const [side, coreMaterial, rimMaterial] of [[near, core, rim], [far, farCore, farRim]]) {
      const rimMesh = new THREE.Mesh(half(GIZMO.tube + GIZMO.rim), rimMaterial);
      const coreMesh = new THREE.Mesh(half(GIZMO.tube), coreMaterial);
      rimMesh.renderOrder = ORDER;
      coreMesh.renderOrder = ORDER + 1;
      side.add(rimMesh, coreMesh);
    }
    const bandMesh = new THREE.Mesh(new THREE.TorusGeometry(GIZMO.ring, 6, 8, 160), band);
    bandMesh.renderOrder = ORDER - 1;
    const wedge = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
      color: colors.amber, transparent: true, opacity: 0.45, depthTest: false, depthWrite: false, fog: false,
      side: THREE.DoubleSide,
    }));
    wedge.renderOrder = ORDER - 1;
    wedge.visible = false;
    const sweep = new THREE.Mesh(new THREE.BufferGeometry(), flat(colors.amber, 0.95));
    sweep.renderOrder = ORDER + 2;
    sweep.visible = false;
    ring.add(near, far, bandMesh, wedge, sweep);
    const label = letter("Yaw", GIZMO.ringNamePx, `#${tone.toString(16).padStart(6, "0")}`);
    label.position.copy(u).add(v).normalize().multiplyScalar(GIZMO.ringName);
    group.add(ring, label);
    handles.push({kind: "gizmo", type: "turn", axis: 2, name: "Yaw", tone, core, rim, band, farCore, farRim, near, far,
                  wedge, sweep, u, v, label, spot: 0});
  }
  const yaw = handles[3];

  const toScreen = (local, camera, rect) => {
    const p = local.clone().applyMatrix4(group.matrixWorld).project(camera);
    return p.z > 1 ? null : new THREE.Vector2(((p.x + 1) / 2) * rect.width, ((1 - p.y) / 2) * rect.height);
  };
  const ringLocal = (angle) => yaw.u.clone().multiplyScalar(Math.cos(angle) * GIZMO.ring)
    .addScaledVector(yaw.v, Math.sin(angle) * GIZMO.ring);

  // Turns the ring's dimmed half away from the camera (seen from straight above it has no far half, so it's drawn
  // whole), and sets its name at whichever of its four diagonals reads best from here: on the near side, clear of
  // the arrows and their letters. The name holds its place until another spot is clearly better, so it doesn't
  // flicker as the view turns.
  function orientRing(camera, rect) {
    const toCamera = camera.position.clone().sub(group.position).normalize();
    const centre = toScreen(new THREE.Vector3(), camera, rect);
    const taken = [];
    for (const direction of AXES) {
      for (const out of [45, 84, GIZMO.label]) {
        const p = toScreen(new THREE.Vector3(...direction).multiplyScalar(out), camera, rect);
        if (p) taken.push(p);
      }
    }
    const a = toCamera.dot(yaw.u);
    const b = toCamera.dot(yaw.v);
    const away = Math.atan2(-b, -a);
    yaw.far.rotation.z = away - Math.PI / 2;
    yaw.near.rotation.z = away + Math.PI / 2;
    const fade = 1 - (1 - GIZMO.far) * Math.min(1, Math.hypot(a, b) / 0.35);
    yaw.farCore.color.copy(yaw.core.color);
    yaw.farCore.opacity = yaw.core.opacity * fade;
    yaw.farRim.opacity = yaw.rim.opacity * fade;
    let best = null;
    NAME_SPOTS.forEach(([s, t], spot) => {
      const direction = yaw.u.clone().multiplyScalar(s).addScaledVector(yaw.v, t).normalize();
      const at = toScreen(direction.clone().multiplyScalar(GIZMO.ringName), camera, rect);
      if (!at) return;
      let score = Math.min(48, ...taken.map((q) => q.distanceTo(at)));  // px of room, enough at 48
      if (direction.dot(toCamera) > 0.05) score += 12;  // against the ring's own bright near arc
      if (centre && at.distanceTo(centre) < GIZMO.ring) score -= 24;  // seen edge-on: inside the ring, over the arrows
      if (spot === yaw.spot) score += 8;
      if (!best || score > best.score) best = {spot, score, direction};
    });
    if (best) {
      yaw.spot = best.spot;
      yaw.label.position.copy(best.direction).multiplyScalar(GIZMO.ringName);
    }
  }

  return {
    group,
    handles,
    toScreen,
    ringLocal,
    // Keeps the gizmo the same size on screen at `position`: one unit of its geometry is one pixel at its depth.
    place(position, camera, rect) {
      group.position.copy(position);
      const depth = position.clone().sub(camera.position).dot(camera.getWorldDirection(new THREE.Vector3()));
      group.scale.setScalar((2 * Math.max(depth, camera.near) * Math.tan((camera.fov * RAD) / 2)) / Math.max(1, rect.height));
      group.updateMatrixWorld(true);
      orientRing(camera, rect);
      group.updateMatrixWorld(true);
    },
    // How many metres one pixel spans at the gizmo's depth.
    metresPerPixel: () => group.scale.x,
    // The handle whose drawn shape passes nearest (px, py) on screen within `reach` px, and where on the ring.
    pick(px, py, camera, rect, reach) {
      let best = null;
      for (const handle of handles) {
        if (handle.type === "move") {
          const axis = new THREE.Vector3(...AXES[handle.axis]);
          const a = toScreen(axis.clone().multiplyScalar(GIZMO.shaft[0]), camera, rect);
          const b = toScreen(axis.clone().multiplyScalar(GIZMO.head[1]), camera, rect);
          if (!a || !b) continue;
          const gap = segmentGap(px, py, a, b);
          if (gap <= reach && (!best || gap < best.gap)) best = {handle, gap};
        } else {
          for (let k = 0; k < 120; k += 1) {
            const angle = (k / 120) * Math.PI * 2;
            const p = toScreen(ringLocal(angle), camera, rect);
            if (!p) continue;
            const gap = Math.hypot(p.x - px, p.y - py);
            if (gap <= reach && (!best || gap < best.gap)) best = {handle, gap, angle};
          }
        }
      }
      return best;
    },
    // Each handle in its axis colour, the hovered one brightened toward white, all dimmed while the arm can't take a
    // plan, and the others dimmed during a drag. A dragged arrow lights amber; a dragged ring lights white, so its
    // amber wedge and arc stand out on it.
    style(hover, dragged, interactive) {
      for (const handle of handles) {
        const active = dragged ? dragged === handle : hover === handle;
        const dim = dragged ? !active : !interactive;
        if (dragged && active) handle.core.color.setHex(handle.type === "move" ? colors.amber : colors.lit);
        else handle.core.color.setHex(handle.tone).lerp(new THREE.Color(colors.lit), active ? 0.35 : 0);
        handle.core.opacity = dim ? 0.3 : 1;
        handle.rim.opacity = dim ? 0.3 : 0.9;
        handle.band.visible = active && (interactive || Boolean(dragged));
        handle.label.material.opacity = dim ? 0.35 : 1;
      }
    },
    // The ring's turn so far, from `startAngle`, as an amber (or red, when blocked) wedge edged by an arc; or none.
    showTurn(startAngle, turn, tone) {
      const {sweep, wedge} = yaw;
      sweep.visible = wedge.visible = Math.abs(turn) > 0.001;
      if (!sweep.visible) return;
      const start = turn >= 0 ? startAngle : startAngle + turn;
      sweep.geometry.dispose();
      sweep.geometry = new THREE.TorusGeometry(GIZMO.ring, GIZMO.tube * 2.2, 6, 120, Math.abs(turn));
      sweep.rotation.z = start;
      wedge.geometry.dispose();
      wedge.geometry = new THREE.CircleGeometry(GIZMO.ring, 120, start, Math.abs(turn));
      sweep.material.color.setHex(tone);
      wedge.material.color.setHex(tone);
    },
    hideTurn() {
      yaw.sweep.visible = yaw.wedge.visible = false;
    },
    // Screen points (canvas pixels) of a handle, for automated checks: partway along an arrow and its tip, or the
    // ring's point furthest from the centre on screen at one of its diagonals; and the centre.
    points(type, axis, camera, rect) {
      const centre = toScreen(new THREE.Vector3(), camera, rect);
      if (!centre) return null;
      if (type === "move") {
        const direction = new THREE.Vector3(...AXES[axis]);
        const point = toScreen(direction.clone().multiplyScalar((GIZMO.shaft[0] + GIZMO.shaft[1]) / 2), camera, rect);
        const tip = toScreen(direction.clone().multiplyScalar(GIZMO.head[1]), camera, rect);
        return point && tip ? {point, tip, centre} : null;
      }
      let point = null;
      for (const angle of [0.25, 0.75, 1.25, 1.75].map((x) => x * Math.PI)) {
        const p = toScreen(ringLocal(angle), camera, rect);
        if (p && (!point || p.distanceTo(centre) > point.distanceTo(centre))) point = p;
      }
      return point ? {point, tip: null, centre} : null;
    },
  };
}
