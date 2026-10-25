// The 3D monitor: the arm built from the URDF the server serves, posed from the SDK's joint values and drawn with
// Hitbot's CAD meshes when they've been made on this PC (with the URDF's primitive shapes when they haven't); the
// floor's millimetre grid and the reach envelope in the SDK's own x and y; and an amber shadow arm, with the tool's
// path, for a planned pose or the move under way. In Joints mode the J1, J2 and J4 rings and the Z handle (J3) are
// grab handles: dragging one plans that joint. In Tool mode a gizmo at the flange slides the tool along x, y or z or
// turns its yaw, and the joints follow by inverse kinematics (kinematics.js). Either way the page stages the plan and
// nothing moves until Move. The dragging is umbratek-code's (scripts/web/arm_view.js): handles picked by their drawn
// shape on screen, a capture-phase grab that keeps the camera still, rings tracked around their plane (or along their
// on-screen tangent when seen edge-on), and arrows and the Z handle along their line (or up and down when end-on).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { createGizmo, GIZMO } from "./gizmo.js";
import { solveTool, toolPath, toolPose } from "./kinematics.js";
import { pathCheck } from "./plan.js";
import { buildRobot, parseUrdf } from "./urdf.js";

const RAD = Math.PI / 180;
const COLOR = {
  well: 0x0e0f10, grid: 0x23262a, gridMain: 0x3d4248, amber: 0xf2aa3c, alarm: 0xff8073, reach: 0x8f8b83,
  axisX: 0xff3352, axisY: 0x8bdc00, axisZ: 0x2890ff, lit: 0xffffff,
};
const TAPES = {j1_shoulder: 0xe3ba4f, j2_elbow: 0x7fb3d9, j3_vertical: 0x92cc8a, j4_wrist: 0xe19769};
const VIEW = {target: [0.22, 0.0, 0.17], position: [1.3, -1.28, 1.24]};  // the home view's direction, before it's fitted
// The home view frames the arm this many pixels inside the canvas's edges (the top clears the SIMULATED mark), and
// above the help line or plan bar along the bottom.
const INSET = {side: 14, top: 44, bottom: 14};
const GRID_MM = {x0: -300, x1: 500, y0: -500, y1: 500};  // in the SDK frame
const MM = 0.001;
const LABEL_PX = 20;  // the floor labels' height on screen (their text is about 12 px), whatever the zoom
// The grab handles, in metres. A ring stands RING_CLEARANCE off the hub it circles; the Z handle rides on the
// column's face beside the J1 housing.
const RING_TUBE = 0.0045;
const RING_CLEARANCE = 0.009;
const RIM = 0.0013;  // the Well Black outline that lets a handle read over the light arm and the dark floor alike
const SLIDE = {half: 0.045, shaft: 0.0026, head: 0.0085, headLength: 0.02, standOff: 0.004};
const PICK_PX = {mouse: 14, touch: 24};  // how close the pointer must come to a handle's drawn shape to grab it
const RING_FACING_MIN = 0.5;  // below this, a ring is too side-on to track the pointer around its plane
const RING_PX_PER_DEG = 2;  // side-on rings turn this many pixels of drag into one degree
const SLIDE_MIN_PX = 24;  // a Z handle shorter than this on screen is seen end-on: it slides with up and down drags
const SNAP = {deg: [0.5, 5], mm: [1, 10]};  // a plan's steps, and with Shift held
const MIN_CHANGE = {deg: 0.25, mm: 0.5};  // a smaller drag is a click, not a plan
const TOOL_SNAP = {move: [1, 10], turn: [0.5, 5]};  // the gizmo's steps: mm along an arrow or degrees of yaw; with Shift
const GIZMO_PICK_PX = {mouse: 12, touch: 20};
const BASE_AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
// Each handle is drawn twice: faintly over everything, so it stays in sight (and in reach) inside the CAD meshes,
// then solid wherever nothing hides it. All of it is transparent, so renderOrder alone sets the order.
const PASSES = [
  {key: "behindRim", rim: true, depthTest: false, opacity: 0.5, order: 20},
  {key: "behindCore", rim: false, depthTest: false, opacity: 0.55, order: 21},
  {key: "frontRim", rim: true, depthTest: true, opacity: 0.9, order: 22},
  {key: "frontCore", rim: false, depthTest: true, opacity: 1, order: 23},
];
const DIM = 0.35;  // handles that can't be dragged now are dimmed to this

async function fetchOk(url, kind) {
  const response = await fetch(url, {cache: "no-store"});
  if (!response.ok) throw new Error(`couldn't load ${url} (HTTP ${response.status})`);
  return kind === "json" ? response.json() : response.text();
}

function lines(points, material) {
  return new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points), material);
}

const wrap = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle));

function segmentGap(px, py, a, b) {
  const ab = b.clone().sub(a);
  const t = Math.max(0, Math.min(1, ((px - a.x) * ab.x + (py - a.y) * ab.y) / Math.max(ab.lengthSq(), 1e-9)));
  return Math.hypot(a.x + ab.x * t - px, a.y + ab.y * t - py);
}

// onPreview(drag readout | null) follows a drag; onRelease(...) ends it: {field, from, to, cancelled, changed} for a
// joint handle, {kind: "tool", coord, amount, pose, limited, cancelled, changed} for the gizmo. onHover(handle under
// the pointer | null) names a handle before it's grabbed. Positions are in page pixels. `mode` is what dragging in
// the view moves: "joints" (the rings and the Z handle) or "tool" (the gizmo).
export async function createView(host, config, {onPreview = () => {}, onRelease = () => {}, onHover = () => {}, onLost,
                                                mode: startMode = "joints"} = {}) {
  const [urdfText, envelope] = await Promise.all([fetchOk(config.urdf, "text"), fetchOk(config.envelope, "json")]);
  const urdf = parseUrdf(urdfText);
  const [sx, sy, sz] = config.sdkOriginM;
  const fields = config.jointMap;  // the SDK's joints in its order (J1, J2, J3, J4), each with its URDF joint
  const limits = config.softLimits;  // the SDK's soft limits, by field
  const zone = config.blindZone || null;
  const lengths = config.armLengthsMm;
  let mode = startMode === "tool" ? "tool" : "joints";

  const renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: "high-performance"});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  host.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(COLOR.well);
  scene.fog = new THREE.Fog(COLOR.well, 4.5, 10);
  const camera = new THREE.PerspectiveCamera(32, 1, 0.01, 30);
  camera.up.set(0, 0, 1);  // the URDF is Z-up; keep it that way
  const controls = new OrbitControls(camera, renderer.domElement);
  // Until the arm is built, look along the home direction; then fitView (below) frames it.
  camera.position.set(...VIEW.position);
  controls.target.set(...VIEW.target);
  controls.update();

  scene.add(new THREE.HemisphereLight(0xf3efe6, 0x1b1d20, 1.5));
  const key = new THREE.DirectionalLight(0xffffff, 2.3);
  key.position.set(1.2, -0.9, 2.5);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, {left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.5, far: 5});
  key.shadow.camera.updateProjectionMatrix();
  key.shadow.bias = -0.0004;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xffffff, 0.7);
  rim.position.set(-1.5, 1.7, 1.1);
  scene.add(rim);

  // ---- The floor: soft contact shadows, then the millimetre grid in the SDK's x and y (10 and 100 mm) ----
  const labels = [];
  function label(text, colour = "#aaa69d") {
    const canvas = document.createElement("canvas");
    canvas.width = 256;
    canvas.height = 64;
    const pen = canvas.getContext("2d");
    pen.font = "600 38px 'Segoe UI Variable Text', 'Segoe UI', system-ui, sans-serif";
    pen.fillStyle = colour;
    pen.textAlign = "center";
    pen.textBaseline = "middle";
    pen.fillText(text, 128, 34);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    // Sized in screen space (see sizeLabels), so the labels stay legible at any zoom and monitor size.
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map: texture, transparent: true, depthWrite: false, sizeAttenuation: false}));
    labels.push(sprite);
    return sprite;
  }
  function sizeLabels(widthPx, heightPx) {
    // Without size attenuation a sprite's height on screen is scale.y * projection[1][1] / 2 of the viewport's height.
    // A narrow monitor gets smaller labels, every 200 mm only.
    const narrow = widthPx < 520;
    const scale = (2 * (narrow ? 16 : LABEL_PX)) / (camera.projectionMatrix.elements[5] * heightPx);
    for (const sprite of labels) {
      sprite.scale.set(scale * 4, scale, 1);
      sprite.visible = !narrow || sprite.userData.every200 !== false;
    }
  }

  const shadowFloor = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.ShadowMaterial({opacity: 0.4, depthWrite: false}));
  shadowFloor.receiveShadow = true;
  scene.add(shadowFloor);
  const floor = new THREE.Group();
  floor.position.set(sx, sy, 0);
  scene.add(floor);
  const minor = [];
  const major = [];
  for (let x = GRID_MM.x0; x <= GRID_MM.x1; x += 10) {
    (x % 100 === 0 ? major : minor).push(new THREE.Vector3(x * MM, GRID_MM.y0 * MM, 0), new THREE.Vector3(x * MM, GRID_MM.y1 * MM, 0));
  }
  for (let y = GRID_MM.y0; y <= GRID_MM.y1; y += 10) {
    (y % 100 === 0 ? major : minor).push(new THREE.Vector3(GRID_MM.x0 * MM, y * MM, 0), new THREE.Vector3(GRID_MM.x1 * MM, y * MM, 0));
  }
  const minorLines = lines(minor, new THREE.LineBasicMaterial({color: COLOR.grid}));
  const majorLines = lines(major, new THREE.LineBasicMaterial({color: COLOR.gridMain}));
  minorLines.position.z = 0.0002;
  majorLines.position.z = 0.0004;
  floor.add(minorLines, majorLines);
  for (let x = GRID_MM.x0; x <= GRID_MM.x1; x += 100) {
    const sprite = label(x === 0 ? "x 0" : String(x).replace("-", "\u2212"));
    sprite.userData.every200 = x % 200 === 0;
    sprite.position.set(x * MM, (GRID_MM.y0 - 22) * MM, 0.004);
    floor.add(sprite);
  }
  for (let y = GRID_MM.y0; y <= GRID_MM.y1; y += 100) {
    const sprite = label(y === 0 ? "y 0" : String(y).replace("-", "\u2212"));
    sprite.userData.every200 = y % 200 === 0;
    sprite.position.set((GRID_MM.x1 + 34) * MM, y * MM, 0.004);
    floor.add(sprite);
  }
  for (const [dir, colour, name] of [[[1, 0, 0], COLOR.axisX, "+x"], [[0, 1, 0], COLOR.axisY, "+y"]]) {
    const arrow = new THREE.ArrowHelper(new THREE.Vector3(...dir), new THREE.Vector3(0, 0, 0.002), 0.12, colour, 0.022, 0.012);
    floor.add(arrow);
    const tag = label(name, colour === COLOR.axisX ? "#ff6b80" : "#a9e64a");
    tag.position.set(dir[0] * 0.15, dir[1] * 0.15, 0.006);
    floor.add(tag);
  }

  // The reach envelope (SDK soft limits) and the blind zone, as a picture on the floor plus a crisp outline.
  const [ex0, ex1, ey0, ey1] = envelope.boundsMm;
  const envelopeTexture = await new THREE.TextureLoader().loadAsync(envelope.png);
  envelopeTexture.colorSpace = THREE.SRGBColorSpace;
  envelopeTexture.magFilter = THREE.NearestFilter;
  const envelopePlane = new THREE.Mesh(
    new THREE.PlaneGeometry((ex1 - ex0) * MM, (ey1 - ey0) * MM),
    new THREE.MeshBasicMaterial({map: envelopeTexture, transparent: true, depthWrite: false}),
  );
  envelopePlane.position.set((ex0 + ex1) / 2 * MM, (ey0 + ey1) / 2 * MM, 0.0007);
  floor.add(envelopePlane);
  const outlineMaterial = new THREE.LineBasicMaterial({color: COLOR.reach, transparent: true, opacity: 0.9});
  for (const line of envelope.outlineMm) {
    const outline = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(line.map(([x, y]) => new THREE.Vector3(x * MM, y * MM, 0.0011))),
      outlineMaterial,
    );
    floor.add(outline);
  }

  // ---- The arm, from the URDF. Each link gets its own materials, so one link can be tinted alone ----
  const materials = new Map();
  const solid = (visual, link) => {
    const [r, g, b, a] = visual.rgba;
    const cacheKey = `${link}:${visual.rgba.join(",")}`;
    if (!materials.has(cacheKey)) {
      materials.set(cacheKey, new THREE.MeshStandardMaterial({
        color: new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace), roughness: 0.55, metalness: 0.08,
        transparent: a < 1, opacity: a,
      }));
    }
    return materials.get(cacheKey);
  };
  const robot = buildRobot(THREE, urdf, solid);
  for (const list of robot.meshes.values()) {
    for (const mesh of list) {
      mesh.castShadow = true;
      mesh.receiveShadow = true;
    }
  }
  scene.add(robot.root);
  const part = (link, name) => (robot.meshes.get(link) || []).find((mesh) => mesh.name === name);

  // The J3 rail: the travel zone, where the J1 housing travels (its underside at the bottom of travel to its top at
  // J3 = 0), on the column's face just outside the housing's side that faces the default view, with a stop across
  // each end. The Z handle rides on it.
  const j3 = urdf.joints.find((joint) => joint.name === "j3_vertical");
  const column = part("base_link", "column");
  const housing = part("carriage", "housing");
  const railX = column ? column.position.x + column.geometry.parameters.width / 2 + SLIDE.standOff : 0;
  const railY = housing ? housing.position.y - housing.geometry.parameters.height / 2 - SLIDE.standOff : 0;
  if (j3 && column && housing) {
    const low = j3.xyz[2] + j3.lower + housing.position.z - housing.geometry.parameters.depth / 2;
    const high = j3.xyz[2] + j3.upper + housing.position.z + housing.geometry.parameters.depth / 2;
    const tape = new THREE.MeshBasicMaterial({color: TAPES.j3_vertical});
    const rail = new THREE.Mesh(new THREE.BoxGeometry(0.002, 0.004, high - low), tape);
    rail.position.set(railX, railY, (low + high) / 2);
    robot.groups.get("base_link").add(rail);
    for (const [z, inward] of [[low, 1], [high, -1]]) {
      const stop = new THREE.Mesh(new THREE.BoxGeometry(0.004, 0.016, 0.0025), tape);
      stop.position.set(railX, railY, z + inward * 0.00125);
      robot.groups.get("base_link").add(stop);
    }
  }

  // ---- The shadow arm: a planned pose, or the goal of the move under way ----
  const ghostMaterial = (color) => new THREE.MeshStandardMaterial({
    color, emissive: color, emissiveIntensity: 0.6, roughness: 0.8, metalness: 0, transparent: true, opacity: 0.42,
    depthWrite: false,
  });
  const ghostOk = ghostMaterial(COLOR.amber);
  const ghostBlocked = ghostMaterial(COLOR.alarm);
  // A depth-only pass first, so the translucent shadow shows only its front skin, not every face inside it.
  const ghostDepth = new THREE.MeshBasicMaterial({colorWrite: false, transparent: true});
  // Where a blocked plan's shadow passes behind the arm or into the column, a faint red x-ray of it shows through:
  // that's where the collision is. It draws only where something nearer hides the shadow.
  const ghostXray = new THREE.MeshBasicMaterial({
    color: COLOR.alarm, transparent: true, opacity: 0.22, depthWrite: false, depthFunc: THREE.GreaterDepth, fog: false,
  });
  const shadow = buildRobot(THREE, urdf, () => ghostOk);
  scene.add(shadow.root);
  const ghostMeshes = [];
  const addGhost = (mesh, link) => {
    mesh.renderOrder = 2;
    const depth = new THREE.Mesh(mesh.geometry, ghostDepth);
    depth.renderOrder = 1;
    const xray = new THREE.Mesh(mesh.geometry, ghostXray);
    xray.renderOrder = 3;
    xray.visible = false;
    mesh.add(depth, xray);
    mesh.visible = false;
    ghostMeshes.push({mesh, link, xray});
  };
  for (const [link, list] of shadow.meshes) for (const mesh of list) addGhost(mesh, link);
  // The movable joints between the base and each link: a link is shadowed when one of them is planned elsewhere.
  const parentJoint = new Map(urdf.joints.map((joint) => [joint.child, joint]));
  const ancestry = new Map([...shadow.groups.keys()].map((link) => {
    const above = [];
    for (let joint = parentJoint.get(link); joint; joint = parentJoint.get(joint.parent)) {
      if (joint.type !== "fixed") above.push(joint.name);
    }
    return [link, above];
  }));

  // ---- Hitbot's CAD meshes (made locally by scripts/fetch_cad_meshes.py), for the arm and its shadow alike ----
  let cad = false;
  let cadError = null;
  if (config.meshes) {
    try {
      const {STLLoader} = await import("three/addons/loaders/STLLoader.js");
      const loader = new STLLoader();
      const loaded = await Promise.all(Object.entries(config.meshes).map(async ([link, url]) => [link, await loader.loadAsync(url)]));
      const tones = {base_link: 0xc9cacc, carriage: 0xe9e8e4, j1_arm: 0xe9e8e4, j2_arm: 0xe9e8e4, flange: 0xa9adb2};
      for (const [link, geometry] of loaded) {
        if (!robot.groups.has(link)) continue;
        const material = new THREE.MeshStandardMaterial({color: tones[link] ?? 0xdddddd, roughness: 0.5, metalness: 0.1});
        materials.set(`${link}:cad`, material);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.userData.kind = "cad";
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        robot.groups.get(link).add(mesh);
        const ghost = new THREE.Mesh(geometry, ghostOk);
        ghost.userData.kind = "cad";
        shadow.groups.get(link).add(ghost);
        addGhost(ghost, link);
      }
      cad = true;
    } catch (error) {
      cadError = error && error.message ? error.message : String(error);
      console.warn("The CAD meshes didn't load, so the view draws the primitive shapes.", error);
    }
  }
  // With the CAD meshes the primitives stay hidden, but for the amber tab that shows J4's angle (it isn't on the arm).
  const drawn = (mesh) => (cad ? mesh.userData.kind === "cad" || mesh.name === "flange_index" : mesh.userData.kind !== "cad");
  for (const list of robot.meshes.values()) for (const mesh of list) mesh.visible = drawn(mesh);

  // ---- Grab handles: a ring per revolute joint, around the hub it turns, and the Z handle for J3 ----
  // They ride on the shadow arm, in each joint's own frame (which doesn't turn with the joint), so a drag sculpts the
  // plan and the handles further along follow it.
  const handles = [];
  const byJoint = new Map(fields.map((f) => [f.joint, f]));
  const handleMaterials = (tone) => Object.fromEntries(PASSES.map((pass) => [pass.key, new THREE.MeshBasicMaterial({
    color: pass.rim ? COLOR.well : tone, transparent: true, opacity: pass.opacity, depthTest: pass.depthTest,
    depthWrite: false, fog: false,
  })]));
  function addPasses(group, core, rims, mats) {
    for (const pass of PASSES) {
      for (const geometry of pass.rim ? rims : core) {
        const mesh = new THREE.Mesh(geometry, mats[pass.key]);
        mesh.renderOrder = pass.order;
        group.add(mesh);
      }
    }
  }
  const overlay = (geometry, color, opacity, order) => {
    const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
      color, transparent: true, opacity, depthTest: false, depthWrite: false, fog: false,
    }));
    mesh.renderOrder = order;
    mesh.visible = false;
    return mesh;
  };
  function addRing(jointName, z, radius) {
    const spec = byJoint.get(jointName);
    if (!spec || !shadow.frames.has(jointName)) return;
    const tone = TAPES[jointName] ?? 0xdad5c9;
    const group = new THREE.Group();
    group.position.set(0, 0, z);
    const mats = handleMaterials(tone);
    addPasses(group, [new THREE.TorusGeometry(radius, RING_TUBE, 10, 128)],
      [new THREE.TorusGeometry(radius, RING_TUBE + RIM, 10, 128)], mats);
    const band = overlay(new THREE.TorusGeometry(radius, 0.011, 8, 64), tone, 0.22, 19);  // hover highlight
    const sweep = overlay(new THREE.BufferGeometry(), COLOR.amber, 0.95, 24);  // how far a drag turns the joint
    group.add(band, sweep);
    shadow.frames.get(jointName).add(group);
    handles.push({kind: "ring", joint: jointName, field: spec.field, unit: "deg", group, radius,
                  tone: new THREE.Color(tone), mats, band, sweep});
  }
  function addSlide() {
    const spec = byJoint.get("j3_vertical");
    const carriage = shadow.groups.get("carriage");
    if (!spec || !carriage || !column || !housing) return;
    const tone = TAPES.j3_vertical;
    const group = new THREE.Group();
    group.position.set(railX, railY, housing.position.z);
    // A double-headed arrow along the column's axis, centred on the housing's height.
    const arrow = (grow) => {
      const shaftLength = 2 * (SLIDE.half - SLIDE.headLength);
      const head = (up) => new THREE.ConeGeometry(SLIDE.head + grow, SLIDE.headLength + 2 * grow, 24)
        .rotateX(up ? Math.PI / 2 : -Math.PI / 2)
        .translate(0, 0, (up ? 1 : -1) * (SLIDE.half - SLIDE.headLength / 2));
      return [new THREE.CylinderGeometry(SLIDE.shaft + grow, SLIDE.shaft + grow, shaftLength, 16).rotateX(Math.PI / 2),
              head(true), head(false)];
    };
    const mats = handleMaterials(tone);
    addPasses(group, arrow(0), arrow(RIM), mats);
    const band = overlay(new THREE.CylinderGeometry(0.014, 0.014, 2 * SLIDE.half + 0.012, 24).rotateX(Math.PI / 2), tone, 0.2, 19);
    group.add(band);
    carriage.add(group);
    handles.push({kind: "slide", joint: "j3_vertical", field: spec.field, unit: "mm", group, tone: new THREE.Color(tone),
                  mats, band});
  }
  const hub = (link, name) => {
    const mesh = part(link, name);
    return mesh ? {z: mesh.position.z, radius: mesh.geometry.parameters.radiusTop} : null;
  };
  const shoulder = hub("j1_arm", "j1_arm_shoulder_end");
  const elbow = hub("j2_arm", "j2_arm_elbow_end");
  const toolEnd = hub("j2_arm", "j2_arm_tool_end");
  const flangeDisc = hub("flange", "flange_disc");
  if (shoulder) addRing("j1_shoulder", shoulder.z, shoulder.radius + RING_CLEARANCE);
  if (elbow) addRing("j2_elbow", elbow.z, elbow.radius + RING_CLEARANCE);
  if (flangeDisc && toolEnd) addRing("j4_wrist", flangeDisc.z, Math.max(flangeDisc.radius, toolEnd.radius) + RING_CLEARANCE);
  addSlide();
  for (const handle of handles) handle.group.visible = false;  // until the first status

  // ---- Tool mode: the gizmo at the shadow's flange (the SDK's pose point), drawn over everything ----
  const gizmo = createGizmo(THREE, {axes: [COLOR.axisX, COLOR.axisY, COLOR.axisZ], well: COLOR.well, amber: COLOR.amber,
                                    lit: COLOR.lit});
  scene.add(gizmo.group);
  const gizmoAt = new THREE.Vector3();  // where the gizmo sits: the shadow's tool0, in world coordinates

  // The tool's path on the joint move to the plan (or the move under way): every joint turns in step, so the tool
  // follows a curve, not a straight line. Drawn over the arm, amber, or red when the plan is blocked.
  const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({
    color: COLOR.amber, transparent: true, opacity: 0.95, depthTest: false, depthWrite: false, fog: false,
  }));
  pathLine.renderOrder = 11;
  pathLine.visible = false;
  scene.add(pathLine);
  function showPath(points, clear) {
    pathLine.visible = Boolean(points);
    if (!points) return;
    pathLine.geometry.dispose();
    pathLine.geometry = new THREE.BufferGeometry().setFromPoints(
      points.map(([x, y, z]) => new THREE.Vector3(sx + x * MM, sy + y * MM, sz + z * MM)));
    pathLine.material.color.setHex(clear ? COLOR.amber : COLOR.alarm);
  }

  // Where the tool goes: a ring and a dashed drop line to the floor.
  const marker = new THREE.Group();
  const markerRing = new THREE.Mesh(new THREE.TorusGeometry(0.016, 0.0022, 10, 64), new THREE.MeshBasicMaterial({color: COLOR.amber}));
  const markerDrop = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineDashedMaterial({color: COLOR.amber, dashSize: 0.01, gapSize: 0.008, transparent: true, opacity: 0.7}),
  );
  marker.add(markerRing, markerDrop);
  marker.visible = false;
  scene.add(marker);

  // ---- State ----
  let actual = null;  // the arm's joints (SDK values), from the latest status
  let goal = null;  // the joints of the move under way
  let goalTool = null;  // the tool's target, for a pose or linear move under way
  let staged = null;  // the page's plan: SDK values by field, null for joints that follow the arm
  let stagedKey = "null";
  let plan = null;  // the planned pose: `staged` filled in from the arm
  let planCheck = {clear: true, reason: null};
  let drag = null;  // a handle drag in progress
  let hover = null;  // the handle under the pointer
  let interactive = false;
  let dirty = true;
  let lastKey = "";

  const toModel = (joints) => {
    const values = {};
    for (const f of fields) {
      if (Number.isFinite(joints?.[f.field])) values[f.joint] = joints[f.field] * f.scale;
    }
    return values;
  };
  const blindMaterials = () => [...materials.entries()].filter(([name]) => name.startsWith("j2_arm:")).map(([, m]) => m);

  function refreshPlan() {
    const set = staged ? Object.entries(staged).filter(([, value]) => Number.isFinite(value)) : [];
    plan = set.length && actual ? {...actual, ...Object.fromEntries(set)} : null;
    planCheck = plan ? pathCheck(zone, actual, plan) : {clear: true, reason: null};
  }

  function showMarker(point, blocked) {
    marker.visible = true;
    marker.position.copy(point);
    markerDrop.geometry.setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -point.z)]);
    markerDrop.computeLineDistances();
    const tone = blocked ? COLOR.alarm : COLOR.amber;
    markerRing.material.color.setHex(tone);
    markerDrop.material.color.setHex(tone);
  }

  // Poses the shadow arm, and the handles riding on it: a drag in progress, else the plan, else the goal of the move
  // under way, else the arm itself. Only the links a changed joint moves are drawn; red when the plan is blocked. The
  // gizmo sits at the shadow's flange, and the tool's path runs from the arm to the shadow.
  const toolPoint = new THREE.Vector3();
  function updateGhost() {
    if (!actual) return;
    const pose = drag ? drag.pose : plan || goal;
    const check = drag ? drag.check : plan ? planCheck : null;
    shadow.setJoints(toModel(pose || actual));
    const turned = new Set(pose ? fields.filter((f) => Math.abs(pose[f.field] - actual[f.field]) > 0.01).map((f) => f.joint) : []);
    const blocked = Boolean(check && !check.clear);
    for (const {mesh, link, xray} of ghostMeshes) {
      mesh.visible = drawn(mesh) && ancestry.get(link).some((joint) => turned.has(joint));
      mesh.material = blocked ? ghostBlocked : ghostOk;
      xray.visible = blocked;
    }
    if (turned.size) showMarker(toolPoint.setFromMatrixPosition(shadow.groups.get("tool0").matrixWorld), blocked);
    else if (goalTool) showMarker(toolPoint.set(sx + goalTool.x_mm * MM, sy + goalTool.y_mm * MM, sz + goalTool.z_mm * MM), false);
    else marker.visible = false;
    gizmoAt.setFromMatrixPosition(shadow.groups.get("tool0").matrixWorld);
    gizmo.group.visible = mode === "tool";
    showPath(turned.size ? toolPath(actual, pose, lengths) : null, !blocked);
    dirty = true;
  }

  // The handles of the mode in use, once the arm's position is known: the rings and the Z handle, or the gizmo.
  function showHandles() {
    for (const handle of handles) handle.group.visible = Boolean(actual) && mode === "joints";
    gizmo.group.visible = Boolean(actual) && mode === "tool";
  }

  // Each handle in its tape colour, brightened under the pointer; the others dim while one is dragged, and all of
  // them while the arm can't take a plan. The gizmo styles its own, in its axis colours.
  const white = new THREE.Color(COLOR.lit);
  function styleHandles() {
    for (const handle of handles) {
      const active = drag ? drag.handle === handle : hover === handle;
      const dim = drag ? drag.handle !== handle : !interactive;
      for (const pass of PASSES) {
        const material = handle.mats[pass.key];
        material.opacity = pass.opacity * (dim ? DIM : 1);
        if (!pass.rim) material.color.copy(handle.tone).lerp(white, active ? 0.35 : 0);
      }
      handle.band.visible = active && (interactive || Boolean(drag));
    }
    gizmo.style(hover && hover.kind === "gizmo" ? hover : null, drag && drag.kind === "tool" ? drag.handle : null, interactive);
    renderer.domElement.style.cursor = drag ? "grabbing" : hover && interactive ? "grab" : "";
    dirty = true;
  }

  // ---- Picking and dragging ----
  const raycaster = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  const canvasRect = () => renderer.domElement.getBoundingClientRect();
  const canvasSize = () => ({width: renderer.domElement.clientWidth || host.clientWidth,
                             height: renderer.domElement.clientHeight || host.clientHeight});
  function aim(event, rect) {
    ndc.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
    raycaster.setFromCamera(ndc, camera);
  }
  function toPixel(world, rect) {
    const p = world.clone().project(camera);
    return p.z > 1 ? null : new THREE.Vector2(((p.x + 1) / 2) * rect.width, ((1 - p.y) / 2) * rect.height);
  }
  const centreOf = (handle) => new THREE.Vector3().setFromMatrixPosition(handle.group.matrixWorld);
  const ringPoint = (handle, angle) => new THREE.Vector3(Math.cos(angle) * handle.radius, Math.sin(angle) * handle.radius, 0)
    .applyMatrix4(handle.group.matrixWorld);
  const slideEnds = (handle) => [-SLIDE.half, SLIDE.half].map((z) => new THREE.Vector3(0, 0, z).applyMatrix4(handle.group.matrixWorld));
  // The gizmo's centre on screen (canvas pixels), sized for the camera as it is now.
  function gizmoCentre(rect) {
    gizmo.place(gizmoAt, camera, rect);
    return gizmo.toScreen(new THREE.Vector3(), camera, rect) || new THREE.Vector2(rect.width / 2, rect.height / 2);
  }

  // The handle whose drawn shape passes nearest the pointer on screen, within reach, and where on a ring it was
  // grabbed. Hidden parts count: the handles show through the arm, so they can be grabbed through it too.
  function pickHandle(event) {
    if (!actual) return null;
    const rect = canvasRect();
    const px = event.clientX - rect.left;
    const py = event.clientY - rect.top;
    if (mode === "tool") {
      if (!gizmo.group.visible) return null;
      gizmo.place(gizmoAt, camera, rect);
      return gizmo.pick(px, py, camera, rect, event.pointerType === "touch" ? GIZMO_PICK_PX.touch : GIZMO_PICK_PX.mouse);
    }
    const reach = event.pointerType === "touch" ? PICK_PX.touch : PICK_PX.mouse;
    shadow.root.updateMatrixWorld(true);
    let best = null;
    for (const handle of handles) {
      if (!handle.group.visible) continue;
      if (handle.kind === "ring") {
        for (let k = 0; k < 120; k += 1) {
          const angle = (k / 120) * Math.PI * 2;
          const p = toPixel(ringPoint(handle, angle), rect);
          if (!p) continue;
          const gap = Math.hypot(p.x - px, p.y - py);
          if (gap <= reach && (!best || gap < best.gap)) best = {handle, gap, angle};
        }
      } else {
        const [a, b] = slideEnds(handle).map((end) => toPixel(end, rect));
        if (!a || !b) continue;
        const gap = Math.max(0, segmentGap(px, py, a, b) - 3);  // the arrow is a few pixels thick
        if (gap <= reach && (!best || gap < best.gap)) best = {handle, gap};
      }
    }
    return best;
  }

  // The pointer's angle around the dragged ring, in the ring's own plane.
  function ringAngle(event) {
    aim(event, drag.rect);
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(drag.normal, drag.center);
    const hit = raycaster.ray.intersectPlane(plane, new THREE.Vector3());
    if (!hit) return null;
    const local = drag.handle.group.worldToLocal(hit);
    return Math.atan2(local.y, local.x);
  }
  // How far along the Z handle's axis (m, from where the drag began) the pointer's ray passes closest, or null when
  // the axis points nearly along the ray.
  function alongAxis(d) {
    const {origin, direction} = raycaster.ray;
    const w = origin.clone().sub(d.center);
    const b = d.axis.dot(direction);
    const denominator = 1 - b * b;
    if (denominator < 1e-4) return null;
    return (d.axis.dot(w) - b * direction.dot(w)) / denominator;
  }
  function metresPerPixel(point, rect) {
    const depth = point.clone().sub(camera.position).dot(camera.getWorldDirection(new THREE.Vector3()));
    return (2 * Math.max(depth, camera.near) * Math.tan((camera.fov * RAD) / 2)) / Math.max(1, rect.height);
  }

  function grab(event) {
    controls.enabled = false;  // this capture-phase listener runs before OrbitControls' own, so the camera stays put
    event.preventDefault();
    event.stopImmediatePropagation();
    try {
      renderer.domElement.setPointerCapture(event.pointerId);
    } catch {
      // not an active pointer (a synthetic event); the drag still follows this element's events
    }
  }

  function startDrag(pick, event) {
    const handle = pick.handle;
    if (handle.kind === "gizmo") {
      startToolDrag(pick, event);
      return;
    }
    const rect = canvasRect();
    // The drag edits the plan (or the arm's pose when nothing is staged) one joint at a time.
    const base = {...(plan || actual)};
    shadow.root.updateMatrixWorld(true);
    const center = centreOf(handle);
    const axis = new THREE.Vector3(0, 0, 1).transformDirection(handle.group.matrixWorld);
    drag = {
      kind: "joint", handle, field: handle.field, unit: handle.unit, pointerId: event.pointerId, rect, base, center,
      from: base[handle.field], actualFrom: actual[handle.field], to: base[handle.field], pose: base,
      check: planCheck, limited: false, x: event.clientX, y: event.clientY,
    };
    aim(event, rect);
    if (handle.kind === "ring") {
      Object.assign(drag, {normal: axis, total: 0, startAngle: pick.angle});
      // Seen nearly edge-on, the ring's plane is useless for tracking the pointer: follow the ring on screen instead.
      drag.planar = Math.abs(raycaster.ray.direction.dot(axis)) > RING_FACING_MIN;
      if (drag.planar) {
        drag.last = ringAngle(event);
        if (drag.last === null) drag.planar = false;
        else drag.startAngle = drag.last;
      }
      if (!drag.planar) {
        const here = toPixel(ringPoint(handle, pick.angle), rect);
        const ahead = toPixel(ringPoint(handle, pick.angle + 0.05), rect);
        const along = here && ahead ? ahead.sub(here) : new THREE.Vector2(1, 0);
        drag.tangent = along.length() > 0.05 ? along.normalize() : new THREE.Vector2(1, 0);
        drag.pxPerRad = RING_PX_PER_DEG / RAD;
      }
    } else {
      drag.axis = axis;
      const along = alongAxis(drag);
      const [a, b] = slideEnds(handle).map((end) => toPixel(end, rect));
      // Seen end-on, the handle can't be followed along its axis: slide it by dragging up and down instead.
      drag.planar = along !== null && Boolean(a && b) && a.distanceTo(b) > SLIDE_MIN_PX;
      drag.t0 = drag.planar ? along : 0;
      drag.metresPerPx = metresPerPixel(center, rect);
    }
    // The handle's centre on the page, so the page can keep its readout clear of the handle.
    const middle = toPixel(center, rect);
    drag.cx = rect.left + (middle ? middle.x : rect.width / 2);
    drag.cy = rect.top + (middle ? middle.y : rect.height / 2);
    hover = null;
    onHover(null);
    styleHandles();
    updateDrag(event);
  }

  function updateDrag(event) {
    const d = drag;
    if (d.kind === "tool") {
      updateToolDrag(event);
      return;
    }
    let raw;
    if (d.handle.kind === "ring") {
      if (d.planar) {
        const angle = ringAngle(event);
        if (angle !== null) {
          d.total += wrap(angle - d.last);
          d.last = angle;
        }
      } else {
        d.total = ((event.clientX - d.x) * d.tangent.x + (event.clientY - d.y) * d.tangent.y) / d.pxPerRad;
      }
      raw = d.from + d.total / RAD;
    } else {
      aim(event, d.rect);
      const along = d.planar ? alongAxis(d) : null;
      const metres = along !== null ? along - d.t0 : d.planar ? d.metres || 0 : (d.y - event.clientY) * d.metresPerPx;
      d.metres = metres;
      raw = d.from + metres * 1000;
    }
    const [low, high] = limits[d.field] || [-Infinity, Infinity];
    const [fine, coarse] = SNAP[d.unit];
    const snap = event.shiftKey ? coarse : fine;
    d.to = Math.min(high, Math.max(low, Math.round(raw / snap) * snap));
    d.limited = raw < low || raw > high;  // pushing past a soft limit: the plan stops at it
    d.pose = {...d.base, [d.field]: d.to};
    d.check = pathCheck(zone, actual, d.pose);
    updateGhost();
    if (d.handle.sweep) {
      // An arc on the ring shows how far, and which way, this drag turns the joint: amber, or red at a limit or blocked.
      const turn = Math.max(-2 * Math.PI, Math.min(2 * Math.PI, (d.to - d.from) * RAD));
      const sweep = d.handle.sweep;
      sweep.visible = Math.abs(turn) > 0.001;
      if (sweep.visible) {
        sweep.geometry.dispose();
        sweep.geometry = new THREE.TorusGeometry(d.handle.radius, RING_TUBE * 1.7, 6, 96, Math.abs(turn));
        sweep.rotation.z = turn >= 0 ? d.startAngle : d.startAngle + turn;
        sweep.material.color.setHex(d.check.clear && !d.limited ? COLOR.amber : COLOR.alarm);
      }
    }
    onPreview({field: d.field, from: d.actualFrom, to: d.to, clear: d.check.clear, reason: d.check.reason,
               limited: d.limited, x: event.clientX, y: event.clientY, cx: d.cx, cy: d.cy});
  }

  // ---- Gizmo drags: slide the tool along an arrow, or turn its yaw about Z while the flange stays put ----
  // The pointer's angle about the yaw ring's centre, in the ring's (horizontal) plane.
  function turnAngle(d) {
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(d.axis, d.center);
    const hit = raycaster.ray.intersectPlane(plane, new THREE.Vector3());
    if (!hit) return null;
    hit.sub(d.center);
    return Math.atan2(hit.y, hit.x);
  }

  function startToolDrag(pick, event) {
    const handle = pick.handle;
    const rect = canvasRect();
    // The drag moves the tool from where the plan puts it (or the arm, when nothing is staged); the joints follow.
    const base = {...(plan || actual)};
    shadow.root.updateMatrixWorld(true);
    gizmoAt.setFromMatrixPosition(shadow.groups.get("tool0").matrixWorld);
    const centre = gizmoCentre(rect);
    drag = {
      kind: "tool", handle, type: handle.type, coord: handle.type === "turn" ? "yaw" : "xyz"[handle.axis],
      pointerId: event.pointerId, rect, base, from: toolPose(base, lengths), center: gizmoAt.clone(),
      axis: new THREE.Vector3(...BASE_AXES[handle.axis]), steps: 0, applied: 0, pose: base, reason: null,
      check: pathCheck(zone, actual, base), x: event.clientX, y: event.clientY, total: 0, startAngle: pick.angle || 0,
      metresPerPx: gizmo.metresPerPixel(), cx: rect.left + centre.x, cy: rect.top + centre.y,
    };
    aim(event, rect);
    if (handle.type === "move") {
      const along = alongAxis(drag);
      const a = gizmo.toScreen(new THREE.Vector3(), camera, rect);
      const b = gizmo.toScreen(drag.axis.clone().multiplyScalar(GIZMO.head[1]), camera, rect);
      // Seen end-on, an arrow can't be followed along its line: slide it by dragging up and down instead.
      drag.planar = along !== null && Boolean(a && b) && a.distanceTo(b) > SLIDE_MIN_PX;
      drag.t0 = drag.planar ? along : 0;
    } else {
      drag.planar = Math.abs(raycaster.ray.direction.dot(drag.axis)) > RING_FACING_MIN;
      if (drag.planar) {
        drag.last = turnAngle(drag);
        if (drag.last === null) drag.planar = false;
        else drag.startAngle = drag.last;
      }
      if (!drag.planar) {
        // Seen nearly edge-on: follow the ring's on-screen direction where it was grabbed, at a fixed rate.
        const here = gizmo.toScreen(gizmo.ringLocal(drag.startAngle), camera, rect);
        const ahead = gizmo.toScreen(gizmo.ringLocal(drag.startAngle + 0.05), camera, rect);
        const along = here && ahead ? ahead.sub(here) : new THREE.Vector2(1, 0);
        drag.tangent = along.length() > 0.05 ? along.normalize() : new THREE.Vector2(1, 0);
        drag.pxPerRad = RING_PX_PER_DEG / RAD;
      }
    }
    hover = null;
    onHover(null);
    styleHandles();
    updateToolDrag(event);
  }

  // Follows the drag toward `wanted` (a whole number of `fine` steps from where it began) one step at a time from the
  // last pose reached, and stops at the last one the arm can reach within its limits, saying why.
  function followTool(d, wanted, fine) {
    const direction = Math.sign(wanted - d.steps);
    d.reason = null;
    d.limitField = null;
    for (let n = d.steps + direction; direction !== 0 && n !== wanted + direction; n += direction) {
      const result = solveTool(d.base, d.coord, n * fine, lengths, limits);
      if (!result.ok) {
        d.reason = result.reason;
        d.limitField = result.field;
        break;
      }
      d.steps = n;
      d.pose = result.joints;
    }
    d.applied = d.steps * fine;
  }

  function updateToolDrag(event) {
    const d = drag;
    aim(event, d.rect);
    let raw;
    if (d.type === "move") {
      const along = d.planar ? alongAxis(d) : null;
      raw = along !== null ? (along - d.t0) * 1000 : d.planar ? d.lastRaw || 0 : (d.y - event.clientY) * d.metresPerPx * 1000;
      d.lastRaw = raw;
    } else {
      if (d.planar) {
        const angle = turnAngle(d);
        if (angle !== null) {
          d.total += wrap(angle - d.last);
          d.last = angle;
        }
      } else {
        d.total = ((event.clientX - d.x) * d.tangent.x + (event.clientY - d.y) * d.tangent.y) / d.pxPerRad;
      }
      raw = d.total / RAD;
    }
    const [fine, coarse] = TOOL_SNAP[d.type];
    const snap = event.shiftKey ? coarse : fine;
    followTool(d, Math.round((Math.round(raw / snap) * snap) / fine), fine);
    d.check = pathCheck(zone, actual, d.pose);
    updateGhost();
    const centre = gizmoCentre(d.rect);
    if (d.type === "turn") gizmo.showTurn(d.startAngle, d.applied * RAD, d.check.clear && !d.reason ? COLOR.amber : COLOR.alarm);
    // The page's readout sits just outside the gizmo, toward the pointer, so it never covers a handle or a letter.
    const toward = new THREE.Vector2(event.clientX - d.rect.left - centre.x, event.clientY - d.rect.top - centre.y);
    if (toward.length() < 1) toward.set(1, 0);
    toward.normalize().multiplyScalar(GIZMO.label + 12);
    onPreview({
      kind: "tool", type: d.type, coord: d.coord, name: d.handle.name, pose: {...d.pose},
      from: d.coord === "yaw" ? d.from.yaw_deg : d.from[`${d.coord}_mm`], amount: d.applied,
      clear: d.check.clear, reason: d.check.reason, limited: d.reason, limitField: d.limitField,
      x: d.rect.left + centre.x + toward.x, y: d.rect.top + centre.y + toward.y,
      cx: d.rect.left + centre.x, cy: d.rect.top + centre.y,
    });
  }

  function endDrag(cancelled) {
    const done = drag;
    drag = null;
    controls.enabled = true;
    if (done.kind === "tool") gizmo.hideTurn();
    else if (done.handle.sweep) done.handle.sweep.visible = false;
    try {
      if (renderer.domElement.hasPointerCapture(done.pointerId)) renderer.domElement.releasePointerCapture(done.pointerId);
    } catch {
      // already released
    }
    styleHandles();
    updateGhost();
    onPreview(null);
    if (done.kind === "tool") {
      onRelease({kind: "tool", type: done.type, coord: done.coord, name: done.handle.name, amount: done.applied,
                 from: done.coord === "yaw" ? done.from.yaw_deg : done.from[`${done.coord}_mm`], pose: {...done.pose},
                 limited: done.reason, cancelled, changed: !cancelled && done.steps !== 0});
      return;
    }
    onRelease({field: done.field, from: done.actualFrom, to: done.to, cancelled,
               changed: !cancelled && Math.abs(done.to - done.from) >= MIN_CHANGE[done.unit]});
  }

  function setHover(pick, event) {
    const handle = pick ? pick.handle : null;
    if (handle !== hover) {
      hover = handle;
      styleHandles();
    }
    if (!handle || !event) {
      onHover(null);
      return;
    }
    const rect = canvasRect();
    if (handle.kind === "gizmo") {
      const centre = gizmoCentre(rect);
      const at = toolPose(plan || actual, lengths);
      const coord = handle.type === "turn" ? "yaw" : "xyz"[handle.axis];
      onHover({kind: "tool", type: handle.type, coord, name: handle.name, from: coord === "yaw" ? at.yaw_deg : at[`${coord}_mm`],
               x: event.clientX, y: event.clientY, cx: rect.left + centre.x, cy: rect.top + centre.y});
      return;
    }
    const middle = toPixel(centreOf(handle), rect);
    onHover({field: handle.field, x: event.clientX, y: event.clientY,
             cx: rect.left + (middle ? middle.x : 0), cy: rect.top + (middle ? middle.y : 0)});
  }

  renderer.domElement.addEventListener("pointerdown", (event) => {
    if (drag) {  // one drag at a time: a second finger or a pen mustn't start a pinch or an orbit under it
      event.preventDefault();
      event.stopImmediatePropagation();
      return;
    }
    if (event.button !== 0 || !interactive || !actual) return;
    const pick = pickHandle(event);
    if (!pick) return;  // empty space: OrbitControls orbits, as before
    grab(event);
    startDrag(pick, event);
  }, true);
  renderer.domElement.addEventListener("pointermove", (event) => {
    if (drag) {
      if (event.pointerId === drag.pointerId) updateDrag(event);
      return;
    }
    if (event.buttons || event.pointerType === "touch") return;  // orbiting, or a finger (fingers don't hover)
    setHover(interactive ? pickHandle(event) : null, event);
  });
  renderer.domElement.addEventListener("pointerleave", () => {
    if (!drag && hover) setHover(null);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    renderer.domElement.addEventListener(type, (event) => {
      if (drag && event.pointerId === drag.pointerId) endDrag(type !== "pointerup");
    });
  }

  // ---- Rendering ----
  // The home view looks along VIEW's direction from just far enough away to show the whole arm, base plate to column
  // top, at the canvas's shape (a tall side panel or a wide desk alike), within INSET of its edges and above the
  // help line or plan bar. In Tool mode it leaves room for the gizmo around the flange. It refits as the canvas
  // resizes until the camera is moved; Reset view fits it again.
  const homeDirection = new THREE.Vector3(...VIEW.position).sub(new THREE.Vector3(...VIEW.target)).normalize();
  let homed = true;
  controls.addEventListener("start", () => { homed = false; });
  // The window (in normalised device coordinates) the arm is framed within.
  function frameWindow() {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    const foot = host.querySelector(".screen-foot");
    const footSpace = foot && foot.offsetHeight ? foot.offsetHeight + 10 + 8 : 0;  // docked 10 px up, and a little air
    const bottom = Math.min(Math.max(INSET.bottom, footSpace), height * 0.35);
    return {left: -1 + (2 * INSET.side) / width, right: 1 - (2 * INSET.side) / width,
            bottom: -1 + (2 * bottom) / height, top: 1 - (2 * INSET.top) / height};
  }
  // A thinned sample of the drawn arm's vertices in world space, taken with the arm at both ends of its Z travel: the
  // framing fits the arm's real shape, not its box, and stays the same whatever J3 is.
  const zJoint = robot.movers.get(fields.find((f) => f.field === "j3_mm")?.joint);
  function framePoints() {
    const points = [];
    const held = zJoint ? zJoint.mover.position.clone() : null;
    for (const end of zJoint && limits.j3_mm ? limits.j3_mm : [null]) {
      if (end !== null) robot.setJoints(toModel({j3_mm: end}));
      robot.root.updateMatrixWorld(true);
      robot.root.traverse((node) => {
        if (!node.isMesh || !node.visible || !node.geometry.attributes.position) return;
        const position = node.geometry.attributes.position;
        const stride = Math.max(1, Math.floor(position.count / 1500));
        for (let i = 0; i < position.count; i += stride) {
          points.push(new THREE.Vector3().fromBufferAttribute(position, i).applyMatrix4(node.matrixWorld));
        }
      });
    }
    if (held) {
      zJoint.mover.position.copy(held);
      robot.root.updateMatrixWorld(true);
    }
    return points;
  }
  function fitView() {
    const points = framePoints();
    if (!points.length) return;
    const area = frameWindow();
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    // In Tool mode, room for the gizmo around the flange: it's a fixed size on screen, so its margin is in pixels,
    // and never more than 45% of the area, so the fit always settles.
    const flange = mode === "tool" && actual ? gizmoAt.clone() : null;
    const margin = [Math.min((2 * GIZMO.reach) / width, 0.45 * (area.right - area.left)),
                    Math.min((2 * GIZMO.reach) / height, 0.45 * (area.top - area.bottom))];
    const centre = new THREE.Box3().setFromPoints(points).getCenter(new THREE.Vector3());
    let distance = 2.5;
    const p = new THREE.Vector3();
    const right = new THREE.Vector3();
    const up = new THREE.Vector3();
    for (let k = 0; k < 12; k += 1) {
      camera.position.copy(centre).addScaledVector(homeDirection, distance);
      camera.lookAt(centre);
      camera.updateMatrixWorld(true);
      let [x0, x1, y0, y1] = [Infinity, -Infinity, Infinity, -Infinity];
      for (const point of points) {
        p.copy(point).project(camera);
        x0 = Math.min(x0, p.x);
        x1 = Math.max(x1, p.x);
        y0 = Math.min(y0, p.y);
        y1 = Math.max(y1, p.y);
      }
      if (flange) {
        p.copy(flange).project(camera);
        x0 = Math.min(x0, p.x - margin[0]);
        x1 = Math.max(x1, p.x + margin[0]);
        y0 = Math.min(y0, p.y - margin[1]);
        y1 = Math.max(y1, p.y + margin[1]);
      }
      // Slide the view so the picture sits in the middle of the area, then zoom so it fills it.
      const halfHeight = distance * Math.tan((camera.fov * RAD) / 2);
      right.setFromMatrixColumn(camera.matrixWorld, 0);
      up.setFromMatrixColumn(camera.matrixWorld, 1);
      centre.addScaledVector(right, ((x0 + x1 - area.left - area.right) / 2) * halfHeight * camera.aspect)
        .addScaledVector(up, ((y0 + y1 - area.bottom - area.top) / 2) * halfHeight);
      const scale = Math.max((x1 - x0) / (area.right - area.left), (y1 - y0) / (area.top - area.bottom));
      distance = Math.max(0.3, distance * scale);
    }
    controls.target.copy(centre);
    camera.position.copy(centre).addScaledVector(homeDirection, distance);
    controls.update();
    homed = true;
    dirty = true;
  }

  // In Tool mode, keep the gizmo whole on screen and clear of the plan bar: when a new plan, or the bar appearing or
  // growing, leaves its arrows near an edge or under the bar, re-frame (unless the camera has been moved, or a drag is
  // on). It's checked on the next frame, once the page has laid the bar out.
  function gizmoCrowded() {
    if (!gizmo.group.visible) return false;
    const rect = canvasRect();
    gizmo.place(gizmoAt, camera, rect);
    const centre = gizmo.toScreen(new THREE.Vector3(), camera, rect);
    if (!centre) return true;
    const room = GIZMO.head[1] + 10;  // the arrow tips, and a little air
    if (Math.min(centre.x, centre.y, rect.width - centre.x, rect.height - centre.y) < room) return true;
    const bar = host.querySelector(".plan");
    if (!bar || bar.hidden) return false;
    const box = bar.getBoundingClientRect();
    const x = rect.left + centre.x;
    const y = rect.top + centre.y;
    return Math.hypot(x - Math.max(box.left, Math.min(x, box.right)), y - Math.max(box.top, Math.min(y, box.bottom))) < room;
  }
  let clearCheck = 0;
  function keepGizmoClear() {
    if (clearCheck) return;
    clearCheck = requestAnimationFrame(() => {
      clearCheck = 0;
      if (mode === "tool" && homed && !drag && gizmoCrowded()) fitView();
    });
  }
  const planBar = host.querySelector(".plan");
  if (planBar) new ResizeObserver(() => keepGizmoClear()).observe(planBar);

  controls.addEventListener("change", () => { dirty = true; });
  const resize = () => {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    sizeLabels(width, height);
    if (homed && !drag) fitView();  // never re-frame under a drag
    dirty = true;
  };
  new ResizeObserver(resize).observe(host);
  resize();
  let running = true;
  renderer.domElement.addEventListener("webglcontextlost", (event) => {
    event.preventDefault();
    running = false;
    if (onLost) onLost();
  });
  const frame = () => {
    if (!running) return;
    requestAnimationFrame(frame);
    if (dirty) {
      dirty = false;
      if (gizmo.group.visible) gizmo.place(gizmoAt, camera, canvasSize());
      renderer.render(scene, camera);
    }
  };
  requestAnimationFrame(frame);

  return {
    hasCad: cad,
    cadError,
    resetView() {
      fitView();
    },
    // The arm's state from the server: its joints, the move under way, and whether the J2 arm is in the blind zone.
    update(status) {
      const joints = status && status.joints;
      if (!joints || !fields.every((f) => Number.isFinite(joints[f.field]))) return;
      const g = status.goal || null;
      const goalJoints = g ? g.jogGoal || g.joints || null : null;
      const hit = Boolean(status.blindZone && status.blindZone.hit);
      const stateKey = JSON.stringify([joints, goalJoints, g && g.tool, hit]);
      if (stateKey === lastKey) return;
      lastKey = stateKey;
      const first = !actual;
      actual = {...joints};
      robot.setJoints(toModel(actual));
      goal = goalJoints ? {...goalJoints} : null;
      goalTool = !goalJoints && g && g.tool ? g.tool : null;
      for (const material of blindMaterials()) {
        material.emissive.setHex(hit ? COLOR.alarm : 0x000000);
        material.emissiveIntensity = hit ? 0.55 : 0;
      }
      if (first) {
        showHandles();
        styleHandles();
      }
      refreshPlan();  // the arm moved under the plan: check its path again
      updateGhost();
      if (first && homed && !drag) fitView();  // frame the arm as it stands, with room for the gizmo in Tool mode
    },
    // The page's plan: SDK values by field, null for joints that follow the arm, or null for no plan. Returns whether
    // the straight joint move from the arm to it keeps the J2 arm out of the column, and if not, why.
    setPlan(next) {
      const nextKey = JSON.stringify(next || null);
      if (nextKey !== stagedKey) {
        stagedKey = nextKey;
        staged = next ? {...next} : null;
        refreshPlan();
        updateGhost();
        keepGizmoClear();  // a new plan moves the gizmo with the shadow's flange
      }
      return planCheck;
    },
    // What dragging in the view moves: "joints" (the rings and the Z handle) or "tool" (the gizmo at the flange). A
    // drag under way is cancelled; the plan stays, so the two can be mixed.
    setMode(next) {
      const wanted = next === "tool" ? "tool" : "joints";
      if (wanted === mode) return;
      if (drag) endDrag(true);
      mode = wanted;
      if (hover) {
        hover = null;
        onHover(null);
      }
      showHandles();
      updateGhost();
      if (homed) fitView();  // make room for the gizmo, or take it back
      styleHandles();
    },
    currentMode: () => mode,
    // How many points the drawn tool path has (0 when none is drawn), for automated checks.
    pathPoints: () => (pathLine.visible && pathLine.geometry.attributes.position ? pathLine.geometry.attributes.position.count : 0),
    // Page positions of a gizmo handle (used by automated checks): partway along an arrow and its tip ("move", axis 0,
    // 1 or 2 for X, Y or Z), or a point on the yaw ring ("turn"); and the gizmo's centre. Null when it isn't shown.
    gizmoPoint(type, axis = 2) {
      if (!actual || !gizmo.group.visible) return null;
      const rect = canvasRect();
      gizmo.place(gizmoAt, camera, rect);
      const points = gizmo.points(type, axis, camera, rect);
      if (!points) return null;
      const page = (p) => p && {x: rect.left + p.x, y: rect.top + p.y};
      return {...page(points.point), tip: page(points.tip), cx: rect.left + points.centre.x, cy: rect.top + points.centre.y};
    },
    // Whether new drags may start. A drag under way carries on; the page cancels it when the arm can't take a plan.
    setInteractive(on) {
      if (interactive === Boolean(on)) return;
      interactive = Boolean(on);
      if (!interactive && hover) {
        hover = null;
        onHover(null);
      }
      styleHandles();
    },
    cancelDrag() {
      if (!drag) return false;
      endDrag(true);
      return true;
    },
    isDragging: () => Boolean(drag),
    // The page position of a point in the robot frame (m), for automated checks of the framing; null behind the camera.
    screenPoint([x, y, z]) {
      const rect = canvasRect();
      const p = toPixel(new THREE.Vector3(x, y, z), rect);
      return p && {x: rect.left + p.x, y: rect.top + p.y};
    },
    // Page positions of a handle (used by automated checks): a ring's outermost point on screen, or a point on the
    // Z handle and its two ends; and the handle's centre.
    handlePoint(fieldName) {
      const handle = handles.find((h) => h.field === fieldName);
      if (!handle || !actual || !handle.group.visible) return null;
      const rect = canvasRect();
      shadow.root.updateMatrixWorld(true);
      const page = (p) => p && {x: rect.left + p.x, y: rect.top + p.y};
      const centre = toPixel(centreOf(handle), rect);
      if (!centre) return null;
      if (handle.kind === "ring") {
        let best = null;
        for (let k = 0; k < 72; k += 1) {
          const p = toPixel(ringPoint(handle, (k / 72) * Math.PI * 2), rect);
          if (p && (!best || p.distanceTo(centre) > best.distanceTo(centre))) best = p;
        }
        return best && {...page(best), cx: rect.left + centre.x, cy: rect.top + centre.y};
      }
      const [low, high] = slideEnds(handle).map((end) => toPixel(end, rect));
      if (!low || !high) return null;
      const at = low.clone().lerp(high, 0.7);
      return {...page(at), cx: rect.left + centre.x, cy: rect.top + centre.y, low: page(low), high: page(high)};
    },
  };
}
