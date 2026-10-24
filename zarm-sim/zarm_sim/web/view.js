// The 3D monitor: the arm built from the URDF the server serves, posed from the SDK's joint values; the floor's
// millimetre grid and the reach envelope in the SDK's own x and y; an amber shadow arm and ring at the goal of the
// move under way; and, when present, CAD meshes made locally from Hitbot's STEP in place of the primitives.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { buildRobot, parseUrdf } from "./urdf.js";

const COLOR = {
  well: 0x0e0f10, grid: 0x23262a, gridMain: 0x3d4248, amber: 0xf2aa3c, alarm: 0xff8073, reach: 0x8f8b83,
  axisX: 0xff3352, axisY: 0x8bdc00,
};
const TAPES = {j1_shoulder: 0xe3ba4f, j2_elbow: 0x7fb3d9, j3_vertical: 0x92cc8a, j4_wrist: 0xe19769};
const VIEW = {target: [0.22, 0.0, 0.17], position: [1.3, -1.28, 1.24]};
const GRID_MM = {x0: -300, x1: 500, y0: -500, y1: 500};  // in the SDK frame
const MM = 0.001;
const LABEL_PX = 20;  // the floor labels' height on screen (their text is about 12 px), whatever the zoom

async function fetchOk(url, kind) {
  const response = await fetch(url, {cache: "no-store"});
  if (!response.ok) throw new Error(`couldn't load ${url} (HTTP ${response.status})`);
  return kind === "json" ? response.json() : response.text();
}

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

function sizeLabels(camera, widthPx, heightPx) {
  // Without size attenuation a sprite's height on screen is scale.y * projection[1][1] / 2 of the viewport's height.
  // A narrow monitor gets smaller labels, every 200 mm only.
  const narrow = widthPx < 520;
  const scale = (2 * (narrow ? 16 : LABEL_PX)) / (camera.projectionMatrix.elements[5] * heightPx);
  for (const sprite of labels) {
    sprite.scale.set(scale * 4, scale, 1);
    sprite.visible = !narrow || sprite.userData.every200 !== false;
  }
}

function lines(points, material) {
  return new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points), material);
}

export async function createView(host, config, {onLost} = {}) {
  const [urdfText, envelope] = await Promise.all([fetchOk(config.urdf, "text"), fetchOk(config.envelope, "json")]);
  const urdf = parseUrdf(urdfText);
  const [sx, sy, sz] = config.sdkOriginM;

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
  const resetView = () => {
    camera.position.set(...VIEW.position);
    controls.target.set(...VIEW.target);
    controls.update();
  };
  resetView();

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

  // The floor: soft contact shadows, then the millimetre grid in the SDK's x and y (10 and 100 mm), labelled in mm.
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

  // The arm, from the URDF. Each link gets its own materials, so one link can be tinted alone.
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

  // Tape rings name the joints in the monitor, in the channel strips' colours.
  const ring = (radius, colour) => new THREE.Mesh(
    new THREE.TorusGeometry(radius, 0.0032, 12, 96),
    new THREE.MeshBasicMaterial({color: colour}),
  );
  const partZ = (link, name) => robot.meshes.get(link).find((mesh) => mesh.name === name);
  const shoulder = partZ("j1_arm", "j1_arm_shoulder_end");
  const elbow = partZ("j2_arm", "j2_arm_elbow_end");
  const flange = partZ("flange", "flange_disc");
  if (shoulder) {
    const r1 = ring(0.06, TAPES.j1_shoulder);
    r1.position.set(0, 0, shoulder.position.z + shoulder.geometry.parameters.height / 2 + 0.001);
    robot.groups.get("j1_arm").add(r1);
  }
  if (elbow) {
    const r2 = ring(0.06, TAPES.j2_elbow);
    r2.position.set(0, 0, elbow.position.z + elbow.geometry.parameters.height / 2 + 0.001);
    robot.groups.get("j2_arm").add(r2);
  }
  if (flange) {
    const r4 = ring(0.049, TAPES.j4_wrist);
    r4.position.set(0, 0, flange.position.z);
    robot.groups.get("flange").add(r4);
  }
  const j3 = urdf.joints.find((joint) => joint.name === "j3_vertical");
  const column = robot.meshes.get("base_link").find((mesh) => mesh.name === "column");
  const housing = robot.meshes.get("carriage").find((mesh) => mesh.name === "housing");
  if (j3 && column && housing) {
    // The J3 rail: where the carriage's housing travels, drawn on the column's face beside the housing.
    const {width: depth, height: width} = column.geometry.parameters;  // the box's x and y sizes
    const low = j3.xyz[2] + j3.lower + housing.position.z - housing.geometry.parameters.depth / 2;
    const high = j3.xyz[2] + j3.upper + housing.position.z + housing.geometry.parameters.depth / 2;
    const rail = new THREE.Mesh(new THREE.BoxGeometry(0.002, 0.006, high - low), new THREE.MeshBasicMaterial({color: TAPES.j3_vertical}));
    rail.position.set(column.position.x + depth / 2 + 0.001, column.position.y + width / 2 - 0.008, (low + high) / 2);
    robot.groups.get("base_link").add(rail);
  }

  // The goal: an amber shadow of the moving links, and a ring where the tool is going.
  const shadowMaterial = new THREE.MeshBasicMaterial({color: COLOR.amber, transparent: true, opacity: 0.3, depthWrite: false});
  const shadow = buildRobot(THREE, urdf, () => shadowMaterial);
  for (const mesh of shadow.meshes.get(urdf.root)) mesh.visible = false;
  shadow.root.visible = false;
  scene.add(shadow.root);
  const marker = new THREE.Group();
  const markerRing = new THREE.Mesh(new THREE.TorusGeometry(0.016, 0.0022, 10, 64), new THREE.MeshBasicMaterial({color: COLOR.amber}));
  const markerDrop = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineDashedMaterial({color: COLOR.amber, dashSize: 0.01, gapSize: 0.008, transparent: true, opacity: 0.7}),
  );
  marker.add(markerRing, markerDrop);
  marker.visible = false;
  scene.add(marker);

  let dirty = true;

  // Optional CAD meshes (made locally by scripts/fetch_cad_meshes.py), drawn in place of the primitives.
  const cad = {groups: [], on: false};
  if (config.meshes) {
    const {STLLoader} = await import("three/addons/loaders/STLLoader.js");
    const loader = new STLLoader();
    const tones = {base_link: 0xc9cacc, carriage: 0xe9e8e4, j1_arm: 0xe9e8e4, j2_arm: 0xe9e8e4, flange: 0xa9adb2};
    try {
      const loaded = await Promise.all(Object.entries(config.meshes).map(async ([link, url]) => [link, await loader.loadAsync(url)]));
      for (const [link, geometry] of loaded) {
        const material = new THREE.MeshStandardMaterial({color: tones[link] ?? 0xdddddd, roughness: 0.5, metalness: 0.1});
        materials.set(`${link}:cad`, material);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.visible = false;
        robot.groups.get(link).add(mesh);
        cad.groups.push({link, mesh});
      }
    } catch (error) {
      console.warn("CAD meshes didn't load; drawing the primitives", error);
      cad.groups.length = 0;
    }
  }
  const setCad = (on) => {
    cad.on = on && cad.groups.length > 0;
    for (const {mesh} of cad.groups) mesh.visible = cad.on;
    for (const {link} of cad.groups) {
      for (const mesh of robot.meshes.get(link)) mesh.visible = !cad.on || mesh.name === "flange_index";
    }
    dirty = true;
  };

  controls.addEventListener("change", () => { dirty = true; });
  const resize = () => {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    sizeLabels(camera, width, height);
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
      renderer.render(scene, camera);
    }
  };
  requestAnimationFrame(frame);

  const toModel = (joints) => {
    const values = {};
    for (const entry of config.jointMap) {
      if (Number.isFinite(joints?.[entry.field])) values[entry.joint] = joints[entry.field] * entry.scale;
    }
    return values;
  };
  const blindMaterials = () => [...materials.entries()].filter(([name]) => name.startsWith("j2_arm:")).map(([, m]) => m);
  let lastKey = "";

  return {
    hasCad: cad.groups.length > 0,
    setCad,
    resetView() { resetView(); dirty = true; },
    update(status) {
      const goal = status?.goal ?? null;
      const stateKey = JSON.stringify([status?.joints, goal, status?.blindZone?.hit]);
      if (stateKey === lastKey) return;
      lastKey = stateKey;
      if (status?.joints) robot.setJoints(toModel(status.joints));
      const goalJoints = goal?.jogGoal ?? goal?.joints;
      shadow.root.visible = Boolean(goalJoints);
      if (goalJoints) shadow.setJoints(toModel(goalJoints));
      marker.visible = Boolean(goal?.tool);
      if (goal?.tool) {
        const {x_mm: x, y_mm: y, z_mm: z} = goal.tool;
        const height = sz + z * MM;
        marker.position.set(sx + x * MM, sy + y * MM, height);
        markerDrop.geometry.setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -height)]);
        markerDrop.computeLineDistances();
      }
      const hit = Boolean(status?.blindZone?.hit);
      for (const material of blindMaterials()) {
        material.emissive.setHex(hit ? COLOR.alarm : 0x000000);
        material.emissiveIntensity = hit ? 0.55 : 0;
      }
      dirty = true;
    },
  };
}
