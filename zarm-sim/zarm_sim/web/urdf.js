// Reads the robot description the server serves (a URDF of boxes and cylinders) and builds it with three.js.
// Each link becomes a Group; each joint a frame at its origin holding a mover that turns or slides the child.

function numbers(text, count, fallback) {
  const values = String(text ?? fallback).trim().split(/\s+/).map(Number);
  if (values.length !== count || values.some((value) => !Number.isFinite(value))) {
    throw new Error(`the URDF has a bad list of ${count} numbers: "${text}"`);
  }
  return values;
}

function origin(element) {
  return {
    xyz: numbers(element?.getAttribute("xyz"), 3, "0 0 0"),
    rpy: numbers(element?.getAttribute("rpy"), 3, "0 0 0"),
  };
}

function shape(element) {
  const tag = element?.tagName;
  if (tag === "box") return {type: "box", size: numbers(element.getAttribute("size"), 3)};
  if (tag === "cylinder") {
    return {type: "cylinder", radius: Number(element.getAttribute("radius")), length: Number(element.getAttribute("length"))};
  }
  if (tag === "sphere") return {type: "sphere", radius: Number(element.getAttribute("radius"))};
  return null;  // meshes: this model has none
}

export function parseUrdf(text) {
  const doc = new DOMParser().parseFromString(text, "application/xml");
  if (doc.querySelector("parsererror")) throw new Error("the URDF isn't valid XML");
  const robot = doc.documentElement;
  if (robot.tagName !== "robot") throw new Error("the URDF's root element isn't <robot>");
  const materials = new Map();
  for (const material of robot.querySelectorAll(":scope > material")) {
    const color = material.querySelector("color");
    if (color) materials.set(material.getAttribute("name"), numbers(color.getAttribute("rgba"), 4));
  }
  const links = new Map();
  for (const link of robot.querySelectorAll(":scope > link")) {
    const visuals = [];
    for (const visual of link.querySelectorAll(":scope > visual")) {
      const geometry = shape(visual.querySelector(":scope > geometry > *"));
      if (!geometry) continue;
      const material = visual.querySelector(":scope > material");
      const color = material?.querySelector("color");
      const rgba = color ? numbers(color.getAttribute("rgba"), 4) : materials.get(material?.getAttribute("name")) ?? [0.8, 0.8, 0.8, 1];
      visuals.push({name: visual.getAttribute("name") ?? "", ...origin(visual.querySelector(":scope > origin")), geometry, rgba});
    }
    links.set(link.getAttribute("name"), {visuals});
  }
  const joints = [];
  for (const joint of robot.querySelectorAll(":scope > joint")) {
    const limit = joint.querySelector(":scope > limit");
    joints.push({
      name: joint.getAttribute("name"),
      type: joint.getAttribute("type"),
      parent: joint.querySelector(":scope > parent")?.getAttribute("link"),
      child: joint.querySelector(":scope > child")?.getAttribute("link"),
      ...origin(joint.querySelector(":scope > origin")),
      axis: numbers(joint.querySelector(":scope > axis")?.getAttribute("xyz"), 3, "1 0 0"),
      lower: Number(limit?.getAttribute("lower") ?? 0),
      upper: Number(limit?.getAttribute("upper") ?? 0),
    });
  }
  for (const joint of joints) {
    if (!links.has(joint.parent) || !links.has(joint.child)) throw new Error(`joint ${joint.name} refers to a missing link`);
  }
  const children = new Set(joints.map((joint) => joint.child));
  const roots = [...links.keys()].filter((name) => !children.has(name));
  if (roots.length !== 1) throw new Error("the URDF must have exactly one root link");
  return {name: robot.getAttribute("name"), links, joints, root: roots[0]};
}

function geometryFor(THREE, spec) {
  if (spec.type === "box") return new THREE.BoxGeometry(...spec.size);
  if (spec.type === "sphere") return new THREE.SphereGeometry(spec.radius, 32, 16);
  const cylinder = new THREE.CylinderGeometry(spec.radius, spec.radius, spec.length, 64);
  cylinder.rotateX(Math.PI / 2);  // three's cylinders run along Y; URDF's along Z
  return cylinder;
}

export function rpyQuaternion(THREE, [roll, pitch, yaw]) {
  // URDF roll-pitch-yaw are fixed-axis X, Y, Z rotations: R = Rz(yaw) Ry(pitch) Rx(roll).
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(roll, pitch, yaw, "ZYX"));
}

// materialFor(visual, linkName) returns the material for one visual.
export function buildRobot(THREE, urdf, materialFor) {
  const groups = new Map();
  const meshes = new Map();
  const movers = new Map();
  const makeGroup = (name) => {
    const group = new THREE.Group();
    group.name = name;
    const list = [];
    for (const visual of urdf.links.get(name).visuals) {
      const mesh = new THREE.Mesh(geometryFor(THREE, visual.geometry), materialFor(visual, name));
      mesh.name = visual.name;
      mesh.position.set(...visual.xyz);
      mesh.quaternion.copy(rpyQuaternion(THREE, visual.rpy));
      group.add(mesh);
      list.push(mesh);
    }
    groups.set(name, group);
    meshes.set(name, list);
    return group;
  };
  const root = makeGroup(urdf.root);
  const pending = [urdf.root];
  while (pending.length) {
    const parent = pending.pop();
    for (const joint of urdf.joints) {
      if (joint.parent !== parent) continue;
      const frame = new THREE.Object3D();
      frame.position.set(...joint.xyz);
      frame.quaternion.copy(rpyQuaternion(THREE, joint.rpy));
      const mover = new THREE.Object3D();
      frame.add(mover);
      mover.add(makeGroup(joint.child));
      groups.get(parent).add(frame);
      movers.set(joint.name, {joint, mover, axis: new THREE.Vector3(...joint.axis).normalize()});
      pending.push(joint.child);
    }
  }
  const setJoints = (values) => {
    for (const [name, value] of Object.entries(values)) {
      const entry = movers.get(name);
      if (!entry || !Number.isFinite(value)) continue;
      if (entry.joint.type === "prismatic") entry.mover.position.copy(entry.axis).multiplyScalar(value);
      else if (entry.joint.type === "revolute" || entry.joint.type === "continuous") {
        entry.mover.quaternion.setFromAxisAngle(entry.axis, value);
      }
    }
    root.updateMatrixWorld(true);
  };
  return {root, groups, meshes, movers, setJoints};
}
