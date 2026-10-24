// The simulator desk: polls the server's status about 30 times a second, sends commands with the page's token, and
// hands the arm's state to the 3D monitor (view.js), loaded only when three.js and WebGL are both available.
// Dragging a ring or the Z handle in the monitor stages a plan; nothing moves until the plan bar's Move sends it.
const CONFIG = JSON.parse(document.getElementById("zarm-config").textContent);
const $ = (id) => document.getElementById(id);
const MINUS = "\u2212";
const JOINTS = [
  {n: 1, field: "j1_deg", name: "Shoulder", unit: "\u00b0"},
  {n: 2, field: "j2_deg", name: "Elbow", unit: "\u00b0"},
  {n: 3, field: "j3_mm", name: "Vertical", unit: " mm"},
  {n: 4, field: "j4_deg", name: "Wrist R", unit: "\u00b0"},
];
const JOINT = Object.fromEntries(JOINTS.map((joint) => [joint.field, joint]));
const FRESH_MS = 1000;
const PLAN_MIN = {j1_deg: 0.25, j2_deg: 0.25, j3_mm: 0.5, j4_deg: 0.25};  // a joint this close to the arm isn't planned
const HANDOVER_MS = 1500;  // a sent plan gives way to the move's goal when the status shows it, or after this long

let status = null;
let view = null;
let failedPolls = 0;
let lastHeard = 0;
let shownLogSeq = -1;
let localMessage = null;  // a problem the page found itself (no answer, bad input)
let serverMessage = null;  // the newest entry of the server's log, and when it first showed up
let prefilled = false;
let jogInFlight = false;
let polling = false;
let pollSoon = false;
let pollTimer = null;
// Planning in the 3D view.
let plan = null;  // staged targets by SDK field (deg, or mm for J3); a joint left out follows the arm
let preview = null;  // the drag under way, from the 3D view
let planCheck = {clear: true, reason: null};
let planSending = false;
let planSentAt = 0;
let rendering = false;
let renderAgain = false;

function fmt(value, digits = 2) {
  if (!Number.isFinite(value)) return "\u2013";
  const text = value.toFixed(digits);
  return /^-0\.?0*$/.test(text) ? text.slice(1) : text.replace("-", MINUS);
}

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

const isLive = () => Boolean(status) && performance.now() - lastHeard < FRESH_MS;
const armBusy = () => Boolean(status && (status.busy || status.moving || status.queued));
const listOf = (names) => (names.length > 1 ? `${names.slice(0, -1).join(", ")} and ${names[names.length - 1]}` : names[0]);

// ---- Commands ----

function say(text, level = "error") {
  localMessage = {text, level, at: performance.now()};
  renderMessage();
}

async function post(path, body = {}) {
  let reply;
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: {"Content-Type": "application/json", "X-Panel-Token": CONFIG.token},
      body: JSON.stringify(body),
    });
    reply = await response.json().catch(() => ({ok: false, message: `The server answered HTTP ${response.status}.`}));
  } catch {
    reply = {ok: false, message: "No answer from the simulator server. Is it still running?"};
    say(reply.message);
    return reply;
  }
  localMessage = null;  // the server logged the outcome; poll now to show it
  poll();
  return reply;
}

function detent(name) {
  return Number(document.querySelector(`input[name="${name}"]:checked`).value);
}

function refuse(button) {
  button.classList.add("refused");
  setTimeout(() => button.classList.remove("refused"), 700);
}

async function jog(joint, dir, button) {
  if (jogInFlight) return;  // one at a time; a held key just skips a beat
  jogInFlight = true;
  try {
    const reply = await post("/api/jog", {joint: joint.n, dir, step: detent("step"), speed: detent("speed")});
    if (!reply.ok) refuse(button);
  } finally {
    jogInFlight = false;
  }
}

const repeats = new Set();  // the release functions of the jog keys, so Stop can end any repeat

function cancelRepeats() {
  for (const release of repeats) release();
}

function bindJog(button, joint, dir) {
  let delay = null;
  let repeat = null;
  let pointer = null;  // the one pointer holding this key; others are ignored until it lets go
  const release = () => {
    clearTimeout(delay);
    clearInterval(repeat);
    delay = repeat = null;
    pointer = null;
    button.classList.remove("held");
  };
  repeats.add(release);
  button.addEventListener("pointerdown", (event) => {
    if (event.button !== 0 || button.disabled || pointer !== null) return;
    release();
    pointer = event.pointerId;
    try {
      button.setPointerCapture(event.pointerId);
    } catch {
      // the pointer was already released; pointerup on this key still ends the hold
    }
    button.classList.add("held");
    jog(joint, dir, button);
    delay = setTimeout(() => { repeat = setInterval(() => jog(joint, dir, button), 160); }, 450);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    button.addEventListener(type, (event) => {
      if (event.pointerId === pointer) release();
    });
  }
  button.addEventListener("click", (event) => {
    if (event.detail === 0) jog(joint, dir, button);  // Enter or Space; pointer clicks were handled on pointerdown
  });
}

function readNumbers(ids) {
  const values = {};
  for (const [key, id, name] of ids) {
    const input = $(id);
    const value = input.value.trim() === "" ? NaN : Number(input.value);
    if (!Number.isFinite(value)) {
      input.focus();
      say(`${name} needs a number.`);
      return null;
    }
    values[key] = value;
  }
  return values;
}

const POSE_INPUTS = [["x_mm", "tx", "X"], ["y_mm", "ty", "Y"], ["z_mm", "tz", "Z"], ["r_deg", "tr", "R"]];
const JOINT_INPUTS = JOINTS.map((joint) => [joint.field, `t${joint.field}`, `J${joint.n}`]);

function fillJoints() {
  if (!status?.joints) return;
  for (const joint of JOINTS) $(`t${joint.field}`).value = status.joints[joint.field].toFixed(2);
}

function fillPose() {
  if (!status?.pose) return;
  for (const [key, id] of POSE_INPUTS) $(id).value = status.pose[key].toFixed(2);
}

// ---- Planning: drag a ring or the Z handle in the 3D view to stage a pose; nothing moves until Move ----

const stagedFields = (p) => (p ? JOINTS.map((joint) => joint.field).filter((field) => Number.isFinite(p[field])) : []);

// The plan with one joint staged at `value`, or taken out when that's null or where the arm already is.
function withJoint(base, field, value) {
  const next = {...(base || {})};
  const arm = status?.joints?.[field];
  next[field] = value === null || (Number.isFinite(arm) && Math.abs(value - arm) < PLAN_MIN[field]) ? null : value;
  return stagedFields(next).length ? next : null;
}

function dragAllowed() {
  return Boolean(view && isLive() && status.ready && !armBusy() && !planSending && !planSentAt);
}

function canMove() {
  return Boolean(plan && planCheck.clear && dragAllowed() && !view.isDragging());
}

// Why Move can't send the plan right now, for a refused press.
function whyNotMove() {
  if (!planCheck.clear) return `Not moving: ${planCheck.reason}. Change the plan, or Clear it.`;
  if (!isLive()) return "Not moving: there's no live status from the simulator.";
  if (status.phase === "connected") return "Not moving: the arm needs Initialize first.";
  if (armBusy() || planSentAt) return "Not moving: wait for the arm to settle, or press STOP.";
  return "Not moving: finish the drag first.";
}

function clearPlan(text) {
  if (view) view.cancelDrag();
  plan = null;
  planSentAt = 0;
  if (text) say(text, "ok");
  render();
}

// Sends the whole plan as one joint move, with the joints left out where the arm is now. The plan stays on screen
// until the status shows it as the move's goal.
async function movePlan() {
  if (!plan || planSending) return;
  if (!canMove()) {
    refuse($("planMove"));
    say(whyNotMove());
    return;
  }
  const target = {...status.joints};
  for (const field of stagedFields(plan)) target[field] = plan[field];
  planSending = true;
  render();
  try {
    const reply = await post("/api/move_joints", {...target, speed: detent("speed")});
    if (reply.ok) planSentAt = performance.now();
    else refuse($("planMove"));
  } finally {
    planSending = false;
    render();
  }
}

// The 3D view's callbacks.
function onPreview(p) {
  preview = p;
  showTag(p && {...p, kind: "drag"});
  render();
}

function onHover(p) {
  if (!preview) showTag(p && {...p, kind: "hover"});
}

function onRelease(d) {
  preview = null;
  showTag(null);
  if (d.cancelled) {
    say("Drag cancelled. The plan is unchanged.", "ok");
  } else if (d.changed) {
    plan = withJoint(plan, d.field, d.to);
    const joint = JOINT[d.field];
    const count = stagedFields(plan).length;
    say(plan && Number.isFinite(plan[d.field])
      ? `J${joint.n} planned at ${fmt(d.to, 1)}${joint.unit}${count > 1 ? ` (${count} joints planned)` : ""}. `
        + "Press Move to go there, or Clear."
      : `J${joint.n} is back where the arm is, so it's out of the plan.`, "ok");
  }
  render();
}

// The readout beside the pointer: a handle's name before it's grabbed, then where the drag takes the joint.
function showTag(p) {
  const tag = $("dragTag");
  if (!p) {
    tag.hidden = true;
    return;
  }
  const joint = JOINT[p.field];
  tag.hidden = false;
  if (p.kind === "hover") {
    const now = status?.joints?.[p.field];
    tag.dataset.tone = "hover";
    tag.replaceChildren(`J${joint.n} ${joint.name.toLowerCase()}${joint.n === 3 ? " (Z)" : ""}  `,
      el("span", "now", `${fmt(now)}${joint.unit}`),
      el("span", "how", joint.n === 3 ? "Drag up or down to plan a move" : "Drag around the ring to plan a move"));
  } else {
    const blocked = !p.clear || p.limited;
    const how = !p.clear ? `Blocked: ${p.reason}`
      : p.limited ? `Stops at the soft limit, ${fmt(p.to, 1)}${joint.unit}` : "Release to plan \u00b7 Esc cancels";
    tag.dataset.tone = blocked ? "blocked" : "ok";
    tag.replaceChildren(`J${joint.n}  ${fmt(p.from, 1)}${joint.unit} `, el("span", "to", `\u2192 ${fmt(p.to, 1)}${joint.unit}`),
      el("span", "how", how));
  }
  placeTag(tag, p);
}

// Sets the readout outward from the handle's centre (cx, cy), past the pointer (x, y), on the pointer's side, and
// keeps it on screen.
function placeTag(tag, p) {
  const rect = $("screen").getBoundingClientRect();
  const width = tag.offsetWidth;
  const height = tag.offsetHeight;
  let dx = p.x - p.cx;
  let dy = p.y - p.cy;
  const reach = Math.hypot(dx, dy) || 1;
  dx /= reach;
  dy /= reach;
  const x = p.x - rect.left + dx * 34;
  const y = p.y - rect.top + dy * 34;
  let side = dx < 0 ? "left" : "right";
  if (side === "right" && x + width > rect.width - 8) side = "left";
  else if (side === "left" && x - width < 8) side = "right";
  const left = Math.min(rect.width - 8 - width, Math.max(8, side === "right" ? x : x - width));
  const top = Math.min(rect.height - 8 - height, Math.max(8, y - height / 2));
  tag.style.left = `${left}px`;
  tag.style.top = `${top}px`;
}

// The plan bar: each staged joint (removable), what Move will do, and the Clear and Move keys.
let planChipsKey = "";
function renderPlan(live) {
  const fields = stagedFields(plan);
  $("plan").hidden = !fields.length;
  $("screenHelp").hidden = fields.length > 0;  // the plan bar takes its place
  if (!fields.length) {
    planChipsKey = "";
    return;
  }
  const now = (field) => (live && Number.isFinite(status.joints?.[field]) ? fmt(status.joints[field], 1) : "?");
  const chipsKey = fields.map((field) => `${field}:${plan[field]}:${now(field)}`).join("|");
  if (chipsKey !== planChipsKey) {
    planChipsKey = chipsKey;
    $("planJoints").replaceChildren(...fields.map((field) => {
      const joint = JOINT[field];
      const chip = el("li", "plan-chip");
      chip.dataset.joint = String(joint.n);
      const remove = el("button", "key unstage", "\u00d7");
      remove.type = "button";
      remove.dataset.field = field;
      remove.setAttribute("aria-label", `Take J${joint.n} out of the plan`);
      const reading = el("span", "reading");
      reading.append(el("span", "from", `${now(field)} `), `\u2192 ${fmt(plan[field], 1)}${joint.unit}`);
      chip.append(el("span", "tag", `J${joint.n}`), reading, remove);
      return chip;
    }));
  }
  const speed = detent("speed");
  const pace = fields.every((field) => field === "j3_mm") ? `${speed} mm/s`
    : fields.includes("j3_mm") ? `${speed}\u00b0/s (J3 ${speed} mm/s)` : `${speed}\u00b0/s`;
  const names = listOf(fields.map((field) => `J${JOINT[field].n}`));
  $("planNote").textContent = planSending ? "Sending the plan\u2026" : planSentAt ? "Moving to the plan\u2026"
    : !planCheck.clear ? `Blocked: ${planCheck.reason}.`
    : `Moves ${names}${fields.length > 1 ? " together" : ""} at ${pace}, the joint speed.`;
  $("plan").classList.toggle("blocked", !planCheck.clear);
  $("planMove").disabled = planSending;
  $("planMove").setAttribute("aria-disabled", String(!canMove()));
  $("planClear").disabled = planSending || Boolean(planSentAt);
}

// ---- Building the desk ----

function buildDetents(fieldset, name, values, unit, initial) {
  for (const value of values) {
    const label = el("label");
    const input = el("input");
    input.type = "radio";
    input.name = name;
    input.value = String(value);
    input.checked = value === initial;
    label.append(input, el("span", null, `${value}${unit}`));
    fieldset.append(label);
  }
}

const channels = new Map();

function buildChannels() {
  const host = $("channels");
  for (const joint of JOINTS) {
    const [low, high] = CONFIG.softLimits[joint.field];
    const row = el("div", "channel");
    row.dataset.joint = String(joint.n);
    const tape = el("div", "tape", `J${joint.n} ${joint.name}`);
    tape.append(el("small", null, `${fmt(low, 0)} to ${fmt(high, 0)}${joint.unit}`));
    const minus = el("button", "key jog-key minus", MINUS);
    const plus = el("button", "key jog-key plus", "+");
    minus.type = plus.type = "button";
    minus.setAttribute("aria-label", `J${joint.n} ${joint.name}: one step down`);
    plus.setAttribute("aria-label", `J${joint.n} ${joint.name}: one step up`);
    bindJog(minus, joint, -1);
    bindJog(plus, joint, 1);
    const readout = el("div", "readout");
    const value = el("span", "value", "\u2013");
    const target = el("span", "target quiet", "\u00a0");
    readout.append(value, target);
    const meter = el("div", "meter");
    meter.setAttribute("aria-hidden", "true");
    const slot = el("div", "slot");
    const zero = el("div", "zero");
    const goal = el("div", "goal");
    const cap = el("div", "cap");
    zero.style.left = `${((0 - low) / (high - low)) * 100}%`;
    goal.hidden = true;
    meter.append(slot, zero, goal, cap);
    row.append(tape, minus, readout, plus, meter);
    host.append(row);
    channels.set(joint.field, {joint, low, high, value, target, cap, goal, keys: [minus, plus]});
  }
}

function buildJointFields() {
  const host = $("jointFields");
  for (const joint of JOINTS) {
    const label = el("label", "field");
    const input = el("input");
    input.id = `t${joint.field}`;
    input.type = "number";
    input.step = "0.1";
    input.inputMode = "decimal";
    label.append(el("span", null, `J${joint.n}`), input, el("span", "unit", joint.unit.trim()));
    host.append(label);
  }
}

// ---- Rendering ----

function stateOf(s, live) {
  if (!s) {
    return failedPolls >= 3
      ? {tone: "alarm", word: "NO LINK", line: "The simulator server isn't answering. Start it again: py -3.12 -m zarm_sim.server"}
      : {tone: "idle", word: "CONNECTING", line: "Waiting for the first status report."};
  }
  if (!live) return {tone: "idle", word: "STALE", line: "Status reports are late; the server may be busy."};
  if (s.phase === "connected") {
    return {tone: "stopped", word: "NEEDS INIT", line: "Stopped and uninitialized (an E-stop latches). Press Initialize."};
  }
  if (s.phase !== "initialized") return {tone: "alarm", word: s.phase.toUpperCase(), line: "The simulated arm isn't available."};
  if (s.moving || s.busy) {
    const line = s.goal?.label ?? "Moving.";
    return {tone: "moving", word: "MOVING", line: s.queued ? `${line} \u00b7 next step queued` : line};
  }
  // With a pose staged, the sentence follows the plan.
  const fields = stagedFields(plan);
  if (fields.length && !planSentAt && !planSending) {
    const names = listOf(fields.map((field) => `J${JOINT[field].n}`));
    if (!planCheck.clear) return {tone: "ready", word: "READY", line: `The plan is blocked: ${planCheck.reason}. Change it, or Clear.`};
    return {tone: "ready", word: "READY", line: `${names} planned. Press Move to send ${fields.length > 1 ? "them" : "it"}, or Clear.`};
  }
  return {tone: "ready", word: "READY", line: "Idle. Jog a joint, drag a ring in the 3D view, or send a move."};
}

function renderMessage() {
  const message = $("message");
  const shown = localMessage && (!serverMessage || localMessage.at >= serverMessage.at) ? localMessage : serverMessage;
  if (!shown) return;
  message.textContent = shown.text;
  message.className = shown.level;
}

function renderLog(entries) {
  const last = entries.length ? entries[entries.length - 1].seq : 0;
  if (last === shownLogSeq) return;
  shownLogSeq = last;
  if (entries.length) {
    const latest = entries[entries.length - 1];
    serverMessage = {text: latest.text, level: latest.level, at: performance.now()};
  }
  const list = $("log");
  list.replaceChildren();
  if (!entries.length) {
    list.append(el("li", "empty", "Nothing yet. Commands and SDK refusals appear here."));
    return;
  }
  for (const entry of [...entries].reverse()) {
    const item = el("li", entry.level);
    const time = el("time", null, entry.time);
    item.append(time, el("span", null, entry.text));
    list.append(item);
  }
}

function placeOnMeter(node, channel, value) {
  const fraction = Math.min(1, Math.max(0, (value - channel.low) / (channel.high - channel.low)));
  node.style.left = `${fraction * 100}%`;
}

// The joint rows: where each joint is, and where a drag, the plan or the move under way takes it.
function renderChannels(live, ready, goalJoints) {
  for (const channel of channels.values()) {
    const {field, unit} = channel.joint;
    const value = status?.joints?.[field];
    channel.value.textContent = Number.isFinite(value) ? `${fmt(value)}${unit}` : "\u2013";
    const pv = preview && preview.field === field ? preview : null;
    const planned = plan && Number.isFinite(plan[field]) ? plan[field] : null;
    const aim = pv ? pv.to : planned;
    const goal = goalJoints?.[field];
    const travelling = Number.isFinite(goal) && Number.isFinite(value) && Math.abs(goal - value) > 0.005;
    let text = "\u00a0";
    let kind = "quiet";
    let mark = null;
    if (aim !== null) {
      // A drag in progress is amber, like a held key; a staged plan is grey with a dashed frame; red when blocked.
      const blocked = pv ? !pv.clear || pv.limited : !planCheck.clear;
      text = pv ? `\u2192 ${fmt(aim, 1)}${unit}${pv.limited ? " at limit" : ""}` : `plan \u2192 ${fmt(aim, 1)}${unit}`;
      kind = [pv ? "" : "planned", blocked ? "blocked" : ""].join(" ").trim();
      mark = aim;
    } else if (travelling) {
      text = `\u2192 ${fmt(goal)}${unit}`;
      kind = "";
      mark = goal;
    }
    channel.target.className = `target ${kind}`.trim();
    channel.target.textContent = text;
    channel.goal.hidden = mark === null;
    channel.goal.className = `goal ${kind === "quiet" ? "" : kind}`.trim();
    if (mark !== null) placeOnMeter(channel.goal, channel, mark);
    channel.cap.hidden = !Number.isFinite(value);
    if (Number.isFinite(value)) placeOnMeter(channel.cap, channel, value);
    for (const key of channel.keys) key.disabled = !ready;
  }
}

function render() {
  if (rendering) {  // a drag callback while rendering: run again once this pass is done
    renderAgain = true;
    return;
  }
  rendering = true;
  try {
    renderOnce();
  } finally {
    rendering = false;
  }
  if (renderAgain) {
    renderAgain = false;
    render();
  }
}

function renderOnce() {
  const live = isLive();
  const lamp = $("linkLamp");
  lamp.className = `lamp ${live ? "live" : failedPolls >= 3 ? "dead" : ""}`;
  $("linkText").textContent = live
    ? `Live \u00b7 ${status.backend}`
    : failedPolls >= 3 ? "No answer from the simulator server" : "Connecting to the simulator\u2026";

  // A sent plan hands over to the move's goal once the status shows it (or after a moment, for a move too short to see).
  if (planSentAt && (status?.goal?.joints || performance.now() - planSentAt > HANDOVER_MS)) {
    plan = null;
    planSentAt = 0;
  }
  if (view) {
    const allowed = dragAllowed();
    if (!allowed && view.isDragging()) view.cancelDrag();  // the arm can't take a plan now
    view.setInteractive(allowed);
    if (status) view.update(status);
    planCheck = view.setPlan(plan);
  }

  const state = stateOf(status, live);
  $("stateWindow").dataset.tone = state.tone;
  $("stateWord").textContent = state.word;
  $("stateLine").textContent = state.line;

  const ready = live && status.ready;
  const needsInit = live && status.phase === "connected";
  const init = $("initialize");
  init.disabled = !needsInit;
  init.classList.toggle("ready", live && status.phase === "initialized");
  $("initLabel").textContent = needsInit ? "Initialize" : "Initialized";
  $("estop").disabled = !live || !["initialized", "connected"].includes(status.phase);
  for (const id of ["moveJoints", "movePose", "moveLinear", "changeHand"]) $(id).disabled = !ready || status.busy;
  for (const id of ["jointsFromState", "poseFromState"]) $(id).disabled = !status?.joints;

  renderChannels(live, ready, status?.goal?.jogGoal ?? status?.goal?.joints ?? null);
  renderPlan(live);

  const pose = status?.pose;
  $("poseX").textContent = fmt(pose?.x_mm);
  $("poseY").textContent = fmt(pose?.y_mm);
  $("poseZ").textContent = fmt(pose?.z_mm);
  $("poseR").textContent = fmt(pose?.r_deg);
  $("toolYaw").textContent = fmt(status?.toolYawDeg);
  const check = $("modelCheck");
  if (status?.modelCheck) {
    check.className = status.modelCheck.ok ? "bank-note" : "bank-note bad";
    check.textContent = status.modelCheck.ok
      ? `Model agrees with the SDK's pose (${fmt(status.modelCheck.errorMm, 3)} mm)`
      : `Model and SDK disagree by ${fmt(status.modelCheck.errorMm, 1)} mm: check the arm lengths`;
  }
  $("blindTag").hidden = !status?.blindZone?.hit;

  if (status && !prefilled && status.joints) {
    prefilled = true;
    fillJoints();
    fillPose();
  }
  if (status?.log) renderLog(status.log);
  renderMessage();
}

async function poll() {
  if (polling) {  // one chain of polls: a request made while one is in flight runs right after it
    pollSoon = true;
    return;
  }
  polling = true;
  clearTimeout(pollTimer);
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    status = await response.json();
    failedPolls = 0;
    lastHeard = performance.now();
  } catch {
    failedPolls += 1;
    if (failedPolls >= 3) status = null;  // ride out a single dropped request
  } finally {
    polling = false;
  }
  render();
  pollTimer = setTimeout(poll, pollSoon ? 0 : CONFIG.pollMs);
  pollSoon = false;
}

// ---- The 3D monitor ----

function webglAvailable() {
  try {
    const canvas = document.createElement("canvas");
    return Boolean(window.WebGL2RenderingContext && canvas.getContext("webgl2")) || Boolean(canvas.getContext("webgl"));
  } catch {
    return false;
  }
}

// Shown under the monitor when it draws the URDF's simple shapes because Hitbot's CAD meshes aren't usable here.
function showCadNote(reason) {
  $("cadReason").textContent = reason;
  $("cadCommand").textContent = CONFIG.cadCommand;
  $("cadNote").hidden = false;
}

function startMonitor() {
  const note = $("screenNote");
  if (!CONFIG.three) {
    note.textContent = "No 3D view: it needs three.js, which the server downloads the first time it starts, and that "
      + "download failed (the server's window says why). Check this PC can reach the internet, then start the server "
      + "again. The readouts and controls below still work.";
    return;
  }
  if (!webglAvailable()) {
    note.textContent = "No 3D view: this browser can't use WebGL (it may be turned off, or hardware acceleration may be "
      + "disabled). Try another browser or turn hardware acceleration on. The readouts and controls still work.";
    return;
  }
  import("./view.js")
    .then(({createView}) => createView($("screen"), CONFIG, {
      onPreview,
      onRelease,
      onHover,
      onLost() {
        note.hidden = false;
        note.textContent = "The 3D view stopped because the graphics driver reset. Reload the page to bring it back.";
      },
    }))
    .then((created) => {
      view = created;
      note.hidden = true;
      $("viewReset").disabled = false;
      if (!view.hasCad) {
        showCadNote(view.cadError ? `Simple shapes: the CAD meshes didn't load (${view.cadError}).`
          : CONFIG.meshNote ? `Simple shapes: ${CONFIG.meshNote}.`
          : "Simple shapes: the CAD meshes haven't been made on this PC.");
      }
      render();
    })
    .catch((error) => {
      note.textContent = `The 3D view couldn't start (${error.message}). The readouts and controls still work.`;
    });
}

// ---- Wiring ----

buildDetents($("stepDetents"), "step", CONFIG.steps, "", CONFIG.defaults.jointStep);
buildDetents($("speedDetents"), "speed", CONFIG.jointSpeeds, "", CONFIG.defaults.jointSpeed);
buildDetents($("linearDetents"), "linear", CONFIG.linearSpeeds, "", CONFIG.defaults.linearSpeed);
buildChannels();
buildJointFields();
$("leadSteps").textContent = String(CONFIG.maxLeadSteps);
$("floorZ").textContent = fmt(CONFIG.floorZMm, 0);

function cancelHolds() {
  cancelRepeats();
  if (view) view.cancelDrag();
}

function stopArm() {
  cancelHolds();
  if (planSentAt) {  // a plan on its way is what's being stopped: plan again from where the arm ends up
    plan = null;
    planSentAt = 0;
  }
  post("/api/stop");
}

$("stop").addEventListener("click", stopArm);
$("estop").addEventListener("click", () => {
  cancelHolds();
  if (planSentAt) {
    plan = null;
    planSentAt = 0;
  }
  post("/api/estop");
});
$("initialize").addEventListener("click", () => post("/api/initialize"));
document.addEventListener("keydown", (event) => {
  if (event.key !== "Escape") return;
  event.preventDefault();
  // Esc backs out of a drag in progress, which moves nothing. Otherwise, and whenever the arm is moving, it stops
  // the arm, like the STOP key.
  const cancelled = Boolean(view && view.cancelDrag());
  if (!cancelled || armBusy()) stopArm();
});
window.addEventListener("blur", cancelHolds);  // a key or drag held while the window loses focus never sees its pointerup
document.addEventListener("visibilitychange", () => {
  if (document.hidden) cancelHolds();
});
$("jointsFromState").addEventListener("click", fillJoints);
$("poseFromState").addEventListener("click", fillPose);
$("moveJoints").addEventListener("click", () => {
  const values = readNumbers(JOINT_INPUTS);
  if (values) post("/api/move_joints", {...values, speed: detent("speed")});
});
$("movePose").addEventListener("click", () => {
  const values = readNumbers(POSE_INPUTS);
  if (values) post("/api/move_pose", {...values, hand: $("hand").value, speed: detent("speed")});
});
$("moveLinear").addEventListener("click", () => {
  const values = readNumbers(POSE_INPUTS);
  if (values) post("/api/move_linear", {...values, speed_mm_s: detent("linear")});
});
$("changeHand").addEventListener("click", () => post("/api/change_hand", {speed: detent("speed")}));
$("viewReset").addEventListener("click", () => view?.resetView());
$("planJoints").addEventListener("click", (event) => {
  const button = event.target.closest("button[data-field]");
  if (!button || planSending || planSentAt) return;
  const field = button.dataset.field;
  plan = withJoint(plan, field, null);
  say(`J${JOINT[field].n} taken out of the plan.`, "ok");
  render();
});
$("planClear").addEventListener("click", () => clearPlan("Plan cleared. The arm didn't move."));
$("planMove").addEventListener("click", movePlan);
$("cadCopy").addEventListener("click", async () => {
  const key = $("cadCopy");
  try {
    await navigator.clipboard.writeText(CONFIG.cadCommand);
    key.textContent = "Copied";
  } catch {
    const range = document.createRange();
    range.selectNodeContents($("cadCommand"));
    const selection = window.getSelection();
    selection.removeAllRanges();
    selection.addRange(range);
    key.textContent = "Selected: Ctrl+C";
  }
  setTimeout(() => { key.textContent = "Copy"; }, 2000);
});

// For automated checks (and the console): the page's state and the 3D view.
window.zarmPanel = {view: () => view, plan: () => plan, preview: () => preview, status: () => status, planCheck: () => planCheck};

render();
poll();
startMonitor();
