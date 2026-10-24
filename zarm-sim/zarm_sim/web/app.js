// The simulator desk: polls the server's status about 30 times a second, sends commands with the page's token, and
// hands the arm's state to the 3D monitor (view.js), loaded only when three.js and WebGL are both available.
const CONFIG = JSON.parse(document.getElementById("zarm-config").textContent);
const $ = (id) => document.getElementById(id);
const MINUS = "\u2212";
const JOINTS = [
  {n: 1, field: "j1_deg", name: "Shoulder", unit: "\u00b0"},
  {n: 2, field: "j2_deg", name: "Elbow", unit: "\u00b0"},
  {n: 3, field: "j3_mm", name: "Vertical", unit: " mm"},
  {n: 4, field: "j4_deg", name: "Wrist R", unit: "\u00b0"},
];
const FRESH_MS = 1000;

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
  return {tone: "ready", word: "READY", line: "Idle. Jog a joint or send a move."};
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

function render() {
  const live = Boolean(status) && performance.now() - lastHeard < FRESH_MS;
  const lamp = $("linkLamp");
  lamp.className = `lamp ${live ? "live" : failedPolls >= 3 ? "dead" : ""}`;
  $("linkText").textContent = live
    ? `Live \u00b7 ${status.backend}`
    : failedPolls >= 3 ? "No answer from the simulator server" : "Connecting to the simulator\u2026";

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

  const goalJoints = status?.goal?.jogGoal ?? status?.goal?.joints ?? null;
  for (const channel of channels.values()) {
    const field = channel.joint.field;
    const value = status?.joints?.[field];
    channel.value.textContent = Number.isFinite(value) ? `${fmt(value)}${channel.joint.unit}` : "\u2013";
    const goal = goalJoints?.[field];
    const travelling = Number.isFinite(goal) && Number.isFinite(value) && Math.abs(goal - value) > 0.005;
    channel.target.className = travelling ? "target" : "target quiet";
    channel.target.textContent = travelling ? `\u2192 ${fmt(goal)}${channel.joint.unit}` : "\u00a0";
    channel.goal.hidden = !travelling;
    if (travelling) placeOnMeter(channel.goal, channel, goal);
    channel.cap.hidden = !Number.isFinite(value);
    if (Number.isFinite(value)) placeOnMeter(channel.cap, channel, value);
    for (const key of channel.keys) key.disabled = !ready;
  }

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
  if (view && status) view.update(status);
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
      onLost() {
        note.hidden = false;
        note.textContent = "The 3D view stopped because the graphics driver reset. Reload the page to bring it back.";
      },
    }))
    .then((created) => {
      view = created;
      note.hidden = true;
      $("viewReset").disabled = false;
      if (view.hasCad) {
        const keys = $("modelKeys");
        keys.hidden = false;
        view.setCad(true);
        for (const key of keys.querySelectorAll("button")) {
          key.addEventListener("click", () => {
            const cad = key.dataset.mode === "cad";
            for (const other of keys.querySelectorAll("button")) {
              other.setAttribute("aria-pressed", String(other === key));
            }
            view.setCad(cad);
          });
        }
      }
      if (status) view.update(status);
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

function stopArm() {
  cancelRepeats();
  post("/api/stop");
}

$("stop").addEventListener("click", stopArm);
$("estop").addEventListener("click", () => {
  cancelRepeats();
  post("/api/estop");
});
$("initialize").addEventListener("click", () => post("/api/initialize"));
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") {
    event.preventDefault();
    stopArm();
  }
});
window.addEventListener("blur", cancelRepeats);  // a key held while the window loses focus never sees its pointerup
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
if (CONFIG.meshNote) $("modelCheck").textContent = CONFIG.meshNote;

render();
poll();
startMonitor();
