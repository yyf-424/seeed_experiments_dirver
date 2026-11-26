"use strict";

const element = (id) => document.getElementById(id);
const inputSource = element("input-source");
const backendSelect = element("backend");
const mujocoSceneControls = element("mujoco-scene-controls");
const mujocoSceneSelect = element("mujoco-scene");
const stiffnessSwitchingControls = element("stiffness-switching-controls");
const stiffnessSwitchingSelect = element("stiffness-switching");
const connectButton = element("connect");
const disableButton = element("disable");
const recenterButton = element("recenter");
const stiffnessLabels = { high: "高刚度", low: "低刚度" };
const sourceLabels = new Map([
  ["web", "网页"], ["policy", "外部策略"], ["external", "外部接口"], ["recenter", "归中"],
]);
let session;
let shownBackend;
let stiffnessSelectionInitialized = false;
let backendReply;
const rows = new Map();
const edited = new Set();
let snapshot = { feedback: {}, limits: {} };
let busy = false;
let recenterPending = false;
let robotView;

function sourceLabel(source) {
  return sourceLabels.get(source) || source || "无";
}

function refreshRobotView() {
  if (!robotView) return;
  const state = snapshot.status;
  const forwarded = state?.last_forwarded;
  const target = { ...(!state?.holding_after_loss && forwarded?.position) };
  let label = forwarded?.source === "recenter" ? "归中目标" : "最近下发目标";
  if (!recenterPending && state?.source !== "recenter") {
    for (const name of edited) {
      const input = rows.get(name).number;
      if (Number.isFinite(input.valueAsNumber) && input.checkValidity()) {
        target[name] = input.valueAsNumber;
        label = "待发送预览";
      }
    }
  }
  robotView.update({
    feedback: snapshot.feedback, target, label, online: !!snapshot.status,
    armJoints: state?.joints, limits: snapshot.limits,
    previewPose: snapshot.preview_pose, modelRevision: snapshot.model_revision,
  });
}

function applyTcpDraft(positions) {
  for (const [name, value] of Object.entries(positions)) {
    const row = rows.get(name);
    if (!row) continue;
    const { min, max } = snapshot.limits[name];
    setValue(row, Math.max(min, Math.min(max, Number(value.toFixed(5)))));
    edited.add(name);
  }
  refresh();
}

function notice(message, error = false) {
  element("notice").textContent = message.replaceAll("finger_left", "gripper");
  element("notice").dataset.error = String(error);
}

function setValue(row, value) {
  row.number.value = value ?? "";
  if (value !== undefined) row.slider.value = value;
}

function makeRows(names) {
  rows.clear();
  edited.clear();
  const list = element("joints");
  list.replaceChildren();
  for (const name of names) {
    const row = document.createElement("div");
    row.className = "joint";
    const title = document.createElement("span");
    title.className = "joint-name";
    const label = name === "finger_left" ? "gripper" : name;
    title.textContent = label;
    const measured = document.createElement("span");
    measured.className = "measured";
    const slider = document.createElement("input");
    const number = document.createElement("input");
    const unit = name === "finger_left" ? "m" : "rad";
    slider.type = "range";
    slider.step = unit === "m" ? "0.0001" : "0.001";
    number.type = "number";
    number.step = "any";
    for (const input of [slider, number]) {
      input.min = snapshot.limits[name].min;
      input.max = snapshot.limits[name].max;
    }
    slider.setAttribute("aria-label", `${label} 目标滑块 ${unit}`);
    number.setAttribute("aria-label", `${label} 目标 ${unit}`);
    slider.addEventListener("input", () => {
      number.value = slider.value;
      edited.add(name);
      refresh();
    });
    number.addEventListener("input", () => {
      slider.value = number.value;
      edited.add(name);
      refresh();
    });
    const references = document.createElement("div");
    references.className = "joint-references";
    const [velocity, tff] = [["速度", `${unit}/s`], ["任务前馈", unit === "m" ? "N" : "N·m"]]
      .map(([caption, referenceUnit]) => {
        const field = document.createElement("label");
        field.textContent = `${caption} (${referenceUnit})`;
        const input = document.createElement("input");
        input.type = "number";
        input.step = "any";
        input.placeholder = "0";
        input.setAttribute("aria-label", `${label} ${field.textContent}`);
        input.addEventListener("input", () => {
          edited.add(name);
          refresh();
        });
        field.append(input);
        references.append(field);
        return input;
      });
    row.append(title, measured, slider, number, references);
    list.append(row);
    rows.set(name, { measured, slider, number, velocity, tff, unit });
  }
}

function showCompensationSource(kind) {
  const output = element(`${kind}-source`);
  const byJoint = snapshot.compensation_sources?.[kind];
  const joints = Object.keys(byJoint || {});
  const external = joints.filter((name) => byJoint[name] === "external");
  let source = "unknown";
  let label = "待回读";
  if (joints.length && external.length === joints.length) {
    source = "external";
    label = "外部传入";
  } else if (external.length) {
    source = "mixed";
    const axes = external.map((name) => name === "finger_left" ? "夹爪" : name.replace(/^joint(\d+)$/, "J$1"));
    label = `混合 · 外部 ${axes.join("、")}`;
  } else if (joints.length) {
    source = "internal";
    label = "内部数据";
  }
  output.dataset.source = source;
  if (output.textContent !== label) output.textContent = label;
}

function refreshBackend() {
  const backend = snapshot.backend;
  if (backend?.name && backend.name !== shownBackend) backendSelect.value = backend.name;
  shownBackend = backend?.name;
  backendSelect.disabled = busy || !backend?.can_connect;
  if (backend?.mujoco_scene) mujocoSceneSelect.value = backend.mujoco_scene;
  mujocoSceneControls.hidden = backendSelect.value !== "mujoco" || !backend
    || !Object.hasOwn(backend, "mujoco_scene")
    || (backend.name === "mujoco" && !backend.mujoco_scene);
  mujocoSceneSelect.disabled = backendSelect.disabled;
  if (!stiffnessSelectionInitialized && backend?.default_stiffness_switching) {
    stiffnessSwitchingSelect.value = backend.default_stiffness_switching;
    stiffnessSelectionInitialized = true;
  }
  stiffnessSwitchingControls.hidden = backendSelect.value === "mock" || !backend?.can_connect;
  stiffnessSwitchingSelect.disabled = backendSelect.disabled;
  connectButton.disabled = busy || !backend?.can_connect;
  disableButton.disabled = busy || !backend?.can_disable;
  const connection = element("connection");
  connection.dataset.connected = String(backend?.phase === "connected");
  const setting = backend?.stiffness_switching;
  const connectionMessage = backend?.message || "等待连接";
  connection.textContent = setting ? `${connectionMessage} · 软硬切换${setting === "true" ? "开启" : "关闭"}`
    : connectionMessage;
  const feedback = element("backend-feedback");
  feedback.dataset.error = String(!!backendReply?.error || backend?.phase === "error");
  feedback.textContent = backend?.log || backendReply?.message || backend?.disable_reason
    || (backendSelect.value === "dm" && backend?.can_connect ? "连接实机将使能电机，请先确认现场可运动。" : "");
  const stiffness = element("stiffness");
  const mode = (backend?.phase === "connected" && snapshot.stiffness) || "unknown";
  if (stiffness.dataset.mode !== mode) {
    stiffness.dataset.mode = mode;
    stiffness.textContent = stiffnessLabels[mode] || "刚度待回读";
  }
  showCompensationSource("gravity");
  showCompensationSource("friction");
}

async function changeBackend(path, payload) {
  backendReply = undefined;
  try {
    const result = await api(path, payload);
    if (payload.backend === "mock") backendReply = { message: result.message };
  } catch (error) {
    backendReply = { message: error.message, error: true };
  }
}

function refresh() {
  const state = snapshot.status;
  const ready = snapshot.backend?.phase === "connected";
  const allowed = !busy && ready && !!session && state?.session === session.publicId;
  refreshBackend();
  element("authority").textContent = sourceLabel(state?.authority);
  element("source").textContent = sourceLabel(state?.source);
  element("age").textContent = snapshot.feedback_age_ms === null || snapshot.feedback_age_ms === undefined
    ? "—" : `${snapshot.feedback_age_ms} ms`;
  const forwarded = state?.last_forwarded;
  element("applied").textContent = state?.holding_after_loss ? "受控持位"
    : !forwarded ? "—" : state.last_applied_id === forwarded.dispatch_id ? "已采纳" : "等待采纳";
  element("acquire").disabled = busy || !ready || !state || !inputSource.value.trim();
  element("release").disabled = busy || (!state?.authority && !state?.session);
  inputSource.disabled = busy || !ready || !state;
  recenterButton.disabled = busy || !ready || !snapshot.recenter_ready;
  element("sync").disabled = !state?.feedback_ready;
  element("send-all").disabled = !allowed || !edited.size;
  for (const [name, row] of rows) {
    const value = snapshot.feedback[name];
    row.measured.textContent = value === undefined ? "—" : `${value.toFixed(4)} ${row.unit}`;
    if (!edited.has(name) && document.activeElement !== row.number && document.activeElement !== row.slider) {
      const desired = !state?.holding_after_loss && forwarded?.session === state?.session
        ? forwarded?.position[name] : undefined;
      setValue(row, desired ?? value);
    }
  }
  refreshRobotView();
}

async function poll() {
  try {
    const response = await fetch("/api/state");
    if (!response.ok) throw new Error("状态读取失败");
    snapshot = await response.json();
    const state = snapshot.status;
    if (session && state) {
      if (state.session === session.publicId) session.observed = true;
      else if (session.observed) session = undefined;
    }
    if (state?.joints && state.joints.join() !== [...rows.keys()].join()) makeRows(state.joints);
    refresh();
  } catch (error) {
    snapshot.status = null;
    snapshot.backend = null;
    snapshot.stiffness = null;
    snapshot.compensation_sources = null;
    refresh();
    notice(error.message, true);
  } finally {
    window.setTimeout(poll, 100);
  }
}

async function api(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const result = await response.json();
  if (!response.ok) {
    const error = new Error(result.message);
    error.status = response.status;
    throw error;
  }
  return result;
}

function renewSession() {
  const current = session;
  if (current && !current.renewing) {
    current.renewing = true;
    api("/api/heartbeat", { session_id: current.id }).catch((error) => {
      if (error.status === 409 && session === current) {
        session = undefined;
        if (!busy) notice("网页控制权已失效，可重新指定网页。", true);
        refresh();
      }
    }).finally(() => { current.renewing = false; });
  }
  window.setTimeout(renewSession, snapshot.heartbeat_period_ms || 100);
}

async function mutate(task) {
  if (busy) return;
  busy = true;
  refresh();
  try {
    await task();
  } catch (error) {
    notice(error.message, true);
  } finally {
    busy = false;
    refresh();
  }
}

async function sendTargets() {
  const positions = Object.fromEntries([...rows].map(([name, {number: input}]) => {
    if (!Number.isFinite(input.valueAsNumber) || !input.checkValidity()) {
      input.reportValidity();
      throw new Error(`${name} 超出关节范围，或未填写数值。`);
    }
    return [name, input.valueAsNumber];
  }));
  const optional = (field) => {
    const values = {};
    for (const [name, row] of rows) {
      const input = row[field];
      if (!input.checkValidity()) {
        input.reportValidity();
        throw new Error(`${name} 的目标值无效。`);
      }
      if (input.value !== "") values[name] = input.valueAsNumber;
    }
    return values;
  };
  const velocities = optional("velocity");
  const tff = optional("tff");
  const reply = await api("/api/target", {
    session_id: session.id, sequence: ++session.sequence, positions, velocities, tff,
  });
  for (const name of edited) {
    const row = rows.get(name);
    if (row.velocity.valueAsNumber === velocities[name]) row.velocity.value = "";
    if (row.tff.valueAsNumber === tff[name]) row.tff.value = "";
    if (row.number.valueAsNumber === positions[name] && !row.velocity.value && !row.tff.value) {
      edited.delete(name);
    }
  }
  notice(`七关节目标已转发，编号 ${reply.dispatch_id}。`);
}

backendSelect.addEventListener("change", () => {
  backendReply = undefined;
  refreshBackend();
});
mujocoSceneSelect.addEventListener("change", () => {
  backendReply = undefined;
  refreshBackend();
});
connectButton.addEventListener("click", () => mutate(() => {
  const backend = backendSelect.value;
  const payload = { backend };
  if (!mujocoSceneControls.hidden) payload.mujoco_scene = mujocoSceneSelect.value;
  if (backend !== "mock") payload.stiffness_switching = stiffnessSwitchingSelect.value;
  return changeBackend("/api/connect", payload);
}));
disableButton.addEventListener("click", () => mutate(() => changeBackend("/api/disable", {})));

inputSource.addEventListener("input", refresh);
element("acquire").addEventListener("click", () => mutate(async () => {
  const source = inputSource.value.trim();
  await api("/api/authority", { source });
  if (source === "web") {
    if (!session || session.publicId !== snapshot.status?.session) {
      const reply = await api("/api/select", {});
      session = { id: reply.session_id, publicId: reply.session, sequence: 0 };
    }
    notice("网页已取得整机控制权。");
  } else {
    session = undefined;
    notice(`已指定 ${sourceLabel(source)}，该来源可申请整机控制权。`);
  }
}));
element("release").addEventListener("click", () => mutate(async () => {
  await api("/api/authority", { source: "" });
  session = undefined;
  notice("已释放整机控制权。");
}));
recenterButton.addEventListener("click", () => mutate(async () => {
  const current = session?.publicId === snapshot.status?.session ? session : undefined;
  recenterPending = true;
  recenterButton.textContent = "正在归中…";
  notice("正在归中…");
  refreshRobotView();
  try {
    const result = await api("/api/recenter", { session_id: current?.id });
    notice(result.message);
  } finally {
    recenterPending = false;
    recenterButton.textContent = "一键归中";
  }
}));
element("sync").addEventListener("click", () => {
  for (const [name, row] of rows) {
    if (snapshot.feedback[name] === undefined) continue;
    setValue(row, snapshot.feedback[name]);
    edited.add(name);
  }
  refresh();
});
element("send-all").addEventListener("click", () => mutate(sendTargets));
window.addEventListener("pagehide", () => { session = undefined; });
import("/robot-view.js").then(({ createRobotView }) => {
  robotView = createRobotView(element("robot-view"), applyTcpDraft);
  refreshRobotView();
}).catch((error) => {
  element("model-status").textContent = `三维渲染不可用：${error.message}`;
});
poll();
renewSession();
