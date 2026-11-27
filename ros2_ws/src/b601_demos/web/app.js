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
const tcpDistance = element("tcp-distance");
const tcpAngle = element("tcp-angle");
const tcpButtons = document.querySelectorAll("[data-tcp-direction]");
const tcpLabels = { forward: "前", backward: "后", left: "左", right: "右", up: "上", down: "下" };
const stiffnessLabels = { high: "高刚度", low: "低刚度" };
const loadSites = new Map([["gripper", "夹爪"], ["upper_arm", "大臂"], ["forearm", "小臂"], ["wrist", "腕部"]]);
const loadStatuses = new Map([
  ["waiting_node", "等待负载节点"], ["waiting_description", "等待后端模型"],
  ["waiting_feedback", "等待关节反馈"], ["invalid_feedback", "关节反馈不完整"],
  ["uncalibrated", "待空载清零"], ["taring", "正在空载清零"], ["warming_up", "正在采样"],
  ["no_contact", "未检测到外力"], ["ambiguous", "力方向或作用点不确定"],
  ["residual_only", "残差无法拟合所选作用点"], ["unresolved", "作用点未辨识"],
  ["provider_error", "负载 API 不可用"], ["unsupported_backend", "当前后端无适用的估计器"],
]);
const sourceLabels = new Map([
  ["web", "网页"], ["policy", "外部策略"], ["external", "外部接口"], ["recenter", "归中"],
]);
let session;
const heartbeat = new Worker("/session-heartbeat.js");
let shownBackend;
let stiffnessSelectionInitialized = false;
let backendReply;
const rows = new Map();
const edited = new Set();
let snapshot = { feedback: {}, limits: {} };
let busy = false;
let recenterPending = false;
let tcpPending = false;
let tcpReply;
let robotView;
let displayedLoad;
let displayedTcpTarget;

function sourceLabel(source) {
  return sourceLabels.get(source) || source || "无";
}

function refreshRobotView() {
  if (!robotView) return;
  const state = snapshot.status;
  const forwarded = state?.last_forwarded;
  const target = { ...(!state?.holding_after_loss && forwarded?.position) };
  let label = forwarded?.source === "recenter" ? "归中目标" : "最近下发目标";
  if (!recenterPending && !tcpPending && state?.source !== "recenter") {
    for (const name of edited) {
      const input = rows.get(name).number;
      if (Number.isFinite(input.valueAsNumber) && input.checkValidity()) {
        target[name] = input.valueAsNumber;
        label = "待发送预览";
      }
    }
  }
  displayedTcpTarget = { label, dispatch_id: forwarded?.dispatch_id, positions: target };
  robotView.update({
    feedback: snapshot.feedback, target, label, online: !!snapshot.status,
    armJoints: state?.joints, limits: snapshot.limits,
    previewPose: snapshot.preview_pose, modelRevision: snapshot.model_revision, load: snapshot.load,
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

function showLoadFeedback() {
  const load = snapshot.load;
  const list = element("load-readings");
  const positions = load?.positions ?? [];
  element("load-feedback").hidden = !positions.length;
  if (!positions.length) return;
  element("load-frame").textContent = load.frame_id;
  const rows = positions.map((name) => {
    const reading = load.estimates[name];
    const status = loadStatuses.get(reading?.status) || "等待估计";
    const magnitude = reading?.magnitude_n;
    const detail = reading?.direction
      ? reading.force_n.map((value, index) =>
        `${"XYZ"[index]} ${value >= 0 ? "+" : ""}${value.toFixed(2)}`).join("  ")
      : status;
    const value = magnitude == null ? detail : `${magnitude.toFixed(2)} N · ${detail}`;
    return `${loadSites.get(name) || name}  ${value}`;
  });
  const key = rows.join("\n");
  if (key === displayedLoad) return;
  displayedLoad = key;
  list.replaceChildren(...rows.map((text) => {
    const item = document.createElement("li");
    item.textContent = text;
    return item;
  }));
}

function showCompensationSource(kind) {
  const output = element(`${kind}-source`);
  const byJoint = snapshot.compensation_sources?.[kind];
  const joints = Object.keys(byJoint || {});
  const axes = (names) => names.map(
    (name) => name === "finger_left" ? "夹爪" : name.replace(/^joint(\d+)$/, "J$1")).join("、");
  const external = joints.filter((name) => byJoint[name] === "external");
  const disabled = joints.filter((name) => byJoint[name] === "disabled");
  let source = "unknown";
  let label = "待回读";
  if (joints.length && disabled.length === joints.length) {
    source = "disabled";
    label = "已关闭";
  } else if (joints.length && external.length === joints.length) {
    source = "external";
    label = "外部传入";
  } else if (external.length) {
    source = "mixed";
    label = `混合 · 外部 ${axes(external)}`;
  } else if (joints.length) {
    source = "internal";
    label = "内部数据";
  }
  if (disabled.length && source !== "disabled") label += ` · 关闭 ${axes(disabled)}`;
  output.dataset.source = source;
  if (output.textContent !== label) output.textContent = label;
}

function refreshGravity() {
  const choice = snapshot.gravity_selection;
  for (const source of ["internal", "external"]) {
    const button = element(`gravity-${source}`);
    button.disabled = busy || !choice || (source === "external" && !choice.api);
    button.setAttribute("aria-pressed", String(choice?.selected === source));
  }
  const status = element("gravity-selection");
  const pendingSource = choice?.requested === "external" ? "外部" : "内部";
  const detail = choice?.error || (choice?.pending ? `等待${pendingSource}重力补偿` : "");
  const sources = Object.values(snapshot.compensation_sources?.gravity || {}).filter((source) => source !== "disabled");
  const fallback = sources.length > 0 && sources.every((source) => source === "internal");
  const interruption = fallback ? "外部更新中断，已回到内部" : "外部重力流停更";
  const text = choice?.stale ? [interruption, detail].filter(Boolean).join(" · ") : detail;
  status.dataset.error = String(Boolean(choice?.error || choice?.stale));
  if (status.textContent !== text) status.textContent = text;
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
  refreshGravity();
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

function refreshTcp(ready, allowed) {
  const jog = snapshot.tcp_jog;
  const available = allowed && jog?.ready;
  for (const button of tcpButtons) button.disabled = !available;
  tcpDistance.disabled = busy || !ready;
  tcpAngle.disabled = busy || !ready;
  if (jog) tcpDistance.min = jog.min_distance_mm;
  const recording = element("tcp-recording");
  const recordingPath = jog?.recording_path || "";
  recording.hidden = !recordingPath;
  const recordingOutput = element("tcp-recording-path");
  if (recordingOutput.textContent !== recordingPath) recordingOutput.textContent = recordingPath;
  const output = element("tcp-jog-status");
  const message = tcpPending || tcpReply?.error ? tcpReply?.message
    : !ready ? "连接后端后可点动。"
    : !jog?.planner_ready ? "等待 MoveIt 规划服务。"
    : !jog.ready ? "等待 MIT 控制器、关节反馈和安全状态。"
    : !session || snapshot.status?.session !== session.publicId ? "请先指定网页控制权。"
    : tcpReply?.message || "平移保持姿态，旋转保持 TCP 位置。";
  output.dataset.error = String(!!tcpReply?.error);
  if (output.textContent !== message) output.textContent = message;
}

async function jogTcp(button) {
  const { tcpDirection: direction, turnSign } = button.dataset;
  const rotating = turnSign !== undefined;
  const input = rotating ? tcpAngle : tcpDistance;
  if (!input.reportValidity()) return;
  const current = session;
  const amount = input.valueAsNumber;
  const label = rotating ? `绕 ${direction.at(-1).toUpperCase()} 轴${turnSign === "1" ? "正转" : "反转"}`
    : `向${tcpLabels[direction]}点动`;
  const viewBefore = displayedTcpTarget;
  const sequence = ++current.sequence;
  let dispatchId;
  tcpPending = true;
  tcpReply = { message: `正在${label} ${amount} ${rotating ? "°" : "mm"}…` };
  refresh();
  try {
    const result = await api("/api/tcp_jog", {
      session_id: current.id, session: current.publicId, sequence,
      direction, view_before: viewBefore,
      ...(rotating ? { angle_deg: Number(turnSign) * amount } : { distance_mm: amount }),
    });
    current.sequence = Math.max(current.sequence, result.sequence);
    dispatchId = result.dispatch_id;
    const delta = result.delta_xyz_mm.map((value) => value.toFixed(2));
    const angleError = rotating ? ` · 姿态误差 ${result.angular_error_deg.toFixed(2)}°` : "";
    tcpReply = { message: `${label}完成 · 回读 ΔXYZ（mm）${delta.join(" / ")}${angleError}` };
  } catch (error) {
    if (Number.isInteger(error.sequence)) current.sequence = Math.max(current.sequence, error.sequence);
    tcpReply = { error: true, message: error.message };
  } finally {
    tcpPending = false;
    refresh();
    api("/api/tcp_view", {
      session: current.publicId, sequence, dispatch_id: dispatchId, view_after: displayedTcpTarget,
    }).catch(() => {});
  }
}

function refresh() {
  const state = snapshot.status;
  const ready = snapshot.backend?.phase === "connected";
  const allowed = !busy && ready && !!session && state?.session === session.publicId;
  refreshBackend();
  refreshTcp(ready, allowed);
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
  showLoadFeedback();
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
      else if (session.observed) setSession(undefined);
    }
    const last = state?.last_forwarded;
    if (session && last?.session === session.publicId) {
      session.sequence = Math.max(session.sequence, last.sequence);
    }
    if (state?.joints && state.joints.join() !== [...rows.keys()].join()) makeRows(state.joints);
    refresh();
  } catch (error) {
    snapshot.status = null;
    snapshot.backend = null;
    snapshot.stiffness = null;
    snapshot.compensation_sources = null;
    snapshot.gravity_selection = null;
    if (snapshot.load) snapshot.load = { ...snapshot.load, estimates: {} };
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
    error.sequence = result.sequence;
    throw error;
  }
  return result;
}

function setSession(next) {
  session = next;
  heartbeat.postMessage(next ? { id: next.id, period: snapshot.heartbeat_period_ms || 100 } : null);
}

heartbeat.addEventListener("message", ({ data }) => {
  if (session?.id !== data.lost) return;
  setSession(undefined);
  if (!busy) notice("网页控制权已失效，可重新指定网页。", true);
  refresh();
});

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

for (const source of ["internal", "external"]) {
  element(`gravity-${source}`).addEventListener("click", () => mutate(async () => {
    const reply = await api("/api/gravity_source", { source });
    notice(reply.message);
  }));
}

inputSource.addEventListener("input", refresh);
element("acquire").addEventListener("click", () => mutate(async () => {
  const source = inputSource.value.trim();
  await api("/api/authority", { source });
  if (source === "web") {
    if (!session || session.publicId !== snapshot.status?.session) {
      const reply = await api("/api/select", {});
      setSession({ id: reply.session_id, publicId: reply.session, sequence: 0 });
    }
    notice("网页已取得整机控制权。");
  } else {
    setSession(undefined);
    notice(`已指定 ${sourceLabel(source)}，该来源可申请整机控制权。`);
  }
}));
element("release").addEventListener("click", () => mutate(async () => {
  await api("/api/authority", { source: "" });
  setSession(undefined);
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
for (const button of tcpButtons) {
  button.addEventListener("click", () => mutate(() => jogTcp(button)));
}
element("send-all").addEventListener("click", () => mutate(sendTargets));
window.addEventListener("pagehide", () => setSession(undefined));
import("/robot-view.js").then(({ createRobotView }) => {
  robotView = createRobotView(element("robot-view"), applyTcpDraft);
  refreshRobotView();
}).catch((error) => {
  element("model-status").textContent = `三维渲染不可用：${error.message}`;
});
poll();
