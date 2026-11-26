import * as THREE from "three";
import { OrbitControls } from "/vendor/OrbitControls.js";
import { STLLoader } from "/vendor/STLLoader.js";
import { solveTcpPosition } from "/tcp-ik.js";

function place(object, frame) {
  object.position.fromArray(frame.xyz);
  object.rotation.set(...frame.rpy, "ZYX");
}

async function visualGeometry(visual, loader) {
  if (visual.kind === "mesh") return loader.loadAsync(visual.url);
  if (visual.kind === "box") return new THREE.BoxGeometry(...visual.size);
  if (visual.kind === "sphere") return new THREE.SphereGeometry(visual.radius, 24, 16);
  if (visual.kind === "cylinder") {
    const body = new THREE.CylinderGeometry(visual.radius, visual.radius, visual.length, 32);
    return body.rotateX(Math.PI / 2);
  }
  throw new Error("不支持的模型几何");
}

class RobotLayer {
  constructor(model, visualGroups, ghostMaterial) {
    this.root = new THREE.Group();
    this.links = new Map();
    for (const [name, visuals] of visualGroups) {
      const link = new THREE.Group();
      for (const { visual, geometry, material } of visuals) {
        const mesh = new THREE.Mesh(geometry, ghostMaterial || material);
        place(mesh, visual);
        if (visual.scale) mesh.scale.fromArray(visual.scale);
        if (ghostMaterial) mesh.renderOrder = 1;
        link.add(mesh);
      }
      this.links.set(name, link);
      this.root.add(link);
    }
    this.joints = [];
    for (const joint of model.joints) {
      const offset = new THREE.Group();
      const motion = new THREE.Group();
      place(offset, joint);
      this.links.get(joint.parent).add(offset);
      offset.add(motion);
      motion.add(this.links.get(joint.child));
      const axis = new THREE.Vector3(...joint.axis).normalize();
      this.joints.push({ ...joint, offset, motion, axis });
    }
  }

  setPose(positions) {
    for (const { name, type, mimic, motion, axis } of this.joints) {
      const position = mimic ? (positions[mimic.joint] ?? 0) * mimic.multiplier + mimic.offset
        : positions[name] ?? 0;
      if (type === "prismatic") motion.position.copy(axis).multiplyScalar(position);
      if (type === "revolute" || type === "continuous") motion.quaternion.setFromAxisAngle(axis, position);
    }
  }

  tipPosition() {
    const tip = this.links.get("end_link");
    return tip?.getWorldPosition(new THREE.Vector3());
  }

  tipText() {
    return this.tipPosition()?.toArray().map((value) => value.toFixed(3)).join("  /  ") ?? "—";
  }
}

export function createRobotView(section, onTcpDraft) {
  const stage = section.querySelector("#robot-stage");
  const notice = section.querySelector("#model-status");
  const actualTip = section.querySelector("#actual-tip");
  const targetTip = section.querySelector("#target-tip");
  const targetLabel = section.querySelector("#model-target-label");
  const showTarget = section.querySelector("#show-robot-target");
  const reset = section.querySelector("#reset-view");
  const tcpHandle = section.querySelector("#tcp-handle");
  const tcpHint = section.querySelector("#tcp-hint");
  let actualLayer;
  let targetLayer;
  let ikLayer;
  let displayedRevision;
  let latest = { feedback: {}, target: {}, label: "", online: false };
  let loading = false;
  let nextAttempt = 0;
  let requestedFrame = 0;
  let assetText = "";
  let viewportWidth = 0;
  let viewportHeight = 0;
  let tcpSelected = false;
  let hasTarget = false;
  let drag = null;
  let dragFrame = 0;
  let ignoreClick = false;
  let controlHeld = false;
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.domElement.setAttribute("role", "img");
  renderer.domElement.setAttribute("aria-label", "B601 实测与目标三维模型");
  stage.prepend(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#0a1924");
  const camera = new THREE.PerspectiveCamera(38, 1, 0.03, 15);
  camera.up.set(0, 0, 1);
  camera.position.set(0.96, -0.80, 0.76);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.32);
  controls.update();
  controls.saveState();
  controls.addEventListener("change", draw);
  reset.addEventListener("click", () => controls.reset());
  showTarget.addEventListener("change", applyPose);
  const grid = new THREE.GridHelper(1.8, 18, "#3e6275", "#203b4b");
  grid.rotation.x = Math.PI / 2;
  scene.add(grid, new THREE.HemisphereLight(0xe8f4ff, 0x6d7f91, 2));
  const key = new THREE.DirectionalLight(0xffffff, 2.0);
  key.position.set(1.8, -1.2, 2.4);
  const fill = new THREE.DirectionalLight(0x9cdbff, 0.9);
  fill.position.set(-1.5, 1.6, 1.0);
  scene.add(key, fill);
  const loadArrows = new Map();
  const ghost = new THREE.MeshBasicMaterial({
    color: "#54def8", transparent: true, opacity: 0.28, depthWrite: false,
    polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1,
  });

  function draw() {
    if (requestedFrame) return;
    requestedFrame = window.requestAnimationFrame(() => {
      requestedFrame = 0;
      const { clientWidth, clientHeight } = stage;
      if (!clientWidth || !clientHeight) return;
      if (viewportWidth !== clientWidth || viewportHeight !== clientHeight) {
        viewportWidth = clientWidth;
        viewportHeight = clientHeight;
        camera.aspect = clientWidth / clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(clientWidth, clientHeight, false);
      }
      renderer.render(scene, camera);
      if (actualLayer) {
        actualTip.textContent = actualLayer.root.visible && !latest.previewPose ? actualLayer.tipText() : "—";
      }
      if (targetLayer) targetTip.textContent = targetLayer.root.visible ? targetLayer.tipText() : "—";
      placeTcpHandle();
    });
  }
  new ResizeObserver(draw).observe(stage);

  function tcpPosition() {
    return (tcpSelected && hasTarget ? targetLayer : actualLayer)?.tipPosition();
  }

  function setTcpSelected(selected) {
    tcpSelected = selected;
    tcpHandle.setAttribute("aria-pressed", String(selected));
    tcpHint.textContent = selected ? "已选中 TCP，按住 Ctrl 拖动目标" : "点选 TCP，再按住 Ctrl 拖动";
    draw();
  }

  function placeTcpHandle() {
    const point = !latest.previewPose && latest.armJoints?.length && actualLayer?.root.visible
      ? tcpPosition() : null;
    if (!point) {
      tcpHandle.hidden = true;
      return;
    }
    const projected = point.project(camera);
    const visible = Math.abs(projected.x) < 1 && Math.abs(projected.y) < 1
      && projected.z > -1 && projected.z < 1;
    tcpHandle.hidden = !visible && !drag;
    if (visible) {
      tcpHandle.style.left = `${(1 + projected.x) * viewportWidth / 2}px`;
      tcpHandle.style.top = `${(1 - projected.y) * viewportHeight / 2}px`;
    }
  }

  function pointerRay(event) {
    const bounds = renderer.domElement.getBoundingClientRect();
    pointer.set(2 * (event.clientX - bounds.left) / bounds.width - 1,
                1 - 2 * (event.clientY - bounds.top) / bounds.height);
    raycaster.setFromCamera(pointer, camera);
    return raycaster.ray;
  }

  function applyDrag() {
    dragFrame = 0;
    if (!drag?.point) return;
    const result = solveTcpPosition(ikLayer, drag.point, drag.seed, latest.armJoints, latest.limits);
    if (!result) {
      tcpHint.textContent = "目标点未找到可达解，保留上次预览";
      return;
    }
    Object.assign(drag.seed, result);
    onTcpDraft(result);
    tcpHint.textContent = "已解算关节角，确认后手动发送";
  }

  function finishDrag(event) {
    if (!drag || event.pointerId !== drag.id) return;
    if (dragFrame) {
      window.cancelAnimationFrame(dragFrame);
      if (event.type === "pointerup") applyDrag();
      dragFrame = 0;
    }
    const id = drag.id;
    drag = null;
    if (tcpHandle.hasPointerCapture(id)) tcpHandle.releasePointerCapture(id);
    draw();
  }

  tcpHandle.addEventListener("click", () => {
    if (!ignoreClick) setTcpSelected(!tcpSelected);
  });
  tcpHandle.addEventListener("pointerdown", (event) => {
    ignoreClick = false;
    if (event.button !== 0 || !(event.ctrlKey || controlHeld) || !tcpSelected || !latest.armJoints?.length) return;
    const anchor = tcpPosition();
    if (!anchor) return;
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(
      camera.getWorldDirection(new THREE.Vector3()), anchor);
    const hit = pointerRay(event).intersectPlane(plane, new THREE.Vector3());
    if (!hit) return;
    event.preventDefault();
    ignoreClick = true;
    drag = { id: event.pointerId, plane, offset: anchor.sub(hit),
             seed: { ...latest.feedback, ...latest.target } };
    tcpHandle.setPointerCapture(event.pointerId);
  });
  tcpHandle.addEventListener("pointermove", (event) => {
    if (!drag || event.pointerId !== drag.id) return;
    if (!(event.ctrlKey || controlHeld)) {
      finishDrag(event);
      return;
    }
    const hit = pointerRay(event).intersectPlane(drag.plane, new THREE.Vector3());
    if (!hit) return;
    drag.point = hit.add(drag.offset);
    if (!dragFrame) dragFrame = window.requestAnimationFrame(applyDrag);
  });
  for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) {
    tcpHandle.addEventListener(event, finishDrag);
  }
  window.addEventListener("keydown", (event) => {
    if (event.key !== "Control") return;
    controlHeld = true;
    if (tcpSelected && !drag) tcpHint.textContent = "Ctrl 已按下，拖动 TCP 目标";
  });
  window.addEventListener("keyup", (event) => {
    if (event.key !== "Control") return;
    controlHeld = false;
    if (tcpSelected && tcpHint.textContent === "Ctrl 已按下，拖动 TCP 目标") {
      tcpHint.textContent = "已选中 TCP，按住 Ctrl 拖动目标";
    }
  });
  window.addEventListener("blur", () => {
    controlHeld = false;
    if (drag) finishDrag({ pointerId: drag.id });
  });

  function applyLoadArrows(hasFeedback) {
    for (const arrow of loadArrows.values()) arrow.visible = false;
    const { load, previewPose } = latest;
    if (!hasFeedback || previewPose || !load) return;
    const frame = actualLayer.links.get(load.frame_id);
    if (!frame) return;
    const rotation = frame.getWorldQuaternion(new THREE.Quaternion());
    for (const [name, reading] of Object.entries(load.estimates)) {
      if (!reading.direction) continue;
      const link = actualLayer.links.get(reading.link);
      if (!link) continue;
      const direction = new THREE.Vector3().fromArray(reading.direction).applyQuaternion(rotation);
      let arrow = loadArrows.get(name);
      if (!arrow) {
        arrow = new THREE.ArrowHelper(direction, new THREE.Vector3(), 0.13, "#ff9d6e", 0.032, 0.022);
        for (const part of [arrow.line, arrow.cone]) {
          part.material.depthTest = false;
          part.renderOrder = 2;
        }
        loadArrows.set(name, arrow);
        scene.add(arrow);
      }
      arrow.position.copy(link.localToWorld(new THREE.Vector3().fromArray(reading.point_xyz_m)));
      arrow.setDirection(direction);
      arrow.visible = true;
    }
  }

  function applyPose() {
    if (!actualLayer) return;
    const { feedback, target, label, online, previewPose } = latest;
    const hasFeedback = actualLayer.joints.every(({ type, name, mimic }) =>
      type === "fixed" || mimic || Number.isFinite(feedback[name]));
    actualLayer.root.visible = hasFeedback || Boolean(previewPose);
    hasTarget = !previewPose && hasFeedback && online && Object.keys(target).length > 0;
    targetLayer.root.visible = hasTarget && showTarget.checked;
    showTarget.disabled = Boolean(previewPose);
    if (actualLayer.root.visible) actualLayer.setPose(previewPose ?? feedback);
    if (hasTarget) targetLayer.setPose({ ...feedback, ...target });
    applyLoadArrows(hasFeedback);
    if (!latest.armJoints?.length && tcpSelected) {
      if (drag) finishDrag({ pointerId: drag.id });
      setTcpSelected(false);
    }
    notice.textContent = previewPose ? "默认姿态预览 · 等待连接后端"
      : !hasFeedback ? "等待关节反馈" : online ? assetText : "反馈中断 · 显示最后一帧";
    targetLabel.textContent = targetLayer.root.visible ? label : "关节目标";
    draw();
  }

  async function loadModel(revision) {
    loading = true;
    nextAttempt = performance.now() + 1800;
    try {
      const response = await fetch(`/api/model?revision=${revision}`);
      if (response.status === 503) {
        notice.textContent = "等待后端发布机械臂模型";
        return;
      }
      if (!response.ok) throw new Error("模型接口不可用");
      notice.textContent = "正在载入机械臂模型…";
      const model = await response.json();
      const loader = new STLLoader();
      const visualGroups = await Promise.all(Object.entries(model.links).map(async ([name, visuals]) =>
        [name, await Promise.all(visuals.map(async (visual) => ({
          visual,
          geometry: await visualGeometry(visual, loader),
          material: new THREE.MeshStandardMaterial({
            color: new THREE.Color(...visual.color).convertSRGBToLinear(), roughness: 0.66,
          }),
        })))],
      ));
      const nextActualLayer = new RobotLayer(model, visualGroups);
      const nextTargetLayer = new RobotLayer(model, visualGroups, ghost);
      const nextIkLayer = new RobotLayer(model, Object.keys(model.links).map((name) => [name, []]));
      if (actualLayer) {
        scene.remove(actualLayer.root, targetLayer.root);
        actualLayer.root.traverse((object) => {
          if (object.isMesh) {
            object.geometry.dispose();
            object.material.dispose();
          }
        });
      }
      actualLayer = nextActualLayer;
      targetLayer = nextTargetLayer;
      ikLayer = nextIkLayer;
      scene.add(actualLayer.root, targetLayer.root);
      displayedRevision = model.revision;
      nextAttempt = 0;
      assetText = model.asset_source === "vendor" ? "B601 官方外观模型" : "B601 简化外观模型";
      reset.disabled = false;
      showTarget.disabled = false;
      section.dataset.ready = "true";
      applyPose();
    } catch (error) {
      notice.textContent = `三维模型加载失败：${error.message}`;
    } finally {
      loading = false;
    }
  }

  return {
    update(next) {
      latest = next;
      applyPose();
      if (next.modelRevision && next.modelRevision !== displayedRevision
          && !loading && performance.now() >= nextAttempt) {
        void loadModel(next.modelRevision);
      }
    },
  };
}
