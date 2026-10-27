/* Offline kinematics panel; all robot motion remains outside this interface. */
(() => {
  "use strict";
  const el = id => document.getElementById(id);
  const keys = ["X (m)", "Y (m)", "Z (m)", "Rx (rad)", "Ry (rad)", "Rz (rad)"];
  keys.forEach((name, i) => {
    const label = document.createElement("label"); label.textContent = name;
    const input = document.createElement("input"); input.type = "number"; input.step = "any"; input.required = true; input.id = `ik-pose-${i}`;
    label.append(input); el("ik-target").append(label);
  });
  let config = null, playback = null, animation = null, pending = false, configRequest = null, generation = 0;
  async function api(path, body) {
    const response = await fetch(`/api/kinematics/${path}`, body ? {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)} : {cache: "no-store"});
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || "请求失败");
    return data;
  }
  function seed() {
    const values = el("ik-seed").value.trim().split(/[\s,，]+/).map(Number);
    if (values.length !== 6 || !values.every(Number.isFinite)) throw new Error("请输入六个有限关节角，单位 rad");
    return values;
  }
  function preview(matrices) { if (matrices && window.ur10MotionArmed) return; window.dispatchEvent(new CustomEvent("ur10-ik-preview", {detail: matrices})); }
  function message(text, failed = false) { el("ik-result").className = failed ? "fail" : ""; el("ik-result").textContent = text; }
  function stop() {
    generation++;
    cancelAnimationFrame(animation); playback = null; preview(null);
    el("ik-pause").disabled = true; el("ik-stop").disabled = true; el("ik-playback").textContent = "";
  }
  async function fill(q, source) {
    stop(); el("ik-seed").value = q.map(x => x.toFixed(8)).join(", ");
    const token = generation;
    const data = await api("fk", {seed_q_rad: q, frame: el("ik-frame").value});
    if (token !== generation) return;
    data.pose_m_rad.forEach((v, i) => { el(`ik-pose-${i}`).value = v.toFixed(9); });
    el("ik-seed-source").textContent = source; message("已填入模型 FK 位姿，可修改目标后计算。");
  }
  function metric(title, value, detail) {
    const card = document.createElement("div"); card.className = "ik-metric";
    for (const [tag, text] of [["span", title], ["b", value], ["small", detail]]) { const node = document.createElement(tag); node.textContent = text; card.append(node); }
    return card;
  }
  async function refresh() {
    try {
      const data = await api("status"); config = data;
      el("ik-health").textContent = data.integrity_ok ? "新安装参数已加载" : "配置变化 · 待重新验收";
      el("ik-health").className = "badge " + (data.integrity_ok ? "badge-on" : "badge-off");
      const independent = data.independent;
      el("ik-summary").replaceChildren(
        metric("手眼标定 · " + data.revision, data.translation_mm.map(v => v.toFixed(2)).join(" / ") + " mm", "相机原点在 tool0 中 · 光轴夹角 " + data.optical_axis_angle_deg.toFixed(3) + "°"),
        metric("全新独立闭环 · 5 组", `${independent.board_origin_rms_mm.toFixed(3)} mm / ${independent.board_orientation_rms_deg.toFixed(3)}°`, "RMS 一致性；非绝对定位精度"),
        metric("离线 IK 验证", `${data.validation.initial_ik_passed} / ${data.validation.tracking_ik_passed} / ${data.validation.camera_target_ik_passed}`, "初始搜索 / 局部跟踪 / 新手眼相机目标"),
        metric("工厂参数模型", `${data.validation.mujoco_trajectory_points} 点 · ${data.validation.watchdog_passed} 项`, "MuJoCo 轨迹 / 缓冲看门狗测试"));
      window.ur10KinematicsConfig = data;
      window.dispatchEvent(new CustomEvent("ur10-kinematics-config", {detail: data}));
      el("ik-caches").replaceChildren();
      data.caches.forEach(cache => {
        const row = document.createElement("div"); row.className = "ik-cache";
        const button = document.createElement("button"); button.textContent = `预览 ${cache.mode} 轨迹 · ${cache.duration_s.toFixed(3)} s`; button.disabled = !data.integrity_ok;
        button.disabled = !data.integrity_ok || Boolean(window.ur10MotionArmed);
        button.onclick = () => play(cache.mode, button);
        const info = document.createElement("small"); info.textContent = `${cache.sample_count} 缓存点 · 最大位置插值误差 ${cache.max_position_error_mm.toFixed(3)} mm · 已查间隙 ${cache.minimum_checked_clearance_mm.toFixed(1)} mm`;
        row.append(button, info); el("ik-caches").append(row);
      });
      el("ik-solve").disabled = !data.integrity_ok || Boolean(window.ur10MotionArmed);
      if (!el("ik-seed").value) await fill(data.example_seed_q_rad, "离线验证示例 · 不是当前真机姿态");
    } catch (error) { el("ik-health").textContent = "参数服务不可用"; el("ik-health").className = "badge badge-off"; message(error.message, true); el("ik-solve").disabled = true; }
  }
  el("ik-live").onclick = async () => {
    try { const state = window.latestRobotState; if (!state?.connected || Date.now() / 1000 - state.timestamp > 1) throw new Error("没有新鲜的真机关节回读"); await fill(state.q, "真机关节快照 · 模型 FK"); } catch (e) { message(e.message, true); }
  };
  el("ik-example").onclick = async () => { try { if (!config) throw new Error("参数尚未加载"); await fill(config.example_seed_q_rad, "离线验证示例 · 不是当前真机姿态"); } catch (e) { message(e.message, true); } };
  el("ik-frame").onchange = async () => {
    el("ik-mode").disabled = el("ik-frame").value === "camera"; if (el("ik-mode").disabled) el("ik-mode").value = "pose";
    try { await fill(seed(), "已按所选坐标系重算目标"); } catch (e) { message(e.message, true); }
  };
  el("ik-form").onsubmit = async event => {
    event.preventDefault(); if (pending) return; if (window.ur10MotionArmed) return message("请先关闭真机点动，再使用离线预览"); pending = true; stop(); el("ik-solve").disabled = true;
    const token = generation;
    try {
      const target = keys.map((_, i) => el(`ik-pose-${i}`).value === "" ? NaN : Number(el(`ik-pose-${i}`).value));
      if (!target.every(Number.isFinite)) throw new Error("目标位姿须为六个有限数值");
      message("正在计算离线 IK 与选定碰撞对…");
      const data = await api("solve", {seed_q_rad: seed(), target_base_pose_m_rad: target, frame: el("ik-frame").value, mode: el("ik-mode").value});
      if (token !== generation) return;
      const r = data.result; el("ik-result").replaceChildren(); el("ik-result").className = r.passes_offline_gates ? "pass" : "fail";
      const title = document.createElement("strong"); title.textContent = !r.success ? "IK 未收敛" : r.passes_offline_gates ? "离线检查通过 · 已显示紫色预览" : "IK 收敛，碰撞门禁未通过";
      const info = document.createElement("p"); info.textContent = `工具任务误差 ${(r.position_error_m * 1000).toFixed(4)} mm / ${(r.orientation_error_rad * 180 / Math.PI).toFixed(4)}° · 已查间隙 ${r.minimum_checked_clearance_mm.toFixed(1)} mm · 种子最大变化 ${r.max_seed_step_deg.toFixed(3)}°`;
      const table = document.createElement("table"); const head = table.insertRow(); ["关节", "目标 rad", "目标 °"].forEach(t => { const th = document.createElement("th"); th.textContent = t; head.append(th); });
      r.q.forEach((v, i) => { const row = table.insertRow(); [`J${i + 1}`, v.toFixed(6), (v * 180 / Math.PI).toFixed(3)].forEach(text => { row.insertCell().textContent = text; }); });
      el("ik-result").append(title, info, table); if (r.passes_offline_gates) preview(data.link_world_matrices);
    } catch (error) { message(error.message, true); }
    finally { pending = false; el("ik-solve").disabled = !config?.integrity_ok || Boolean(window.ur10MotionArmed); }
  };
  async function play(mode, button) {
    if (window.ur10MotionArmed) return message("请先关闭真机点动，再使用离线预览");
    stop(); button.disabled = true;
    const token = generation;
    try {
      const data = await api("cache?mode=" + mode);
      if (token !== generation) return;
      playback = {data, started: performance.now(), paused: false, index: 0};
      el("ik-pause").disabled = false; el("ik-stop").disabled = false; el("ik-pause").textContent = "暂停预览";
      function tick(now) {
        if (!playback || token !== generation) return;
        const state = playback; const t = state.paused ? state.elapsed : Math.min(data.duration_s, (now - state.started) / 1000);
        while (state.index + 1 < data.frames.length && data.frames[state.index + 1].t_s <= t) state.index++;
        preview(data.frames[state.index].link_world_matrices);
        el("ik-playback").textContent = `${mode} 紫色预览 ${t.toFixed(2)} / ${data.duration_s.toFixed(2)} s`;
        if (t >= data.duration_s) { playback = null; el("ik-pause").disabled = true; return; }
        animation = requestAnimationFrame(tick);
      }
      animation = requestAnimationFrame(tick);
    } catch (error) { message(error.message, true); } finally { button.disabled = !config?.integrity_ok || Boolean(window.ur10MotionArmed); }
  }
  el("ik-pause").onclick = () => { if (!playback) return; const now = performance.now(); if (playback.paused) playback.started = now - playback.elapsed * 1000; else playback.elapsed = (now - playback.started) / 1000; playback.paused = !playback.paused; el("ik-pause").textContent = playback.paused ? "继续预览" : "暂停预览"; };
  el("ik-clear").onclick = stop; el("ik-stop").onclick = stop;
  el("ik-refresh").onclick = () => { if (!configRequest) configRequest = refresh().finally(() => { configRequest = null; }); };
  window.addEventListener("ur10-motion-mode", event => { if (event.detail) stop(); el("ik-solve").disabled = event.detail || !config?.integrity_ok; document.querySelectorAll("#ik-caches button").forEach(button => { button.disabled = event.detail || !config?.integrity_ok; }); });
  el("ik-solve").disabled = true; refresh();
})();
