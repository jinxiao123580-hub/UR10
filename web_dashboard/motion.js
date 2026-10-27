/* Explicitly armed, bounded native-movel steps. No automatic movement on load. */
(() => {
  const el = id => document.getElementById(id);
  let token = null, busy = false, timer = null;
  let submitting = false, pendingId = null, lastObserved = 0, localError = null, draft = null;
  const buttons = [];
  function error(text) { localError = text; el('motion-result').textContent = text; el('motion-result').className = 'fail'; }
  function motionMode(on) {
    window.ur10MotionArmed = on;
    window.dispatchEvent(new CustomEvent('ur10-motion-mode', {detail: on}));
    if (on) window.dispatchEvent(new CustomEvent('ur10-ik-preview', {detail: null}));
  }
  async function api(action, body = {}) {
    const abort = new AbortController(), timeout = setTimeout(() => abort.abort(), 5000);
    try {
      const response = await fetch('/api/motion/' + action, {method: 'POST', signal: abort.signal, headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
      const data = await response.json(); if (!response.ok || !data.ok) throw new Error(data.error || '点动请求失败'); return data;
    } finally { clearTimeout(timeout); }
  }
  function disabled() { buttons.forEach(button => { button.disabled = !token || busy || submitting; }); el('motion-stop').disabled = !token || !(busy || submitting); }
  function show(data) {
    const job = data.job;
    if (pendingId && job?.id !== pendingId) return;
    if (data.observed_at && data.observed_at < lastObserved) return;
    lastObserved = data.observed_at || lastObserved;
    busy = Boolean(data.busy); disabled();
    const phases = {controller: '核对控制器状态', tcp: '读取当前TCP偏移', plan: `检查完整路径 ${job?.checked_samples || 0}/${job?.total_path_samples || 0}`, path: `检查路径 ${job?.checked_samples || 0}/25`, recheck: '发送前复核状态'};
    const names = {checking: phases[job?.phase] || '正在校验路径', moving: '真机分段运动中', complete: '本次点动已完成', rejected: '路径或状态门禁拒绝', stopped: '本次点动已停止'};
    el('motion-badge').textContent = token ? (busy || submitting ? '点动进行中' : '点动已启用') : '未启用';
    el('motion-badge').className = 'badge ' + (token ? 'badge-on' : '');
    if (draft || !job?.state || (localError && !data.busy)) return;
    el('motion-result').textContent = (names[job.state] || job.state) + (job.segments_total > 1 ? ` · 已完成 ${job.segments_completed}/${job.segments_total} 段 · 当前第 ${job.segment_index || 0} 段` : '') + (job.error ? '：' + job.error : '') + (job.gate ? ` · 已查间隙 ${job.gate.clearance_mm.toFixed(1)} mm` : '') + (job.finished_at ? ` · 总计 ${(job.finished_at - job.started_at).toFixed(1)} s` : '');
    el('motion-result').className = ['rejected', 'stopped'].includes(job.state) ? 'fail' : job.state === 'complete' ? 'pass' : '';
    const axis = job.axis.toUpperCase(), sign = job.direction > 0 ? '+' : '−';
    el('motion-request').textContent = `${job.frame === 'base' ? 'base' : 'TCP'} ${job.kind === 'rotation' ? 'R' : ''}${axis} ${sign}${job.step}${job.kind === 'translation' ? ' mm' : '°'}`;
    const vector = job.kind === 'translation' ? job.actual_delta_mm : job.actual_rotation_delta_deg;
    if (vector) {
      const actual = vector['xyz'.indexOf(job.axis)];
      const errorValue = job.kind === 'translation' ? job.final_error_mm : job.final_error_deg;
      el('motion-actual').textContent = `${actual >= 0 ? '+' : ''}${actual.toFixed(3)} ${job.kind === 'translation' ? 'mm' : '°'}` + (errorValue != null ? ` · 到位差 ${errorValue.toFixed(3)} ${job.kind === 'translation' ? 'mm' : '°'}` : '');
    }
  }
  async function disarm() {
    clearDraft(); const old = token; token = null; clearInterval(timer); el('motion-arm').checked = false; motionMode(false); disabled();
    if (old) try {
      let state = await api('disarm', {token: old}); show(state);
      for (let i = 0; state.busy && i < 15 && !token; i++) {
        await new Promise(resolve => setTimeout(resolve, 150));
        state = await fetch('/api/motion/status', {cache: 'no-store'}).then(response => response.json());
        if (!token) show(state);
      }
    } catch (e) { if (!localError) error(e.message); }
    el('motion-badge').textContent = '未启用';
  }
  el('motion-arm').onchange = async () => {
    if (!el('motion-arm').checked) return disarm();
    el('motion-arm').disabled = true; localError = null; lastObserved = 0; pendingId = null;
    try {
      const data = await api('arm'); token = data.token; busy = false; motionMode(true);
      el('motion-result').className = ''; el('motion-result').textContent = '已启用真机点动，紫色离线预览已清除。选择方向后按请求总量执行。'; show({busy: false});
      const owner = token; let ownerRequest = false;
      timer = setInterval(async () => {
        if (token !== owner || ownerRequest) return; ownerRequest = true;
        try {
          await api('heartbeat', {token: owner});
          if (token !== owner) return;
          const response = await fetch('/api/motion/status', {cache: 'no-store'}), state = await response.json();
          if (token === owner) show(state);
        } catch (e) { if (token === owner) { error(e.message + '；若服务已更新，请刷新页面后重新启用'); await disarm(); } }
        finally { ownerRequest = false; }
      }, 350);
    } catch (e) { el('motion-arm').checked = false; error(e.message); }
    finally { el('motion-arm').disabled = false; disabled(); }
  };
  for (const [kind, labels] of [['translation', ['前 +X', '左 +Y', '上 +Z', '后 −X', '右 −Y', '下 −Z']], ['rotation', ['Rx +', 'Ry +', 'Rz +', 'Rx −', 'Ry −', 'Rz −']]]) {
    labels.forEach((label, i) => {
      const button = document.createElement('button'); button.type = 'button'; button.textContent = label; button.disabled = true; button.dataset.motion = kind;
      button.onclick = async () => {
        if (!token || busy || submitting) return;
        const input = el(kind === 'translation' ? 'motion-distance' : 'motion-angle');
        const step = Number(input.value), maximum = kind === 'translation' ? 50 : 2;
        if (input.value.trim() === '' || !Number.isFinite(step) || step < .5 || step > maximum) {
          error(kind === 'translation' ? '请输入0.5–50 mm的平移总量' : '请输入0.5–2°的单步旋转量'); input.focus(); return;
        }
        const request = {kind, axis: 'xyz'[i % 3], direction: i < 3 ? 1 : -1, step, frame: el('motion-frame').value};
        clearDraft(); localError = null;
        if (kind === 'translation' && step > 5) {
          draft = {request, label: button.textContent};
          el('motion-confirm').hidden = false;
          el('motion-confirm-text').textContent = `确认沿 ${request.frame} ${button.textContent} 移动 ${step} mm？整条路径先检查，再分 ${Math.ceil(step / 4.5)} 段低速执行，每段最多5 mm。`;
          el('motion-result').className = ''; el('motion-result').textContent = '等待确认，尚未开始移动。';
          el('motion-request').textContent = `${request.frame} ${button.textContent} ${step} mm（待确认）`;
          el('motion-actual').textContent = '尚未提交移动';
          return;
        }
        await submit(request, button.textContent);
      };
      buttons.push(button); el('motion-' + kind).append(button);
    });
  }
  function clearDraft() { draft = null; el('motion-confirm').hidden = true; }
  async function submit(request, label) {
    if (!token || busy || submitting) return;
    clearDraft(); localError = null; submitting = true; busy = true; disabled();
    pendingId = crypto.randomUUID ? crypto.randomUUID() : 'jog_' + Date.now() + '_' + Math.random().toString(16).slice(2);
    el('motion-result').className = ''; el('motion-result').textContent = `请求已提交：${label} ${request.step}${request.kind === 'translation' ? ' mm' : '°'}，正在核对状态与路径…`;
    el('motion-badge').textContent = '提交与校验中'; el('motion-request').textContent = `${request.frame} ${label} ${request.step}${request.kind === 'translation' ? ' mm' : '°'}`; el('motion-actual').textContent = '等待实际回读…';
    try {
      await api('heartbeat', {token});
      const data = await api('jog', {...request, token, request_id: pendingId});
      submitting = false; show(data);
    } catch (e) { submitting = false; pendingId = null; busy = true; disabled(); error(e.message); }
  }
  el('motion-confirm-run').onclick = () => {
    if (!draft) return;
    const {request, label} = draft; submit({...request, confirm_large: true}, label);
  };
  el('motion-confirm-cancel').onclick = () => { clearDraft(); localError = '已取消，尚未开始移动。'; el('motion-result').textContent = localError; };
  ['motion-distance', 'motion-angle'].forEach(id => el(id).addEventListener('input', () => { if (draft) { clearDraft(); error('参数已改变，请重新选择方向。'); } }));
  document.querySelectorAll('#motion-presets button').forEach(button => { button.onclick = () => {
    clearDraft(); el('motion-distance').value = button.dataset.mm;
    if (!busy && !submitting) { localError = `平移量已设为 ${button.dataset.mm} mm，请选择方向。`; el('motion-result').textContent = localError; }
  }; });
  el('motion-stop').onclick = async () => { try { localError = null; show(await api('stop', {token})); } catch (e) { error(e.message + '；请使用示教器停止'); } };
  el('motion-frame').onchange = () => {
    if (draft) { clearDraft(); error('坐标系已改变，请重新选择方向。'); }
    const labels = el('motion-frame').value === 'base' ? ['前 +X', '左 +Y', '上 +Z', '后 −X', '右 −Y', '下 −Z'] : ['TCP +X', 'TCP +Y', 'TCP +Z', 'TCP −X', 'TCP −Y', 'TCP −Z'];
    buttons.filter(button => button.dataset.motion === 'translation').forEach((button, i) => { button.textContent = labels[i]; });
  };
  function live(state) {
    if (!state?.connected || !Array.isArray(state.tcp)) return;
    el('motion-live-tcp').textContent = ['X', 'Y', 'Z'].map((axis, i) => `${axis} ${(state.tcp[i] * 1000).toFixed(2)}`).join(' / ');
  }
  window.addEventListener('ur10-realtime-state', event => live(event.detail)); live(window.latestRobotState);
  document.addEventListener('visibilitychange', () => { if (document.hidden && token) disarm(); });
  window.addEventListener('pagehide', () => { if (token) navigator.sendBeacon('/api/motion/disarm', new Blob([JSON.stringify({token})], {type: 'application/json'})); });
})();
