/* Local six-channel plot: force and torque use separate scales, no CDN. */
window.createWrenchChart = function (host) {
  const colors = ["#e74c3c", "#f39c12", "#27ae60", "#2980b9", "#b874d4", "#16a085"];
  const canvas = document.createElement("canvas"), legend = document.createElement("div"), tip = document.createElement("div");
  host.style.position = "relative";
  canvas.style.cssText = "position:absolute;inset:0;width:100%;height:100%";
  legend.style.cssText = "position:absolute;top:2px;left:45px;right:45px;display:flex;gap:5px;flex-wrap:wrap;z-index:1";
  tip.style.cssText = "display:none;position:absolute;z-index:2;pointer-events:none;padding:7px;background:#0e1116;border:1px solid #52606f;border-radius:5px;font:11px monospace;white-space:pre";
  host.append(canvas, legend, tip);
  const state = {series: [], xAxis: {min: 0, max: 10}}, hidden = new Set();
  let width = 300, height = 220;
  function range(series, torque) {
    if (!document.getElementById("auto-scale").checked) return torque ? [-10, 10] : [-100, 100];
    const values = series.flatMap((s, i) => ((i >= 3) === torque && !hidden.has(s.name)) ? (s.data || []).map(p => p[1]).filter(Number.isFinite) : []);
    if (!values.length) return [-1, 1];
    let low = Math.min(...values), high = Math.max(...values);
    const pad = Math.max((high - low) * .12, torque ? .02 : .2);
    return [low - pad, high + pad];
  }
  function draw() {
    const ratio = Math.min(devicePixelRatio || 1, 2), ctx = canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, width, height);
    const left = 48, right = width - 48, top = 36, bottom = height - 26;
    if (right <= left || bottom <= top) return;
    const force = range(state.series, false), torque = range(state.series, true);
    ctx.font = "10px sans-serif"; ctx.fillStyle = "#8b98a9";
    ctx.fillText("N", 12, 27); ctx.fillText("N·m", right + 9, 27);
    const min = state.xAxis.min || 0, max = state.xAxis.max || 10;
    for (let i = 0; i <= 4; i++) {
      const f = i / 4, y = top + (bottom - top) * f;
      ctx.strokeStyle = "#29323e"; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
      ctx.fillStyle = "#8b98a9"; ctx.textAlign = "right"; ctx.fillText((force[1] - f * (force[1] - force[0])).toFixed(1), left - 5, y + 3);
      ctx.textAlign = "left"; ctx.fillText((torque[1] - f * (torque[1] - torque[0])).toFixed(2), right + 5, y + 3);
      const x = left + (right - left) * f;
      ctx.textAlign = "center"; ctx.fillText((min + (max - min) * f).toFixed(1) + "s", x, bottom + 17);
    }
    ctx.save(); ctx.beginPath(); ctx.rect(left, top, right - left, bottom - top); ctx.clip();
    state.series.forEach((series, i) => {
      if (hidden.has(series.name)) return;
      const bounds = i >= 3 ? torque : force; let started = false;
      ctx.strokeStyle = colors[i]; ctx.lineWidth = 1.5; ctx.beginPath();
      for (const [t, value] of series.data || []) {
        if (!Number.isFinite(t) || !Number.isFinite(value)) { started = false; continue; }
        const x = left + (t - min) / Math.max(max - min, .001) * (right - left);
        const y = bottom - (value - bounds[0]) / (bounds[1] - bounds[0]) * (bottom - top);
        if (!started) ctx.moveTo(x, y); else ctx.lineTo(x, y); started = true;
      }
      ctx.stroke();
    }); ctx.restore();
    canvas.dataset.points = String(state.series.reduce((n, s) => n + (s.data || []).length, 0));
  }
  const chart = {
    setOption(option) {
      if (option.xAxis) state.xAxis = {...state.xAxis, ...option.xAxis};
      if (option.series) {
        state.series = option.series.map((s, i) => ({...(state.series[i] || {}), ...s}));
        if (!legend.children.length) state.series.forEach((s, i) => {
          const button = document.createElement("button"); button.type = "button"; button.textContent = s.name;
          button.style.cssText = `font-size:10px;padding:2px 5px;background:transparent;color:${colors[i]};border:1px solid #2a3441`;
          button.onclick = () => { if (hidden.has(s.name)) hidden.delete(s.name); else hidden.add(s.name); button.style.opacity = hidden.has(s.name) ? .4 : 1; draw(); };
          legend.append(button);
        });
      }
      draw();
    },
    resize() {
      width = Math.max(host.clientWidth, 160); height = Math.max(host.clientHeight, 180);
      const ratio = Math.min(devicePixelRatio || 1, 2); canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio); draw();
    }
  };
  canvas.onmousemove = event => {
    const t = (state.xAxis.min || 0) + Math.max(0, Math.min(1, (event.offsetX - 48) / Math.max(width - 96, 1))) * ((state.xAxis.max || 10) - (state.xAxis.min || 0));
    const lines = [t.toFixed(2) + " s"];
    state.series.forEach((s, i) => { if (!(s.data || []).length) return; const point = s.data.reduce((a, b) => Math.abs(a[0] - t) < Math.abs(b[0] - t) ? a : b); lines.push(s.name + ": " + point[1].toFixed(3) + (i < 3 ? " N" : " N·m")); });
    tip.textContent = lines.join("\n"); tip.style.display = "block"; tip.style.left = Math.min(event.offsetX + 10, width - 140) + "px"; tip.style.top = "35px";
  };
  canvas.onmouseleave = () => { tip.style.display = "none"; };
  new ResizeObserver(() => chart.resize()).observe(host); chart.resize(); return chart;
};
