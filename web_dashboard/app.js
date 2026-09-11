/* UR10 视觉 · 力觉 监控台
 * 连接 scripts/ros_web_bridge.py 的 WebSocket，订阅：
 *   /ft_sensor/wrench       六维力（ATI Net F/T）
 *   /mechmind/color_image   相机彩色（Mech-Eye，服务触发）
 *   /mechmind/depth_map     深度图
 *   /mechmind/point_cloud   点云统计
 */
"use strict";

// ---------------------------------------------------------------- 全局
const WS_PORT = 9090;
const CAPTURE_POLL_MS = 200;        // 检查上一帧完成后立即采下一帧
const AUX_CAPTURE_INTERVAL = 15000; // 深度与点云交替采集间隔 ms
const WRENCH_WINDOW = 10;           // 固定宽度滚动时间窗（秒）
const WRENCH_MAX = 2500;            // 约 12.5 秒原始数据（输入约 200Hz）
const CHART_MAX_POINTS = 1200;      // 绘图抽样上限，避免长窗口拖慢浏览器
const RENDER_HZ = 15;               // 折线图刷新帧率

const $ = (id) => document.getElementById(id);

const chart = echarts.init($("ft-chart"));
const AXES = [
  { key: "fx", label: "Fx", color: "#e74c3c", unit: "N" },
  { key: "fy", label: "Fy", color: "#f39c12", unit: "N" },
  { key: "fz", label: "Fz", color: "#27ae60", unit: "N" },
  { key: "tx", label: "Tx", color: "#2980b9", unit: "N·m" },
  { key: "ty", label: "Ty", color: "#8e44ad", unit: "N·m" },
  { key: "tz", label: "Tz", color: "#16a085", unit: "N·m" },
];

chart.setOption({
  animation: false,
  grid: { left: 46, right: 16, top: 30, bottom: 26 },
  legend: { top: 2, data: AXES.map(a => a.label), textStyle: { fontSize: 11 } },
  tooltip: { trigger: "axis", axisPointer: { type: "line" } },
  xAxis: { type: "value", name: "时间(秒)", nameLocation: "middle", nameGap: 22 },
  yAxis: { type: "value", scale: true, splitLine: { lineStyle: { type: "dashed" } } },
  series: AXES.map(a => ({
    name: a.label, type: "line", showSymbol: false, sampling: "lttb",
    lineStyle: { width: 1.6, color: a.color },
    itemStyle: { color: a.color }, data: [],
  })),
});

// ---------------------------------------------------------------- 状态
const wsBadge = $("ws-badge"), demoBadge = $("demo-badge"), camBadge = $("cam-badge");
let ws = null, wsOk = false;
let demoMode = false;
let cameraOnline = false;           // 相机服务可用
let cameraFrames = 0;

const wrenchBuf = [];               // {t, fx,fy,fz,tx,ty,tz}  t=相对起始秒
let wrenchStart = null;
let lastRealWrench = 0;
const ftStatusEl = $("ft-status"), ftRateEl = $("ft-rate");

function setWs(ok) {
  wsOk = ok;
  wsBadge.textContent = ok ? "已连接" : "已断开";
  wsBadge.className = "badge " + (ok ? "badge-on" : "badge-off");
}
function setCam(ok) {
  cameraOnline = ok;
  camBadge.textContent = ok ? "相机在线" : "相机离线";
  camBadge.className = "badge " + (ok ? "badge-on" : "badge-off");
}
function setDemo(on) {
  demoMode = on;
  demoBadge.classList.toggle("hidden", !on);
}

// ---------------------------------------------------------------- WebSocket
function wsSend(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.hostname}:${WS_PORT}`);
  ws.onopen = () => {
    setWs(true);
    wsSend({ op: "subscribe", topic: "/ft_sensor/wrench_raw", kind: "wrench" });
    wsSend({ op: "subscribe", topic: "/ft_sensor/wrench_compensated", kind: "wrench" });
    wsSend({ op: "subscribe", topic: "/mechmind/color_image", kind: "image" });
    wsSend({ op: "subscribe", topic: "/mechmind/depth_map", kind: "depth" });
    wsSend({ op: "subscribe", topic: "/mechmind/point_cloud", kind: "pcl_stats" });
    $("cam-color-info").textContent = "已订阅，等待相机数据…";
  };
  ws.onclose = () => { setWs(false); setTimeout(connect, 2000); };
  ws.onerror = () => { setWs(false); };
  ws.onmessage = (ev) => handleMessage(JSON.parse(ev.data));
}

// ---------------------------------------------------------------- 消息分发
function handleMessage(m) {
  if (m.op === "pong" || m.op === "subscribed" || m.op === "unsubscribed") return;
  if (m.op === "call_service") { handleServiceReply(m); return; }
  if (!m.topic) return;
  if (m.topic === "/ft_sensor/wrench_raw") onWrench(m.data, "raw");
  else if (m.topic === "/ft_sensor/wrench_compensated") onWrench(m.data, "comp");
  else if (m.topic === "/mechmind/color_image") onColorImage(m.data);
  else if (m.topic === "/mechmind/depth_map") onDepthImage(m.data);
  else if (m.topic === "/mechmind/point_cloud") onPclStats(m.data);
  else if (m.topic === "/ur_link/realtime_state") onRobotState(m.data);
}

// ---------------------------------------------------------------- UR10 实时状态
const JOINTS = [
  ["肩回转", 360], ["肩抬升", 360], ["肘关节", 180],
  ["腕 1", 360], ["腕 2", 360], ["腕 3", 360],
];
const jointGrid = $("joint-grid");
JOINTS.forEach(([name, limit], i) => {
  jointGrid.insertAdjacentHTML("beforeend", `<div class="joint-row" id="joint-${i}">
    <div class="joint-top"><span class="joint-name">J${i + 1} ${name}</span>
    <b class="joint-value" id="joint-value-${i}">--</b><span class="joint-limit">±${limit}°</span></div>
    <div class="limit-track"><span class="limit-fill" id="joint-fill-${i}"></span></div></div>`);
});

let robotState = null;
let sceneYaw = -0.72, scenePitch = 0.48, sceneZoom = 1;
function onRobotState(d) {
  const badge = $("ur-badge");
  if (!d.connected) {
    badge.textContent = "机械臂离线"; badge.className = "badge badge-off";
    $("ur-rate").textContent = d.error || "30003 无数据";
    return;
  }
  robotState = d;
  badge.textContent = "机械臂在线"; badge.className = "badge badge-on";
  $("ur-rate").textContent = `${d.port} ${d.source_hz.toFixed(1)} Hz · 网页 20 Hz · ${d.frame_size} B`;
  d.q.forEach((rad, i) => {
    const deg = rad * 180 / Math.PI, limit = JOINTS[i][1], ratio = Math.abs(deg) / limit;
    $(`joint-value-${i}`).textContent = `${deg >= 0 ? "+" : ""}${deg.toFixed(2)}°`;
    const fill = $(`joint-fill-${i}`), clamped = Math.max(-1, Math.min(1, deg / limit));
    fill.style.left = `${50 + Math.min(0, clamped) * 50}%`;
    fill.style.width = `${Math.abs(clamped) * 50}%`;
    const row = $(`joint-${i}`); row.classList.toggle("warn", ratio >= .8 && ratio < .95); row.classList.toggle("danger", ratio >= .95);
  });
  ["x", "y", "z", "rx", "ry", "rz"].forEach((key, i) => {
    $(`tcp-${key}`).textContent = d.tcp[i].toFixed(i < 3 ? 4 : 3);
  });
  drawTcpScene();
}

function rotvecMatrix(v) {
  const a = Math.hypot(...v); if (a < 1e-9) return [[1,0,0],[0,1,0],[0,0,1]];
  const [x,y,z] = v.map(n => n / a), c = Math.cos(a), s = Math.sin(a), C = 1-c;
  return [[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]];
}
function drawTcpScene() {
  const cv = $("tcp-scene"), ctx = cv.getContext("2d"), rect = cv.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1, w = Math.max(320, rect.width), h = Math.max(220, rect.height);
  cv.width = w*dpr; cv.height = h*dpr; ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,w,h);
  const project = ([x,y,z]) => {
    const cy=Math.cos(sceneYaw), sy=Math.sin(sceneYaw), cp=Math.cos(scenePitch), sp=Math.sin(scenePitch);
    const x1=cy*x-sy*y, y1=sy*x+cy*y, y2=cp*y1-sp*z, z2=sp*y1+cp*z;
    const scale=145*sceneZoom, persp=1/(1+Math.max(-.8,z2)*.18);
    return [w*.5+x1*scale*persp, h*.72-y2*scale*persp];
  };
  const line = (a,b,color,width=1) => { const p=project(a),q=project(b); ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke(); };
  for(let n=-1;n<=1.001;n+=.25){ line([n,-1,0],[n,1,0],"#25303b"); line([-1,n,0],[1,n,0],"#25303b"); }
  line([0,0,0],[1,0,0],"#d95757",2); line([0,0,0],[0,1,0],"#55b86b",2); line([0,0,0],[0,0,1],"#5598df",2);
  [["X",[1,0,0],"#d95757"],["Y",[0,1,0],"#55b86b"],["Z",[0,0,1],"#5598df"]].forEach(([t,p,c])=>{const q=project(p);ctx.fillStyle=c;ctx.fillText(t,q[0]+4,q[1]-4);});
  if (!robotState) return;
  const p=robotState.tcp.slice(0,3), origin=project(p); line([0,0,0],p,"#91a0af",1.5);
  ctx.beginPath();ctx.arc(origin[0],origin[1],5,0,Math.PI*2);ctx.fillStyle="#f2f5f8";ctx.fill();
  const R=rotvecMatrix(robotState.tcp.slice(3)), len=.18, colors=["#ff6b6b","#63d47a","#63a9ef"];
  for(let j=0;j<3;j++) line(p,[p[0]+R[0][j]*len,p[1]+R[1][j]*len,p[2]+R[2][j]*len],colors[j],3);
}
const scene = $("tcp-scene"); let drag = null;
scene.addEventListener("pointerdown", e => { drag=[e.clientX,e.clientY]; scene.setPointerCapture(e.pointerId); });
scene.addEventListener("pointermove", e => { if(!drag)return; sceneYaw+=(e.clientX-drag[0])*.008; scenePitch=Math.max(-1.2,Math.min(1.2,scenePitch+(e.clientY-drag[1])*.008)); drag=[e.clientX,e.clientY]; drawTcpScene(); });
scene.addEventListener("pointerup", () => { drag=null; });
scene.addEventListener("wheel", e => { e.preventDefault(); sceneZoom=Math.max(.55,Math.min(2,sceneZoom*(e.deltaY>0?.9:1.1))); drawTcpScene(); }, {passive:false});

// ---------------------------------------------------------------- 六轴力
const wrenchBuffers = { raw: wrenchBuf, comp: [] };
const wrenchStarts = { raw: null, comp: null };
let wrenchMode = "raw";

function onWrench(d, mode) {
  lastRealWrench = Date.now();
  if (demoMode) setDemo(false);                    // 有真数据就退出演示
  const stamp = d.sec + d.nsec / 1e9;
  if (wrenchStarts[mode] == null) wrenchStarts[mode] = stamp;
  const t = stamp - wrenchStarts[mode];
  const buffer = wrenchBuffers[mode];
  buffer.push({ t, fx: d.fx, fy: d.fy, fz: d.fz, tx: d.tx, ty: d.ty, tz: d.tz });
  while (buffer.length > WRENCH_MAX) buffer.shift();
  if (mode !== wrenchMode) return;
  // 当前值
  $("v-fx").textContent = d.fx.toFixed(2);
  $("v-fy").textContent = d.fy.toFixed(2);
  $("v-fz").textContent = d.fz.toFixed(2);
  $("v-tx").textContent = d.tx.toFixed(3);
  $("v-ty").textContent = d.ty.toFixed(3);
  $("v-tz").textContent = d.tz.toFixed(3);
}

function renderChart() {
  const data = demoMode ? demoWrenchData() : wrenchBuffers[wrenchMode];
  const topic = wrenchMode === "raw" ? "/ft_sensor/wrench_raw" : "/ft_sensor/wrench_compensated";
  if (!data.length) { ftStatusEl.textContent = `等待 ${topic} 数据…`; return; }
  // 只保留时间窗内
  const tEnd = data[data.length - 1].t, tStart = tEnd - WRENCH_WINDOW;
  const seg = data.filter(p => p.t >= tStart);
  const stride = Math.max(1, Math.ceil(seg.length / CHART_MAX_POINTS));
  const plot = seg.filter((_, i) => i % stride === 0 || i === seg.length - 1);
  const series = AXES.map(a => plot.map(p => [p.t, p[a.key]]));
  chart.setOption({
    xAxis: { min: Math.max(0, tStart), max: Math.max(WRENCH_WINDOW, tEnd) },
    series: series.map((s, i) => ({ name: AXES[i].label, data: s })),
  });
  const last = seg[seg.length - 1];
  const label = wrenchMode === "raw" ? "原始" : "重力补偿";
  ftStatusEl.textContent = `${label} · F = (${last.fx.toFixed(1)}, ${last.fy.toFixed(1)}, ${last.fz.toFixed(1)}) N · T = (${last.tx.toFixed(2)}, ${last.ty.toFixed(2)}, ${last.tz.toFixed(2)}) N·m`;
  const duration = Math.max(0, tEnd - seg[0].t);
  ftRateEl.textContent = `${Math.round((seg.length / duration) || 0)} Hz · ${Math.round(duration)}s`;
}

// 演示数据：6 路不同频率的正弦
let demoSeed = 0;
function demoWrenchData() {
  const now = (Date.now() / 1000);
  const t = (now - demoSeed);
  demoSeed = demoSeed || now;
  return [{
    t, fx: 5 * Math.sin(now * 1.3), fy: 4 * Math.sin(now * 0.9 + 1), fz: 62 + 6 * Math.sin(now * 0.6),
    tx: 2.2 + 0.4 * Math.sin(now * 2.1), ty: 2.3 + 0.3 * Math.sin(now * 1.7 + 2), tz: 0.25 * Math.sin(now * 3.1),
  }];
}

setInterval(renderChart, 1000 / RENDER_HZ);

function selectWrenchMode(mode) {
  wrenchMode = mode;
  $("mode-raw").classList.toggle("active", mode === "raw");
  $("mode-comp").classList.toggle("active", mode === "comp");
  const data = wrenchBuffers[mode];
  if (data.length) onWrenchValues(data[data.length - 1]);
}

function onWrenchValues(d) {
  $("v-fx").textContent = d.fx.toFixed(2);
  $("v-fy").textContent = d.fy.toFixed(2);
  $("v-fz").textContent = d.fz.toFixed(2);
  $("v-tx").textContent = d.tx.toFixed(3);
  $("v-ty").textContent = d.ty.toFixed(3);
  $("v-tz").textContent = d.tz.toFixed(3);
}

$("mode-raw").onclick = () => selectWrenchMode("raw");
$("mode-comp").onclick = () => selectWrenchMode("comp");

// ---------------------------------------------------------------- 相机
function drawCanvas(cv, jpeg, infoEl, label) {
  if (!jpeg || jpeg.format !== "jpeg") { drawPlaceholder(cv, "无数据"); return; }
  const img = new Image();
  img.onload = () => {
    cv.width = jpeg.width; cv.height = jpeg.height;
    const ctx = cv.getContext("2d");
    ctx.save();
    ctx.translate(0, cv.height);
    ctx.scale(1, -1);
    ctx.drawImage(img, 0, 0);
    ctx.restore();
    if (infoEl) infoEl.textContent = `${jpeg.width}×${jpeg.height}  ${label}`;
  };
  img.src = "data:image/jpeg;base64," + jpeg.data;
}

function drawPlaceholder(cv, text) {
  const ctx = cv.getContext("2d");
  cv.width = 640; cv.height = 480;
  ctx.fillStyle = "#151a20";
  ctx.fillRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = "#3a4654";
  ctx.font = "20px sans-serif"; ctx.textAlign = "center";
  ctx.fillText(text, cv.width / 2, cv.height / 2);
}

function onColorImage(d) {
  cameraFrames++;
  setCam(true);
  drawCanvas($("cam-color"), d, $("cam-color-info"), "彩色画面");
}
function onDepthImage(d) {
  cameraFrames++;
  setCam(true);
  drawCanvas($("cam-depth"), d, $("cam-depth-info"),
    d.min != null ? `深度 ${d.min}~${d.max} m` : "深度图");
}
function onPclStats(d) {
  const el = $("pcl-info");
  if (d.has_data) {
    el.classList.remove("hidden");
    el.textContent = `点云: ${d.count} 点 · 中心 (${(d.px||0).toFixed(3)}, ${(d.py||0).toFixed(3)}, ${(d.pz||0).toFixed(3)}) m`;
  } else if (d.count) {
    el.classList.remove("hidden");
    el.textContent = `点云: ${d.count} 点（全为无效）`;
  }
}

// 自动采集：只允许一个在途请求，防止慢速工业相机积压服务队列。
let reqId = 0, activeCapture = null;
let nextAuxCapture = Date.now() + AUX_CAPTURE_INTERVAL;
let nextAuxKind = "depth";
function autoCapture() {
  if (!wsOk || activeCapture) return;
  const now = Date.now();
  if (now >= nextAuxCapture) {
    const service = nextAuxKind === "depth" ? "/capture_depth_map" : "/capture_point_cloud";
    nextAuxKind = nextAuxKind === "depth" ? "point_cloud" : "depth";
    nextAuxCapture = now + AUX_CAPTURE_INTERVAL;
    requestCapture(service);
    return;
  }
  requestCapture("/capture_color_image");
}

function requestCapture(service) {
  reqId++;
  activeCapture = { service, requestId: reqId };
  wsSend({ op: "call_service", service, kind: "mecheye_capture", request_id: reqId });
}
function handleServiceReply(m) {
  if (activeCapture && m.request_id === activeCapture.requestId) activeCapture = null;
  if (m.service === "/capture_color_image" || m.service === "/capture_depth_map") {
    if (m.ok) { setCam(true); }
    else { setCam(false); }
  }
  if (m.service === "/ft_sensor/tare") {
    alert("去皮完成: " + (m.ok ? JSON.stringify(m.response) : m.error));
  }
}

$("btn-capture").onclick = () => {
  if (!wsOk) return alert("未连接桥");
  if (!activeCapture) requestCapture("/capture_color_image");
};
$("btn-tare").onclick = () => {
  if (!wsOk) return alert("未连接桥");
  reqId++;
  wsSend({ op: "call_service", service: "/ft_sensor/tare", kind: "trigger", request_id: reqId });
};
setInterval(() => { if ($("auto-capture").checked) autoCapture(); }, CAPTURE_POLL_MS);

// ---------------------------------------------------------------- 演示与看门狗
// 2 秒没有真数据就进演示模式（页面仍可用，标注 DEMO）
setInterval(() => {
  if (!wsOk) return;
  const age = Date.now() - lastRealWrench;
  if (age > 2000 && wrenchBuf.length === 0 && !demoMode) {
    setDemo(true);
    ftStatusEl.textContent = "无真实力数据，正在显示演示波形…";
  }
  if (cameraFrames === 0) {
    drawPlaceholder($("cam-color"), "相机离线 · 等待 Mech-Eye SDK");
    drawPlaceholder($("cam-depth"), "相机离线");
  }
}, 1000);

// 时钟
setInterval(() => { $("clock").textContent = new Date().toLocaleTimeString(); }, 1000);

window.addEventListener("resize", () => { chart.resize(); drawTcpScene(); });
connect();
drawTcpScene();
drawPlaceholder($("cam-color"), "连接中…");
drawPlaceholder($("cam-depth"), "连接中…");
