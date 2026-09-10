#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS → WebSocket 桥 —— 把 ROS 2 话题推到浏览器

用途：网页里看相机画面（Mech-Eye）+ 六轴力折线图（ATI Net F/T）。
     rclpy 在后台线程转话题，asyncio 在主线跑 WebSocket 服务。

协议（客户端 → 服务器，JSON）:
    {"op":"subscribe","topic":"/ft_sensor/wrench","kind":"wrench"}
    {"op":"subscribe","topic":"/mechmind/color_image","kind":"image"}
    {"op":"subscribe","topic":"/mechmind/depth_map","kind":"depth"}
    {"op":"subscribe","topic":"/mechmind/point_cloud","kind":"pcl_stats"}
    {"op":"unsubscribe","topic":"/..."}
    {"op":"call_service","service":"/capture_color_image",
     "kind":"mecheye_capture","request_id":1}
    {"op":"call_service","service":"/ft_sensor/tare","kind":"trigger","request_id":2}
    {"op":"ping"}

协议（服务器 → 客户端）:
    {"topic":"/ft_sensor/wrench","data":{fx,fy,fz,tx,ty,tz,sec,nsec}}
    {"topic":"/mechmind/color_image","data":{format:"jpeg",width,height,data:base64}}
    {"topic":"/mechmind/depth_map","data":{format:"jpeg",width,height,data:base64,min,max}}
    {"topic":"/mechmind/point_cloud","data":{count,px,py,pz,has_data}}
    {"ok":true,"op":"call_service","service":...,"response":{...},"request_id":n}
    {"ok":false,"op":"call_service","service":...,"error":"...","request_id":n}
    {"op":"status","topics":{topic:{"kind":kind,"count":N}}}

用法:
    python3 scripts/ros_web_bridge.py                 # WS 9090 + HTTP 8080
    python3 scripts/ros_web_bridge.py --ws-port 9091 --http-port 8081
    python3 scripts/ros_web_bridge.py --no-http       # 只开 WS
"""
import argparse
import asyncio
import base64
import functools
import http.server
import json
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import WrenchStamped

try:
    import websockets
except ImportError:
    raise SystemExit("缺少 websockets，请先：python3 -m pip install --user websockets")

# 梅卡曼德的服务类型（可能没编译成功，导入失败时降级）
try:
    from mecheye_ros_interface.srv import CaptureColorImage, CaptureDepthMap, CapturePointCloud
    MECHEYE_OK = True
except Exception:
    MECHEYE_OK = False
    CaptureColorImage = CaptureDepthMap = CapturePointCloud = None

WS_PORT = 9090
HTTP_PORT = 8080
WEB_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web_dashboard")

import cv2  # noqa: E402


class RosWebBridge(Node):
    def __init__(self, ws_port=WS_PORT, http_port=HTTP_PORT):
        super().__init__("ros_web_bridge")
        self.ws_port = ws_port
        self.http_port = http_port
        self.loop = None
        self.ws_clients = set()
        self._subs = {}
        self._stats = {}
        self._service_clients = {}  # service 名 -> ServiceClient
        if MECHEYE_OK:
            self._service_clients.update({
                "/capture_color_image": self.create_client(
                    CaptureColorImage, "/capture_color_image"),
                "/capture_depth_map": self.create_client(
                    CaptureDepthMap, "/capture_depth_map"),
                "/capture_point_cloud": self.create_client(
                    CapturePointCloud, "/capture_point_cloud"),
            })
        self._service_clients["/ft_sensor/tare"] = self.create_client(
            Trigger, "/ft_sensor/tare")
        self._lock = threading.Lock()
        self._http = None
        print("[桥] rclpy 节点 ros_web_bridge 就绪；Mech-Eye 服务类型: %s"
              % ("可用" if MECHEYE_OK else "不可用(mecheye_ros_interface 未编译)"))

    # ---------------- 推送到所有网页端 ----------------
    def push(self, payload):
        """从 rclpy 回调线程推送到 asyncio 里的所有网页端

        注意 websockets >= 14 改了 API：没有 send_str，只有 await ws.send()，
        所以跨线程必须用 run_coroutine_threadsafe 把协程丢回事件循环。
        """
        if not self.loop:
            return
        text = json.dumps(payload)
        for ws in list(self.ws_clients):
            try:
                asyncio.run_coroutine_threadsafe(_safe_send(ws, text), self.loop)
            except Exception:
                pass

    # ---------------- 订阅（rclpy 回调线程执行） ----------------
    def do_subscribe(self, topic, kind):
        with self._lock:
            if topic in self._subs:
                return "已订阅"
            self._stats[topic] = {"kind": kind, "count": 0}
        cb = None
        if kind == "wrench":
            def cb(m, t=topic):
                self._stats[t]["count"] += 1
                self.push({"topic": t, "data": {
                    "fx": m.wrench.force.x, "fy": m.wrench.force.y, "fz": m.wrench.force.z,
                    "tx": m.wrench.torque.x, "ty": m.wrench.torque.y, "tz": m.wrench.torque.z,
                    "sec": m.header.stamp.sec, "nsec": m.header.stamp.nanosec}})
            sub = self.create_subscription(WrenchStamped, topic, cb, 10)
        elif kind == "image":
            def cb(m, t=topic):
                self._stats[t]["count"] += 1
                enc = _image_to_jpeg(m)
                if enc:
                    self.push({"topic": t, "data": enc})
            sub = self.create_subscription(Image, topic, cb, 1)
        elif kind == "depth":
            def cb(m, t=topic):
                self._stats[t]["count"] += 1
                enc = _depth_to_jpeg(m)
                if enc:
                    self.push({"topic": t, "data": enc})
            sub = self.create_subscription(Image, topic, cb, 1)
        elif kind == "pcl_stats":
            def cb(m, t=topic):
                self._stats[t]["count"] += 1
                self.push({"topic": t, "data": _pcl_stats(m)})
            sub = self.create_subscription(PointCloud2, topic, cb, 2)
        else:
            return "不支持的 kind: %s" % kind
        with self._lock:
            self._subs[topic] = sub
        return "ok"

    def do_unsubscribe(self, topic):
        with self._lock:
            if topic in self._subs:
                self.destroy_subscription(self._subs.pop(topic))
            self._stats.pop(topic, None)
        return "ok"

    # ---------------- 服务调用 ----------------
    def call_service(self, srv_name, kind, request_id):
        loop = self.loop
        fut = loop.create_future()

        if kind == "trigger":
            srv_type = Trigger
            req = Trigger.Request()
        elif kind == "mecheye_capture":
            if not MECHEYE_OK:
                fut.set_result({"ok": False, "error": "mecheye_ros_interface 未编译（SDK 没装？）",
                                "service": srv_name, "request_id": request_id})
                return fut
            srv_map = {"capture_color_image": CaptureColorImage,
                       "capture_depth_map": CaptureDepthMap,
                       "capture_point_cloud": CapturePointCloud}
            base = srv_name.rsplit("/", 1)[-1]
            srv_type = srv_map.get(base)
            if srv_type is None:
                fut.set_result({"ok": False, "error": "未知服务 %s" % srv_name,
                                "service": srv_name, "request_id": request_id})
                return fut
            req = srv_type.Request()
        else:
            fut.set_result({"ok": False, "error": "未知调用类型 %s" % kind,
                            "service": srv_name, "request_id": request_id})
            return fut

        def worker():
            try:
                with self._lock:
                    client = self._service_clients.get(srv_name)
                    if client is None:
                        client = self.create_client(srv_type, srv_name)
                        self._service_clients[srv_name] = client
                if not client.wait_for_service(timeout_sec=4.0):
                    loop.call_soon_threadsafe(fut.set_result, {
                        "ok": False, "error": "服务 %s 不存在或无响应" % srv_name,
                        "service": srv_name, "request_id": request_id})
                    return
                rf = client.call_async(req)

                def service_done(done):
                    try:
                        result = {"ok": True, "response": _dump_resp(done.result()),
                                  "service": srv_name, "request_id": request_id}
                    except Exception as exc:
                        result = {"ok": False, "error": str(exc),
                                  "service": srv_name, "request_id": request_id}
                    loop.call_soon_threadsafe(fut.set_result, result)

                rf.add_done_callback(service_done)
            except Exception as e:
                loop.call_soon_threadsafe(fut.set_result, {
                    "ok": False, "error": str(e), "service": srv_name, "request_id": request_id})

        threading.Thread(target=worker, daemon=True).start()
        return fut

    # ---------------- HTTP 静态页 ----------------
    def start_http(self):
        import http.server
        import functools

        handler = functools.partial(_StaticHandler, directory=WEB_ROOT)
        srv = http.server.ThreadingHTTPServer(("0.0.0.0", self.http_port), handler)
        self._http = srv
        t = threading.Thread(target=srv.serve_forever, daemon=True)
        t.start()
        print("[桥] HTTP 服务 http://127.0.0.1:%d/  （web_dashboard/）" % self.http_port)


class _StaticHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def end_headers(self):
        # 允许网页在本机任意端口加载 CDN 等资源时不出跨域问题
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def _image_to_jpeg(msg, quality=82):
    """sensor_msgs/Image (bgr8) → {format,width,height,data:base64}；失败返回 None"""
    try:
        if msg.encoding not in ("bgr8", "rgb8", "mono8"):
            return {"format": "raw", "encoding": msg.encoding, "width": msg.width,
                    "height": msg.height}
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if msg.encoding == "mono8":
            ok, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        else:
            ok, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        return {"format": "jpeg", "width": msg.width, "height": msg.height,
                "data": base64.b64encode(buf.tobytes()).decode("ascii")}
    except Exception:
        return None


def _depth_to_jpeg(msg):
    """sensor_msgs/Image (32FC1) → 归一化伪彩色 jpeg"""
    try:
        if msg.encoding not in ("32FC1", "32FC3"):
            return {"format": "raw", "encoding": msg.encoding, "width": msg.width,
                    "height": msg.height}
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, -1))
        if msg.encoding == "32FC3":
            arr = arr[:, :, :1]
        v = arr[:, :, 0]
        finite = v[np.isfinite(v) & (v > 0)]
        lo = float(np.percentile(finite, 2)) if finite.size else 0.0
        hi = float(np.percentile(finite, 98)) if finite.size else 1.0
        if hi - lo < 1e-6:
            hi = lo + 1.0
        norm = np.clip((v - lo) / (hi - lo), 0, 1)
        gray = (norm * 255).astype(np.uint8)
        colored = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        ok, buf = cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            return None
        return {"format": "jpeg", "width": msg.width, "height": msg.height,
                "data": base64.b64encode(buf.tobytes()).decode("ascii"),
                "min": round(lo, 3), "max": round(hi, 3)}
    except Exception:
        return None


def _pcl_stats(msg):
    """点云只发统计（网页画不了真 3D，够用）：点数 + 均值中心

    注意：每个字段的字节偏移在 msg.fields[i].offset（不是 msg.offset），
    而且点云里常混有 rgb/intensity 字段，所以必须按字段名取偏移。
    """
    import struct as _st
    try:
        off = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
        if len(off) != 3:
            return {"count": 0, "has_data": False,
                    "error": "点云缺少 x/y/z 字段（有：%s）" % ",".join(f.name for f in msg.fields)}
        n = int(msg.width * msg.height)
        if n == 0:
            return {"count": 0, "has_data": False}
        total = np.zeros(3, dtype=np.float64)
        count = 0
        step = max(1, n // 20000)          # 采样，避免卡死
        data = msg.data
        f32 = _st.Struct("<f")
        for i in range(0, n, step):
            base = i * msg.point_step
            x = f32.unpack_from(data, base + off["x"])[0]
            y = f32.unpack_from(data, base + off["y"])[0]
            z = f32.unpack_from(data, base + off["z"])[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and not (x == 0 and y == 0 and z == 0):
                total += (x, y, z)
                count += 1
        if count:
            px, py, pz = (total / count).tolist()
        else:
            px = py = pz = None
        return {"count": n, "sampled": count, "px": px, "py": py, "pz": pz, "has_data": count > 0}
    except Exception as e:
        return {"count": 0, "has_data": False, "error": str(e)}


def _dump_resp(resp):
    """把响应消息转成 JSON（通用：挑出所有标量字段）"""
    if resp is None:
        return {}
    out = {}
    for name in dir(resp):
        if name.startswith("_"):
            continue
        try:
            v = getattr(resp, name)
            if isinstance(v, (int, float, str, bool)) or v is None:
                out[name] = v
        except Exception:
            pass
    return out


# ---------------- WebSocket 处理 ----------------
async def _safe_send(ws, text):
    """发送失败（客户端已断开等）不抛异常，避免污染其它客户端"""
    try:
        await ws.send(text)
    except Exception:
        pass


async def ws_handler(ws, bridge):
    bridge.ws_clients.add(ws)
    try:
        async for raw in ws:
            try:
                req = json.loads(raw)
            except Exception:
                await ws.send(json.dumps({"op": "error", "error": "JSON 解析失败"}))
                continue
            op = req.get("op")
            if op == "subscribe":
                res = bridge.do_subscribe(req.get("topic"), req.get("kind", "wrench"))
                await ws.send(json.dumps({"op": "subscribed", "topic": req.get("topic"),
                                              "result": res}))
            elif op == "unsubscribe":
                bridge.do_unsubscribe(req.get("topic"))
                await ws.send(json.dumps({"op": "unsubscribed", "topic": req.get("topic")}))
            elif op == "call_service":
                fut = bridge.call_service(req.get("service"), req.get("kind", "trigger"),
                                          req.get("request_id"))
                result = await asyncio.wait_for(asyncio.shield(fut), timeout=20.0)
                result["op"] = "call_service"
                await ws.send(json.dumps(result))
            elif op == "ping":
                await ws.send(json.dumps({"op": "pong"}))
            elif op == "status":
                with bridge._lock:
                    await ws.send(json.dumps({"op": "status", "topics": dict(bridge._stats)}))
    finally:
        bridge.ws_clients.discard(ws)


def rclpy_spin(node):
    rclpy.spin(node)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws-port", type=int, default=WS_PORT)
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--no-http", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    bridge = RosWebBridge(args.ws_port, args.http_port)

    th = threading.Thread(target=rclpy_spin, args=(bridge,), daemon=True)
    th.start()

    if not args.no_http:
        bridge.start_http()

    async def serve():
        bridge.loop = asyncio.get_running_loop()
        print("[桥] WebSocket ws://127.0.0.1:%d/" % args.ws_port)
        async with websockets.serve(functools.partial(ws_handler, bridge=bridge),
                                    "0.0.0.0", args.ws_port,
                                    ping_interval=None, max_size=64 * 1024 * 1024):
            await asyncio.Future()

    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        print("\n[桥] 已退出")


if __name__ == "__main__":
    main()
