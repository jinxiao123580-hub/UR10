#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mech-Eye 3D 相机接入助手（梅卡曼德 Mech-Eye → ROS 2）

把官方流程里**最容易出错的几步自动化**：
  1. 自动发现相机（GVCP 广播，连不在本网段的也能找到）并读出 IP / 型号 / 版本 / 序列号
  2. 检查 SDK 与依赖，缺什么就打印**可直接复制的命令**
  3. 克隆/更新官方 mecheye_ros2_interface
  4. **自动打补丁**：让 camera_ip 参数真正生效（官方源码里按 IP 连接那段是注释掉的，
     且写死了 Version("2.3.4") —— 直接照抄会连不上）
  5. 生成专用 launch 文件（不带 xterm、不带假的 static TF）
  6. colcon 编译 + 启动 + 调 capture_point_cloud 验证点云

用法:
    python3 scripts/setup_mecheye.py --check      # 只做预检，不改任何东西
    python3 scripts/setup_mecheye.py              # 完整流程（SDK 未装时会停下并给指引）
    python3 scripts/setup_mecheye.py --ip 192.168.1.33
    python3 scripts/setup_mecheye.py --verify     # 只做启动+点云验证
"""
import argparse
import os
import re
import socket
import struct
import subprocess
import sys

SDK_DIR = "/opt/mech-mind/mech-eye-sdk"
WS = os.path.expanduser("~/colcon_ws")
PKG_DIR = os.path.join(WS, "src", "mecheye_ros2_interface")
REPO = "https://github.com/MechMindRobotics/mecheye_ros2_interface.git"
SDK_HELP = """
──────────────────────────────────────────────────────────────
 需要先装 Mech-Eye SDK（需官网注册后下载，有国内镜像更快）：

   国内镜像: https://downloads.mech-mind.com.cn/?tab=tab-sdk
   国际站点: https://downloads.mech-mind.com/?tab=tab-sdk

 下载得到 .zip（如 Mech-Eye_API_2.6.0_amd64.zip），然后：

   sudo apt-get install libarchive-tools      # 用于校验 CRC-32
   crc32 Mech-Eye_API_2.6.0_amd64.zip         # 与下载页给的校验码比对
   unzip Mech-Eye_API_2.6.0_amd64.zip         # 解压出 .deb
   sudo dpkg -i Mech-Eye_API_2.6.0_amd64.deb  # 安装
   dpkg -l | grep mecheyeapi                  # 确认装上
   ls /opt/mech-mind/mech-eye-sdk/            # 本脚本就认这个路径

 装完再跑本脚本即可。
──────────────────────────────────────────────────────────────
"""


# ---------------------------------------------------------------- 工具
def run(cmd, **kw):
    return subprocess.run(cmd, shell=isinstance(cmd, str), **kw)


def step(n, total, msg):
    print("\n[%d/%d] %s" % (n, total, msg))


# ---------------------------------------------------------------- 相机发现
def discover_camera(timeout=6.0):
    """GVCP 广播发现 GigE Vision 相机，返回 dict 列表"""
    pkt = bytes([0x42, 0x11, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01])   # DISCOVERY_CMD
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.settimeout(timeout)
    for dst in ("255.255.255.255", "192.168.1.255"):
        try:
            s.sendto(pkt, (dst, 3956))
        except OSError:
            pass
    found, seen = [], set()
    import time
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            data, addr = s.recvfrom(2048)
        except socket.timeout:
            break
        if addr[0] in seen:
            continue
        seen.add(addr[0])
        # 抽可打印字符串：厂商 / 型号 / 版本 / 序列号（顺序固定）
        strs = [x.decode(errors="replace").strip() for x in re.findall(rb"[ -~]{4,}", data)]
        strs = [x for x in strs if x and not x.isdigit()]
        vendor = strs[0] if strs else "?"
        model = strs[1] if len(strs) > 1 else "?"
        # 版本形如 2.5.0；序列号取最后一个"够长且不是厂商/型号/版本"的串
        # （GVCP 应答里厂商名常出现两次，不能按固定下标取序列号）
        ver_re = re.compile(r"^\d+(\.\d+)+$")
        version = next((x for x in strs if ver_re.match(x)), "?")
        serial = next((x for x in reversed(strs)
                       if len(x) >= 8 and x not in (vendor, model) and not ver_re.match(x)), "?")
        ip = None
        m = re.search(rb"\xc0\xa8[\x00-\xff]{2}", data)      # 192.168.x.x 兜底
        if m:
            ip = ".".join(str(b) for b in m.group(0))
        found.append({
            "addr": addr[0],
            "ip": ip or addr[0],
            "mac": ":".join("%02x" % b for b in data[20:26]) if len(data) >= 26 else "?",
            "strings": strs,
            "vendor": vendor,
            "model": model,
            "version": version,
            "serial": serial,
        })
    s.close()
    return found


# ---------------------------------------------------------------- 检查
def check_sdk():
    ok = os.path.isdir(SDK_DIR)
    print("   SDK 安装目录 %s: %s" % (SDK_DIR, "✔ 存在" if ok else "✘ 不存在"))
    if ok:
        for sub in ("include", "lib"):
            p = os.path.join(SDK_DIR, sub)
            if os.path.isdir(p):
                n = len(os.listdir(p))
                print("      %s/ : %d 个文件" % (sub, n))
    return ok


def check_deps():
    need = {
        "libopencv-dev": "OpenCV 开发库",
        "ros-humble-cv-bridge": "ROS 图像桥",
        "libpcl-dev": "PCL 点云库",
        "ros-humble-pcl-conversions": "ROS 点云转换",
        "python3-colcon-common-extensions": "colcon 构建",
    }
    missing = []
    for pkg, desc in need.items():
        r = run(["dpkg", "-l", pkg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        installed = r.returncode == 0
        print("   %-36s %s" % (pkg, "✔ 已装" if installed else "✘ 缺失 (%s)" % desc))
        if not installed:
            missing.append(pkg)
    if missing:
        print("\n   请执行：\n     sudo apt install " + " ".join(missing))
    return not missing


# ---------------------------------------------------------------- 打补丁
MARK_BEGIN = "// >>> UR10-setup"
MARK_END = "// <<< UR10-setup"


def _strip_blocks(text):
    """移除上次插入的补丁块（保证可重复运行）"""
    return re.sub(re.escape(MARK_BEGIN) + r".*?" + re.escape(MARK_END) + r"\n", "", text, flags=re.S)


def patch_source(ip, version):
    """让 camera_ip 参数真正生效（官方源码里按 IP 连接那段是注释掉的）"""
    cpp = os.path.join(PKG_DIR, "src", "MechMindCamera.cpp")
    hdr = os.path.join(PKG_DIR, "include", "MechMindCamera.h")
    for f in (cpp, hdr):
        if not os.path.isfile(f):
            print("   ✘ 找不到 %s" % f)
            return False

    src = _strip_blocks(open(cpp, encoding="utf-8").read())
    h = _strip_blocks(open(hdr, encoding="utf-8").read())

    # ① 头文件加成员
    anchor = "    std::string camera_ip;"
    if anchor not in h:
        print("   ✘ 头文件里找不到 camera_ip 成员（上游代码变了？）")
        return False
    if "camera_fw_version" not in _strip_blocks(h):
        h = h.replace(anchor, anchor + "\n" + MARK_BEGIN +
                      "\n    std::string camera_fw_version;\n" + MARK_END, 1)

    # ② 声明参数
    decl = '    node->declare_parameter<std::string>("camera_ip", "");'
    if decl not in src:
        print("   ✘ 找不到 camera_ip 参数声明（上游代码变了？）")
        return False
    src = src.replace(decl, decl + "\n" + MARK_BEGIN +
                      '\n    node->declare_parameter<std::string>("camera_fw_version", "%s");\n'
                      % version + MARK_END, 1)

    # ③ 读取参数
    getp = '    node->get_parameter("camera_ip", camera_ip);'
    src = src.replace(getp, getp + "\n" + MARK_BEGIN +
                      '\n    node->get_parameter("camera_fw_version", camera_fw_version);\n'
                      + MARK_END, 1)

    # ④ 按 IP 直连（保留官方自动发现作为 else 分支；用 else 接原语句，最小改动）
    call = "    if (!findAndConnect(camera))"
    if call not in src:
        print("   ✘ 找不到 findAndConnect 调用（上游代码变了？）")
        return False
    block = """%(b)s
    // camera_ip 非空 → 按 IP 直连指定相机；为空 → 保持官方默认的自动发现。
    // （官方源码里按 IP 连接那段是注释掉的，且写死 Version("2.3.4")，照抄会连不上）
    if (!camera_ip.empty())
    {
        mmind::eye::CameraInfo info;
        info.ipAddress = camera_ip;
        info.port = 5577;
        if (!camera_fw_version.empty())
        {
            info.firmwareVersion = mmind::eye::Version(camera_fw_version.c_str());
        }
        auto connect_status = camera.connect(info);
        if (!connect_status.isOK())
        {
            throw connect_status;
        }
        std::cout << "按 IP 直连相机成功: " << camera_ip
                  << " (固件 " << camera_fw_version << ")" << std::endl;
    }
    else
%(e)s
""" % {"b": MARK_BEGIN, "e": MARK_END}
    src = src.replace(call, block + call, 1)

    open(cpp, "w", encoding="utf-8").write(src)
    open(hdr, "w", encoding="utf-8").write(h)
    print("   ✔ 已打补丁：camera_ip=%s, 固件版本=%s" % (ip, version))
    print("     （按 IP 直连，不再需要交互式选相机序号）")
    return True


def write_launch(ip, version):
    """生成我们自己的 launch（不带 xterm、不发布假的 static TF）"""
    path = os.path.join(PKG_DIR, "launch", "start_camera_ur10.py")
    content = '''# 由 scripts/setup_mecheye.py 自动生成
# 与官方 start_camera.py 的区别：
#   1. 不用 xterm（官方用 prefix="xterm -e"，会多弹一个窗口）
#   2. 按 IP 直连，不需要手动输入相机序号
#   3. 不发布 map→相机 的假 static TF（那会让点云"看起来"在机器人坐标里但位置是错的）
#      真实变换请用手眼标定结果（easy_handeye2 / tf2_ros static_transform_publisher）
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mecheye_ros_interface",
            executable="start",
            name="mechmind_camera_publisher_service",
            output="screen",
            parameters=[
                {"save_file": False},
                {"camera_ip": "%(ip)s"},
                {"camera_fw_version": "%(ver)s"},
            ],
        ),
    ])
''' % {"ip": ip, "ver": version}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w", encoding="utf-8").write(content)
    print("   ✔ 已生成 %s" % path)
    return path


# ---------------------------------------------------------------- 构建 / 验证
def clone_or_update():
    os.makedirs(os.path.join(WS, "src"), exist_ok=True)
    if os.path.isdir(os.path.join(PKG_DIR, ".git")):
        r = run(["git", "-C", PKG_DIR, "pull", "--ff-only"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print("   " + (r.stdout or "").strip().splitlines()[-1] if r.stdout else "   (无输出)")
    else:
        r = run(["git", "clone", "--depth", "1", REPO, PKG_DIR],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print("   " + (r.stdout or "").strip())
    return os.path.isdir(PKG_DIR)


def build():
    cmd = ("source /opt/ros/humble/setup.bash && cd %s && colcon build "
           "--packages-select mecheye_ros_interface" % WS)
    r = run(["bash", "-c", cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = r.stdout or ""
    tail = "\n".join(out.strip().splitlines()[-8:])
    print("   " + tail.replace("\n", "\n   "))
    if r.returncode != 0:
        print("\n   ✘ 编译失败。常见原因：")
        print("     · Mech-Eye SDK 没装 / 版本不匹配（%s）" % SDK_DIR)
        print("     · 相机固件版本与 SDK 不匹配（本机相机: %s）" % "见上方发现结果")
        return False
    return True


def verify():
    """启动节点 + 调服务采集点云"""
    print("   启动相机节点（后台 20 秒）...")
    launch = os.path.join(PKG_DIR, "launch", "start_camera_ur10.py")
    bg = ("source /opt/ros/humble/setup.bash && source %s/install/setup.bash && "
          "timeout 25 ros2 launch %s > /tmp/mecheye_launch.log 2>&1 &" % (WS, launch))
    run(["bash", "-c", bg])
    import time
    time.sleep(12)

    def sh(c):
        return run(["bash", "-c", "source /opt/ros/humble/setup.bash && source %s/install/setup.bash && %s"
                    % (WS, c)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True).stdout or ""

    topics = sh("ros2 topic list | grep mechmind")
    print("   话题:\n     " + (topics.strip().replace("\n", "\n     ") or "(无)"))
    if not topics.strip():
        print("   ✘ 没有 /mechmind/* 话题。启动日志：")
        try:
            print("     " + open("/tmp/mecheye_launch.log").read()[-800:].replace("\n", "\n     "))
        except OSError:
            pass
        return False

    r = sh("ros2 service call /capture_point_cloud mecheye_ros_interface/srv/CapturePointCloud")
    print("   采集点云: " + ("✔ 服务调用成功" if "successful" in r or "response" in r.lower()
                             else "△ 输出: " + r.strip()[:200]))
    hdr = sh("timeout 8 ros2 topic echo /mechmind/point_cloud --field header --once")
    print("   点云 header:\n     " + (hdr.strip().replace("\n", "\n     ") or "(无)"))
    return True


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=None, help="相机 IP（默认自动发现）")
    ap.add_argument("--check", action="store_true", help="只预检，不改动")
    ap.add_argument("--verify", action="store_true", help="只做启动+点云验证")
    args = ap.parse_args()

    print("=== Mech-Eye 相机接入助手 ===")

    step(1, 6, "发现相机（GVCP 广播）")
    cams = discover_camera()
    if not cams:
        print("   ✘ 没发现任何 GigE Vision 相机")
        print("     检查：① 相机上电  ② 网线在同一个二层网络  ③ 防火墙")
        return 1
    for c in cams:
        print("   ★ %s  %s %s (固件 %s, SN %s)  MAC %s"
              % (c["ip"], c["vendor"], c["model"], c["version"], c["serial"], c["mac"]))
    cam = cams[0]
    if args.ip:
        cam = next((c for c in cams if c["ip"] == args.ip), dict(cam, ip=args.ip))
    ip, version = cam["ip"], cam["version"]

    step(2, 6, "检查 Mech-Eye SDK")
    sdk_ok = check_sdk()

    step(3, 6, "检查依赖")
    deps_ok = check_deps()

    if args.check:
        print("\n=== 预检结论 ===")
        print("   相机: %s ✔" % ip)
        print("   SDK : %s" % ("✔ 已装" if sdk_ok else "✘ 未装 —— 见下面的安装指引"))
        print("   依赖: %s" % ("✔ 齐全" if deps_ok else "✘ 有缺失 —— 见上面的 apt 命令"))
        if not sdk_ok:
            print(SDK_HELP)
        return 0 if (sdk_ok and deps_ok) else 1

    if args.verify:
        return 0 if verify() else 1

    if not sdk_ok:
        print(SDK_HELP)
        return 1

    step(4, 6, "克隆/更新官方 ROS 2 接口")
    if not clone_or_update():
        return 1
    print("   ✔ %s" % PKG_DIR)

    step(5, 6, "打补丁 + 生成 launch")
    if not patch_source(ip, version):
        return 1
    write_launch(ip, version)

    step(6, 6, "colcon 编译")
    if not build():
        return 1
    print("   ✔ 编译通过")

    print("\n=== 完成 ===")
    print("""
 启动相机：
   source /opt/ros/humble/setup.bash && source %s/install/setup.bash
   ros2 launch %s/launch/start_camera_ur10.py

 采集（工业相机是**服务触发**，不是连续推流）：
   ros2 service call /capture_point_cloud         mecheye_ros_interface/srv/CapturePointCloud
   ros2 service call /capture_depth_map           mecheye_ros_interface/srv/CaptureDepthMap
   ros2 service call /capture_color_image         mecheye_ros_interface/srv/CaptureColorImage

 看结果：
   rviz2            # 添加 PointCloud2，话题 /mechmind/point_cloud
   ros2 topic echo /mechmind/point_cloud --field header --once

 注意：点云的 frame_id 是 mechmind_camera/point_cloud，
       要接到机器人坐标里必须做**手眼标定**（见 docs/08 第 3.5 节），
       千万别用官方 launch 里那个 map→相机 的假 static TF。
""" % (WS, PKG_DIR))
    return 0


if __name__ == "__main__":
    sys.exit(main())
