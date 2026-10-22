# 硬笔TCP与写字原型（2026-10-16）

## 状态和证据

- `config/hard_pen_tcp.yaml`：法兰到笔尖`[0.113898,-1.661301,329.069491] mm`，仅位置标定，不写控制器TCP。
- `outputs/tcp_calibration/ballpoint-pen-20261016.json`：9组训练、3组留出；训练RMS0.347mm、留出最大0.991mm，是固定点一致性，不是外部绝对定位精度。
- `outputs/paper_writing/a4-imported-20261016.json`：三个无序角仅定义XY，Z不定义写字面。长边按基座+X（竖直时+Y）推断，纸无标记存在180°歧义，先空走核对。
- `writing-20261016-143507-030047-contact.json`记录Z26.745mm触纸后抬2mm，同名前缀主JSON写至第三笔时5N保护停止，不证明完整字迹验收。
- `writing-20261016-144931-263967.json`记录`completed_fixed_height_motion`及对应触纸成功，只证明运动序列完成，不是后续修改或字迹质量验收。12点采集历史在`outputs/paper_calibration/`，单点高度不能纠正倾斜。

## 环境与操作

依赖ROS2 Humble、系统NumPy/SciPy；真机还需rclpy、geometry_msgs、Pinocchio和既有UR10模型。保留全部相邻脚本及模型，不再依赖私有分析脚本。硬件依赖不自动安装。

```bash
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
bash scripts/start_dashboard.sh

# 无硬件测试
PYTHONNOUSERSITE=1 python3 scripts/test_write_ballpoint.py
PYTHONNOUSERSITE=1 python3 scripts/test_paper_plane_auto.py

# 换纸示教任意三个不同角，使用新文件
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/write_ballpoint.py --teach \
--registration outputs/paper_writing/a4-new.json

# 仅离线规划
PYTHONNOUSERSITE=1 python3 scripts/write_ballpoint.py \
--registration outputs/paper_writing/a4-new.json --paper-z-mm 26.968

# 以下需单独现场确认；空走在估计面上方5mm
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/write_ballpoint.py --registration outputs/paper_writing/a4-new.json \
--paper-z-mm 26.968 --air --execute

# 实写：首笔起点重新触纸，随后使用实测高度
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/write_ballpoint.py --registration outputs/paper_writing/a4-new.json \
--paper-z-mm 26.968 --execute
```

历史配准与纸、TCP哈希、控制器TCP及笔姿态绑定，克隆不代表现场仍匹配。换纸/安装重示教；起点离纸3–100mm，无其他运动程序，现场监看且急停可用。

## 保护与遗留

横向目标30mm/s，有限小段使平均速度更低；竖直保持慢速。相对原始基线力向量变化≥5N停；触纸阈值1N、最多下探6mm。横向≥2N先停，仅尝试每次向上0.1mm卸载，总量≤1mm，不向下追压、不接触清零。这不是已验证恒法向力控制。

接触失联停止，空闲/空走重连有界；限位和模型检查保留。笔、线缆、台面未完整建模需现场检查。新速度/卸载只有离线覆盖，尚需新鲜状态、斜面、可见完整字迹、全纸面压力和从头到尾真机验证。
