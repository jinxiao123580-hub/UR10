# 09 · Mech-Eye 相机接入清单

本机已安装 Mech-Eye SDK `2.5.0`，相机地址为 `192.168.1.33`。当前有效性不以“节点能启动”
判断，而以一次采集的图像、深度和点云元数据判断。

```bash
cd ~/UR10
python3 scripts/setup_mecheye.py --check

# validate_mecheye_capture.py 与 /device_info 都依赖 mecheye_ros_interface 的 srv 类型，
# 该包装在 ~/colcon_ws，而 ~/.bashrc 只 source 了 ~/ros2_ws，必须先手动 source，
# 否则 ros2 会报 "The passed service type is invalid"：
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
python3 scripts/validate_mecheye_capture.py
# 读型号/序列号/固件（相机节点需先启动）：
ros2 service call /device_info mecheye_ros_interface/srv/DeviceInfo "{}"
```

通过时，将当次 JSON 和生成文件保存到 `outputs/camera/`。确认以下事实：

1. 普通点云 XYZ 是米制浮点数据，存在合理比例的有限点。
2. 组织图、深度图和点云有相同采集批次的时间信息。
3. 所谓颜色/纹理输出是单色，不可把三通道格式当作彩色图。这是 PRO XS 的硬件规格
   （官方规格表 `2D image color: Monochrome`，型号表中 PRO XS 无 `C` 后缀彩色版本），
   不是配置错误；成因与证据见 [`08`](08-六维力与3D相机.md) 与
   `outputs/camera/model-identification-20260918.json`。
4. 相机坐标到机器人 `base` 的变换只能使用已验收的 eye-in-hand 标定参数。

重新安装 SDK、改变 ROS 接口版本或修改相机网络配置后，必须重新执行两条检查命令，并在同一
提交中更新本页、[`08`](08-六维力与3D相机.md) 和 [`../HANDOVER.md`](../HANDOVER.md)。
