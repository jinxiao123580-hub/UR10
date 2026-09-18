# 09 · Mech-Eye 相机接入清单

本机已安装 Mech-Eye SDK `2.5.0`，相机地址为 `192.168.1.33`。当前有效性不以“节点能启动”
判断，而以一次采集的图像、深度和点云元数据判断。

```bash
cd ~/UR10
python3 scripts/setup_mecheye.py --check
python3 scripts/validate_mecheye_capture.py
```

通过时，将当次 JSON 和生成文件保存到 `outputs/camera/`。确认以下事实：

1. 普通点云 XYZ 是米制浮点数据，存在合理比例的有限点。
2. 组织图、深度图和点云有相同采集批次的时间信息。
3. 所谓颜色/纹理输出目前是单色，不可把三通道格式当作彩色图。
4. 相机坐标到机器人 `base` 的变换只能使用已验收的 eye-in-hand 标定参数。

重新安装 SDK、改变 ROS 接口版本或修改相机网络配置后，必须重新执行两条检查命令，并在同一
提交中更新本页、[`08`](08-六维力与3D相机.md) 和 [`../HANDOVER.md`](../HANDOVER.md)。
