# 2026-10-09 固定棋盘格误差来源实验

本目录是供离线复算的原始实验记录，不是新手眼标定结果。实验期间棋盘格未移动（现场用户确认）；机械臂只作低速位移，未修改 DH、相机内参或手眼外参。标定文件仍为 `config/handeye_eye_in_hand_20260921.yaml`。

## 数据和顺序

| 顺序 | 数据集 | 有效样本 | 说明 |
| --- | --- | ---: | --- |
| 1 | `dataset.json` | 5 | 首个静止位姿，之后机械臂换位置和姿态 |
| 2 | `pose-change/dataset.json` | 3 | Y 平移实验的起点 A |
| 3 | `translation-only/dataset.json` | 3 | 从 A 纯平移约 +155.19 mm Y，姿态基本不变 |
| 4 | `return-to-pose/dataset.json` | 3 | 低速沿 Y 原路返回 A |
| 5 | `xminus-160/dataset.json` | 3 | 从 A 纯平移约 -159.97 mm X，姿态基本不变 |
| 6 | `x-return/dataset.json` | 3 | 低速沿 X 原路返回 A |

每个数据集旁的 `samples/<sample_id>/` 包含彩色原图 `image.png`、角点图 `corners.png`、原始有组织点云 `cloud.npz`、逐帧 UR 状态 `robot_frames.json` 和 `sample.json`。数据集 JSON 内的 `files` 字段保留采集电脑的绝对路径；在其他电脑上请按**数据集所在目录**寻找同名 `samples/`，不要使用旧绝对路径。仓库的 `scripts/analyze_handeye_closure.py` 已按相对目录读取点云。

## 已复算的观测值

- Y 起点到 +155.19 mm 位：OpenCV PnP 的固定棋盘格基座坐标差 1.420 mm、朝向差 0.3918°；原始深度角点刚体拟合位置差 1.672 mm、朝向差 0.078°。返回位与 Y 起点实际 TCP 相差 0.089 mm / 0.009°，PnP 位置差 0.542 mm，深度位置差 0.426 mm。
- X 起点到 -159.97 mm 位：实际 TCP 朝向差 0.007°；PnP 的固定棋盘格位置差 1.251 mm、朝向差 0.477°。返回位与 X 起点实际 TCP 相差 0.019 mm / 0.009°，PnP 位置差 0.158 mm、朝向差 0.032°，深度位置差 0.216 mm。
- X 侧视角仅约 35–36 个角点有可用深度，且点云角点一致性约 2 mm；该视角深度拟合**不可用于独立精度裁决**。原位三次重复的 PnP 板原点 RMS 通常约 0.008–0.032 mm，这只反映短时重复性，不等于绝对精度。

固定手眼平移在工具朝向不变的纯平移差分中抵消。固定手眼旋转误差可能造成跨视角位置误差，却不能单独解释工具朝向几乎不变时 PnP 棋盘格朝向变化约 0.4–0.5°。仍需核对相机模型、角点检测、深度配准、实体安装与棋盘格平面性；不要由这组数据直接修改 DH 或手眼参数。

## 在另一台电脑复算

从仓库根目录执行（不连接机器人）：

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/analyze_handeye_closure.py \
  --dataset outputs/handeye/error-source-20261009/xminus-160/dataset.json \
  --calibration config/handeye_eye_in_hand_20260921.yaml \
  --output outputs/handeye/error-source-20261009/xminus-160/recomputed-closure.json
```

将 `--dataset` 改为表中的其他五组即可；现有 `static-closure.json` 是采集机上的参考输出。对两组的 `static-closure.json`，比较 `board_geometry.reference_base_from_target.matrix_4x4` 的平移差和旋转夹角；不要把静止组内 RMS 当作跨视角误差。旧多姿态基线为 `outputs/handeye/error-source-baseline-20261009.json`，对应已归档的 `outputs/handeye/manual-eye-in-hand-20260921-r2.json`。

依赖为 Python 3、NumPy、OpenCV、SciPy 和 PyYAML。离线脚本不发送运动指令；`motion-result.json` / `return-motion-result.json` 是已经完成的低速返回记录，不应重放。
