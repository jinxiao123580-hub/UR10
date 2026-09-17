# ATI 重力补偿数据存档（截至 2026-09-16）

本目录是索引，不复制原始数据。原始 CSV/JSON、拟合候选和失败实验均保留在仓库
原路径，并已在提交 `4aa8030` 首次完整归档。校验值见同目录 `SHA256SUMS`。

## 当前结论

- 所有 `config/ft_gravity_calibration*.yaml` 均为实验候选或历史配置，当前没有
  通过最终独立验收的模型；不得用于力控、碰撞停止或抓取判断。
- 新采集和以后复算只使用 `/ft_sensor/wrench_raw`，ATI 硬件 Bias 应保持六轴全零。
- 示教播放最终数据共 12 组、每组 5 个窗口；第 11 组由操作者确认是在运动中
  误采，必须排除。使用组 `{1..10,12}` 拟合时，设计矩阵条件数 `8.207`，但力
  RMS `4.208 N`、最大残差 `15.411 N`，仍失败。
- 朝下候选曾在 3 个独立姿态得到力/力矩 RMS `0.643606 N / 0.055344 N·m`，
  随后新姿态反例力 RMS `1.565931 N`，已撤销。
- 全范围鲁棒候选曾在 4 个独立姿态得到 `0.374993 N / 0.049220 N·m`，随后
  在线静态反例为 `2.000 N / 0.110 N·m`，已撤销。

## 数据分组

### 开机基线

- `outputs/ft_baseline/baseline-20260914-162630.json`
- `outputs/ft_baseline/baseline-20260915-pre-restart.json`
- `outputs/ft_baseline/baseline-20260915-post-restart.json`

### 手动训练与验证原始数据

- `outputs/ft_calibration/downward-20260915.csv`
- `outputs/ft_calibration/downward-validation-20260915.csv`
- `outputs/ft_calibration/fullrange-20260915.csv`
- `outputs/ft_calibration/fullrange-robust-validation-20260915.csv`
- `outputs/ft_calibration/repeat12-20260915.csv`
- `outputs/ft_calibration/repeat12-cable-loose-20260915.csv`
- 同目录 JSON 是对应诊断或验证摘要。

### 示教播放重复采样

- 主数据：`outputs/ft_repeats/pendant-playback-20260916-final.csv`
- 元数据：`outputs/ft_repeats/pendant-playback-20260916-final.json`
- `retry`、无后缀版本保留为采集器调试和中断证据，不应与主数据直接拼接。
- `planned-waypoints-20260916.*` 是未完成的被动计划点采集，不是完整训练集。

### 候选模型

- `config/ft_gravity_calibration.downward-20260915.yaml`
- `config/ft_gravity_calibration.fullrange-20260915.yaml`
- `config/ft_gravity_calibration.fullrange-robust-20260915.yaml`
- `config/ft_gravity_calibration.manual.yaml`
- `config/ft_gravity_calibration.manual.round2.yaml`

以上全部保持 `valid: false`。以后复算时应新建文件，不覆盖这些历史候选。

## 关键实验说明

- `experiments/2026-09-11-重力补偿标定与失败验收.md`
- `experiments/2026-09-15-朝下工作区重力补偿独立验收.md`
- `experiments/2026-09-15-全范围鲁棒候选独立验收.md`
- `experiments/2026-09-16-示教播放第11组运动中采集排除.md`
- `experiments/2026-09-16-示教播放静态组拟合失败.md`

## 恢复使用前的最低条件

重新处理时必须保留训练/验证隔离、线缆状态、逐姿态残差和离群点清单。模型需要在
未参与拟合与模型选择的多个姿态上通过预先声明的门限，才允许把 `valid` 改为
`true`。当前项目主目标已切换为手眼标定，重力补偿工作暂停但数据完整保留。
