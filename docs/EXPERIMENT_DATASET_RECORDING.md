# 本地双流实验采集

`scripts/record_experiment_dataset.py`监听ROS原始六轴力和UR实时状态（默认30013），不清零、不发运动、不自动上传。`--run`包装的实验可能运动机械臂，须单独现场确认。

```bash
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

# 只采集300秒，Ctrl+C保留中止数据
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/record_experiment_dataset.py --name manual-test --duration 300

# 静止30秒暂停落盘，运动自动恢复
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/record_experiment_dataset.py --name motion-watch --watch

# 静止漂移、重力试验必须连续记录
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/record_experiment_dataset.py --name static-drift --continuous --duration 300
```

## 输出

目录`outputs/experiment_datasets/<时间>-<名称>/`：

- `force.csv.gz`：接收墙钟/单调时钟、ROS头时间、frame_id、原始六轴力N/Nm。
- `ur.csv.gz`：接收时间、控制器运行时间、关节rad、当前TCP米/旋转向量rad及速度。
- `ur-frames.bin.gz`：仅`--raw-frames`时额外保存原始帧，默认关闭避免重复。
- `force-50hz.csv.gz`：20ms接收时间分箱分量均值，源时间取箱内最后一条。
- `ur-50hz.csv.gz`：箱内最后状态，不是抗混叠或硬件同步的仿真重采样。
- `idle-force-summary.csv.gz`：最多30秒静止窗口的均值、总体标准差和数量，常量内存计算。
- `markers.csv`、`metadata.json`、`experiment.log`：状态标记、计数/命令/连接/错误、子进程输出。

两种源时钟未同步，不能按CSV相同行号配对；ROS头与接收时间差不等于物理采样延迟。保留源时间和接收时间作后续对齐。

## 包装实验与门禁

`--run`必须最后，后续全部参数属于实验。双流各收到至少20帧且最新年龄小于250ms才启动子实验，15秒未就绪则不启动。就绪后任一流失效超过0.5秒向子进程组发SIGINT，10秒未退标记可能仍运行；这是数据完整性措施，不是认证急停，子实验必须安全处理信号且现场保留急停。

独立采集默认60秒，`--watch`直到Ctrl+C；包装模式持续到子进程退出，不按`--duration`截断。采集完成不代表实验通过，检查双流计数、时间连续性、退出码及日志；失败数据不当完整配对数据。

## 有界存储

默认先落盘，静止超过`--idle-seconds`（默认30秒）暂停；TCP线速度≥0.1mm/s、角速度≥0.001rad/s，或累计关节变化≥0.0005rad/位移≥0.1mm时恢复。极慢微小运动可能漏触发，需`--continuous`。

暂停仍监听双流，`--pre-seconds`默认2秒、最多10秒有界预缓存，静止统计继续。暂停/恢复明确标记，禁止把停存空窗当已记录原始数据补插值。新数据边写边无损压缩，历史数据不改；分析大文件应分块。附带保存工具、负载、TF和走线说明，新增状态客户端占资源，运动前先只读验证。
