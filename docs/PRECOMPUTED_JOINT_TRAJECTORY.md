# 一次预计算，缓存关节轨迹回放

`scripts/precompute_joint_trajectory.py`是离线原型，不连接机器人，不发送servoj或ROS控制指令。旧运动脚本不变。

2026-10-27相机重新安装并重做手眼标定，附件配置已选择`config/handeye_eye_in_hand_20261027.yaml`。之前生成的缓存及其碰撞间隙报告基于旧安装，须使用新安装参数重新验收，不能直接沿用旧通过结论。外壳/支架包络仍待现场复核。

本次迁移已重新生成pose/axis示例缓存，位于`~/.handoff/reports/20261027-handeye-ik-migration/cache/`，并通过MuJoCo FK、解析速度/加速度界和选定附件碰撞对检查。新缓存记录`model_urdf`及其SHA-256，模型为`outputs/ik/ur10_factory_20261027.urdf`；这些元数据供核对，旧回放入口不会自动把历史缓存变成当前模型的已验证缓存。

## 流程

1. 输入整条笛卡尔路径、初始关节角、TCP偏移和任务模式。
2. 加密位置/姿态目标，逐点使用上一个关节解作种子。3°局部IK失败时细分目标段，最多8层，仍失败则整条规划失败。
3. 关节路径采用端点速度为零的三次样条，内部节点无需逐点停车。解析计算样条的关节位置、速度、加速度极值；全局拉长时长以满足默认0.15rad/s、0.3rad/s²限制。
4. 按8ms周期预生成全部角度、速度和加速度，检查输出点FK误差（0.2mm、0.1°）及选定附件碰撞对20mm间隙；碰撞插值采样不超过0.5°。
5. 只有整条通过才写出完整缓存JSON。回放只读缓存，不再计算IK。

规划失败时不会覆盖原有同名缓存，因此不要在失败后误用旧文件。缓存中始终标记`execution_allowed=false`。

## 输入格式

JSON字段：`seed_q_rad`六个初始关节角，`targets_base_tcp_m_rad`各目标`[x,y,z,rx,ry,rz]`，`mode`为`pose`或`axis`，`tip_offset_m`是tool0坐标下的位置偏移，`requested_duration_s`为期望总时长。

目标用base系、米和轴角弧度，不是RPY；首目标必须与seed的FK一致，不自动创建初始接近运动。`axis`允许绕工具轴旋转，不适合要求完整夹爪方向的任务。完整位姿任务建议`pose`。

## 仅本地测试指令

```bash
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

# 使用已经生成的仿真示例输入（不是现场目标）
PYTHONNOUSERSITE=1 python3 scripts/precompute_joint_trajectory.py prepare \
  --input ~/.handoff/reports/20261022-precomputed-trajectory/input-pose.json \
  --output outputs/ik/precomputed-demo.json

# 仅向终端输出缓存的时间和关节角，不发送给机器人
PYTHONNOUSERSITE=1 python3 scripts/precompute_joint_trajectory.py replay \
  --input outputs/ik/precomputed-demo.json --realtime
```

不带`--realtime`则尽快输出缓存，适合仿真消费；带该选项则用单调时钟调度，落后50ms就中止，不突发追赶。Linux/Python及终端输出不是硬实时，不能把此功能当作已经验收的机器人伺服发送器。

## 仿真与限制

脚本`~/.dsh-analysis/test_precompute_joint_trajectory.py`和证据`~/.handoff/reports/20261022-precomputed-trajectory/summary.json`：完整位姿/轴方向两种模式，5mm半径轨迹；检查解析速度加速度界、起止零速度、缓存回放无IK调用、MuJoCo逐点FK一致性，以及起点不一致/碰撞/超时拒绝。

插值后的笛卡尔误差验收比IK节点本身更宽，不能把节点10µm求解门限当作整条插值轨迹或真实机械臂的精度保证。自由轴居中会增加运动量，时间参数化会自动延长轨迹，不以超限换速度。曲线拐角会被关节样条平滑，只有误差门限内才接受，不是精确过所有原始笛卡尔折线。

尚未完成：完整场景碰撞、奇异性全程门禁、动态负载/电机力矩/jerk验证及真机发送。接入真机前必须检查起始状态、安装/模型/缓存一致性，采用控制器缓冲和看门狗；不能逐行打印后直接认为具备可靠伺服控制。纸面接触反馈仍须运行时保留，不能整条预计算后取消保护。改变现场目标或笔安装后应重新规划。

## 离线缓冲与状态看门狗（2026-10-27）

新增`scripts/buffered_joint_replay.py`，模拟生产者填充与消费者逐周期取点。它没有机器人、ROS或网络传输入口，不调用IK；旧`replay`命令仍是终端回放工具。只在下述新模块中使用更严格的缓存检查和缓冲状态机。

```bash
cd ~/UR10
PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/buffered_joint_replay.py \
  --input ~/.handoff/reports/20261027-offline-ik-cache/baseline-cache/cache-pose.json
```

该命令使用确定性仿真时钟与理想反馈，输出摘要，不等待真实时钟、不发送关节目标。`BufferedReplay`接口提供`fill(count)`、`start(now, observation)`、`tick(now, observation)`和`stop(reason)`。`tick`每次最多返回一个缓存目标，提前调用返回空；`Observation`包含同一单调时钟域的时间戳、六关节角和就绪状态。

- 缓存加载时检查版本、离线标志、单位、关节名唯一性、数组形状、有限值、均匀递增时间、起点、速度/加速度界和端点零速度。复制关节数据，防止调用者随后修改原始JSON改变回放目标。SHA-256用于标识所加载的JSON内容，不是来源认证或模型一致性证明。
- 默认队列64点，至少预填16点；仿真剩余不超过16点时补充。队列容量只限制待消费索引，完整缓存仍驻留内存。缓存不足16点时须全部预填。
- 在8ms缓存上，默认消费超时门限4ms、反馈有效期24ms、最大关节跟踪偏差0.05rad。它们是仿真参数，未获真机验收。超时门限必须小于一个周期，避免越过整周期后继续追赶。
- 启动和消费时检查反馈有效、就绪、时间戳非未来且不倒退、反馈未过期、关节角与参考相符。仿真假定反馈对应上一已消费目标；真实伺服延迟、插值和控制器时钟尚未建模。
- 状态为`NEW → PRIMED → RUNNING → COMPLETE`。运行故障进入锁存的`STOPPED`，清空队列，拒绝继续消费或补充；用户取消同样锁存。缓冲耗尽、消费超时、时钟倒退或反馈门禁失败均停止输出，须新建实例才能重新准备。

2026-10-27证据：`~/.handoff/reports/20261027-offline-ik-cache/`。`baseline-ik/offline-tests.json`和`baseline-cache/summary.json`为本次重跑，历史报告未覆盖。`buffer-watchdog-tests.json`记录两种模式的逐点无重无漏回放、队列容量、取消及故障注入；验收脚本为`~/.dsh-analysis/test_buffered_joint_replay_20261027.py`。

这里的停止仅表示本地模拟器不再返回目标，没有发出保持、减速或硬件停止命令。消费者线程/进程本身挂起时，此Python对象无法实施保护；真实控制器需具备独立看门狗及经过验证的停机行为。当前缓存检查也没有重新核验FK、URDF/安装版本、关节顺序与控制器对应关系、碰撞或奇异性，不能代替规划验收与现场门禁。真机传输、动力学、接触反馈和控制器缓冲协议仍未接入。
