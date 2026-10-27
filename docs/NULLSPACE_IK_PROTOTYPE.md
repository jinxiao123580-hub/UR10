# 自写零空间 IK 原型（2026-10-22）

实现：`scripts/ur10_nullspace_ik.py`。仅离线求解，没有网络、控制器IK、官方逆解、ROS运动或自动执行入口；旧求解器和运动脚本不变。

## 方法与坐标

参考本项目`ur_pose_ik.py`的关节约束及现有URDF几何定义，但重新实现URDF串联变换、几何雅可比、阻尼最小二乘迭代和SVD零空间投影。依赖NumPy及SciPy的旋转表示，不调用它们的IK优化器。

- 目标矩阵：`base → TCP`，单位m。`base`不是`base_link`，内部从URDF固定变换计算区别。
- `tip_offset`：tool0坐标中的TCP位置偏移，单位m；输出姿态方向仍与tool0一致，不支持额外TCP旋转偏置。
- 使用URDF关节顺序；至少保留3°限位余量，不将关节角自动折回±π，避免隐藏多圈跳变。
- `pose`：完整6D位姿。常规满秩时零空间维数0，没有冗余可以优化。
- `axis`：位置＋工具Z轴方向，绕工具轴的旋转自由，常规秩5，零空间维数1。只适用于任务允许自由转笔/转工具时，不能直接用于要求夹爪完整朝向的抓取。

迭代式：`dq = J_damped_inverse e - k N gradient(cost)`，`N = Z Zᵀ`，Z是SVD提取的零空间基。没有将`I - J_damped_inverse J`当作严格零空间投影。主任务限步长、回溯并检查关节范围；到达精度后，仅在主任务仍满足误差门限时继续降低关节居中代价。

### 仿真发现的问题及修复

首次MuJoCo轨迹复核发现：单次求解内部迭代累计的自由轴转动达到62.27°，尽管末端任务误差很小，仍不能作为连续轨迹使用。现将两种用途明确分开：

- `solve(...)`默认用于局部跟踪，相对于输入seed每关节最多变化3°，主任务不能在这个邻域内完成则返回失败，不自动扩大范围；3°是每次IK输出角差门限，不是速度限制，也不等同于限位余量。
- `solve_initial(...)`显式用于初始构型搜索，搜索邻域45°。结果不是接近运动指令，更不是可直接执行的路径；必须另行规划接近轨迹。它仍是单初值局部搜索，不是全局逆解保证。
- 次任务只在主任务达标后启用，避免居中优化先消耗有限跟踪范围。

原随机测试初值是标准差约2.29°的高斯扰动，各关节最大扰动可能超过3°，不能要求局部跟踪器全部求解成功。修复后分别测试初始解搜索与小扰动跟踪；另外保留超过3°目标的跟踪拒绝测试，不以修改测试门限隐藏拒绝。

## 离线示例

```python
import numpy as np
from ur10_nullspace_ik import UR10NullspaceIK

ik = UR10NullspaceIK(tip_offset=(0, 0, 0.329), margin_deg=3)
seed = np.array([0.35, -1.15, 1.25, -1.65, -1.1, 0.4])
target, _ = ik.fk_jacobian(seed)  # 示例：用模型生成目标，不是实际纸面
target[0, 3] += 0.005
result = ik.solve(target, seed, mode='axis')
print(result)
```

从项目运行时设置`PYTHONPATH="$PWD/scripts"`；URDF默认`outputs/ik/ur10_factory_20261027.urdf`，可通过构造函数`urdf=`显式指定。该模型由官方描述包和既有`config/ur10_factory_calibration.yaml`生成，不是用手眼标定去拟合机器人连杆参数。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONNOUSERSITE=1 python3 scripts/build_factory_urdf.py
```

2026-10-27迁移复验：初始搜索80/80、局部跟踪80/80；25个记录的控制器位姿经新手眼完整旋转与平移转换为相机目标后，转换回tool0均可求解。相机目标转换为`base_from_tool0 = base_from_camera @ inverse(tool0_from_camera)`，不能只减去相机平移，也不能把近似同轴当作旋转矩阵为单位阵。

MuJoCo三条轨迹363点及两种缓存已按新手眼、工厂参数模型重新验收，证据在`~/.handoff/reports/20261027-handeye-ik-migration/`。数值IK误差、控制器FK一致性和手眼闭环精度是不同指标；本轮没有发送真机运动指令。

## 已验证与未完成

离线测试证据：`~/.handoff/reports/20261022-nullspace-ik/offline-tests.json`；脚本：`~/.dsh-analysis/test_ur10_nullspace_ik.py`。

- 固定随机种子，40个随机姿态×两种模式，初值在已知解附近高斯扰动，用初始搜索接口测试；另用独立随机种子生成每关节±0.004rad的小扰动，测试3°局部跟踪接口。门限位置10µm、姿态/轴方向0.0001rad。仅同模型数值闭环精度，不是机械臂真实精度。最新通过数量见测试JSON。
- 自写雅可比与有限差分最大元素差7.59e-8；与Pinocchio独立FK比对（含329mm偏移、base坐标）最大矩阵元素差5.56e-16。
- 常规完整位姿零空间维数0，轴方向模式维数1；加3°跟踪范围后示例居中代价0.8238→0.8117，不等价于证明所有关节的最小裕量都增加。
- 测试包含不可达目标拒绝、种子超出限位余量拒绝及`J N≈0`检查。

MuJoCo运动学独立核对及附件-机械臂选定碰撞对检查，报告位于`~/.handoff/reports/20261022-nullspace-ik-simulation/simulation.json`。三条圆形/折线轨迹，每条121点；当前测试通过363点，最大位置差约0.0015mm、选定碰撞对最小间隙约33.1mm、单步关节角差小于3°。这些是离线、标称模型的运动学结果，没有运行伺服动力学。失败修复前结果另存`failed-before-continuity-fix.json`。

整条轨迹的速度与加速度时间参数化已在`scripts/precompute_joint_trajectory.py`实现；离线缓冲和状态看门狗原型见[缓存轨迹文档](PRECOMPUTED_JOINT_TRAJECTORY.md)。二者均不发送真机指令。

尚未交付：多初值全局成功率、接近180°轴方向误差的恢复、广泛连续轨迹/奇异点穿越、完整自碰撞和外部障碍检查、TCP/模型与真机一致性验收、动力学与真机控制器缓冲验收。数值奇异点额外零空间不能当成可靠冗余。算法为局部迭代，失败必须保留失败状态。

所有返回结果都标记`execution_allowed=false`和`collision_checked=false`。求解成功不代表可以执行；未来必须外接路径密采样、碰撞、奇异性、关节连续性和现场安全门禁，禁止直接把结果发送真机。
