# COCO 仿真实验状态与后续方案

更新时间：2026-09-03

## 1. 当前仓库状态

- 作者远端：`https://github.com/JustoDario/coco-bot.git`
- 作者仿真分支本地缓存：`origin/feature/simulation`，提交 `0e56c47`
- 当前审计分支：`audit/paper-faithful-20260902`
- 当前审计快照：标签 `audit-snapshot-20260903`
- 作者远端没有被修改。

本次无法完成新的 `git fetch`，原因是 GitHub/代理连接超时。因而不能声称 `0e56c47` 是 2026-09-03 当天远端的最新提交；它是当前机器上已经缓存的作者仿真版本。

## 1.1 清理记录

Windows 工作区已删除过程性残留：`tmp/`、`_xxx_staging/`、`_current_*`、`_edit_*` 以及临时运行脚本。保留 `docs/`、`results/`、`third_party/` 和 `wsl_coco` 链接。

WSL 仓库中已删除可重建的 `build/`、`install/`、`log/` 和 Python 缓存。WSL 家目录中的旧实验目录已清理，仅保留：

- `~/coco_experiments_audit_20260902`：审计过程数据；
- `~/coco_experiments_paper_faithful_20260902`：最新严格固定距离协议数据。

Windows `results/` 中的汇总 CSV、JSON 和图像未删除。

## 2. 控制链路

当前已经验证的链路为：

```text
/cmd_vel
  -> FTG/CPG 足端步态与轨迹
  -> RSA per-leg step scale
  -> KT/逆运动学
  -> JointTrajectoryController
  -> Gazebo Sim ros2_control
  -> /odom、/bno055/imu、/rsa/state
```

其中 RSA 修改的是足端步长/推进比例，然后再进入逆运动学。`reaction_horizon_steps` 主要控制 `goal_pose_` 的长期目标更新频率；RSA 探测的观测窗口由 `learning_observation_steps` 控制，默认是 2 个半步。因此当前实现的 H 不是完整 MPC 意义上的预测时域。

## 3. 已完成并已提交的修复

以下修改位于审计分支，并可通过快照标签恢复：

- 修复相位不匹配和 no-movement 数据污染；
- 用完整 gait cycle 计算横向漂移；
- 固定距离实验在目标距离处插值，并在目标后继续一个完整周期；
- 每个 trial 使用独立的 RSA learning YAML；
- 记录 Gazebo seed、实际参数、起止 scale、confidence 和实验次数；
- 修复 ROS launch 参数被公共 YAML 或残留节点覆盖的问题；
- 加入仿真启动稳定化夹具：暂停、抬高、展开、恢复重力；
- 加入仿真专用的单腿横向偏置和推进比例夹具；
- RSA 候选选择和验收均使用相同的有符号 gait-frame drift 目标；
- 失败探测回滚 scale，并重置局部漂移参考；
- RSA 直接单测通过，完整工作区构建通过。

这些修复分为两类：控制算法/可观测性修复，以及仅用于仿真启动和实验的夹具。后者不能当作论文算法贡献。

## 4. 已证明的事实

1. RSA 的 step scale 确实能通过 FTG、KT 和 JointTrajectoryController 进入 Gazebo 植物。
2. Gazebo 中可以记录 odometry、IMU、关节状态和 RSA 状态。
3. RSA 会进行探索、接受候选、回滚候选并保存 learning YAML。
4. 改变单腿 scale 会改变每周期横向漂移，说明控制变量到观测量之间存在物理链路。
5. 当前实验指标已经改为固定距离、相位匹配的横向误差和每周期漂移。

## 5. 尚未证明的事实

1. 当前 Gazebo 下 online RSA 能跨 seed 稳定识别受扰腿。
2. RSA 开启后能稳定降低横向漂移。
3. `H=1,5,10` 必然产生单调的性能趋势。
4. `plant_step_scale` 或 `actuator_bias_deg` 等价于真实执行器力矩/速度/延迟误差。
5. 当前仿真接触和执行器模型足以支撑论文正面结论。

## 6. 为什么之前结果不可靠

### 6.1 实验停止协议曾经混用

早期数据混用了固定时间、固定 RSA 步数和固定距离，且部分记录虽标记为 1 m，实际并未在准确 1 m 处采样。这些数据不能和最新固定距离数据合并。

### 6.2 H 的作用曾被误当作 RSA 探测时域

当前代码中 H 主要改变长期 `goal_pose_` 的推进间隔，不直接改变 RSA 候选的观测窗口。RSA 实际观测长度是 `learning_observation_steps`。因此不应预先假定 H 越大越差。

### 6.3 扰动信号小于仿真重复波动

RSA 默认一次只改变 1% 的单腿 scale，观测两个半步。当前 Gazebo 四足接触、控制回调调度和 WSL 实时性造成的漂移波动可达数毫米到几十毫米，而单次 RSA 探测的影响只有几毫米，信噪比不足。

### 6.4 当前扰动不是真实执行器模型

`plant_step_scale` 是轨迹层缩放夹具；`actuator_bias_deg` 是关节命令角度偏置。它们没有建模真实电机的 torque limit、速度限制、死区、延迟、摩擦和 backlash。

### 6.5 RSA 的单腿辨识存在整机耦合

单腿步长对机身横向漂移的影响会受到其他腿接触状态、身体姿态和地面摩擦影响。RSA 轮流试探四条腿，短距离运行时样本不足，可能把接触噪声误认为某条腿的 Jacobian。

## 7. 现有数据应如何解释

已有 1 m H=1..10 数据中，RSA-off 平均横向误差约为 `31.4 +/- 17.6 mm`。RSA-on 各 H 的均值约为 `25..36 mm`，H=7 还出现一次严重异常。该数据表现为高方差和偶发失稳，不是可信的 H 单调曲线。

此前 plant scale=0.80、单腿受扰的 frozen 对比也没有跨 seed 稳定优于 off。online RSA 最终没有可靠识别受扰腿。因此这些结果只能作为工程诊断，不能直接写成论文中的 RSA 有效性结论。

## 8. 推荐的实验顺序

### 阶段 A：最小植物敏感性实验

关闭 online RSA，固定 gait、控制器、启动流程和目标距离。只改变一条腿的 plant propulsion scale，例如 `0.80、0.90、1.00、1.10`，每个条件使用至少 20 个独立 seed。记录有符号的每周期横向漂移、目标距离横向误差、yaw、roll/pitch 和跌倒率。

判据：scale 改变引起的均值变化必须明显大于重复实验标准差。若不满足，先校准 Gazebo，不进入 RSA 实验。

### 阶段 B：frozen RSA

使用阶段 A 得到的受扰腿和预先确定的补偿 scale，比较 RSA-off 与 RSA-frozen。采用配对 seed 和相同初始状态，报告每个 seed 的差值，而不是只比较两组均值。

只有 frozen 在多个 seed 上稳定改善，才说明 RSA 的执行路径和局部补偿有足够信号。

### 阶段 C：online RSA

每个 trial 使用全新的 learning YAML。运行长度必须足以让四条腿都完成最低样本数，并报告最终 scale、confidence、samples、接受/回滚次数以及是否识别到真实受扰腿。

### 阶段 D：H 消融

先确认作者论文中 H 的准确语义。如果 H 只控制长期目标更新，应该将其作为目标更新频率消融，而不能宣传为 RSA 预测时域。只有当 H 明确改变 RSA 决策/观测机制时，才测试 `H=1,5,10` 的 RSA 性能。

### 阶段 E：执行器不确定性

在 Gazebo 中增加有物理含义的模型：关节角偏置、速度限制、力矩限制、延迟、死区或 backlash。每种误差都要单独定义，不能把轨迹缩放夹具直接称为 actuator torque model。

## 9. 工具选型建议

- Gazebo：保留，用于 ROS 2、URDF、控制器和完整 COCO 管线联调。
- MuJoCo：可用于更快、更易重复的接触动力学消融；但 ROS 2 集成需要额外适配。
- Isaac Sim/Isaac Lab：适合 GPU 大规模训练，但当前 WSL 部署和资源开销更大，不应作为眼前修复手段。

推荐采用“Gazebo 做系统联调，MuJoCo 做受控消融，实机做最终验证”的分工，而不是继续在当前 Gazebo 配置上盲目增加运行次数。

## 10. 当前决策

当前不应继续批量跑 H=1..10，也不应调整参数去制造预期曲线。下一步应先完成阶段 A 的植物敏感性实验，并由作者确认 RSA 扰动模型和 H 的论文定义。若阶段 A 不能得到稳定单调的物理响应，应先修复/更换仿真模型；若阶段 A 成立，再继续 frozen 和 online RSA。
