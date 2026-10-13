# Py-Apple Dynamics V7.3 → Applepy_v1/V10 软件迁移矩阵（0.8.14）

本表只评估当前Applepy_v1硬件可由软件实现的V7.3主控行为。`implemented/adapted`表示源码与假件测试完成，**不表示实物、负载、供电、COM或落地验收完成**。原仓库按GPL-3.0记录，只读路径见[原项目](ORIGINAL_PROJECT.md)。

| 原模块/公开行为 | 0.8.0 C++/API | 证据 | 状态 | 适配/缺口 |
|---|---|---|---|---|
| `main.py` 5ms Timer/mainloop | Arduino `loop()`、10ms IMU、20ms运动真实dt | host motion/gap/rollover | adapted | 不复制每调用增益；无RTOS硬实时保证 |
| `PA_SERVO.angle/release` | `Pca9685` + `ServoController` coordinated/single/off | host/PCA/fault tests | implemented | PCA仍50Hz/12bit、8次顺序写；无硬件OE急停 |
| 8路中位/网页标定 | `ChannelCalibration`、`applepycal/c{channel}` | 重启/失败/兼容测试 | implemented | 原NVS键完全保留 |
| `PA_IMU` MPU6050 | HXY ICM-42688P驱动 + `AttitudeEstimator` | host IMU/校准/reset | adapted | 六轴、无磁力计；Yaw相对漂移 |
| `PA_IK`串联腿 | `V10LegIk` 80/61.357mm + 三点shank映射 | IK/FK/q测试 | adapted | V7.3 69mm和`ma_case`并联腿不适用于现V10 |
| `PA_ATTITUDE.cal_ges` Pitch/Roll/X | `V10BodyPose` body +X前/+Y左/+Z上 | zero/sign/manual pose测试 | adapted | 8DOF丢弃侧向分量，不是完整3D刚体 |
| 手动Pitch/Roll/高度/X网页 | 四字段原子`pose_live_set`；CG Walk独立应用 | Node latest-wins + motion原子/transition测试 | implemented | Pitch−7..+5、Roll±10、X±8；每个tuple完整Stand预检 |
| 手机高度/Pitch/Roll/body_x滑块 | 150ms合并、单请求在途、读回/失败回退 | Node八控件门控/并发/迟到ACK/回退 + host no-implicit-ARM | safe adaptation | OFF只改RAM；Stand/Hold平滑重定位；Profile保存P/R/X，motion保存height |
| `PA_STABLIZE.stab`静态自稳 | `V10StaticBalance`双轴有界PD | reference/±扰动/dt/reset/双轴速率 | adapted | 默认Pitch/Roll均`body=-sensor`；只限Stand |
| 原Pitch/Roll Kp/Kd网页参数 | Profile独立Pitch Kp/Kd、Roll Kp/Kd | profile重启/轴独立值测试 | implemented | 控制器固定±1.5°、4°/s；八路输出无额外命令角速度拒绝 |
| `PA_GAIT.trot` | `V10Gait::Trot` + `PyAppleFootTrajectory` | 原函数样点/对角/连续/前后/差速/踏步多周期 | adapted | 0.8.5恢复原归一化曲线/相位并做−S/2坐标适配；显式秒周期，不声称原0.1s实物等价 |
| 原项目直接发送目标角 | 0.8.6协调帧不做软件角速度拒绝 | 原速度专属失败现通过、几何/范围/故障回归 | adapted | 不发布峰值速度；硬件跟随、滞后和冲击仍需实机验证 |
| `PA_GAIT.walk` | 普通Walk + opt-in planned CG Walk | 顺序/支撑/planner真实轨迹测试 | adapted | 修复原Walk忽略`r`参数；支持有符号左右步幅 |
| `padog.move(spd,L,R)` | RAM `V10DriveIntent(forward,turn,stepping)` | 前后/左右转/踏步 fake-PWM | adapted | 旧`step_length` NVS仍非负幅值，不改变含义 |
| 实物直行左右推进差异 | RAM-only `straight_trim`→有效turn→原左右scale | 纯函数/RobotApp/Node dirty-readback | safe adaptation | 0.8.11内置/重启默认0.25；手动值不进Profile/motion NVS；无Yaw/足压/关节反馈，不能自动辨识根因 |
| Trot动态抬头角度前馈 | RAM-only `trot_pitch_offset_deg`→仅Trot有效Pitch→BodyPose | 两组中位×五intent、模式隔离、RobotApp/API/UI revision回归 | safe adaptation | 范围`-0.5..0°`、默认0；开环且不持久化，不是原IMU移动闭环；为端点把Trot lift降至12.8mm |
| Web摇杆`f/t` | 点击锁定前/后/左/右/原地踏步方向pad | Node click/switch/lease/network/visibility tests | adapted | 未做连续模拟摇杆；每次begin固定意图，renew只续租；500ms lease保留 |
| RC输入 GPIO32/26/27/14 | 无 | BOARD事实/无适配器 | hardware gap | 本板引脚/电气未确认，不能声称支持 |
| UART2任意Python代码/命令 | USB严格白名单命令 | parser/API tests | safe replacement | 不执行代码、不wire-compatible；未启用未确认UART2引脚 |
| `PA_GAIT.foward_cg_stab` | `V10MovingCg` + planned global body_x | ±pitch/滤波/包络/support/等待测试 | adapted | 使用bias-corrected body pitch/rate；默认±0.5mm、2mm/s，显式开启 |
| Walk等待`X_S`到位再抬腿 | `V10CgPlanner`非阻塞阶段等待 | planner timeout/actual-vs-nominal周期 | implemented | 这是命令到位，不是实际COM到位 |
| 前腿−30/后腿+40移身 | 支撑三角形推导的signed目标 + margin | 正/反/左右转actual projected margin | adapted | 不照搬旧幅值；无可行组合按具体预检原因拒绝 |
| `height*tan(error*1.1)+gyro*5` | `height*tan(body_error*gain)+body_pitch_rate*gain` | 纯控制正负符号/NaN/gap | adapted | 用body pitch导数，不把旧GyX轴直接照抄 |
| `padog.py` Trot按`spd`前/后/转向重心前馈（4/4/2） | 可选intent-aware Trot feedforward，独立f/b/turn/max Profile | 正/负/左右转/踏步零、关闭回退、profile重启、全周期/runtime | adapted | 默认关闭回退0.8.5当前Trot kernel且body_x前馈为0；启用后按signed stride/10自动算body_x并与moving offset叠加 |
| 网页gyro开关 | `balance_capture/on/off`、`moving_cg_on/off` | zero-PWM请求/priority tests | implemented | 两闭环互斥；enable/reference不持久化 |
| 网页gait Trot/Walk开关 | 手动Walk/Trot与drive session mode | RobotApp/Node | implemented | 手动模式保持原永久ARM规则；租约只约束新方向会话 |
| 控制器参数保存 | 独立`applepyprof/profile`版本记录 | missing/corrupt/NaN/IO/readback/reboot | implemented | 与`applepycal`、`applepygait`完全独立；不存enable/reference/intent/session |
| `config_s.py`机械参数保存 | V10几何编译期固定 | build/IK evidence | intentional restriction | 不允许运行时改旧69mm腿、`ma_case`或未测硬件 |
| GPIO25舵机电源使能 | 无 | Applepy_v1原理图事实 | hardware gap | 本板未确认可控舵机电源；软件Off不是物理断电 |
| GPIO33电池低压ADC | 无 | BOARD事实 | hardware gap | 本板无已确认分压/ADC资源，不能伪造低压保护 |
| GPIO13蜂鸣器/alarm | 无 | BOARD事实 | hardware gap | 无已确认蜂鸣器资源；网页/串口告警替代但非声光告警 |
| Wi-Fi AP网页 | `DebugWebServer` POST白名单+token | Node/mock/API tests | implemented | token不是用户身份认证；AP密码仍实验配置 |
| 原舞蹈/动作库 | 无对应项 | 原核心源码核对 | not present upstream | 这份V7.3是手动姿态/步态，没有可迁移的内建舞蹈库 |

## 完成声明边界

0.8.0可称“当前Applepy_v1/V10硬件条件下，V7.3公开主控软件行为已实现或给出明确安全适配/硬件缺口”。不能称整机完成或实物通过：仍缺真实电源/低压链、硬件急停、RC/UART2引脚、电流/负载、关节/足端/接触反馈、实际COM、闭环增益和落地稳定验证。
