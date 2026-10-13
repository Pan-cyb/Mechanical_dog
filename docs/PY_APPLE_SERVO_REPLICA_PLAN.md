# Py-Apple 舵机机器狗完整复刻方案

日期：2026-08-26

## 1. 最终选择

不要直接把聚合仓库 `ToanTech/py-apple-quadruped-robot` 的三个顶层文件夹当成同一版本。聚合仓库只包含 Dynamics V6.8，而独立 Dynamics 仓库已经更新到 V7.3。

本方案冻结以下非无刷组合：

| 子系统 | 冻结版本 | 固定提交 | 结论 |
|---|---|---|---|
| 软件 | Py-Apple Dynamics V7.3 | [`c69ab76d`](https://github.com/ToanTech/py-apple-dynamics/tree/c69ab76df92a9a162b6608ea0ffb72d23c1e8990) | 最新舵机软件；ESP32 MicroPython；8DOF |
| 控制板 | Py-Apple Controller V4.0 直插版 | [`83d989b5`](https://github.com/ToanTech/py-apple-controller/tree/83d989b5d24b92cb699e61e1c798fc7b837c6df5) | 最新公开控制器；ESP32 + PCA9685 + MPU6050 |
| 机械 | Py-Apple Structure V10.0 串联腿 | [`84e3048e`](https://github.com/ToanTech/py-apple-structure/tree/84e3048eb545fca0ae44138a09eeb14e54feba5c) | 最新目录；实际仍是 V7.0 8DOF BOM/几何 |

排除所有 BLDC 文件、`py-apple-bldc-quadruped-robot`、Structure 的 4008 无刷腿，以及 Dynamics V4.0/V4.5/V6.0/V6.5/V6.8 历史版本。

## 2. 必须先接受的边界

这套上游是完整的 **8DOF 舵机狗**，不是 12DOF：

- 每条腿 2 个 MG90S，共 8 个舵机；
- IK 只输出每腿大腿/小腿两个角度；
- 姿态模型只把四足足端投影到二维腿平面；
- V10 压缩包无 STEP/参数化源文件，只有 STL。

若最终目标仍是 OQ12，建议先复刻 8DOF 作为可运行参考机，再独立开发 12DOF 机械、3D IK 和 12 路标定表。

## 3. 软件提取清单

上游目录：`Py Apple Dynamics V7.3 SRC/PA-Dynamics V7.3/`

| 文件 | 复刻责任 | 去向 |
|---|---|---|
| `main.py` | 240 MHz、5 ms 周期、Web 线程、模式入口 | `Application/RobotApp` + 调度器 |
| `padog.py` | 状态、命令、舵机映射、低压/IK 停机 | 拆成 `RobotController`、`SafetyManager`、`ServoMap` |
| `PA_SERVO.py` | PCA9685 和 PWM 映射 | `IServo` + `Pca9685Servo` |
| `PA_IK.py` | 串联/并联 8DOF IK | `Domain/Kinematics8Dof`，保留主机测试 |
| `PA_ATTITUDE.py` | 机身姿态到足端偏移 | `Domain/BodyPose` |
| `PA_GAIT.py` | TROT/WALK 轨迹 | `Domain/GaitGenerator` |
| `PA_STABLIZE.py` | IMU 校准与 P/D 姿态补偿 | `Domain/Stabilizer` |
| `PA_IMU.py` | MPU6050 读取与姿态融合 | `IImu` + `Mpu6050Adapter` + `AttitudeEstimator` |
| `web_c.py` | HTTP 遥控、参数保存 | 安全 REST/串口命令适配器；禁止 `exec()` |
| `cal.html` | 8 舵机中位标定 | 校准页面/上位机 |
| `control.html` | 网页遥控与参数页 | 可复用交互逻辑，重写协议 |
| `config_s.py` | 中位、杆长、机身尺寸、增益 | 版本化配置 + 校验 + NVS/文件存储 |

### 原始硬件接口

| 功能 | 原始定义 |
|---|---|
| PCA9685 | 地址 `0x40`，SoftI2C `SDA18/SCL19`，50 Hz，500--2500 us |
| MPU6050 | 地址 `0x68`，SoftI2C `SDA21/SCL22` |
| 电池/舵机电压 ADC | `GPIO33` |
| 舵机电源使能 | `GPIO25` |
| 蜂鸣器 | `GPIO13` |
| UART2 | 115200，`TX26/RX27` |
| 航模遥控输入 | `GPIO32/26/27/14`，与 UART 部分冲突 |
| 实际舵机通道 | `2,3,13,12,10,11,5,4` |
| 主控制周期 | 5 ms 定时器；PCA9685 实际输出 50 Hz |

## 4. 硬件提取清单

### 4.1 Controller V4.0 直插版

| 类别 | 原始器件/模块 | 数量 | 复刻建议 |
|---|---|---:|---|
| MCU | ESP32 开发板 | 1 | 先保留模块，后续再做板载模组 |
| PWM | PCA9685 16 路舵机板 | 1 | 必须接出 OE，默认失能 |
| 舵机降压 | XL4005 可调 5 A 模块 | 1 | 只适合原始 8×MG90S 路线；需实测峰值与温升 |
| 逻辑稳压 | AMS1117 模块 | 1 | 复刻板改用更可靠的 3.3 V 稳压方案 |
| IMU | GY-521/MPU6050 | 1 | 固定安装方向，远离舵机电源噪声 |
| 电池 | 2S 800 mAh 25C | 1 | 原始值；按实测续航和峰值电流决定替代品 |
| 电源/连接 | JST-2P、XH2.54、排针/排母、20 AWG 电源线 | 1 套 | 检查连接器持续电流，不用杜邦线承载舵机总电流 |
| 总开关 | KCD1-101 10×15 船型开关 | 1 | 另加保险丝和急停/OE 链路 |

原仓库还提供 Altium `SchDoc`、`pcbdoc`、`PrjPcb` 和库文件。它们可用于读图和功能核对，但不得在未完成电流、保护、连接器和 DRC 复核时直接下单生产。

### 4.2 Structure V10.0 串联腿

| 类别 | 原始物料 | 数量 |
|---|---|---:|
| 舵机 | MG90S | 8 |
| 轴承 | 6704ZZ，20×27×4 mm | 4 |
| 球头 | 2 mm | 8 |
| 连杆 | M2×L30 mm | 4 |
| 轮胎/脚垫 | 单车皮 | 1 小段 |
| 紧固件 | M2/M3 自攻、细牙、防松螺母 | 1 套 |

V10 压缩包包含：`Total.stl`、`body_bottom_114(x1).stl`、`front_gap_34(x2).stl`、`test_gap_31(x1).stl`、`b1_2(x2).stl`、`b2_2(x2).stl`、`ham_54(x2).stl`、`ham_servo_55(x2).stl`、`shank_48(x4).stl`。没有 STEP、SolidWorks 或参数化装配源。

## 5. 上游不能原样照搬的部分

1. `web_c.py` 直接 `exec()` 网络输入，必须重写。
2. `PA_SERVO.release()` 只释放 0--14，漏掉通道 15。
3. `PA_SERVO.py` 的 radians 分支缺少 `math` 导入。
4. UART 与航模输入复用 GPIO26/27，只能互斥。
5. 低压阈值和 ADC 拟合没有硬件校准来源。
6. 软件停机依赖主循环和 GPIO25，不是独立硬件急停。
7. IMU 采样周期、滤波增益和机械安装方向没有统一配置契约。
8. V10 BOM 实际复用了 V7.0 表，采购前必须按 STL 装配逐项复核。
9. 当前仓库根许可证是 GPL-3.0，旧文件头写 Apache-2.0；对外发布按 GPL-3.0 做合规准备。

## 6. 复刻实施顺序

### 阶段 A：资料冻结

- 保存三个固定提交和完整文件哈希；
- 建立关节编号、坐标系、通道、舵机方向和机械尺寸表；
- 明确 8DOF 参考机与 12DOF 目标机是两个分支。

验收：任何代码、PCB、CAD 都能追溯到固定提交和原始文件。

### 阶段 B：单舵机/单腿开环

- ESP32 + PCA9685 + 独立舵机电源；
- 验证 50 Hz、最小/中位/最大脉宽、急停/OE；
- 测 2 个候选舵机的空载、负载、堵转电流、回差和温升；
- 打印一条 V10/V7 串联腿并核对装配。

验收：单腿可重复回零，无机械干涉；急停后 PWM 和电源均撤销。

### 阶段 C：8DOF 软件复刻

- 先对 IK、轨迹、限位、坐标变换做主机测试；
- 再接 `IServo/IImu/IStorage/ITransport/ISafety` 适配器；
- 标定数据版本化，拒绝越界参数；
- Web/UART 只允许固定命令和数值范围。

验收：8 路舵机按表动作；错误输入不能执行代码或越过限位。

### 阶段 D：静态与慢速步态

- 静态站立、姿态读取、倾倒停机；
- WALK 后 TROT；
- 记录电池、电源模块、连接器和舵机温升；
- 断网、低压、IMU 断开、IK 越界分别做故障注入。

验收：站立 10 分钟、低速行走 20 分钟，无随机复位/甩腿；故障均进入安全状态。

### 阶段 E：12DOF 扩展（可选）

- 每腿新增髋外展/内收关节；
- 新建 3D IK、12 路关节模型、三维限位和姿态坐标系；
- 机械 STEP 重建，不以 V10 STL 布尔拼接作为主模型；
- 重做电源预算、线束和结构负载测试。

验收：12DOF 单腿三轴先开环验证，再进入四腿闭环；8DOF 与 12DOF 测试数据分开保存。

## 7. 当前最合适的下一步

先做“一个 ESP32 + 一个 PCA9685 + 两个 MG90S + 一条打印腿”的台架，不立即画整板。它能一次性验证最关键的不确定项：舵机、电源、V10 结构配合、通道方向、脉宽和 IK 零位。

完成标准：示波器记录 PWM、实测电流/温升、单腿回零视频/日志、机械干涉清单、最终舵机型号与电源规格。完成后才能冻结 PCB 和整机 BOM。
