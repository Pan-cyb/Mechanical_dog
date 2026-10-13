# 参考架构与兼容点

> 本文件先前的 OQ12/2S/MPU6050/ESP32-WROOM-32E 假设是历史方案。2026-09-24 的实物板为 `Applepy_v1`，以 [BOARD_V1_BRINGUP.md](BOARD_V1_BRINGUP.md) 和 `../SCH_Schematic1_2026-09-24.pdf` 为现行硬件依据；下文原始 Py-Apple 地图仍是研究参考，绝非新板引脚表。

## 模块地图

| 参考 | 责任 | 关键公开资料 | OQ12 处理 |
|---|---|---|---|
| OpenCat ESP32 | 机器人动作、技能、校准、串口/无线接口 | `OpenCatEsp32.ino`, `src/OpenCat.h`, `src/espServo.h`, `src/imu.h` | 借鉴上层行为，舵机驱动抽象为 `IPwmServo` |
| BiBoard V0/V1 | ESP32、IMU、USB、12 PWM、供电/扩展功能 | Petoi 指南 | 仅复用功能目标和可见引脚/电源要求，不复制 PCB |
| Sesame | 完整开源硬件项目的目录、BOM、Gerber、PCB 装配流程 | `hardware/` | 作为 OQ12 制造文件和开源许可组织样板 |
| MKme NovaSM3 | 12DOF 舵机四足的 STL、部件与接线内容 | `Nova-SM3/` | 仅作机械与试验参考，因许可不明确而不直接派生 |
| ElectroPup | 高级执行器、KiCad、STEP、仿真组织 | `cad/`, `pcbs/`, `src/` | V2 研究参考，不作为 V1 的执行器路线 |

## 参考运行流

OpenCat 风格运行流为：启动 -> 读取关节校准/IMU -> 等待串口或无线命令 -> 根据动作/步态生成关节角度 -> PWM 输出 -> IMU 姿态补偿/故障处理。

OQ12 保留此逻辑顺序，但加入独立硬件安全链：E-stop -> PCA9685/缓冲器输出禁用；物理总电源开关切断 UBEC 输入。首版没有独立 PWR12 舵机电源板，安全链不依赖 Wi-Fi、上位机或主循环按时运行。

## OQ12 接口契约

| 接口 | 数据/单位 | 初版实现 |
|---|---|---|
| `IPwmServo` | 关节编号、目标角度 deg、脉宽 us | PCA9685 50 Hz 输出 |
| `IImu` | 四元数/欧拉角、加速度 m/s²、角速度 rad/s | MPU6050 I2C，100 Hz |
| `IPowerMonitor` | VSERVO/VLOGIC 的 V，电流 A（V1.1）；VBAT 需额外 sense 线 | ESP32 ADC + 分压 |
| `ISafety` | 急停、掉线、倾倒、低压状态 | GPIO 中断 + 1 kHz 状态机 |
| `ITransport` | 串口动作/遥测帧 | USB UART、Pi UART；BLE/Wi-Fi 后加 |
| `IStorage` | 校准偏置、关节配置、动作数据 | NVS，预留 I2C EEPROM |

## 硬件约束

- 2S 电池标称 7.4 V、满电 8.4 V；舵机 6.0 V 与逻辑 5 V 分域。
- 12 个位置舵机以 50 Hz 输出；实际脉宽、零位和方向由单台标定，不写死。
- IMU 安装方向是机械/软件共同契约，必须在 CAD 和固件配置中记录。
- 降压热量由外部 UBEC 承担；OQ12 本板仅分配舵机 6 V，并以 2 oz 外层、短宽母线和粗线束承受大电流。

## 可保留和必须重设计的部分

保留：关节编号表、动作数据逻辑、校准流程、串口协议思路、IMU 姿态补偿。

重设计：PCB、BOM、器件封装、供电路径、安全链、机械外形、安装孔、连接器、开源许可和生产测试。

## Py-Apple 舵机基线的原始代码地图

以下地图针对选定的 `py-apple-dynamics@c69ab76d`、`py-apple-controller@83d989b5`、`py-apple-structure@84e3048e`。聚合仓库的 V6.8 目录不再作为软件主线。

### 软件运行流

`main.py` 上电先调用 `PA_SERVO.release()`，将 ESP32 CPU 设为 240 MHz，然后以 `Timer(1)` 每 5 ms 调用 `padog.mainloop()` 和 `alarm_run()`。`CC_M=0` 时启动 AP 模式和 `web_c.py` 网页控制；`CC_M=1` 读取航模 PWM 输入；`CC_M=2` 读取 UART2 命令。`config_s.py` 由网页标定/参数页面写回 Flash 文件系统。

### 核心文件与责任

| 原始路径（V7.3） | 责任 | 关键硬件/接口事实 |
|---|---|---|
| `Py Apple Dynamics V7.3 SRC/PA-Dynamics V7.3/main.py` | 启动、5 ms 定时器、Web 线程、模式切换 | ESP32 MicroPython；240 MHz；`Timer(1)` |
| `.../padog.py` | 全局状态、舵机输出映射、运动/高度/姿态命令、低压与 IK 故障停机 | ADC `GPIO33`；舵机电源使能 `GPIO25`；蜂鸣器 `GPIO13`；UART2 `TX26/RX27`；遥控输入 `GPIO32/26/27/14` |
| `.../PA_SERVO.py` | PCA9685 驱动与角度到 PWM 映射 | PCA9685 `0x40`；SoftI2C `SDA18/SCL19`；50 Hz；500--2500 us；实际使用通道 `2,3,13,12,10,11,5,4`（8 路） |
| `.../PA_IK.py` | 串联腿/并连腿逆运动学 | `case=0/1`；输入四足 8 个足端坐标，输出 8 个角度 |
| `.../PA_ATTITUDE.py` | 机身俯仰/滚转/高度/重心变换 | 默认几何参数来自 `config_s.py`：`l1=80,l2=69,l=142,b=92.8,w=108` mm |
| `.../PA_GAIT.py` | TROT/WALK 足端轨迹、支撑/摆动相、重心前移稳定 | `Tf=0.5`；由主循环推进相位 |
| `.../PA_STABLIZE.py` | 启动水平校准、姿态 P/D 补偿 | IMU `SDA21/SCL22`，地址 `0x68`；约 200 个周期校准；俯仰/滚转限幅约 25/20° |
| `.../PA_IMU.py` | MPU6050 初始化、14 字节采样、互补/四元数姿态估计 | 400 Hz 采样配置、21 Hz 数字低通、陀螺 ±500 dps、加速度 ±4 g |
| `.../web_c.py`、`cal.html`、`control.html` | HTTP 控制、舵机中位标定、机械/步态参数保存 | 明文 GET 参数；网页直接修改 `config_s.py` |
| `.../config_s.py` | 舵机中位、杆长、机身尺寸、步态和控制增益 | 8 个中位变量 `init_1h/init_1s...init_4h/init_4s` |

### V7.3 手机网页标定路径核对（2026-09-26）

截图里的“Py-apple Dog 舵机标定”确是原项目的 `cal.html`，不是串口 UI。`main.py:49-52` 调用 `padog.do_connect_AP()` 再启动 `web_c.py` 线程；`padog.py:193-211` 使用 `network.AP_IF`，配置 SSID `Py Apple Dog - 菠萝狗`，**源码没有显式设置密码**，IP 则读取 `wifi.ifconfig()[0]`（截图为 `192.168.4.1`，不是代码硬编码）。`web_c.py:7-21` 在这个本机 IP 的 80 端口监听 HTTP；`control.html:359-361` 的“标定及设置”发 `?key=ss`，由 `web_c.py:97-105` 切到 `cal.html`。

`cal.html:53-83` 的腿 `l1..l4` 用于选择，`hi/hd`、`si/sd` 分别让选中腿大腿/小腿的 `init_?h/init_?s` 中位每点击一次增减 1°；`web_c.py:128-188` 实现动作。“开始标定”键 `t9` 会调用 `padog.servo_init(1)`，结合 `padog.py:243-269,471-477`，让**原项目八路舵机同时**输出对应中位，并非只动所选腿；“保存并退出”键 `sc` 用 `w+` 覆写 `config_s.py` 的八个中位以及机械/步态参数，切回控制页及非标定模式。旧网页 `web_c.py:34-36` 对网络 GET 参数执行 `exec()`，其他中位调整也用动态 `exec()`；既没有安全的参数白名单，也不适合直接搬到新板。旧项目 `CC_M=2` 的 UART 控制另属运行模式，不是这张网页的通信链路。

### Applepy_v1 早期0.3.x调试接口历史快照（与当前0.8.0区分）

以下描述是0.3.x早期快照，不是当前API全集：当时SoftAP页面只有`arm|off|pulse|nudge|save`等单路动作、1000–2000µs台架范围和候选物理通道，尚无整机标定、V10关节映射、步态、自稳、Profile或方向会话。当前0.8.0仍使用`GET /api/state`、白名单`POST /api/action`和`X-Applepy-Token`请求校验，但完整操作与安全语义以[当前指南](WEB_UI_USER_GUIDE.md)和[迁移矩阵](MIGRATION_MATRIX.md)为准；仍没有原项目GET `exec()`。

早期候选丝印Servo1–4/9–12对应PCA通道`{0,1,2,3,8,9,10,11}`；0.8.0已按L1H/L1S…L4H/L4S固定V10映射并支持500–2500µs电气校准、八路整机中位和三份独立NVS。加载/展示/编辑/轮询仍不会自动运动。普通手动ARM/PWM仍保持到OFF或故障；只有新方向pad会话使用500ms run lease。本板`OE#`下拉、无硬件急停，物理断电边界不变。

### 关键数据/安全路径

足端目标（毫米） → `PA_ATTITUDE.cal_ges()` → `PA_GAIT.trot/walk()` → `PA_IK.ik()` → `padog.servo_output()` → `PA_SERVO.angle()` → PCA9685 PWM → 8 个模拟舵机。`padog.alarm_and_servo_control()` 在 ADC 拟合电压低于约 5.5 V 持续 200 次时关闭 `GPIO25` 舵机电源；逆解异常也会释放 PWM 并关闭电源。

这条安全链是“软件 GPIO + 外部电源模块”的实现，不等价于独立硬件急停。`GPIO25`、ADC 拟合系数、阈值、通道映射必须在复刻中改为集中配置，并增加硬件级 PWM/OE 与物理急停。

### 硬件文件地图

Controller V4.0 直插版提供 Altium `SchDoc/pcbdoc/PrjPcb`、库文件、PDF 焊接/使用说明和 XLSX BOM；BOM 明列 ESP32、PCA9685、XL4005 可调降压、AMS1117、MPU6050、XH2.54 排针/排母、JST-2P、KCD1-101 开关和 800 mAh 2S 25C 电池。Structure V10.0 串联腿压缩包只含 STL：`Total.stl`、机身/间隙件、`ham`/`ham_servo`、`shank` 等，配套 BOM 明列 `mg90s` 舵机 8 个、6704ZZ 轴承 4 个、2 mm 球头 8 个、M2×L30 连杆 4 个及螺钉包。

### 复刻时必须修正的原始缺口

- 原始 `PA_SERVO.py` 的 `radians` 分支缺少 `math` 导入；`release()` 只遍历 0--14，未覆盖 PCA9685 第 15 路。
- `UART2` 与遥控输入复用了 GPIO26/27，依赖 `CC_M` 互斥，不能同时工作。
- `web_c.py` 用 `exec()` 执行 GET 参数，必须在复刻中改为白名单解析，禁止直接执行网络输入。
- `PA_IMU.py`/`PA_STABLIZE.py` 的滤波与采样周期写死，需用统一时基并加入传感器超时、零值和 I2C 错误处理。
- 8DOF IK 不能通过改通道表变成 12DOF；12DOF 需要重新定义关节状态、3D IK、舵机映射和机械坐标系。
- V7.3 `padog.py` 的启动打印仍写 `V7.2` 和 `Apache License 2.0`，这是版本/许可证元数据残留，不能作为实际版本或许可证证据。

## 现行 Applepy_v1 移植架构（2026-09-24）

原项目 Py-Apple 的 `PA_SERVO.py`、`PA_IMU.py`、`PA_ATTITUDE.py`、`PA_GAIT.py`、`PA_IK.py`、`padog.py` 分别负责 PWM、旧 MPU6050、姿态/步态/8DOF IK 和总调度；C++ 复刻先研究接口与单位，再分阶段移植行为。Py-Apple 的 8DOF、两条 SoftI2C、GPIO25 使能及 GPIO33 电压监测都**不是**本板的现有资源，网页 `exec()` 命令解析亦不可移植。

分层风格参考 `G:\PROJECT\robotic_arm\之前的工作\旧开源资料\Dummy-Robot-2.Firmware\Ctrl-Step-Driver-STM32F1-fw`：`UserApp/main.cpp` 是组合根与任务回调，`Ctrl/Motor` 依赖 `Ctrl/Driver/driver_base.h` 和 `Ctrl/Sensor/Encoder/encoder_base.h`，`Port/*_stm32.cpp` 实现硬件细节；`Core/Src/main.c` 只接入入口，`Drivers/` 是 STM32 库。这是**架构参考**而非本项目功能或频率参考。目标依赖方向：`Application -> Ctrl -> Interface <- Port -> Arduino/ESP32`；上层算法不依赖 `Wire`、Arduino 和 GPIO 编号。

这段记录的是2026-09-24早期开环bring-up路径：`setup()/loop()` → `Port`初始化CH340K串口与GPIO4/GPIO5共用I²C → PCA/IMU探测 → 单通道台架脉宽。当时纯计算8DOF尚未连接协调PWM；此状态已被后续0.7.x实现取代。仍有效的硬件边界是：IMU中断线未确认连接；PCA9685 `OE#`经10kΩ接地不可软件切断；未见ADC电压采样或舵机供电使能，实物观察仍待验证。

用户现已确认 Py-Apple 同版 8DOF/MG90/串联腿；选用 Servo1–4、9–12 为候选接口。未来步态移植仍须先确认接头→关节映射、逐只零位/限幅、安全机制和实测数据。16 个 PCA9685 输出不自动定义 12DOF；原项目 8DOF IK 的算法参数见 [APPLEPY_8DOF_PROFILE.md](APPLEPY_8DOF_PROFILE.md)。

## 原项目与 M0 工程的姿态输出边界（2026-09-28 核对）

Py-Apple Dynamics V7.3 的 `PA_IMU.py::IMUupdate(gx,gy,gz,ax,ay,az)` 使用六轴加速度/陀螺数据；源码计算 `Pitch`、`Roll`，返回数组 `a[0]`/`a[1]` 是俯仰/横滚的混合估计，`a[2]`/`a[3]` 是加速度倾角，`a[4..6]` 是陀螺读数。**没有输出 Yaw**，也没有磁场输入。原项目的 PCA9685 `PA_SERVO.py` 是 PWM 输出，不能读取姿态或舵机真实位置。

用户 M0 工程 `MSPM0G3507_CPP_Template/Legacy/IMU/IMU.c` 的 `IMU_getYawPitchRoll()` **确实返回 Yaw/Pitch/Roll**（约第 357–369 行），但调用 `IMU_AHRSupdate()` 时传入的 `mygetqval[6..8]` 没有磁力计采样赋值（静态初始化为零）；磁场参考与误差反馈代码在约第 262–292 行被注释。它的 Yaw 是积分得到的相对朝向，会因陀螺零偏漂移，不是磁北/绝对航向。当前HXY只提供六轴。用户已确认抬头时sensor Pitch为负、左倾（左低右高）时sensor Roll为正，因此0.7.1的body Pitch/Roll都默认取sensor反号；网页RPY仍显示原sensor frame。物理增益和完整安装轴验证仍待上板。

## V7.3姿态、自稳与Walk移身补充核对（2026-10-01）

- `PA_ATTITUDE.cal_ges()`对名义机身`l=142,b=92.8`做Pitch/Roll刚体变换，但最终只把每腿x/z送8DOF IK，侧向分量丢弃，Yaw固定0。
- `PA_STABLIZE.stab()`按每次调用累计Pitch误差×0.04+gyro×0.6、Roll误差×0.04+gyro×0.35并限幅；`padog.mainloop()`只在静止时调用，运动时目标清零。它不是可直接按秒复用的PD。
- `PA_GAIT.foward_cg_stab()`令`X_goal=in_y+gait_need+110*tan(pitch_error*1.1)+gyro_x*5`。Walk在腿1/2前用−30、腿3/4前用+40，并等待`abs(X_S-X_goal)<1`才推进；末尾四足回程。这是全局机身X目标，不是0.6.1局部stride bias。
- `main.py`5ms Timer；Trot `.05`相位增量理论约0.1s；Walk每有效tick`.01`，无等待约1.25s，实际更长。`walk_h=50/walk_speed=.02`在这份执行路径只定义未使用。
- 完整字段、网页范围和启动`spd=0`边界见[原版参数证据](ORIGINAL_V73_PARAMETERS.md)。0.7.0没有迁移移动中IMU前后CG反馈，只实现静态双轴反馈与确定性的计划式全局CG阶段。

## 0.8.0主控行为适配结论

0.8.0进一步适配`padog.move`的前后/左右意图、`foward_cg_stab`的Pitch/角速度全局X修正、Walk命令侧等待、网页方向控制和控制参数保存。实现采用真实dt、受限包络、signed左右步幅、非阻塞planner、epoch/seq/lease与三份独立NVS记录；不复制旧`exec()`或任意Python代码协议。

原UART2协议由USB严格白名单语义替代，不声称wire-compatible。RC GPIO32/26/27/14、GPIO25舵机电源、GPIO33低压ADC和GPIO13蜂鸣器在Applepy_v1没有已确认可用资源，保持硬件缺口；没有新建虚假GPIO适配。完整逐项账本见[MIGRATION_MATRIX](MIGRATION_MATRIX.md)。
