# 2026-09-28 HXY 寄存器只读诊断与实板交接

## 目标、板卡与参考

目标是定位 Applepy_v1 静置 HXY ICM-42688P 加速度合量约 `5.50 m/s²` 而非 `9.81 m/s²` 的原因，先用最小只读诊断确认寄存器、状态和未换算的六轴计数。用户授权连接当前 COM 串口并烧录；本轮未发送 `arm/pwm`，但**没有核查舵机是否物理连接**，不修改加速度比例或姿态算法。现行固件为 `mechanical_dog/` 的 PlatformIO ESP32-C3-WROOM-02-H4 + Arduino C++ 分层工程；原 Py-Apple Dynamics V7.3 是 MicroPython 8DOF 行为参考，M0 工程 HXY 驱动是寄存器参考，均不替代本板测量。

手册为用户提供的 `C:\Users\0\Downloads\C46550687_姿态传感器-陀螺仪_ICM-42688P-HXY_规格书_LGA_14_2_5X3X1_00_ICM_42688P_HXY.PDF`（HXY 46 页 PDF）。按含中文路径的文档流程复制为 ASCII 工作副本，使用 PyMuPDF 抽取并渲染所列页，已人工看过相关表格。这里的“第 N 页”指 PDF 阅读器从 1 开始计数：

| PDF 页 | 与此诊断相关的定义 |
|---:|---|
| 4 | 加速度量程和 16 位灵敏度：±4g 时约 `0.122 mg/count`，即约 `8192 count/g`；注明电路工厂按 1.8V 校准，工作电压 `1.71–3.6V`。 |
| 15 | HXY 的寄存器地址并非 TDK 同名芯片寄存器图：`WHO_AM_I=0x01` 默认 `0x6A`，`COM_CFG=0x05` 默认 `0x50`，`HPF&LPF_CFG=0x08` 默认 `0x80`，`DATA_STAT=0x0B`，六轴数据窗口 `0x0C–0x17`。 |
| 21 | `DATA_STAT(0x0B)` bit4/bit5 为 `ACC_CONF_ERR/GYR_CONF_ERR`，bit0/bit1 为瞬时 ACC/GYR 数据就绪；配置无错误不等于灵敏度或整个器件健康。 |
| 22 | `0x0C–0x17` 的三轴加速度/角速度均为高字节在前的二补码；表中 `0x2000=8192` 在 `FS=01` 时为 1g。表格后还给出右移 4 位示例，但不改变 16 位表示下 `8192 count/g` 的关系。 |
| 32–34 | `ACC_CONF(0x40)=0xA8` 可解析为高性能、100Hz；`ACC_RANGE(0x41)` bit1:0 `01=±4g`、默认 `10=±8g`；非法配置可使错误位置位。 |
| 34–36 | `GYR_CONF(0x42)=0xA8` 为高性能、100Hz，`GYR_RANGE(0x43)=0x01` 为 ±1000dps；`PWR_CTRL(0x7D)=0x0E` 使能温度/加速度/陀螺，并规定配置顺序/等待。手册也提供需要**写寄存器**的自检流程，本轮未执行。 |

## 实现和接口边界

- `include/Driver/HxyIcm42688p.h`、`src/Driver/HxyIcm42688p.cpp` 新增 `HxyDiagnostics`、`readDiagnostics(...)`：只在已初始化时分别读取所需配置/状态寄存器和一帧 `0x0C–0x17`，大端二补码拼接 16 位有符号原始计数。读取任一步失败时不发布部分结果。既不重配 HXY，也不更新常规 IMU 样本。
- `src/main.cpp` 接收串口精确命令 `imu_diag`，输出三行 `HXY_DIAG`：寄存器值、错误位/就绪位/六轴原始计数、12 字节 HEX 帧。命令本身不经过 ARM/PWM 调度、不写 NVS；普通 `help` 另提示此命令。上电初始化仍会写 IMU 配置/PCA 软件 all-off，`off` 则是另一个主动向 PCA 发关断命令的操作，不能将整个会话概括为全程零写入。
- `src/UserApp/RobotApp.cpp` 把版本标识改为 `0.3.2`；保留 `0.3.1` 的相对静止标定逻辑。`test/host/host_tests.cpp` 加入假总线读回/大端计数回归与读取失败不发布部分结果检查。没有对网页、舵机目标、量程设置或换算系数做变更。
- 此轮同步更新 `mechanical_dog/README.md`、`docs/REPLICA_STATUS.md` 和 `docs/REPLICA_PROGRESS.md`；原有版本历史保留为快照。

## 验证和现场读数

- `g++ -std=c++17 -Wall -Wextra -Werror` 编译 `test/host/host_tests.cpp` 及工程 `Ctrl/Driver/UserApp` 对应 `.cpp`，执行主机假总线回归 `host_exit=0`。使用技能脚本 `platformio_builder.py --project-dir G:\PROJECT\Mechanical_dog\mechanical_dog --env esp32-c3-devkitc-02` 构建成功，`firmware.bin` 约 834.5 KB；`pio_flasher.py --flash --project-dir G:\PROJECT\Mechanical_dog\mechanical_dog --env esp32-c3-devkitc-02 --upload-port COM4` 烧录成功（约 27.5 s）。脚本 `serial_monitor.py --port COM4 --baud 115200 --duration 3 --timestamp` 看到上电 `Applepy_v1 debug firmware 0.3.2`、PCA `0x40 ready`、HXY `0x19 ID=0x6A ready`。
- 随后的 PowerShell .NET SerialPort 会话只发送 `status/imu_diag/imu/imu_diag/imu/rpy`，`status` 为 `ARMED=no ACTIVE=-1`，`rpy` 已显示数值，说明之前姿态一直“未完成”的症状在这次串口路径没有重现，但物理角度仍未标定。**`off` I²C 应答来自烧录前旧版固件**的 `status/off/status` 独立会话，未在 `0.3.2` 重测。所有串口会话在 `finally` 关闭 COM4，当前没有占用串口的脚本；全程未发送 `arm/pwm`，但并未检查有无物理连接的舵机。
- `imu_diag` 读回 `WHO=6A COM=50 FILTER=80 STAT=04/05 ACC_CONF=A8 ACC_RANGE=01 GYR_CONF=A8 GYR_RANGE=01 PWR=0E`；两次状态中的 `ACC_CONF_ERR=0`、`GYR_CONF_ERR=0`，就绪位可随读取时刻不同。原始加速度约 `(42,565,4557)` count，约 `sqrt(42²+565²+4557²)=4592 count`，按当前 **±4g 8192 count/g** 换算约 `5.50 m/s²`。这是静置的一面、近似读数，不是六面校准值；该样本的陀螺与完整 12 字节 HEX 详见原始串口，不凭空填造具体数值。
- `ACC_RANGE=01` 与配置错误位 0 排除了“0x41 根本未写入，仍保持寄存器默认 `±8g`”这个**具体**猜测，但没有证明内部有效量程、模拟链路、供电/装贴、原始数据比例都正常。若按手册 1g，本应有约 8192 count；目前约 4592 count 是大幅偏差，不应把 `5.50` 直接硬乘 `9.81/5.50`，也不能立即归因芯片损坏。`STAT` bit0/bit1 为瞬时 DRDY，某次为零不构成传感器停转证据；自测寄存器和厂商测试程序未运行。

## 下一步及安全约束

1. 先确认舵机已物理断电/拔下，在绝缘面上将板子分别朝传感器 `+X/-X/+Y/-Y/+Z/-Z` 静置，每面数秒，重复 `imu_diag`（原始 count）和 `imu`（当前软件 m/s²）。必要时先识别 IMU 轴朝向而不是按板框/狗身轴猜测。检查三轴随翻转的正负、每轴 `(正面读数-反面读数)/2` 是否接近 `8192 count/g`、偏置 `(正面+反面)/2`、其余轴交叉分量和各面模长。
2. 若三轴每面约同一比例偏小，继续核对真实数据窗口、配置时序/滤波设置和供应商兼容性；若只某轴或某个面异常，优先排查装贴、焊点、机械受力或该轴。必要时在板实际工作、无舵机负载时测芯片电源与地的 3V3（手册工作范围 `1.71–3.6V`），避免探针短路。必要的自测**需要改写寄存器**，须单独计划，不能假称本轮已做。
3. 网页姿态页、PWM 示波器、MG90、机械限位、供电负载与 3 秒软件超时均无本轮验收证据。PCA9685 `OE#` 下拉，软件 `off`/I²C 应答不是硬件急停。加速度物理量纲未确证前，仅将 RPY 用于方向性观察，不能用于平衡闭环或安全判断；保存六面原始值后再决定驱动校准或器件处理。
