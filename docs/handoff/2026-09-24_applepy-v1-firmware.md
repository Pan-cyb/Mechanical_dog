# Applepy_v1 首版固件交接（2026-09-24）

## 当前目标与参考

用户已有 Applepy_v1 打样板，要在 `mechanical_dog/` 实现 ESP32-C3 Arduino/PlatformIO 的 C++ 分层控制固件。硬件事实以 `SCH_Schematic1_2026-09-24.pdf` 第 1–3 页为准，不沿用旧 OQ12/ESP32-WROOM-32E 引脚。上游 Py-Apple Dynamics V7.3 本地路径 `third_party/py-apple-servo/software/py-apple-dynamics/` 为 8DOF GPL-3.0 参考；本次独立实现底层，不复制其源码。HXY IMU 寄存器和 ID 参考 `G:\PROJECT\Electronics Design Contest\the second semester of junior year\M0-chip-package\Myproj\MSPM0G3507_CPP_Template\Legacy\ICM42688\icm42688.c` 及其 2026-07-28 板测记录。STM32 步进驱动工程只借 `UserApp/Ctrl/Port` 架构边界。

## 本次实现与决策

- `include/Board/ApplepyV1.h` 集中 GPIO4/5、PCA9685 `0x40`、HXY IMU `0x19`、50 Hz 和保守台架脉宽。
- `Interface/` 纯 C++ 契约；`Driver/` 独立 PCA9685 和 HXY ICM-42688P 寄存器实现；`Port/` 使用 Arduino Wire；`Ctrl/` 管理白名单串口解析、限幅、ARM/超时；`UserApp/` 轮询 IMU 100 Hz、串口调度。`main.cpp` 只组装对象与处理串口字节。
- 上电 PCA9685 16 路 FULL_OFF，不自动回中位。必须先识别 IMU、手动 `arm` 后才能 `pwm <ch> <us>`；3 秒失联尝试全关。I²C 全关失败会锁存故障并提示物理切断舵机电源。
- PDF 显示 PCA9685 `OE#` 由 10 kΩ 下拉，固件不可控制硬急停；舵机 +5 V 常供电。GPIO6 虽标 INT，IMU INT1 图上疑似未连。无已证实的 ADC 低压保护。没有机械/舵机标定信息，不猜 8DOF/12DOF 映射，不实现 IK/步态。

## 验证

- PlatformIO `esp32-c3-devkitc-02` / Arduino 构建成功，生成 `mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.bin` 和 ELF。
- MinGW g++ `-std=c++17 -Wall -Wextra -Werror` 编译主机假件测试并运行，退出码 0。测试含 PCA 16 通道 all-off/CH15 使能、HXY ID/量程转换、非法串口命令、ARM/限幅、3 秒超时、共总线故障时关断失败告警。
- 没有烧录实物；电压、电源电流、IMU 实际响应、PWM 波形和舵机安全性全部**未验证**。

## 下一步

按 `docs/BOARD_V1_BRINGUP.md`：先断开舵机、确认供电轨/保护，再 USB 下载及 `status`/`imu`，再空载示波器量单路 PWM，最后才用限流供电和脱开机构的单舵机测试。若实测地址/寄存器与预期不同，以器件采购型号及板测修订驱动；确认关节自由度、通道/方向、零位、限位、舵机额定电压及腿几何后，再实现校准、IK、步态。不能以软件 all-off 取代硬件急停。

## 同日追加：用户确认 8DOF MG90 与原机构

用户随后明确：八自由度、180° MG90 舵机、按 Py-Apple 原开源机构拼装、拟选 Servo1–4 与 Servo9–12。原 `config_s.py` 的八个 90° 中位和 `l1=80/l2=69/l=142/b=92.8/w=108` mm 是可调初值；原 `padog.py` 有四腿方向及小腿连杆经验式。新增 `mechanical_dog/include/Ctrl/Applepy8Dof.h` 与实现，纯 C++ 计算串联腿二维 IK、方向/机械补偿、中位及 0–180°/500–2500 µs **标称**映射，主机黄金值与边界测试覆盖；没有接入硬件 PWM 或改变台架 1000–2000 µs 限幅。详情见 `docs/APPLEPY_8DOF_PROFILE.md`。

若 ServoN 是从 1 开始的接口丝印，选定接口池在软件中为 PCA 零基 `{0,1,2,3,8,9,10,11}`；原 PCB 连接器位号 `CN` 不连续，不能与丝印混淆。**未确定**八个腿部 H/S 关节如何一一连接这些接口；须用户确认线束/实际口号后，才能实现并开放 `JointMapper` 的实机角度命令。相同机械装配不保证 MG90 花键中位完全一样；逐只标定仍必要。原步态速度未移植。

本次追加后的 PlatformIO 构建和纯 C++ 主机测试通过；仍无实物烧录/电测/PWM 波形证据。下一步是确认线束对应表，并先按原先台架流程拿到真实空载/单舵机数据。
