# 参考项目与复用边界

> 2026-09-24 现行实物是 `Applepy_v1`（ESP32-C3-WROOM-02-H4、HXY ICM-42688P、PCA9685），工程在 `../mechanical_dog/`。下文 OQ12/ESP32-WROOM-32E/MPU6050/2S 是此前研究及方案背景，**不作为现行板的引脚、电源或验收依据**；详见 [Applepy_v1 首次联调](BOARD_V1_BRINGUP.md)。

## 现行输入与参考资料（2026-09-24）

| 类别 | 位置 | 使用边界 |
|---|---|---|
| 已打样主控板 | `G:\PROJECT\Mechanical_dog\SCH_Schematic1_2026-09-24.pdf`，3 页，图框标 `Applepy_v1` | 固件引脚、地址、电源、外设设计依据；不能替代电测、装配和 PCB 连通性检查。 |
| 固件工程 | `G:\PROJECT\Mechanical_dog\mechanical_dog` | PlatformIO `esp32-c3-devkitc-02` + Arduino，当前上板验证独立于旧方案。 |
| 舵机/行为参考 | `G:\PROJECT\Mechanical_dog\third_party\py-apple-servo` | Py-Apple Dynamics V7.3 是 MicroPython 8DOF。固定提交及 GPL-3.0 授权说明见下文；研究算法和数据流，不将 8DOF 通道/IK 误认为新板 12DOF 定义，不直接复制代码而隐去许可。 |
| IMU 工程参考 | `G:\PROJECT\Electronics Design Contest\the second semester of junior year\M0-chip-package\Myproj` | 仅针对芯片驱动、寄存器和量程实现逐文件审查；不同 MCU 的 BSP/引脚/调度不可照搬，器件批次和许可待核。 |
| C++ 分层参考 | `G:\PROJECT\robotic_arm\之前的工作\旧开源资料\Dummy-Robot-2.Firmware\Ctrl-Step-Driver-STM32F1-fw` | 只借用 `UserApp`、`Ctrl`、`Port`、`Core` 职责边界；不移植 STM32 步进电机控制/FOC、HAL 或中断频率。 |

软件参考的固定上游版本及许可记录保留在下文，原始硬件资料和新板设计不可混用。

本项目不是对某一块商业控制板的 PCB 一比一复制，而是设计一套可自主制造、兼容部分 OpenCat 软件思路的四足机器人平台。

## 主要参考项目

- OpenCat / NyBoard: https://github.com/PetoiCamp/OpenCat-Quadruped-Robot
- OpenCat ESP32 / BiBoard 软件: https://github.com/PetoiCamp/OpenCatEsp32-Quadruped-Robot
- BiBoard V0 指南: https://guide.petoi.com/zhong-wen-ban/biboard/kuai-su-shang-shou-zhi-nan
- BiBoard V1 指南: https://guide.petoi.com/zhong-wen-ban/biboard/biboard-v1-yong-hu-zhi-nan
- Sesame Robot: https://github.com/dorianborian/sesame-robot
- MKme SpotMicro / NovaSM3: https://github.com/MKme/quadrupedal-robot
- ElectroPup: https://github.com/reubenstr/ElectroPup

## 复用原则

1. 优先复用明确许可的代码、文档和机械文件；Sesame 为 Apache-2.0，OpenCat 固件为 MIT。
2. 对没有明确硬件许可的 BiBoard、MKme、ElectroPup，仅使用公开接口、功能和尺寸作为参考，不复制 PCB 版图或受限文件。
3. 本项目的 PCB、BOM、固件适配层和机械 CAD 采用独立命名、独立布局和独立设计记录。
4. 目标是“功能兼容和可替换”，不是宣称与 Bittle/BiBoard 一比一等价。

## Py-Apple 项目筛选结论（2026-08-26）

用户指定的聚合仓库：<https://github.com/ToanTech/py-apple-quadruped-robot>

本次分析的聚合仓库临时快照：`C:\Users\0\AppData\Local\Temp\codex-upstreams\py-apple-quadruped-robot-efdf6e4542ca4663a9e05f88ce49defd`；后续复刻以 GitHub 固定提交 URL 为准，不依赖该临时路径。

永久本地资料根目录：`G:\PROJECT\Mechanical_dog\third_party\py-apple-servo`。其中 `software/`、`hardware/`、`mechanical/` 分别保存固定提交和 ASCII 工作副本；下载哈希记录见 `third_party/py-apple-servo/docs/DOWNLOAD_MANIFEST.md`。

聚合仓库 `master` 当前提交为 `d2bb34f82ff1d1c69c657c50e3af1608ebb6d1fb`（2022-04-11），但仓库内打包的舵机软件只有 `Py Apple Dynamics V6.8`；其三个顶层目录的版本并不构成同一代产品，且 `二代菠萝（Py-Apple Structure V10.0 串联）` 中的 BOM 标题仍为“菠萝狗 V7.0”，只列出 8 个 `mg90s` 舵机。

因此，本项目把“最新、非无刷、可形成整套舵机狗”的上游基线固定为三个子仓库在各自 `master` 的提交，而不是直接复制聚合仓库：

| 子系统 | 选定提交 | 版本/事实 | 选择理由 |
|---|---|---|---|
| 软件 | `ToanTech/py-apple-dynamics@c69ab76df92a9a162b6608ea0ffb72d23c1e8990` | `Py-Apple Dynamics V7.3`；8DOF；ESP32 MicroPython | 子仓库 README 明确称 V7.3 为最新舵机控制软件；2022-05-20 的提交仅修正 OpenMV 例程，核心 V7.3 代码仍保持可追溯 |
| 控制器 | `ToanTech/py-apple-controller@83d989b5d24b92cb699e61e1c798fc7b837c6df5` | `Py-Apple Controller V4.0` 直插分立元件板 | README 明确称 V4.0 为最新控制器；支持 ESP32、MPU6050、PCA9685、5--8 V PWM 舵机、最多 16 路 PWM |
| 结构 | `ToanTech/py-apple-structure@84e3048eb545fca0ae44138a09eeb14e54feba5c` | `Py-Apple Structure V10.0` 串联腿 | README 称 V10.0 为最新舵机结构；实际交付为 STL 压缩包，BOM/零件与 V7.0 8DOF 快装腿一致 |

明确排除：`Py-Apple BLDC`/`py-apple-bldc-quadruped-robot`、Structure 中“无刷版本 V0.1 腿部设计”、以及 Dynamics 的 V4.0/V4.5/V6.0/V6.5/V6.8 古早目录。它们不是本次舵机狗基线；V6.8 仅作为聚合仓库的历史快照保留。

重要边界：上述上游“完整舵机狗”是 8DOF（每腿髋/小腿 2 个舵机），不是 12DOF（每腿 3 个舵机）。如果 OQ12 仍以 12DOF 为目标，上游只能提供控制器、电源、IMU、PWM、网页协议和部分步态/姿态算法参考，不能直接作为 12DOF 机械与 IK 成品。

### 许可记录

三个当前子仓库根目录的 `LICENSE` 均为 GPL-3.0；部分 Python 文件头和旧教程仍写 Apache-2.0，存在声明不一致。复刻时按仓库根目录 GPL-3.0 处理：保留版权与许可证、标注修改、发布衍生软件对应源码；PCB/CAD 文件的许可范围需逐文件确认，不能仅凭 Python 文件头推断。
