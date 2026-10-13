# 2026-10-10 · 固件0.8.6移除协调速度拒绝

## 目标与结果

按用户授权，在不改变目录分层、足端轨迹和机械映射的前提下，彻底删除协调运动的舵机关节命令角速度限幅/拒绝。实现完成，主机/Node严格全套测试与PlatformIO构建通过；未上传、未访问硬件或NVS。

## 实现

- 删除协调速度配置、速度类预检错误、峰值速度报告和快照字段。
- 删除普通/CG预检、首段transition、current→target切换、运行时20ms帧、静态平衡和移动CG包络中的速度计算与拒绝。
- API、页面、mock和测试fixture不再包含峰值速度字段或速度错误文案。
- 保留1.5s smoothstep、周期/调度、控制器自身slew，以及IK、髋窗口、小腿表、0..180°、支撑边距、ARM/OFF、IMU/I²C故障门控。
- 固件版本更新为0.8.6；NVS格式和默认16/5/0.8/114/0/2未变。

## 回归与构建

`mechanical_dog\test\host\run_all_tests.ps1`最终exit 0。新增/改写场景证明原20/6/0.7/down115 Trot、默认高动态Walk、Drive以及current→target切换不再因速度门限失败，同时原IK、ShankMap、ServoRange、支撑边距、总线/IMU/调度故障测试继续通过。

`build-platformio`：首次普通PATH探测失败，确认CLI位于`C:\Users\0\.platformio\penv\Scripts`后，将该目录加入本次命令PATH；随后`--detect`、`--list-envs`、`--env esp32-c3-devkitc-02`构建均exit 0。未clean、未upload。

- 环境/板卡：`esp32-c3-devkitc-02`，espressif32，Arduino
- RAM：41,524/327,680 bytes（12.7%）
- Flash：983,444/1,310,720 bytes（75.0%）
- BIN：`G:\PROJECT\Mechanical_dog\mechanical_dog\.pio\build\esp32-c3-devkitc-02\firmware.bin`
- 大小/时间：1,020,432 bytes；2026-10-10 20:40:25
- SHA-256：`6B781A07A9B0F502C3F79016D99813CB30FDE055F923003223855066195F9C8F`

## 备份与风险

修改前备份：`G:\PROJECT\Mechanical_dog\.work\pre-no-speed-limit-0.8.6-20261010-202227.zip`，包含`mechanical_dog/`与`docs/`并排除`.pio`。项目不是Git仓库。

真实舵机可能无法跟随快速命令并产生滞后、冲击、压降或干涉；软件现在不会替硬件判断这些风险。本轮未烧录、未实机验证，也未放宽小腿未标定的另一侧。
