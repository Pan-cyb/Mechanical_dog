# 0.5.1 扩展调试范围与新照片交接

## 用户要求与实现边界

用户提供新照片并要求扩大限幅，以测舵机行程/中位。实现默认1000–2000µs、全局OFF后显式解锁500–2500µs；不主动烧录/发送PWM。电气输入范围不是机械安全行程，尚无每关节限位保存/API或自动扫限位。

`ServoController::setExtendedRange`集中约束单路、中位微调与八路预检；输出/ARM/标定期间拒绝切范围，初始化恢复锁定。主组合根给扩展范围500–2500，ChannelCalibration用此硬范围加载0–180°旧编码，PWM仍受运行范围约束。即使重启加载了范围外中位，默认八路预检全拒绝，不默认重写90°。原`applepycal/c{ch}`格式未改。

网页POST`range_extended`需页面token和`confirm=EXTENDED_RANGE_RISK`，`range_normal`恢复，状态带`extended_range/pulse_min_us/pulse_max_us`；串口`range extended/normal`显式指令。网页新增名义命令角与脉宽双向草稿，输入和同步不发PWM；“中位±1°”明确与测试区分。

## 新照片

新照命令角未确认，暂猜70°但不能标已知。相比旧90°，脚端轴投影差约4.0°、摇臂约6.8°；视角/取点和命令待确认，不据此拟合新曲线。原0.55候选不接硬件。报告 `results/V10_shank_new_sample_20260930.md`，脚本 `.work/measure_shank_new_sample.py`、参数化 `.work/overlay_shank_pair.py`，原图未改。

## 验证证据 / 构建配置

- 严格g++板级回归（`-std=c++17 -Wall -Wextra -Werror`）退出0，新增默认拒绝、解锁无PWM/NVS、500/2500端点和499/2501拒绝、通道白名单、ARM/标定模式冲突、单路OFF仍ARM、八路端点微调/保存、重启保留中位但锁范围、默认预检无部分输出、配置/关断故障拒绝解锁。
- V10 IK、V10两点映射主机回归退出0，未改公式。
- Node UI烟雾退出0：名义0/70/90/180→500/1278/1500/2500µs、非法输入清草稿、范围/通道切换无请求、STOP优先及旧回归。
- PlatformIO技能脚本先clean再build退出0，Core6.2.0，环境/board `esp32-c3-devkitc-02`，platform `espressif32`，framework `arduino`。
- ELF：`G:/PROJECT/Mechanical_dog/mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.elf`；烧录BIN：同目录`firmware.bin`，875424字节。
- 新版本0.5.1 **未烧录**，没有串口/HTTP硬件指令，没有实际NVS写入。没有真实扩展波形/行程验收。

下一步用户确认新图命令角/脉宽，再自主小步台架测行程并留裕量。PCA OE下拉，无硬急停；连杆受力、卡滞、异常持续嗡鸣时切舵机V+。不在此阶段开放站姿/WALK/TROT。
