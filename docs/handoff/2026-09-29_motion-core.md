# 2026-09-29 Applepy_v1 开环运动控制交接（固件 0.4.0 本地候选）

## 目标与证据边界

用户报告已烧录 `0.3.8`，完成八路中位标定，重启后数值保持；这是用户报告，非本轮代理实板复测。用户要求补上原 V7.3 的站姿、WALK、TROT 等。参考源位于 `third_party/py-apple-servo/software/py-apple-dynamics/Py Apple Dynamics V7.3 SRC/PA-Dynamics V7.3/`，核心 `padog.py`、`PA_GAIT.py`、`PA_ATTITUDE.py`、`PA_STABLIZE.py`、`web_c.py`；固定版本和许可见 `docs/ORIGINAL_PROJECT.md`。工程目录为 `mechanical_dog/`，PlatformIO 环境 `esp32-c3-devkitc-02`。

## 本轮改动

- `Ctrl/ApplepyGaitPreview`：增加纯计算静态站姿；TROT/WALK 摆腿 X/Z 曲线按上游行为重建，两侧步幅支持正负比例（腿1/4左、2/3右），前后足端偏移、身高、Pitch/Roll 与 IK 共用。WALK 的原版 IMU/重心等待未搬来，当前是定时开环序列。
- `Ctrl/MotionController`：保存的八路中位、拟定接线与每关节实测脉宽界限进入同一预检；**初次直接输出八路保存中位（没有关节位置反馈，不保证从实际角度缓动）**，之后20ms/≤8µs逐帧缓动站立，预对齐步态首帧，有限周期运行，再缓动站立。整周期201点和运行每帧检查 IK/脉宽；运动心跳2s租约、非站立状态的150ms调度空窗、IMU过期、I²C中途写失败均尝试全关。`ServoController` 增加与单路、八路标定互斥的运动模式；整帧先校验再顺序写，但 I²C 八次写**不是硬件原子**，关断失败须切断舵机 V+。
- `ChannelCalibration` 跟踪每一路是否从有效 NVS 读出或成功保存，RAM ±1°微调会撤销此标志；八路全有有效持久中位才可能解锁运动。默认90°仍能供安全松开机构后的标定，但不能直接作为步态标定证据。
- `RobotApp`、`Port/DebugWebServer`、网页：新增受限动作 `motion_stand/walk/trot/stop/heartbeat`、运行状态和预览；只读预览用已加载的八个中位。不使用原版网络 `exec()`；GET、页面载入不发运动脉冲。页面保留单路调试“手动保持”语义，而运动是短租约。紧急软件 OFF 按钮不应被普通请求的 busy 状态禁用。

## 当前锁定与实板下一步

`Board/ApplepyV1.h` 的 `kJointWiringConfirmed=false`、`kMotionProfileConfirmed=false`、八个关节的 `kMotionMinPulseUs/MaxPulseUs={0}`，使真实固件 `motion_enabled=false`。用户已有八路中位，但尚无八关节对应接口/机械正方向逐只记录、碰撞前安全脉宽、八路带载供电/压降与可物理切断舵机 V+ 证据。**本轮未上传、未接管 COM、未向真实舵机发运动指令。**不能只翻布尔量或把台架1000–2000µs当作机械安全界限。

先在用户指导下逐个记录 Servo1–4/9–12 → 腿1–4 H/S 及方向、保守机械上下限、电压与电流；填入八个实测 limit 并确认接线，再在物理限流/切电条件下先空载示波器检查八路波形/关断，再托起机身启动静态站姿，之后一次慢 WALK、一次慢 TROT，最后在地面测试。原板可控舵机电源、低压 ADC 与遥控引脚在本板不存在；HXY 约5.5m/s²异常和机身轴向未验收，`PA_STABLIZE.py` 闭环不开放。请勿把当前软件实现称为完整的实板步态验收。

## 主机验证

- `g++ -std=c++17 -Wall -Wextra -Werror` 编译既有 `host_tests.cpp` 并运行通过（需包含新增 `src/Ctrl/MotionController.cpp`）。
- 同样编译运行新增 `test/host/motion_tests.cpp`：假PWM/NVS覆盖锁、站姿缓动、WALK/TROT、周期完成、租约、模式互斥、预检拒绝以及八次写第1/4/8次及 all-off 失败。无实物证明。
- `node test/host/web_ui_smoke.cjs` 网页脚本和受限 API 入口烟雾通过。
- `build-platformio` 在 `esp32-c3-devkitc-02` 构建成功；ELF `mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.elf`、BIN 同目录 `firmware.bin`。测试/构建成功不等于实机安全或步态可用。

下一步若发现 NVS 单路缺失，先在标定页确认并逐路保存，不能用回退90°自动解锁。关断是软件 I²C 请求，PCB 上 `OE#` 固定下拉，没有硬件急停；任何失联、卡住或故障均优先物理断开舵机电源。
