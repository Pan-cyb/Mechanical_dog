# 2026-09-29 Applepy_v1 TROT/WALK/“重心”网页预览

## 当前目标与版本边界

用户指出网页缺少原 `py-apple-servo` 的 TROT、WALK 与重心调节，要求增加完整调试功能。核对选定的 `Py Apple Dynamics V7.3 SRC/PA-Dynamics V7.3`：`control.html` 的控制入口 → `web_c.py` 命令 → `padog.py::mainloop` → `PA_GAIT.trot/walk` → `PA_ATTITUDE.cal_ges` → `PA_IK.ik` → `servo_output`。本板固件 `0.3.6` 候选以前只有单路 PWM/标定与纯计算 Applepy8Dof，确实没有网页步态入口。工作区新增 `0.3.7` **无输出预览**，未上传实板；已实板确认的最后版本仍是 `0.3.4`。

## 原项目事实与移植差异

- `PA_GAIT.py:18–36` TROT 对角腿 1/3 与 2/4 交替；`:40–86` WALK 依腿 1→2→3→4 摆动，末段共同支撑。原 WALK 会等待 IMU 前后偏移目标收敛才推进相位，有全局历史状态。`padog.py:315–351` 是分段摆动/支撑曲线；默认 5 ms 循环、`speed=.05` 的 TROT 理论周期约 100 ms，不能按此作为 MG90 安全速度。
- `PA_ATTITUDE.py:3–37` 的高度、Pitch/Roll、X 平移是四足目标相对机身的几何变换，网页所谓“重心”不是实物质心测量；旧版还有动态陀螺自稳及向前/向后/旋转重心增益。本轮仅提供**静态 X 目标偏移**与几何参数，不实现 IMU 闭环/等待和真实质量中心估计。
- 原旧控制器的八路 `(2,3),(13,12),(10,11),(5,4)` 与本板候选 Servo1–4/9–12 不同；本板 `kJointWiringConfirmed=false`，单路 `ServoController` 会在切通道前全关。不能把旧通道表或候选通道排序当作物理八关节映射。

## 实现

- 新 `include/Ctrl/ApplepyGaitPreview.h` 与 `src/Ctrl/ApplepyGaitPreview.cpp` 完全不依赖 Arduino/I²C/PCA：输入 TROT/WALK、0–1 相位、迈步/抬腿、身高、前后 X 偏移、Pitch/Roll；TROT 对角两组、WALK 四次逐腿+末段支撑，用有界平滑曲线生成足端目标，并按上游 `PA_ATTITUDE` 在 Yaw=0 下的机身几何化简叠加。调用已有 `Applepy8Dof` 逆解、镜像方向与名义脉宽计算，标注不可达/伺服角无效/超本板 1000–2000 µs 台架范围；只读且无关节→物理通道写入。曲线是参考意图的新 C++ 实现，**不与上游分段曲线逐点等价**。
- `DebugWebServer.cpp` 新 `GET /api/gait/preview`，严格校验枚举、有限数字和范围；局部对象纯计算后返回四腿目标/角/名义脉宽或 null 及 `dry_run=true, actuation_enabled=false`。不经过 `RobotApp::execute`，不写 PCA/NVS，也没有 `/api/gait/start`。`DebugWebPage.h` 新增预览表单、相位滑块、逆解/脉宽提示、响应字段校验和固定禁用的“执行步态（未开放）”；正常单路按钮保持旧路由。真实可执行态不会因为单一 `kJointWiringConfirmed` 常量变 true 而自动打开。

## 验证与未完成

严格 g++ 主机回归通过：TROT 两组对角、WALK 1→4 顺序/支撑、周期首尾、相位分界两侧连续性、X 平移、极值有限性、非法相位/NaN/枚举不发布结果；Node `vm.Script` 网页语法与只读 GET/禁用按钮/无 start 路由烟雾测试通过；PlatformIO `esp32-c3-devkitc-02` 构建成功，`firmware.bin` 约 867.8 KB。尚无真实浏览器、HTTP 边界 400、示波器、多路 PWM、机械/电源负载验收；**不能宣称已完成实体 TROT/WALK**。新网页源码仅在未烧录候选中，现有设备页面不会自行更新。

要继续到实机，用户须提供并逐只验证 `腿1H/S、腿2H/S、腿3H/S、腿4H/S → Servo1–4/9–12` 的实际映射、零位/方向、机械安全脉宽上下限、装配方向、空载/带载电流与压降、可立即物理断开舵机 V+ 的手段。随后另做多路同步输出/限幅/故障全关与网络失联策略，先静态四腿/单腿、再慢 WALK、最后 TROT。当前板 PCA `OE#` 10k 下拉，手动保持候选断网仍可能持续 PWM；软件停止不是硬件急停。加速度 `m/s²` 比例仍异常，机身安装轴亦未验证，不能开启姿态闭环。

## 2026-09-29 用户授权的拟定接线补充

用户随后确认仅用丝印 Servo1–4/9–12（PCA CH0–3/8–11）并授权项目决定实际八关节的接线顺序。已将 `kProposedJointChannels` 定为 `{0,1,2,3,8,9,10,11}`，语义顺序 `L1H,L1S,L2H,L2S,L3H,L3S,L4H,L4S`，网页预览 JSON/表格显示拟定的 Servo/CH，主机测试核验顺序/唯一性；**不是已实测的线束**，`kJointWiringConfirmed=false` 和 disabled 实体按钮不变。左右方位是本项目新约定，原 V7.3 可靠证明腿1/2与腿3/4前后分组及腿1/4、2/3镜像分组，但未明确标注左右字样；接线/校准表见[JOINT_WIRING_PLAN.md](../JOINT_WIRING_PLAN.md)。上述“须提供实际映射”现改为“已有拟定顺序，须用户按表接线并逐只验证”，其他安全前提不变。映射标签加入后的 g++ 主机、Node 网页烟雾和 PlatformIO 再验证均退出 0，最终候选 `firmware.bin` 约 869.0 KB；仍没有上传 COM4。
