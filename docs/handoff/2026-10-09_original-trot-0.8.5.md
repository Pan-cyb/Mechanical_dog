# 0.8.5 原项目Trot轨迹交接

## 授权与范围

用户确认主体机械结构几乎同原项目，仅小腿部分修改，授权恢复Py-Apple V7.3的Trot归一化足端曲线/相位。本轮只替换Trot kernel与候选参数；Walk/CG Walk、IK、改装小腿q表、通道/中位、异步预检和安全取消均不改。

参考来源：`padog.py::swing_curve_generate/support_curve_generate`、`PA_GAIT.py::trot`、`PA_ATTITUDE.py::cal_ges`、`PA_IK.py::ik`。上游根许可按GPL-3.0处理，但原文件文字存在Apache声明不一致，已在原参数文档注明。

## 实现

- 新纯Ctrl `PyAppleFootTrajectory`实现原归一化摆动三次曲线与支撑匀速曲线，并把原`u=.75`未赋值点按连续极限补成x=1。
- Trot维持50%占空、腿1/3和2/4半周期错相。原0..S输出仅减S/2适配现中心坐标；forward→IK负号仍仅一次。
- 原软件默认姿态/`in_y=17`与随`spd`变化的4/4/2偏置没有原样恢复；0.8.5 Trot前馈和IMU反馈仍默认关闭，Profile/NVS不新增默认偏置。
- 原touchdown垂直速度非零；不再对Trot强制raised-cosine。Walk/单腿/CG原分支和逐帧hash保持。
- 协调运动`max_servo_speed_deg_s`从人为60改为用户授权的120；预检与runtime共用，不绕过。该值不是MG90硬件认证。
- step/lift输入上限改30mm，但任何不可达、q越界、0..180°越界或>120°/s组合仍拒绝。

## 参数证据

候选`16/5/0.8/114/0/2`在用户中位`90,100,90,90,90,97,90,102`和合理非对称中位、前/后/左右转/踏步全部PASS；最坏为用户中位前进98.032°/s，距120约21.968°/s。默认不启IMU移动反馈或Trot前馈。

按当前已标定模型/软件门限，原`step30/lift30/height110`即使period10仍在phase .087先以`hip_window`拒绝；step=0隔离水平后phase .092以`shank_map`拒绝。原理论0.1s低于当前0.4s周期输入下限。未夹紧、未提高门限继续放行；这不是实物机构物理极限结论。

## 测试

- 红灯：旧smoothstep Trot在摆动`u=.125`不满足原版首1/4保持x的冻结断言，exit1。
- 独立冻结原函数样点`u=0/.25/.5/.75/1`，覆盖u=.75修补、幅度、支撑匀速、非零touchdown垂速、方向/转向/踏步、相位与C0闭合。
- 候选参数、120°/s真实拒绝、30mm不可达和NVS默认/保存优先均有host回归；Walk/CG对0.8.4旧源码的全周期逐帧FNV hash不变。
- `test/host/run_all_tests.ps1`最终全套严格exit0；主审与独立审核核心PASS，mock/doc一致性二审PASS。
- 第一次固件构建暴露旧RISC-V工具链不接受聚合赋值；仅把`out={...}`改为等价字段赋值，重新跑全套测试后构建成功，未改变轨迹算法。
- PlatformIO 6.2.0 detect/list/build均exit0，未clean/upload。最终`firmware.bin`为1,023,456 bytes，生成时间2026-10-09 01:28:22，SHA-256 `41CF2DD7D2EAE097B28E446F235A0E2412B88BC44FC32606BAAFD3A2DC327B7A`；`firmware.elf`为12,675,864 bytes，SHA-256 `ACCA1B9274BBC346F5A2B0D3C6229917EDB2DAD8C4B55EE19F54377307EAB256`。
- PlatformIO内存报告：RAM 41,812/327,680 bytes（12.8%），Flash 986,322/1,310,720 bytes（75.3%）。

## 未验证

未烧录、未访问板卡/串口/真实NVS。软件轨迹与主机预检通过不证明已经达到原项目落地速度、步幅、冲击或稳定性；首次仍须悬空、限流、可立即断舵机V+。烧录后已有NVS步态不会被默认覆盖，需手动选择“Trot参数→应用参数”。
