# 0.8.4 增量步态预检交接

## 根因证据

用户0.8.3真实`/api/state`：Walk参数`6/2/2.5/108/0/2`，body全0、CG/FF/移动反馈关闭，PCA/IMU正常且ARM/Stand；八中位按新映射为`90,100,90,90,90,97,90,102`。预检实际报告`error=none`、峰值35.73°/s，但`duration_ms=476`。

旧`requestMotionWithAdvanced()`同步完成全周期计算后再次检查100ms IMU freshness，因此合法报告因计算耗时被误拒绝；下一主循环恢复采样，网页IMU又变绿。host探针用真实RobotApp与合成0/100/101/150ms跳时复现完全相同的拒绝边界，且无新PWM。

## 实现

- `V10PreflightCursor`每次step最多一帧或一次有界8关节收尾；`preflightAdvanced/preflightPlannedCg`同步wrapper也驱动同一cursor，避免双算法漂移。
- 普通预检保留5ms密采样、周期首尾、初始smoothstep峰值；planned CG保留三动态场景、20ms planner、阶段8/9、等待timeout、每场景2000步和实际周期。
- RobotApp每tick处理最多一帧。checking独立于`motion_pending`；旧运动照常调度，检查期不发新目标PWM。
- PASS提交时重验参数/profile revision、中位、姿态参考、IMU/ARM/fault、Drive lease，并用当时实际最后帧复核切换峰值；随后设置`motion_pending`，仍等下一新IMU样本。
- DriveBegin先建立500ms provisional session并立即ACK queued，原网页可150ms renew。释放、expiry、Stop/Off、BalanceOff/MovingCgOff、故障、参考重置都会取消job；失败不修改旧mode/intent/反馈。
- `/api/state.motion.preview`新增`pending/stage/result/frames_checked/max_slice_ms`；主页面只显示简短checking/通过/拒绝/取消，高级区保留具体码。

## 测试

- 正式红灯：用户参数和中位、合成476ms旧同步成本下`queued_walk.ok`失败，exit1。
- Green：推进型FakeClock将476ms成本分布到slice计时期间；断言累计duration≥476ms、max slice≥1ms、每tick frame增量≤1、IMU多次真实read、checking零PWM、PASS后仍等新IMU才启动。
- 覆盖100/101ms边界、普通/慢速/3s/多周期、CG三场景与移动包络、旧运动不饿死、Stop/Off/反馈关闭、IMU fail/reference reset、stale completion、Drive renew/release/expiry/reject与旧intent保留。
- 使用本轮zip中的0.8.3旧`V10Gait.cpp`生成独立fixtures，逐项固定`ok/error/maxrate/leg/joint/phase/supportMargin/actualCycle`，覆盖Stand/Walk/Trot、反转/转向/踏步、不同中位、超速、IK、shank、ServoRange和planned CG。
- `test/host/run_all_tests.ps1`最终全套严格exit0。独立高风险复审另跑9个exe+Node、逐一核对15组0.8.3基线fixture，结论PASS。
- PlatformIO 6.2.0 detect/list/build均exit0，未clean/upload。最终`firmware.bin`为1,022,496 bytes，生成时间2026-10-08 23:56:36，SHA-256 `AAEA529DDA1C862BB292E478DCE474EE7DF5AD2BA9058FA112BB03319DC476DD`。

## 未验证

用户的476ms是0.8.3整次同步计算实测；host的分布成本与`max_slice_ms`只是合成调度证据。0.8.4尚未上板，ESP32-C3单帧最坏耗时、真实网页终态与运动切换仍待另行授权验证。所有100/200ms、5ms密采样、60°/s、几何/支撑限制均未放宽。
