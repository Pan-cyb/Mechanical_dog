# 0.8.0 V7.3主控软件适配交接

日期：2026-10-01。基线0.7.1；备份`.work/pre-v10-0.8.0-migration-20261001-143458.zip`包含源码/测试/docs和基线BIN，不含整棵`.pio`。原`third_party`只读。未连接板卡/串口、未烧录、未发送真实PWM或写板上NVS。

## A：运动/移动CG/安全会话

- `V10DriveIntent`以独立RAM forward/turn/stepping生成左右signed scale；旧非负step NVS仍为幅值。普通Walk/Trot和CG Walk支持前后、左右差速/原地转向与踏步。
- CG signed目标按三足支撑交线推导；planner以20mm/s命令slew到位，未到容差四足支撑并暂停phase，15s超时错误保持。阶段8先回body_x、阶段9再收足端offset；预检逐20ms仿真真实planner而非旧解析回程。
- `V10MovingCg`使用`height*tan(body_pitch_error*gain)+body_pitch_rate*gain`，默认滤波τ0.1s、±0.5mm、2mm/s。与planner速度独立；Stand只初始化，Walk/Trot才叠加统一body_x。
- 预检覆盖真实planner等待、signed意图、±动态包络、反馈slew、实际投影support margin、IK/q/角度/60°s，并发布预检耗时和预测实际周期。runtime仍逐帧检查。
- `V10DriveSession`使用500ms lease、server epoch和严格递增seq。begin先只读epoch检查和完整预检，lease从预检后时钟开始；renew只续期；end/Stop/Off/movingCGoff使旧包失效。过期renew会latch事件，下一tick原子Hold，不能马上begin绕过。
- 浏览器方向pad只做长按前/后/左/右/踏步；pointerup/cancel/lostcapture/visibilitychange均释放。gesture generation、单renew-in-flight与priority stop阻止晚begin/旧renew复活。
- lease/松手只冻结最后完整帧并保留舵机力矩，不承诺站稳或四脚落地；Trot可能停在抬腿帧。

## B：Profile/UI/迁移账本

- 新`applepyprof/profile`记录独立于`applepycal/c{channel}`和`applepygait/motion`，保存body/CG、planner、movingCG、独立Pitch/Roll Kp/Kd及可选Trot方向前馈f/b/turn/max。版本、长度、flags、所有有限数、整数wait timeout和FNV-1a完整记录校验均验证，保存后回读。
- Profile不保存ARM、静态/移动enable、姿态reference、Roll临时选择、drive intent、epoch/seq或会话。启动只加载参数，永不自动运动。
- Web/API/USB串口均严格白名单。输入缓冲扩为256字节以容纳合法control命令，但每loop仍最多读取32字节。未启用任意Python代码、未确认UART2或RC引脚。
- 新网页卡片显示方向会话、lease、moving CG shift、planner等待/stage、名义/实际时间、预检耗时；三组参数草稿均保留revision/readback保护。
- [迁移矩阵](../MIGRATION_MATRIX.md)逐项记录原模块/函数/网页按钮到C++/API/证据/硬件缺口；没有编造舞蹈动作库或GPIO能力。

## 红—绿与测试

- `v10_drive_control_tests`先因缺模块编译exit1；实现后严格通过。覆盖signed intent、expiry/wrap/replay、planner wait/timeout、movingCG正负/限幅/gap和非法配置。
- `v10_gait_tests`覆盖普通/CG前后左右转/踏步、实际planner轨迹、动态包络/support/60°s。
- `v10_motion_tests`真实RobotApp/fake PWM覆盖5方向多周期、lease renew不重启、迟到包零复活、movingCG等5s+transition rebase、正负非零补偿、NaN/reference/pending/transition、所有Hold别名和Profile持久化失败路径。
- `web_ui_smoke.cjs`覆盖真实pointer handlers、delayed begin松手、晚ACK、旧renew→新gesture、Stop抢占、profile/body/gait dirty与轮询。
- A/B两轮独立复审最终PASS；`powershell -NoProfile -ExecutionPolicy Bypass -File test/host/run_all_tests.ps1`由worker与主代理分别重编运行均exit0。
- 主代理最终本地mock浏览器验收（非实物）：未应用Trot前馈草稿会锁运动启动；body_x=0.3、Pitch Kp=0.6、Roll Kp=0.7/Kd=0.05及Trot前进增益3可分别应用、保存、改RAM并从Profile正确恢复。Stand→capture→movingCG开启时Stand不移身，进入Trot后保留移动反馈；Stop/回Stand不会被迟到timer复活。方向快速按下/松开最终为inactive/Hold且moving关闭。
- `build-platformio`技能：Core6.2.0 detect exit0；唯一环境`esp32-c3-devkitc-02` list exit0；首次build暴露Arduino工具链花括号赋值兼容问题，最小字段赋值修复后最终普通build exit0，未clean/upload。
- 最终`mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.bin`：生成时间2026-10-01 15:50:01，1,016,352 bytes，SHA-256 `BC26870BE1A0CC05A5BAFD38A0EF053B40579FB07EC20E5838C9A942556A088A`。二进制只读核对包含`0.8.0`、`drive_begin`、`profile_save`、`trot_ff_set`、`applepyprof`和`lease_expired_hold`。

## 仍缺条件

GPIO25电源、GPIO33低压ADC、GPIO13蜂鸣器、RC GPIO32/26/27/14和UART2在本板无确认资源，未实现虚假适配。仍缺真实电源/电流、硬急停、关节/足端/接触、实际COM、闭环调参与落地验收；软件适配不能表述为整机完成。
