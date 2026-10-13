# 2026-10-12 · 实时姿态与点击锁定方向 · 固件0.8.13

## 目标与结果

- 高度/Pitch/Roll不再需要应用按钮。网页以150ms debounce把最新完整tuple发送为`pose_live_set height pitch roll`；最多一请求在途，请求期间继续拖动仅保留最新值。成功须状态读回匹配；拒绝恢复板端有效值；旧ACK不覆盖新输入。
- 固件先构造候选motion/Profile并完成Stand的BodyPose、IK、hip window、shank map和servo range预检，再一次提交三字段。无部分提交。OFF/idle只更新RAM、零PWM、不隐式ARM；已ARM的Stand/Hold等待新IMU后复用1.5s smooth transition。连续拖动可从当前完整过渡帧重新定向。
- 前/后/左/右/原地踏步改成click锁定。活动按钮`aria-pressed=true`；点击另一方向先`drive_end`并读回新epoch，再`drive_begin`；点击当前按钮或顶部Stop结束并保持。页面隐藏/renew网络失败停止本地续租，MCU 500ms lease仍兜底冻结。Off继续走优先重试路径。
- 默认Trot/Walk、straight trim、body_x、Trot Pitch、motion/Profile NVS schema均未改。

## 角度probe

`.work/live_pose_range_probe_0_8_12.cpp/.txt/.md`固定`body_x=8`，扫描down100..115、0.1°并覆盖两组舵机中位，直接复用当前BodyPose/IK/map/servo规则。

- 跨包络单轴最大连续Pitch约−7.0/+5.1°，Roll至少±10°（在±10处由旧BodyPose软件上限截断）。最终统一硬范围取Pitch−7..+5°、Roll−10..+10°。
- down108：Pitch−6/+4、Roll±7.9、四象限同幅±3.2°。
- down114：Pitch±3.2、Roll±5.7、四象限同幅±2.0°。
- down115：Pitch±2.4、Roll±4.1、四象限同幅±1.5°。
- 最佳四象限同幅区约down110.2..110.7，为±3.7°。硬范围不是任意组合保证，因此每次tuple必须预检。

## 验证

- 修改前备份：`.work/pre-live-pose-click-drive-0.8.13-20261012-225228.zip`，1,083,661 bytes，SHA-256 `DBC2DAC058E46C943740C8CC78AFF2C1513A0FA24EF2CF243E463043468DE596`。
- TDD先把Web长期按压/零自动POST预期改成实时/click需求，Node按预期红于旧Pitch范围；实现后`test\host\run_all_tests.ps1`严格全套exit 0。
- Host覆盖命令解析、OFF零PWM不ARM、硬边界与非法tuple原子不提交、Stand/Hold实时transition；原drive epoch/seq/replay/预检取消矩阵保留。
- Node覆盖三滑块合并、单并发、latest-wins、成功读回、失败回退；click begin/renew、pointerup无绑定、活动态、方向切换、step=1、visibility/network/Stop-Off。
- 主审核修复了实时控件disabled状态被旧body门控覆盖的问题，并封住普通action与实时姿态请求并发；新增对应Node回归后严格全套测试仍exit 0。
- `build-platformio`脚本detect/list/build `esp32-c3-devkitc-02`全部成功；未clean、未upload。
- RAM 41,556/327,680 bytes（12.7%）；Flash 1,002,786/1,310,720 bytes（76.5%）。
- `firmware.bin`：1,039,920 bytes，时间2026-10-12 23:12:27，SHA-256 `684531E750DF23FA9225D9E75A4A572DDF3026A02C2E3FE41482CACF9BAD194A`。

## 改动文件

- `include/Ctrl/V10BodyPose.h`、`include/Ctrl/Command.h`、`include/UserApp/RobotApp.h`
- `src/Ctrl/V10BodyPose.cpp`、`V10ControlProfile.cpp`、`V10Gait.cpp`、`Command.cpp`
- `src/UserApp/RobotApp.cpp`、`src/Port/DebugWebServer.cpp`、`DebugWebPage.h`
- `test/host/v10_body_balance_tests.cpp`、`v10_gait_tests.cpp`、`v10_motion_tests.cpp`、`web_ui_smoke.cjs`、`mock_web_ui_server.cjs`及版本断言
- README、状态/进度/指南/迁移矩阵/运动学、项目交接与技术文档、本handoff

## 未验证与风险

未上传、烧录、连接板卡/串口、发送真实PWM或写NVS。真实手机触控、网络延迟、连续重定向时舵机跟随/电流/冲击、机构干涉、点击方向实体响应均未验证。软件Off不是物理断电，首次必须悬空、限流并准备立即切断舵机V+。
