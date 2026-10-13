# 2026-10-13 · 实时body_x滑块 · 固件0.8.14

## 结果

- `pose_live_set`由三字段改为`height pitch roll body_x`四字段；旧三字段明确Invalid。HTTP参数固定为`height/pitch/roll/x`。
- RobotApp同时构造motion/Profile候选并完整Stand预检，成功后一次提交height/Pitch/Roll/body_x；越界或不可达时四字段及revision全部保持。
- OFF/idle仍只更新RAM、零PWM、不ARM；Stand/Hold仍等待新IMU并复用1.5s平滑重定位。
- 网页新增`bXRange`，−8..+8mm、step0.1、默认+8，与number双向同步并加入150ms合并、单并发/latest-wins/读回/失败回退。八个实时range/number共享moving CG、自稳、校准、非稳定mode、预检和故障门控。
- body_x不再产生body dirty；旧“应用X/CG设置”按钮及JS删除。CG Walk enable/shift仍由高级区“应用重心设置”提交；Profile仍保存Pitch/Roll/body_x，motion NVS仍保存height等六参数，schema未改。
- 默认Trot/Walk、trim、Trot Pitch、方向click/lease、Stop/Off与项目结构未改。

## 备份与TDD

- 修改前备份：`.work/pre-live-body-x-slider-0.8.14-20261013-133721.zip`，1,096,577 bytes，SHA-256 `6051DF9AD14D2CCCFAA271E02A430D555FDC9D2B6334E012AE2B2167C599C19C`。
- 先更新Web断言，旧0.8.13按预期红于缺少`bXRange`；实现后Node与严格全套测试通过。
- Host覆盖CLI四字段/三字段Invalid、OFF零PWM不ARM、Stand/Hold、有效组合原子提交、body_x±8通过、±8.1与不可达tuple原子拒绝/revision不变、Profile保存重启恢复x。
- Node覆盖bX range-number双向、四字段合并/latest-wins/迟到ACK/失败回退、queued/inflight轮询保护、bX不置body dirty、八控件一致门控；旧方向/踏步/Stop/lease回归通过。

## 验证与产物

- `test\host\run_all_tests.ps1`：严格C++、Node、Web全套exit 0。
- `build-platformio`：PlatformIO Core 6.2.0 detect成功，list-envs确认`esp32-c3-devkitc-02`，指定环境build成功；未clean、未upload。
- RAM 41,556/327,680 bytes（12.7%）；Flash 1,003,376/1,310,720 bytes（76.6%）。
- `firmware.bin`：1,040,512 bytes，时间2026-10-13 13:43:06，SHA-256 `8473C502941C254F6FFEC5B9A9D041C9C562AA6FBB84E81357E68C5766C827B7`。

## 改动文件

- `include/Ctrl/Command.h`、`include/UserApp/RobotApp.h`
- `src/Ctrl/Command.cpp`、`src/UserApp/RobotApp.cpp`
- `src/Port/DebugWebServer.cpp`、`src/Port/DebugWebPage.h`
- `test/host/v10_motion_tests.cpp`、`web_ui_smoke.cjs`、`mock_web_ui_server.cjs`及版本断言
- README、状态/进度/指南/迁移矩阵/运动学、项目交接/技术文档、本handoff

## 未验证

未上传、烧录、连接板卡/串口、发送真实PWM或写真实NVS。真实手机body_x拖动手感、Wi-Fi时延、连续重定位时的舵机跟随/电流/冲击、机构干涉与真实重心变化均待悬空限流实测；软件Off仍不是物理断电。
