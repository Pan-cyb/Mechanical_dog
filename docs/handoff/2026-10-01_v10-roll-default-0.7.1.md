# 0.7.1 Roll默认方向确认补丁交接

日期：2026-10-01。基线0.7.0；补丁前备份：`.work/pre-v10-0.7.1-roll-default-20261001-141731.zip`。未连接板卡、未开串口、未烧录、未发真实PWM、未写NVS。

## 已确认事实与实现

- 用户确认：“左倾=左侧降低、右侧抬高，sensor Roll为正；右倾相反。”
- body坐标定义Roll正=左侧抬高，所以`body_roll=-sensor_roll`，默认sensor sign固定为−1。Pitch既有`body_pitch=-sensor_pitch`不变。
- 唯一默认常量`dog::kDefaultRollSensorSign=-1`位于纯Ctrl头`V10StaticBalance.h`；`DebugSnapshot`、`RobotApp`成员和`begin()`均引用该常量，避免header/启动重置不一致。
- 仍保留手动−1/0/+1：0现在表示主动关闭Roll闭环，+1表示手动覆盖。改变符号仍清参考。
- 重启只恢复已确认方向约定−1；balance enable/reference仍关闭/清空，机器人不ARM、不发PWM、不写中位或步态NVS。
- 网页默认选择−1并显示“Roll已确认 body=-sensor”；RPY遥测继续显示原sensor frame，没有被反号。
- 固件版本与启动/状态版本升级为0.7.1。中位、通道、几何、CG阶段、控制增益和NVS格式均未改变。

## 验证

- 默认−1无需`balance_roll_sign`命令：boot后未ARM、无reference、无PWM、无NVS写；Stand→capture→on后sensor Roll正（机身左低）产生正Roll修正，sensor Roll负产生负修正。
- 默认Roll与Pitch同时扰动的八路命令速率保持≤60°/s。显式0仍为Pitch-only；改符号清reference；Off不自启动；新`Rig`模拟重启恢复−1但不恢复闭环。
- Node覆盖首次板端−1自动选中且不POST、0禁用文案和Roll符号草稿轮询保护。
- `powershell -NoProfile -ExecutionPolicy Bypass -File test/host/run_all_tests.ps1`：完整严格回归exit0。
- `build-platformio`技能：Core6.2.0 detect exit0；唯一环境`esp32-c3-devkitc-02` list exit0；普通build exit0，未clean/upload。
- `mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.bin`：967,088 bytes；SHA-256 `559057E232F526349EE478BF1692705E243514A97D19132FC8F553D3478B7900`。二进制只读核对包含`0.7.1`、`roll defaults -1 confirmed`、`Roll balance explicitly disabled`和`body=-sensor`。

## 未验证

方向观察已确认，但真实闭环增益、舵机/连杆响应、负载和落地稳定仍未上板验证。首次仍须悬空、限流并准备物理断电。
