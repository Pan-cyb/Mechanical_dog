# 2026-10-12 · 固定直行修正默认0.25 · 固件0.8.11

## 当前目标

把现有RAM直行修正的启动默认值固定为`+0.25`，保留手机网页手动调整；手动值不写Profile或motion NVS，重启恢复0.25。不得修改Trot Pitch offset、步态算法、工程分层或NVS schema。

## 修改前证据

- 定向备份：`.work/pre-fixed-straight-trim-0.8.11-20261012-204423.zip`，SHA-256 `54511FE3D95DD196BBA38AB4B5ABBFDD80F7D87EB57F4E6580C16E5C27A7937C`；包含源码、测试、docs及0.8.10 `firmware.bin`，未包含完整`.pio`目录。
- 用真实0.8.10源码单独编译`.work/fixed_trim_0_8_10_probe.cpp`：Trot `17.9/12.8/0.5/down114/body_x8`在两组舵机中位、前进/倒退/左转/右转/踏步、Pitch offset `0/-0.5°`共20个组合全部PASS。
- 纯函数结果：前进有效intent为`forward=1, turn=-0.25`、scale `1.00/0.75`；倒退为`forward=-1, turn=+0.25`、scale `-1.00/-0.75`；显式转向和踏步不变。
- 红测：先把RobotApp boot断言改为0.25，在旧实现上按预期失败于`v10_motion_tests.cpp`，随后再实现默认常量和重置路径。

## 完成工作

- `V10DriveControl.h`增加唯一默认常量`kDefaultStraightTrim=0.25f`。
- RobotApp成员初值与`begin()`重置都使用该常量；`V10DriveIntent.turn`仍默认0，trim仍只经`applyStraightTrim()`产生有效turn。
- 固件版本升为0.8.11；串口help和设置响应明确“重启恢复默认0.25”。
- 手机网页静态初值改0.25，状态轮询仍以板端读回覆盖；范围、dirty/readback、迟到ACK和运动中门控保持。
- Stop/Off保留手动值；Profile保存/加载及motion NVS保存/加载不改变当前值；新RobotApp启动恢复0.25。
- 更新README、状态/进度、网页指南、运动学、迁移矩阵、原版参数证据、项目交接和技术文档。

## 验证

- `powershell -NoProfile -ExecutionPolicy Bypass -File test\host\run_all_tests.ps1`：exit 0；严格C++编译使用`-Wall -Wextra -Werror`，C++/Node/Web全通过。
- PlatformIO技能脚本`--detect`：Core 6.2.0；`--list-envs`：`esp32-c3-devkitc-02`；指定环境构建成功，12.5秒。未clean、未upload。
- PlatformIO最终报告：RAM 41,556/327,680 bytes（12.7%），Flash 992,066/1,310,720 bytes（75.7%）。
- `firmware.bin`：1,029,136 bytes；时间`2026-10-12 20:50:42 +08:00`；SHA-256 `4F789C66C4381470C0F13FC11F24ECA9BA8A1D156A4FA2DB38C7FA4B872D35F0`。

## 未验证与风险

- 未烧录、未连接板卡/串口、未发送真实PWM、未读写真实NVS。
- 0.25来自用户实机调参决定；软件预检只证明命令轨迹处于当前模型边界内，不证明实体能够长期直行，也不消除摩擦、装配、中位、负载和舵机响应不对称。
- 首次验证应确认网页版本0.8.11、板端trim读回0.25，并在可立即断开舵机V+的条件下短距离测试。
