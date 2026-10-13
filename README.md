# Applepy_v1 / Structure V10 传统控制固件

这是面向Applepy_v1控制板与Structure V10串联腿的8DOF四足机器人C++复刻工程。当前传统控制里程碑为固件`0.8.14`，工程入口在[`mechanical_dog/`](mechanical_dog/)。

用户已通过实机体验认可当前传统控制部分：基础站立、Walk/Trot、前后左右点击锁定、原地踏步、实时高度/Pitch/Roll/body_x、八路中位标定、参数预检与保存等均已达到本阶段使用目标。这里的“里程碑完成”不等于实体闭环完全验证：系统仍是8DOF开环舵机命令，没有足压、触地或关节位置反馈。

## 当前能力

- ESP32-C3 + PCA9685八路舵机协调控制，HXY ICM-42688P姿态采集。
- Stand、Walk、Trot、前进/后退/左右转、点击锁定方向与原地踏步。
- 手机网页实时调节髋轴到足端`down`高度、Pitch、Roll和前后重心`body_x`；完整IK/映射/舵机范围预检后原子提交。
- 直行修正、Trot Pitch前馈、静态IMU姿态补偿、移动Pitch→body_x补偿与CG Walk。
- 八路中位、相对中位、小腿映射台架工具；motion、Profile与中位三类独立NVS记录。
- Stop保持最后完整帧，Off尝试软件全关；方向会话保留500ms租约。

## 已知限制

- 用户实机观察：原地踏步仍会轻微后退，判断与前向重心不足有关。当前默认`body_x=+8mm`且网页范围为`-8..+8mm`，在没有足端/关节/接触反馈的条件下，本里程碑暂不继续调修。该现象不是“无法修复”的结论，后续可结合机械配重、足端接触测量或闭环反馈重新评估。
- 软件预检通过不证明舵机实际跟随、供电裕量、打印件无干涉或落地长期稳定。
- PCA9685 `OE#`固定下拉；软件Off不是物理急停，实机测试必须具备舵机V+物理断电手段。

## 文档入口

- [当前实现状态](docs/REPLICA_STATUS.md)
- [复刻进度](docs/REPLICA_PROGRESS.md)
- [网页操作指南](docs/WEB_UI_USER_GUIDE.md)
- [V10运动学与范围](docs/V10_KINEMATICS.md)
- [原项目与许可证据](docs/ORIGINAL_PROJECT.md)
- [传统控制里程碑交接](docs/handoff/2026-10-13_traditional-control-milestone-0.8.14.md)

## 测试与构建

```powershell
cd mechanical_dog
.\test\host\run_all_tests.ps1
```

```powershell
cd mechanical_dog
pio run -e esp32-c3-devkitc-02
```

构建需要PlatformIO Core 6.2.0或兼容版本。

最终`0.8.14`固件：`mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.bin`，1,040,512 bytes，SHA-256：`8473C502941C254F6FFEC5B9A9D041C9C562AA6FBB84E81357E68C5766C827B7`。

## 许可

软件参考上游`ToanTech/py-apple-dynamics@c69ab76df92a9a162b6608ea0ffb72d23c1e8990`，其仓库根许可证为GNU GPL v3。本复刻软件发布遵循根目录[`LICENSE`](LICENSE)中的GPL-3.0。`third_party/`中的硬件、CAD、文档及其他第三方材料仍分别受其原始许可证或权利声明约束，不能因本软件采用GPL-3.0而推定为同一许可。
