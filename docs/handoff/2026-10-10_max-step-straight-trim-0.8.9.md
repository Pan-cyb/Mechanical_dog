# 0.8.9 最大步长与RAM直行修正交接

## 结果

- 内置/Trot参数改为`17.9/13.1/0.5/114/0/2`；网页普通Walk按钮改为`25.3/7.1/0.5/108/0/2`。慢速与CG Walk 3.2s专用预设不变。
- 新增`straight_trim_set <-0.30..+0.30>`及手机网页“直行修正”。默认0、只在RAM中，Stop/Off保留，重启归零，不写motion/Profile NVS。
- 正trim只对直行生效：前进使用`effective_turn=-trim`，倒退使用`+trim`，两者都减弱物理右侧；用于纠正持续左偏。负值镜像减弱左侧。显式左右转和踏步不变。
- 设置只允许OFF/idle或稳定Stand/Hold，调用本身零PWM；网页未应用草稿锁定启动并回读trim/effective turn。

## 修改前边界证据

使用0.8.8真实`V10Gait::preflightAdvanced()`，`body_x=+8`，中位：

```text
{90,100,90,90,90,97,90,102}
{95,100,85,80,100,80,85,100}
```

分别扫描前进、倒退、左转、右转、踏步：

- Trot `lift=13.1, period=.5, down=114, forward=0, bias=2`：共同0.1mm上限17.9。18.0在倒退/左右转于phase .340触发`hip_window`；前进单独可到23.4，踏步因前后幅值为0可接受到输入上限。
- Walk `lift=7.1, period=.5, down=108, forward=0, bias=2`：共同0.1mm上限25.3。25.4在非踏步intent于phase .040或.160触发`hip_window`；踏步同样不使用step幅值。
- 30mm对实际有前后幅值的组合均不是共同可用值；30.1由`parametersValid()`硬上限拒绝。

因此17.9与25.3是各模式在指定组合中的共同最大0.1mm档，不是实物负载、摩擦、舵机跟随或防干涉认证。

## 左偏诊断边界

普通Walk/Trot名义命令为`forward=1, turn=0`，原`driveScales()`左右相等。实物持续左转更符合左右真实推进、摩擦、舵机输出、中位、装配或载荷不一致。当前无Yaw闭环、足压、足端或关节反馈，代码不能自动判定根因或计算补偿；本轮不猜默认值，保持trim=0。

手机调参：停止到稳定Hold或Stand → “直行修正”先填`+0.05` → “应用修正（RAM）” → 启动Trot/Walk观察。仍左偏则小幅增加；偏向反侧则减小或改负值。每次保持悬空/限流/可物理断电条件。

## 文件与兼容性

主要修改：`V10DriveControl.h/.cpp`、`V10Gait.h`、`Command.h/.cpp`、`RobotApp.h/.cpp`、`DebugWebServer.cpp`、`DebugWebPage.h`及host/Node测试和当前文档。项目结构未改。

motion/Profile NVS schema/version未改；合法旧motion NVS仍覆盖新的内置Trot默认。trim不进入任何NVS记录。

修改前备份：`.work/pre-max-step-straight-trim-0.8.9-20261010-224808.zip`。

## 验证

- Red：新增纯函数测试先因`applyStraightTrim`不存在而编译失败。
- Green：`test\host\run_all_tests.ps1`全套C++/Node/Web回归exit 0。
- 覆盖step边界、trim 0兼容、±0.05前后物理侧衰减、显式转向/踏步不变、NaN/越界拒绝、RobotApp预检/preview/方向会话、设置零PWM/运动中拒绝/Stop-Off保留/重启0及网页dirty/readback。
- `build-platformio` detect/list/build均成功，环境`esp32-c3-devkitc-02`，未clean/upload。RAM 41,540/327,680（12.7%），Flash 987,458/1,310,720（75.3%）。`firmware.bin` 1,024,528 bytes，SHA-256 `B9AFBD333D0D68D9AE34B6D8BD39B23BF978F51972DE2155A30B6DFD424E774B`。

未上传、未接板、未访问串口、未发送真实PWM、未写板上NVS。
