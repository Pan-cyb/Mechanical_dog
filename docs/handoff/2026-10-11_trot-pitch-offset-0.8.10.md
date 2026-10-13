# 0.8.10 Trot专用Pitch补偿交接

日期：2026-10-11。未上传、未连接板卡/串口、未发送真实PWM、未写真实NVS。

## 行为变化

- 内置与网页Trot预设从`17.9/13.1/0.5/114/0/2`调整为`17.9/12.8/0.5/114/0/2`；Walk保持`25.3/7.1/0.5/108/0/2`。
- `V10AdvancedMotionParameters`新增默认0的`trot_pitch_offset_deg`。仅Trot执行`effective pitch=Profile pitch+offset`后进入`V10BodyPose`；Stand、Walk、Leg和CG Walk忽略它。
- 新命令/API为`trot_pitch_set <-0.5..0.0>`。负值压低前端；只允许OFF/idle或稳定Stand/Hold且反馈不忙。每次设置用当前motion参数、八路中位、Profile和straight trim所得默认前进Trot做完整预检；失败不修改，成功设置零PWM。
- 手机网页新增“Trot Pitch补偿 °”，0.1°步进，带dirty锁、板端offset/有效Pitch/revision读回和迟到ACK隔离。Stop/Off保留该值，重启归零；Profile与motion NVS均不保存或恢复它。
- moving CG仍保持默认±0.5mm/2mm/s且默认关闭；本功能是开环Trot角度前馈，不是IMU移动闭环。此前“最大步态与moving CG包络冲突”的诊断保留在`.work/moving_cg_range_probe_20261010.md`，本轮没有扩大其范围或改控制器。

## 边界证据

真实0.8.9 `V10Gait`、`body_x=+8`、两组中位`{90,100,90,90,90,97,90,102}`与`{95,100,85,80,100,80,85,100}`、前/后/左/右/踏步五intent扫描：

- `-0.5°`配lift13.1时不存在可行step，step0仍在phase .240腿1触发`shank_map`。
- 保持step17.9时最大共同lift为12.8；12.9下一档在中位A/前进、phase .250、腿1触发`shank_map`。
- `17.9/12.8/-0.5°`两组中位×五intent完整周期全部PASS。
- 余量参考：`-1.0°`保step17.9最多lift12.1，但0.8.10没有开放到-1.0°。

诊断文件：`.work/trot_pitch_tradeoff_probe_20261011.md`。

## 回归覆盖

- 先加入缺字段/版本/行为测试，旧代码以0.8.10版本断言失败，随后实现。
- offset0与默认路径兼容；负offset改变Trot前后腿BodyPose/关节输出；Stand/Walk/四个Leg完整帧hash不变；组合有效Pitch超±3°拒绝。
- RobotApp覆盖范围/NaN、零PWM、失败事务回滚、运动中拒绝、preview/start/direction、Stop/Off保留、reboot0、Profile save/load隔离、snapshot offset/effective/revision。
- Node/Web覆盖字段、中文语义、dirty/start lock、读回、revision、迟到ACK、范围和零动作mock。

## 0.8.9保存

- 归档：`.work/saved-0.8.9-before-trot-pitch-20261011-000719.zip`
- 归档SHA-256：`50E3E3C6D7886BE93B47B114820D3AF87B24AA27E7354BB9EDA4B581BD27C624`
- 原0.8.9 `firmware.bin` SHA-256：`B9AFBD333D0D68D9AE34B6D8BD39B23BF978F51972DE2155A30B6DFD424E774B`

## 最终验证与产物

- `test/host/run_all_tests.ps1`：exit 0；严格C++、Node与Web全套通过。
- `build-platformio`：首次未带PlatformIO penv的detect如实失败；将已安装的`C:\Users\0\.platformio\penv\Scripts`加入本次进程PATH后，detect确认PlatformIO 6.2.0，list-envs确认唯一环境`esp32-c3-devkitc-02`，指定环境build成功。未clean、未upload、未列设备。
- RAM：41,556/327,680 bytes（12.7%）。
- Flash：991,990/1,310,720 bytes（75.7%）。
- `mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.bin`：1,029,072 bytes。
- SHA-256：`6E03D4EA4EA1269DD3D71D332C536FFB5678A3693D981123C481375B85814F11`。

## 实机注意

旧motion NVS若保存13.1mm lift会覆盖新内置12.8；此时`-0.5°`应被预检拒绝。先选择Trot预设并应用12.8，再应用Pitch补偿。软件PASS不证明舵机跟随、地面接触、供电或动态抬头必然改善；首次仍须悬空、限流并准备物理断电。
