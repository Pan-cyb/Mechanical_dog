# 0.8.8 默认机身前移8mm交接

日期：2026-10-10。范围：只调整内置控制Profile默认前后位置及配套网页初值、测试、版本和当前文档；未改项目结构、运动六参数、足端轨迹、IK、小腿映射、范围或NVS schema/version。

## 行为与算法

- 无有效Profile NVS时，`V10ControlProfile.body.body_x_mm`默认由0变为`+8.0mm`；通用`V10BodyPoseCommand{}`仍默认为0。
- Pitch/Roll为0时，`V10BodyPose::apply()`严格输出`forward=nominal_forward-body_x`、`down=nominal_down`。因此+8让四腿足端命令在机身坐标统一后移8mm；若接触足在世界坐标不动，等效于机身向+X/机头前移8mm。
- 该命令改变机身质量相对支撑足的前后杠杆，不是单独降低前脚。固件没有压力、接触、关节或实际COM反馈，无法证明真实重心偏后；+8来自用户本机观察后的选择。
- 原项目有`in_y=17`和`X_S/X_goal/tran_mov`全局移身，Walk还使用`gait_need`阶段目标。两者概念相关，但当前+8不是原17的等值移植，也没有启用原动态4/4/2前馈。

## NVS与网页

- 合法旧Profile记录中的body_x=0或其他值仍优先；缺失、损坏、NaN或IO读取失败才回退+8。保存与回读路径不变。
- 网页“前后 X mm”的静态HTML初值改为8，首次状态轮询仍以板端实际RAM值覆盖，所以旧NVS存在时看到0是预期行为。
- 保存+8：关闭输出 → 前后X填8 → 应用姿态 → 高级设置与遥测 → 保存Profile。六参数“保存已应用值到NVS”不保存body_x。
- CG Walk、Trot方向前馈、静态平衡和移动反馈使能仍默认关闭；启动仍不ARM、不运动、不写PWM。

## Probe与回归

在0.8.7产品代码未改时先增加新期望，显式+8探针全部通过，随后内置Profile=8断言按预期失败，证明red有效。探针覆盖两组中位：

```text
{90,100,90,90,90,97,90,102}
{95,100,85,80,100,80,85,100}
```

每组均覆盖forward/reverse/left/right/stepping五种intent，Stand、Trot `16/13.1/0.5/down114`与Walk `6/7.1/0.5/down108`完整周期预检全部通过。纯姿态测试另确认四腿`nominal-8`且down不变。

审核阶段又用真实0.8.8代码对相同十种组合做二分连续扫描并检查0.1mm档：

- Trot各组合边界约13.4477～13.4979mm，保守共同边界13.4477mm；13.2与13.4全部通过，13.5全部由`shank_map`拒绝。
- Walk十种组合边界约7.49786mm；7.2与7.4全部通过，7.5全部由`shank_map`拒绝。

因此13.17996/7.17996明确只属于0.8.7、`body_x=0`的历史扫描。0.8.8 +8改变了径向距离；13.1/7.1应称为沿用且继续通过的保守预设，不是+8下绝对最大。本轮用户只授权body_x默认变化，生产参数没有顺带提高。

最终严格回归：

```powershell
cd G:\PROJECT\Mechanical_dog\mechanical_dog
.\test\host\run_all_tests.ps1
```

结果exit 0。覆盖Profile missing/invalid/IO回退+8、合法旧body_x=0及其他值覆盖、保存/readback、网页bX初值和现有应用/保存Profile路径。

## 构建与产物

按`build-platformio`技能脚本执行detect、list-envs和指定`esp32-c3-devkitc-02`构建，最终均成功；未clean、未upload。首轮构建发现Arduino旧C++标准不接受带默认成员初始化结构的三值聚合初始化，随后改为`V10ControlProfile`默认构造函数只赋body_x=8，主机与嵌入式构建均通过。

- RAM：41,524 / 327,680 bytes（12.7%）
- Flash：983,384 / 1,310,720 bytes（75.0%）
- BIN：`mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.bin`
- 大小：1,020,368 bytes
- 时间：2026-10-10 22:14:18
- SHA-256：`A5384B3F57500A4E3A9102F91DC90F7B3D50F1CD5493B32BB3E592D8CEBCFAFA`

修改前备份：`.work/pre-forward-cg-default-0.8.8-20261010-221136.zip`。未烧录、未接板、未访问串口、未发送真实PWM、未写板上NVS。

## 剩余风险

+8位于当前允许范围端点，虽软件模型在本轮组合全部通过，仍不代表真实足底载荷、机械余量或落地稳定性合格。已有Profile NVS会遮蔽新默认；实机验证前应先核对网页版本0.8.8、Profile状态和读回X值，并保持悬空/限流/可物理断电条件。
