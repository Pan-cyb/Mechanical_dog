# 0.7.0 V10机身姿态、静态自稳与CG Walk交接

日期：2026-10-01。前置备份：`.work/pre-v10-0.7.0-20261001-114035.zip`。原V7.3只读。**未连接硬件、未开串口、未烧录、未发送真实PWM、未写板上NVS。**

## 实现

- `Ctrl/V10BodyPose`定义body +X前/+Y左/+Z上、Pitch正抬头、Roll正左高，使用原项目名义142×92.8mm髋轴间距（本机未测），输出到现有8DOF前后/上下投影，最终IK X仍只反号一次。
- 手动Pitch/Roll±3°、body_x±8mm与CG开关/余量为RAM参数；草稿与板端读回分离。`motion_save`仍只保存原六项，不改变`applepygait/motion`；`applepycal/c{channel}`完全不变。
- `Ctrl/V10StaticBalance`为真实dt有界PD，不生搬原每调用累计增益。Pitch安装符号按用户观察固定`body=-sensor`；Roll符号0/+1/−1，默认0门控。参考抓取20个稳定样本，限制2°/s和0.6°窗口span；每轴±1.5°、4°/s，最终八路逐帧仍受60°/s。
- 自稳只在稳定Stand周期更新。capture/on请求零PWM且必须当前Stand帧匹配已应用参数；等待数秒后on会以当前样本重置导数时基但保留参考。Stop/BalanceOff取消pending/transition并冻结最后完整帧；Walk/Trot从当前补偿帧smoothstep接续并暂停闭环；全关或IMU/reference/dt/tilt/几何/PWM错误清控制器和参考并按既有路径全关。
- CG Walk默认关闭。阶段：0–10%四足移前腿目标；10–25%腿1；25–27.5%全支撑；27.5–42.5%腿2；42.5–52.5%四足转后腿目标；52.5–67.5%腿3；67.5–70%全支撑；70–85%腿4；85–100%四足回程。前腿body_x=`stand_forward−margin`，后腿=`stand_forward+step+margin`，回程回手动body_x。开启时旧stride bias不参与。
- CG预检用frame实际x/z投影、声明的body旋转与髋位重建支撑足世界XY，以名义机身体中心为COM检查三足支撑边距；这不是实际质心测量。默认6/2mm、margin2.5mm采用3.2s；3.0s峰值60.92°/s被拒，未放宽限制。Roll2°的已知负边距组合以`support_margin`拒绝。
- 网页新增机身/自稳/CG卡片、双草稿revision与Roll符号dirty保护、中文预检摘要、名义尺寸/CG命令范围/自稳状态。mock支持基本流程且醒目标明不是真板。

## 原版证据

详见[原版参数证据](../ORIGINAL_V73_PARAMETERS.md)。原`main.py`5ms Timer；`speed=.05`；Trot理论约0.1s；Walk无等待约1.25s且有`X_S`到位等待；`l=142,b=92.8,w=108,in_y=17`。原静态自稳按调用累计；原Walk前/后腿用−30/+40全局X目标。0.7.0没有迁移移动中IMU前后CG反馈，只实现静态双轴反馈与计划式全局CG。

## 红—绿与验证

- 新纯body/balance测试先因缺头文件/实现编译exit1；实现后严格编译运行exit0。
- 主代理最终执行`powershell -NoProfile -ExecutionPolicy Bypass -File test/host/run_all_tests.ps1`：严格重新编译并运行host/gait/motion/body_balance/IK/shank与Node UI，exit0。
- 独立Astra两轮核心/UI安全复审最终PASS；曾发现的非零Roll+CG支撑边距P1已用实际模型投影修复并锁回归。
- 本地mock仅监听127.0.0.1，页面醒目标明“不是真板”；浏览器覆盖两组草稿、Roll符号保持、预检、Stand、20样本抓取、自稳pending/配置禁用、BalanceOff仍有力提示。浏览器还发现mock在转Walk后错误保留balance状态；已用generation取消延迟回调并在motion/Stop/Off/BalanceOff清pending，随后`node --check`与UI smoke exit0。mock不会调用串口、PWM或NVS。
- 主代理在generation修复后完成最终浏览器模拟复核：选择“CG Walk 3.2s”后分别应用步态与机身/CG两组草稿，友好预检显示“通过”，CG机身位移命令范围为−2.5..8.5mm；随后Stand→20样本capture→Pitch-only静态自稳运行（Roll默认0）→Walk，静态自稳状态正确变为`paused_for_motion_transition`→全关。所有已取消的arm/capture/on延迟回调均未迟到复活。截图：[v10_ui_0.7.0_balance_preview.jpg](../../results/v10_ui_0.7.0_balance_preview.jpg)（本地模拟，非实物）。
- `build-platformio`技能：Core6.2.0 detect exit0；唯一环境`esp32-c3-devkitc-02` list exit0；普通build exit0，未clean、未upload。
- `mechanical_dog/.pio/build/esp32-c3-devkitc-02/firmware.bin`：966,944 bytes；主代理最终实际复核SHA-256为`856C06DEC465900F7E583DAEB81D183BD9716413A7688D0B338EBD7A58DF6780`。二进制只读核对含`0.7.0`、`balance_capture`、`support_margin`、`CG Walk 3.2s`、`Roll未确认`。

## 未验证/风险

- 没有实物轴距、COM、Roll安装符号、负载、地面接触、供电、PWM波形或闭环增益验证。Roll默认关闭是有意门控。
- 8DOF丢弃侧向分量；名义支撑三角形不是完整3D接触模型。移动中IMU补偿未实现，不能称运动自稳。
- PCA50Hz/12bit量化、顺序8次I2C写和无硬件OE急停边界保持不变。
