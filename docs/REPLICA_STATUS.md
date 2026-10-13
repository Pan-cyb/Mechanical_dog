# Applepy_v1 当前状态 · Structure V10

更新：2026-10-13。工作区为`0.8.14`实时body_x滑块版。`pose_live_set`从三字段扩为height/Pitch/Roll/body_x四字段原子链；body_x范围−8..+8mm、step0.1、默认+8不变。默认Trot/Walk/trim/Trot Pitch、NVS schema、方向click/lease均不变。未上传、未连接板卡/串口、未发送真实PWM、未修改板上NVS。修改前备份：`.work/pre-live-body-x-slider-0.8.14-20261013-133721.zip`。

## 传统控制里程碑（2026-10-13）

用户已通过实机体验认可当前传统控制部分，0.8.14作为本阶段保存/发布候选。已实现并集成：基础站立、Walk/Trot、点击锁定前后左右、明确原地踏步、实时高度/Pitch/Roll/body_x、八路舵机中位与小腿映射标定、整周期/Stand预检、motion/Profile/中位独立保存，以及IMU静态姿态补偿、移动Pitch→body_x补偿和可选CG Walk。严格host/Web全套测试exit 0，最终构建证据见“验证”。

里程碑结论限定为“传统开环控制体验达到当前用户目标”，不是实体闭环完全验证。当前没有足压、触地或关节位置反馈，足端和舵机角遥测仍是命令值。

已知实机限制：原地踏步仍会轻微后退，用户判断与前向重心不足有关。当前内置`body_x=+8mm`且允许范围为`-8..+8mm`；在现有开环8DOF、无足端/关节/接触反馈的证据边界下，本里程碑暂不继续修改。此记录不表示问题不可能修复；下一阶段可结合机械配重、足端接触/关节反馈或新的运动学包络重新评估。

## 已实现

- 网页新增`bXRange`并与`bX`双向同步；四个实时量在同一150ms合并/单并发/latest-wins链，moving CG、自稳、校准、非稳定mode、预检和故障门控完全一致。body_x不再置body dirty；旧`bApply`移除，CG Walk仍用`bCgApply`。
- CLI/HTTP/Command统一四字段，旧三字段Invalid。RobotApp候选同时修改stand down和Profile的Pitch/Roll/body_x，完整Stand预检后一次提交；±8通过、±8.1和不可达组合原子拒绝且revision不变。OFF/Stand/Hold语义及Profile持久化链保持。

- `pose_live_set`对height/Pitch/Roll/body_x四元组做完整Stand预检后一次提交；非法tuple不改变motion/profile或revision。OFF/idle零PWM且不ARM；Stand/Hold等待新IMU后复用1.5s transition，并支持从当前完整过渡帧重定向。
- Web实时链150ms debounce、单请求在途、latest-value-wins、成功读回、失败回退；实时四滑块不再进入motion/body dirty锁。其它步态和CG Walk入口保留，motion/Profile保存归属不变。
- 方向改为click锁定：前后左右/原地踏步复用原intent，活动按钮`aria-pressed`；切换执行旧`drive_end`读回后再begin新epoch。Stop/Off优先；隐藏/renew网络失败停止本地续租并靠500ms固件lease冻结。
- 姿态probe：down100..115、两组中位、0.1°，跨包络单轴Pitch约−7.0/+5.1、Roll至少±10；硬范围统一为Pitch−7..+5、Roll±10。默认down114单轴Pitch±3.2/Roll±5.7，同幅组合±2.0，故每个tuple仍预检。

- 手机网页新增`mHeightRange` 100..115mm、`bPitchRange/bRollRange` -3..+3°，均0.1步进；与原number双向同步，预设/轮询/reset/load同步，dirty revision、启动锁、disabled和迟到ACK保护保持。拖动零POST/零动作；高度仍走`motion_set/motion_save`，Pitch/Roll仍走`body_set/profile_save`。
- 最新只读probe固定lift12.8/body_x8/trim.25并覆盖两组中位×五intent×Pitch 0/-0.5°：down114共同step上限18.5（18.6 `hip_window`），仅前进两Pitch 19.8，仅前进/Pitch0 24.1；down115共同21.0。故默认17.9不是最新共同极限，down114也不是网页范围内的最大步长最优down；本轮不改参数。

- 新增RAM-only `trot_pitch_set <-0.5..0.0>`与手机控件。仅Trot在BodyPose前叠加，负值压低前端；Stand/Walk/Leg逐帧不变。设置按当前参数/中位/Profile/straight trim默认前进Trot完整预检，失败不修改，Stop/Off保留，重启/Profile/motion NVS均不保存。
- 诊断显示原17.9/13.1在`-0.5°`下即使step0也因摆动峰值`shank_map`失败；Trot lift降到12.8后两组中位×五intent全过，12.9下一档在phase .250腿1失败。`-1.0°`余量参考需17.9/12.1，但本版范围不开放到-1.0°。

- 0.8.9修改前probe覆盖两组中位×五intent：Trot 17.9全部通过，18.0在倒退/左右转因`hip_window`拒绝；Walk 25.3全部通过，25.4在非踏步intent因`hip_window`拒绝。踏步前后幅值为0；30.1由参数硬上限拒绝。
- 内置/Trot步长改17.9，普通Walk网页预设改25.3；慢速与CG Walk专用预设、motion NVS schema/version不变，合法旧NVS仍优先。
- RAM-only `straight_trim_set`与手机控件范围仍为±0.30；0.8.11以单一常量固定启动/重启默认0.25。普通前进/倒退有效turn为-0.25/+0.25，对应左右scale 1/0.75与-1/-0.75；显式转向和踏步不变。设置零PWM，运动中拒绝，Stop/Off保留手动值，Profile/motion NVS保存加载均不影响它。
- 状态发布`straight_trim`与`effective_turn`；网页保护未应用草稿并锁定启动。无足压/足端/关节反馈，不能自动确认左偏来自哪项硬件差异或自动求补偿。
- 0.8.8仅把内置`V10ControlProfile.body.body_x_mm`设为+8并同步网页静态初值；通用`V10BodyPoseCommand`默认仍为0，六运动参数、轨迹、映射、范围和Profile schema/version均未改。
- 零Pitch/Roll时四腿严格满足`forward_out=nominal-8`且`down`不变；两组中位×前/后/左/右/踏步下，Stand及现有13.1/7.1预设Walk/Trot完整预检全部通过。
- 0.8.8按相同十种组合重扫+8边界：Trot共同连续边界约13.4477mm（13.4通过、13.5 `shank_map`）；Walk约7.49786mm（7.4通过、7.5 `shank_map`）。13.1/7.1是0.8.7在body_x=0选定、+8下继续通过的保守预设，不是+8下绝对最大。
- Profile NVS缺失/损坏/IO失败回退+8；合法旧记录body_x=0或其他值仍覆盖内置默认。CG Walk、Trot前馈、静态/移动反馈使能继续默认关闭。
- 0.8.6删除速度配置、速度类预检错误、峰值速度报告/API/UI，以及整周期、首段过渡、运行时、模式切换、静态平衡和移动CG的速度拒绝；没有用大常量替代。
- 原先只因速度门限失败、但几何/映射/0..180°合法的Trot 20/6/0.7/down115、默认高动态Walk及current→target切换现在预检通过；IK、髋窗口、小腿表、支撑边距和故障测试仍保留。
- 0.8.7用真实V10Gait在`body_x=0`、两组中位×五种intent扫描：Trot连续边界13.17996mm，13.1通过而13.2在phase .25触发`shank_map`；Walk连续边界7.17996mm，7.1通过而7.2在phase .10触发`shank_map`。这是保留的0.8.7历史事实。
- 0.8.5 Trot足端kernel继续使用原项目摆动三次曲线、支撑匀速、50%对角错相，x坐标仅减S/2适配现接口。
- 0.8.4把重型预检改为共享`V10PreflightCursor`：同步纯算法wrapper与异步RobotApp job驱动同一逐帧实现；普通周期仍5ms密采样/周期闭合，CG仍三场景、planner阶段8/9、等待超时、2000步上限与动态包络/slew。
- checking独立于`motion_pending`。每个主循环tick最多一帧，旧运动正常调度；PASS提交时重验IMU/ARM/参考/参数revision/中位，之后仍等新IMU才启动。
- provisional Drive lease、Stop/Off/BalanceOff/MovingCgOff、故障/参考重置取消、终态错误与短UI提示均已接入；检查期间所有参数/NVS冲突写拒绝。
- 0.8.3将L3H映射从Servo9/CH8迁到Servo13/CH12，协调顺序为`0,1,2,3,12,9,10,11`，mask为`0x1E0F`。c12存在时优先；仅c12缺失时只读回退c8；显式保存只写c12，启动不迁移写入。
- 0.8.2网页精简为控制/标定双视图：默认一张运动卡、唯一Walk/Trot选择、紧凑方向pad与四常用参数；姿态、CG、Profile、预检、单腿和原始遥测默认收起。标定主流程只保留当前中位、接口、±1°、保存/退出，高级单路功能未删除。
- 新UI仍保留异步迟到ACK隔离、Stop/Off抢占、草稿revision/readback、epoch/seq/lease与明确ARM规则；切视图/展开/选通道/编辑/轮询零动作，隐藏高级草稿同样锁启动。
- 0.8.1仅修首页传输：75,507字节UTF-8模板使用精确`Content-Length`、不超过1,024字节的Flash分块和单独16字节CSRF token发送，不再构造超过64 KB的整页`String`；HTML、响应安全头、POST token校验、控制逻辑与NVS格式不变。
- PCA/HXY、SoftAP/串口、单路台架、相对中位、四小腿 q、八路中位/NVS 保持原格式与安全门控。
- V10 二连杆 IK、共享小腿线性表、各腿方向/标定中位接入八路协调帧；0.8.3通道顺序 L1H/L1S/L2H/L2S/L3H/L3S/L4H/L4S → CH0/1/2/3/12/9/10/11。
- Stand、Walk、Trot、腿1..4单腿测试。Walk 最多一腿摆动，顺序1→2→3→4，并保留短全支撑间隔；Trot 对角交替；周期首尾连续。
- 默认 `walk_body_shift_mm=2` 保留兼容，但实际语义是每腿局部相位的前后轨迹偏置/stride bias。它使摆动端点和支撑后移幅度增加，不是全局机身/CG平移。全周期数值导数回归验证每个连续支撑片段的 `foot_forward` 不增加。
- 启动请求整周期预检且零 PWM；下一新 IMU 样本先写八路中位，再 1.5 s smoothstep 过渡。过渡曲线仍保留，但不再按命令角速度拒绝。
- Stop 保持最后完整命令帧；Off 取消 pending/transition/run 并尝试全关。IMU失败、>200 ms调度空窗、运行时几何失败和中途总线失败均有回归。
- 网页顶部集中放置参数、预设、“使能并站立”、Stand/Walk/Trot/腿1..4、模式可选只读预检、Stop/Off。按钮明确为“应用参数（RAM）”“保存已应用值到NVS”，并说明全关→应用→保存；草稿、板上RAM、NVS状态分开。
- 参数应用/预检允许 idle/OFF、稳定 Stand、Stop 后 Hold，调用自身零 PWM且保持帧不变；运动/过渡/pending仍拒绝。应用只做有限数/范围/Stand几何检查，开始与显式预检才检查目标模式完整周期。
- `arm_stand`先验证再ARM+排队Stand，失败不残留armed；旧`arm`仍是单路调试解锁。上电加载步态NVS也永不自动运动。
- 步态参数独立保存为`applepygait/motion`完整版本记录，保存后回读；原`applepycal/c{channel}`完全保留。无效/缺失/加载失败使用内置默认并显式报告。
- 默认Trot为17.9/12.8mm、0.5s、down114；网页普通Walk为25.3/7.1mm、0.5s、down108；轨迹函数和30mm输入硬上限均未改。预检5ms密采样、运行20ms不追帧，并发布frame/gap/missed统计。
- `Ctrl/V10BodyPose`采用+X前/+Y左/+Z上、Pitch正抬头/Roll正左高和名义142×92.8mm髋轴间距，把姿态/全局body_x投影到现有8DOF前后/上下目标；本机轴距未量，侧向分量丢弃边界明确。
- 静态Stand双轴有界PD已接真实调度：Pitch固定`body=-sensor`；Roll按用户观察默认−1，即`body=-sensor`。0仍主动关闭Roll，+1仍可手动覆盖；改变符号会清参考。重启只恢复方向约定−1，不恢复reference/enable、不ARM、不输出。
- CG Walk默认关闭，开启后旧stride bias不参与；前腿目标`forward−margin`、后腿`forward+step+margin`，穿插全支撑移身/回程。实际投影足端世界XY+名义体中心仍做全周期支撑边距、IK和映射预检；6/2mm推荐3.2s。
- 移动中Pitch反馈已适配为受限全局body_x修正，但仍只有命令模型，没有实测COM、接触或关节反馈，不能称实物运动闭环验收。
- 0.8.0新增RAM signed drive intent：普通Walk/Trot及CG Walk支持前后、左右差速/原地转向和踏步；旧非负step NVS只作幅值。网页方向会话用epoch/seq/500ms lease，renew只续租，超时/松手冻结最后完整帧并保持力矩。
- 新`V10MovingCg`用body Pitch误差/导数、真实dt、滤波和±0.5mm/2mm/s修正；新`V10CgPlanner`以独立20mm/s命令slew和容差/超时在全支撑阶段暂停相位。预检仿真真实planner、等待、阶段8/9、±反馈包络及20ms帧；页面发布预检耗时与名义/实际周期。
- 新`applepyprof/profile`完整记录保存body/CG、planner、移动CG和独立Pitch/Roll Kp/Kd；旧中位与六步态记录不变。Profile不存ARM/enable/reference/drive/session，启动永不自动运动。
- 可选Trot intent前馈默认关闭，启用后按signed stride自动区分前进/后退/原地转向，独立4/4/2默认增益与max写入Profile；踏步为0，并与moving offset共用完整安全检查。
- 当前硬件缺口与安全替代逐项见[MIGRATION_MATRIX](MIGRATION_MATRIX.md)。软件适配完成不能表述为整机实物完成。

## 设计边界

0.8.11只改变直行trim的启动/重启默认值和网页说明；0.8.10的Trot专用RAM角度前馈、0.8.9步长/差动算法、0.8.8静态body_x默认、0.8.6无软件命令角速度拒绝行为保持。姿态、IK、小腿改装、Walk/CG、控制器自身slew与状态机仍是C++适配，不宣称整套逐行等价。

第一次协调帧是八路保存中位，没有实际关节位置反馈，因此首次必须悬空、限流并可立即切断 V+。PCA OE# 固定下拉，软件 Off 不是硬急停。当前构建/假设备测试不证明真实接线、机械限位、供电能力、I²C 部分帧后果、落地步态或稳定性。

## 验证

集中`test/host/run_all_tests.ps1`以`-Wall -Wextra -Werror`最终全套exit0，新增覆盖四字段CLI/HTTP、三字段Invalid、body_x±8/±8.1原子性、OFF/Stand/Hold、Profile持久化与Web八控件合并/latest-wins/失败回退；旧方向click/lease矩阵不退化。0.8.14最终BIN 1,040,512 bytes，SHA-256 `8473C502941C254F6FFEC5B9A9D041C9C562AA6FBB84E81357E68C5766C827B7`；RAM 41,556/327,680（12.7%）、Flash 1,003,376/1,310,720（76.6%），未clean/upload。

入口：[0.8.14使用指南](WEB_UI_USER_GUIDE.md) · [迁移矩阵](MIGRATION_MATRIX.md) · [原版参数证据](ORIGINAL_V73_PARAMETERS.md) · [进度](REPLICA_PROGRESS.md) · [V10运动学](V10_KINEMATICS.md) · [接线](JOINT_WIRING_PLAN.md)。
