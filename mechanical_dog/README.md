# Applepy_v1 · V10适配版V7.3主控固件0.8.14

当前工作区是`0.8.14`实时前后重心滑块版。`body_x`加入高度/Pitch/Roll的同一`pose_live_set`四元组：约150ms合并后完整Stand预检并原子提交，OFF只更新RAM且不ARM，Stand/Hold平滑重定位，失败四字段全部保持。范围仍为−8..+8mm、0.1mm步进、默认+8；方向点击/lease和0.8.13其它行为不变。未上传/烧录/连接硬件。

## 0.8.14 实时body_x

- 手机网页新增“前后重心 / body_x mm（+向前）”range+number，范围−8..+8、步进0.1、默认8。body_x与height/Pitch/Roll共同参加latest-value-wins、单请求在途、读回与失败回退。
- CLI/HTTP统一为`pose_live_set <height> <pitch> <roll> <body_x>`；未发布的旧三字段形式明确Invalid。RobotApp一次构造motion/Profile候选并完整Stand预检，任何字段越界或组合不可达均不提交任何字段/revision。
- body_x不再制造body dirty，也不需要应用按钮；旧“应用X/CG设置”移除。CG Walk enable/shift仍由高级区“应用重心设置”明确提交，并携带当前已实时应用的body_x。
- 不自动保存：height仍进motion六参数NVS；Pitch/Roll/body_x仍随Profile保存。NVS schema、默认Trot/Walk/trim/Trot Pitch和项目结构未改。

## 0.8.13 实时姿态与点按锁定方向

- 新命令`pose_live_set <height> <pitch> <roll>`先对完整候选tuple做Stand的BodyPose→IK→髋窗口→shank map→舵机范围预检，失败不提交任何字段。高度仍归motion NVS，Pitch/Roll仍归Profile，不自动保存。
- Probe按down100..115、0.1°、两组中位扫描：跨包络单轴最大Pitch约−7.0/+5.1°、Roll至少±10°；统一硬范围取Pitch −7..+5°、Roll −10..+10°。硬范围不是组合可达保证：默认down114单轴仅Pitch±3.2°、Roll±5.7°，四象限Pitch/Roll同幅仅±2.0°，所以每次tuple仍必须预检。
- 网页拖动三滑块会在150ms后发送最新完整tuple；同一时间至多一个请求。请求期间继续拖动只保留最新值，旧ACK不覆盖；拒绝时恢复板端有效值并显示错误。其它步态、X/CG和Profile参数保留原明确应用入口。
- 前/后/左/右/原地踏步改为点击一次持续运动，活动按钮用`aria-pressed`显示。点击当前按钮或“停止”结束并保持；切换按钮按`drive_end→状态读回→drive_begin`。页面隐藏/续租网络失败停止本地续租，固件500ms lease仍是兜底。

## 0.8.12 手机姿态滑块与步长复核

- 高度滑块范围100..115mm、Pitch/Roll滑块范围均为-3..+3°，步进0.1；number键入、预设、状态轮询、放弃修改和加载回读都会同步对应滑块，disabled状态一致。
- “高度”字段是髋轴到足端的向下目标`down`，不是外壳实测离地高度；Pitch正值抬头，Roll正值左侧高。没有增加后端命令、NVS字段或运动算法。
- 用当前0.8.11真实模型复核：Trot `lift12.8/body_x8/trim.25`、两组中位×五intent×Pitch offset `0/-0.5°`时，down114共同0.1mm步长上限为18.5，18.6倒退触发`hip_window`；仅前进且覆盖两种Pitch为19.8，仅前进/Pitch0为24.1。因此默认17.9继续通过，但不是最新共同极限，30只是输入硬上限。
- down100.0..113.9在相同12.8mm抬脚包络下即使step0也因`shank_map`不可行；114.0..115.0的共同上限由18.5逐步升至21.0。网页范围内down115的步长余量最大，故down114不是“最长步长对应的最优高度”；本轮只记录，不自动修改参数。

## 0.8.11 固定直行修正默认值

- 唯一默认常量`kDefaultStraightTrim=0.25f`由RobotApp成员初值和`begin()`重置共同使用；`V10DriveIntent.turn`仍默认0，只有普通直行在`applyStraightTrim()`中转换为有效turn。
- 前进比例为左/右`1.00/0.75`，倒退为`-1.00/-0.75`；显式左转、右转和踏步不叠加trim。Stop/Off保留当前手动值，重启恢复0.25。
- 网页初值、板端读回和提示语同步为0.25。手动设置范围仍为`-0.30..+0.30`，不写Profile或motion NVS；保存/加载这两种记录都不会改变当前trim。
- 修改前真实0.8.10模型已覆盖两组中位×五intent×Pitch 0/-0.5°共20个Trot组合，全部完整预检通过；未放宽IK、髋窗口、小腿映射、舵机范围或支撑边距。

## 0.8.10 Trot专用Pitch补偿

- `trot_pitch_set <-0.5..0.0>`只在Trot中令`effective_pitch=Profile pitch+offset`；负值压低前端，Stand、Walk、单腿、CG Walk完全忽略该角度。它是开环角度前馈，不是IMU移动闭环，也不改变moving CG默认±0.5mm。
- 真实0.8.9模型探测显示原`17.9/13.1`在`-0.5°`下两组中位×五种intent全部因摆动峰值`shank_map`失败；减step无效。将Trot lift最小下调0.3mm至12.8后，`17.9/-0.5°`十种组合完整周期通过，12.9下一档在phase 0.250腿1触发`shank_map`。
- 设置只允许OFF/idle或稳定Stand/Hold且静态/移动反馈不忙；对当前参数、中位、Profile、straight trim所得默认前进Trot做完整预检，失败不修改。命令本身零PWM；Stop/Off保留，重启归零，Profile与motion NVS都不保存该值。
- 手机网页提供0.1°步进输入、草稿锁定、状态读回、revision和迟到ACK隔离。建议先试`-0.5°`；若旧motion NVS仍加载13.1mm，须先选择新的Trot预设并应用，否则补偿会被正确拒绝。

## 0.8.9 最大步长与直行修正

- 修改前0.8.8真实预检在`body_x=+8`、两组中位×前/后/左/右/踏步扫描：Trot共同0.1mm步长上限17.9mm，18.0在倒退/转向的phase 0.340触发`hip_window`；普通Walk共同上限25.3mm，25.4在非踏步intent的phase 0.040或0.160触发`hip_window`。踏步本身前后幅值为0，因此输入step不改变轨迹。30.1仍由参数硬上限拒绝。
- 内置/Trot预设改为17.9，普通Walk网页预设改为25.3；慢速与CG Walk `6/2/3.2/108/0/2`不改。已有合法motion NVS仍覆盖内置Trot默认，记录schema/version不变。
- 直行修正只在`turn≈0`、非踏步且`forward!=0`时生效：`trim>0`在前进生成`effective_turn=-trim`、倒退生成`+trim`，所以两种方向都减弱物理右侧；用于纠正实物左偏。负值镜像减弱左侧。显式左/右转与踏步完全不变。
- 修正不写Profile或motion NVS；0.8.11启动/重启默认值为0.25。设置只允许OFF/idle或稳定Stand/Hold，命令本身零PWM。网页未应用草稿锁定启动，并回读当前trim与effective turn。
- 持续左偏只能说明左右真实推进、摩擦、装配或标定存在不对称；当前没有足压、足端或关节反馈，固件不能自动测出正确补偿值。实机从`+0.05`开始，逐次小幅调整。

## 0.8.8 默认机身前移8mm

- `V10BodyPose`在Pitch/Roll为0时得到`forward_out=nominal_forward-body_x`、`down_out=nominal_down`。因此`body_x=+8`表示在固定足端世界位置的解释下，把机身整体向+X/机头方向前移8mm；四腿随后仍走相同IK和舵机映射。它改变机身质量相对支撑足的前后杠杆，不是单独把前脚压低。
- 用户观察到初始姿态偏后、后腿负载偏大、前腿偏轻；但固件没有足端压力、接触、关节位置或实际COM反馈，代码本身不能证明真实重心确实偏后。+8mm是本机实物观察后的默认命令，不是测得的COM标定。
- 原项目有`in_y=17`基准及`X_S/X_goal/tran_mov`全局移身，Walk还会按`gait_need`等待移身到位；当前+8与它们方向/目的相关，但数值、坐标适配和控制流程不同，不宣称等价于原17mm。
- Profile记录格式/版本不变。缺失、损坏或读取失败时使用+8；已有合法旧记录中的0或其他值继续覆盖。网页静态初值为8，首次状态轮询后会显示板端实际已应用值。
- Profile的CG Walk、Trot前馈、静态/移动反馈使能仍默认关闭；本轮没有把原项目动态移身或前馈默认打开。
- 用真实0.8.8代码在两组中位×五种intent重扫：+8下Trot十种组合的共同连续边界约13.4477mm，13.4通过、13.5触发`shank_map`；Walk共同边界约7.49786mm，7.4通过、7.5触发`shank_map`。13.1/7.1仍保持用户已采用的保守预设，本轮不顺带提高参数。

## 0.8.7 最大已标定抬脚预设与NVS提示

- 0.8.7在`body_x=0`时测得连续边界约Trot 13.17996mm、Walk 7.17996mm；网页以0.1mm为步进，故当时采用13.1mm和7.1mm。0.8.8的+8会改变径向距离，其新边界见上节。
- 普通Walk/Trot周期均为0.5s。慢速按钮和CG Walk专用`6/2/3.2/108/0/2`保持不变。
- 内置缺省采用新Trot值；已有有效`applepygait/motion`仍优先，记录格式/版本没有改变。
- 网页保存顺序写明为“关闭输出 → 应用其它步态参数（RAM） → 保存已应用值到NVS”。高度实时值同属这套六参数；Walk/Trot共用，不会同时保存两个按钮预设。

## 0.8.6 移除协调运动速度拒绝

- 删除速度配置字段、速度类预检错误、峰值速度报告/API/UI字段，不用无穷大或大常量伪装关闭。
- 整周期预检、首段1.5s过渡、运行时帧、模式切换、静态平衡、移动CG和方向会话不再按命令角速度拒绝动作；原先仅因该门限失败而几何/映射/电气合法的动作现可继续。
- 仍保留有限数/参数范围、IK、髋窗口、小腿已标定映射、舵机0..180°、支撑边距、ARM/OFF、IMU、调度/I²C故障等检查；没有放宽小腿映射另一侧。
- 不再计算或发布“峰值舵机速度”。舵机实际速度取决于硬件跟随能力；命令轨迹仍由周期、20ms更新和1.5s smoothstep过渡决定，不能据此声称实机能够无滞后跟随。

## 0.8.5 原项目Trot轨迹

- Trot使用原项目50%占空：腿1/3与腿2/4半周期错相；摆动采用原三次曲线，支撑x匀速后移。修补原Python在严格`u=.75`时未赋值的漏洞为连续极限x=1；原touchdown垂直速度非零，不再强制raised-cosine零速。
- 原函数输出x为`0..S`，当前适配只减`S/2`变为`-S/2..+S/2`，保持`step_length`总行程和已存`stand_forward/body_x`语义；forward到IK仍只取一次负号。Walk/CG Walk轨迹未改。
- 原软件还会默认叠加`PA_ATTITUDE`姿态/`in_y=17`与随`spd`变化的4/4/2重心前馈；0.8.8只根据本机观察把静态Profile `body_x`设为+8，没有原样恢复这些旧动态偏置，当前Trot前馈与IMU反馈仍默认关闭，Profile/NVS格式不变。
- 0.8.5当时的候选是`16/5/0.8/114/0/2`；0.8.7已由实测模型边界预设替代，并保留前后/左右转/踏步几何与映射预检。
- 步长/抬脚输入上限为30mm，但所有组合仍须通过IK/q/角度预检。按当前已标定模型，原`step30/lift30/height110`即使周期10s仍先触发hip窗口；step=0隔离水平后触发shank映射，原理论0.1s也低于现0.4s周期下限；这不是实物机构物理极限结论。
- 旧NVS六参数格式不变且已保存值优先。要用当前按钮值，需点对应预设再“应用其它步态参数（RAM）”；要跨重启保留还需全关后“保存已应用值到NVS”。

## 0.8.4 增量步态预检

- 用户0.8.3真实状态显示合法Walk预检结果`error=none`、峰值35.73°/s，但同步计算耗时476ms，随后100ms IMU freshness检查把它误拒绝；网页请求结束后采样恢复，因此IMU仍显示正常。
- 0.8.4将同一完整预检算法改为cursor：每次`tick()`最多计算一帧，两次之间继续IMU、HTTP、串口和旧运动调度。0.8.6保留该逐帧几何/映射/支撑检查，只移除命令角速度统计与拒绝。
- 请求ACK只表示“已排队检查”。完整PASS且健康/版本/中位/参考仍一致后，才设置真正`motion_pending`，并继续等待后续新IMU样本启动；检查期不发新目标PWM。
- DriveBegin立即建立500ms provisional lease，网页可按原150ms续租；释放、超时、Stop/Off、反馈关闭、故障或参考重置都会取消job，迟到结果不能复活。

## 0.8.3 Servo9→Servo13

- 协调顺序改为PCA `{0,1,2,3,12,9,10,11}`；索引4仍是L3H，网页按通道动态显示Servo13。
- 新通道中位优先读取`applepycal/c12`；只有c12键不存在时才只读回退旧`c8`。c12即使值非法也不会被c8覆盖，仍按既有规则使用90°默认。
- 启动和回退读取不写NVS；显式保存CH12只写c12，不删除或改写c8。其余七个中位键、`applepygait/motion`与`applepyprof/profile`不重释。
- 代码变化不会改变板上接线或固件。必须先断开舵机电源，把插头从Servo9拔到Servo13，再另行授权烧录0.8.3。

## 0.8.2 精简网页

- 控制页默认只保留状态、站立/停止/关闭输出、唯一Walk/Trot选择、开始行走、方向十字键、四项常用参数、三组预设和一行RPY。
- 姿态/水平校准、CG/planner/PD/移动补偿/前馈、手动预检和原始遥测集中到默认收起区域；隐藏草稿仍锁定启动并在主界面提示。
- 标定页按“安全确认→开始→选接口→中位±1°→保存并退出”组织；开始使用当前已存/内存中位，不重置固定90°。原始us、q和单路调试保留在高级折叠。
- 切换视图本身零动作；若点击锁定方向正在运行则明确结束本次会话。预设只改其它步态草稿，Walk/Trot预设同步唯一步态选择，应用/保存仍须明确点击。

## 0.8.1 首页热修复

- 75,507字节UTF-8页面不再复制到单个`String`；响应先设置精确`Content-Length`，Flash正文以不超过1,024字节的块发送，CSRF token单独写入RAM。
- HTML、CSP/防缓存等响应头、token唯一替换和POST校验均保持；GET首页不调用控制业务、状态推进或输出。
- 严格主机回归覆盖真实页面、128 KB模板、token跨分块边界、缺失/重复占位符拒绝、传输失败停止以及重复GET字节一致。

## 当前能力

- ESP32-C3-WROOM-02-H4；PCA9685 `0x40`/50 Hz 与 HXY IMU `0x19` 共用 GPIO4/5、100 kHz I²C；SoftAP 页面位于 `http://192.168.4.1/`。
- 保留单路原始 PWM、相对中位 `offset`、四小腿 `q` 换算、八路中位标定和 NVS 保存。默认电气输入为 500–2500 µs／名义 0–180°，不是机构安全行程。
- 新增 V10 `stand`、`walk`、`trot`、`leg_test 1..4`、`motion_stop`，以及网页对应按钮和只读全周期预检。0.8.3步态输出固定使用 L1H/L1S…L4H/L4S → PCA CH0/1/2/3/12/9/10/11。
- 足端经显式 `(80, 61.357)` mm 二连杆 IK，再经共享小腿节点 `(u,q)=(0,0),(30,9),(50,23)` 与每路独立中位/方向生成八路命令。任何几何、0–180°、髋窗口或小腿表检查失败都会在输出前拒绝。
- 新增纯`V10BodyPose`：机身+X前/+Y左/+Z上，Pitch正抬头、Roll正左高，使用原项目名义142×92.8mm髋轴间距（本机未测）并明确只投影到8DOF前后/上下平面。手动Pitch −7..+5°、Roll±10°、body_x±8mm均先做Stand检查。
- 静态Stand小幅有界PD：Pitch保持`body_pitch=-sensor_pitch`。用户确认左倾（左低右高）时sensor Roll为正，而body Roll正定义为左高，因此默认`body_roll=-sensor_roll`（符号−1）；仍可手动选0关闭Roll或+1覆盖。参考需20个稳定样本；每轴修正仍受±1.5°与控制器4°/s自身斜率约束，但八路输出不再另设协调角速度拒绝。
- 新增可回退CG Walk：显式全支撑移身→腿1/2→全支撑换向→腿3/4→全支撑回程。CG开启时旧stride bias不参与；支撑边距用实际模型投影后的足端世界XY和名义体中心检查，不代表实测COM。
- 有符号RAM drive intent支持前进、后退、左右差速/原地转向和踏步；旧`step_length_mm>=0` NVS仍是幅值，不重释。普通Walk/Trot均支持；CG Walk按左右实际步幅推导前/后支撑目标并完整预检。
- 移动CG参考原`height*tan(error*1.1)+rate*5`，改为已确认的body Pitch及其估计导数、真实dt、滤波、默认±0.5mm和2mm/s；与全支撑planner默认20mm/s明确分离。Stand只初始化，不自动移身；Walk/Trot才在完整帧中叠加。
- 新方向会话约束网页点击锁定方向pad：begin要求已有ARM，renew只续500ms租约且不重启相位；页面隐藏/网络断开停止续租并在租约到期后冻结最后完整帧。原手动ARM/Walk/Trot保持无软件超时规则。
- 独立Profile保存body/CG、planner、移动CG参数以及Pitch/Roll各自Kp/Kd；不保存ARM、enable、参考、drive intent或会话。旧`motion_save`仍只保存六项步态。
- 可选Trot方向前馈默认关闭；启用后按signed左右步幅自动区分前进/后退/原地转向，独立f/b/turn增益默认4/4/2并受最大body_x与IK/q限制，踏步为0。

默认运动参数为Trot起步：步长17.9mm、抬脚12.8mm、周期0.5s、`down=114mm`、`forward=0mm`、兼容bias 2mm（Trot不使用bias）；内置Profile另加全局`body_x=+8mm`。网页普通Walk为`25.3/7.1/0.5/108/0/2`，另保留慢速草稿与CG Walk 3.2s专用预设。Trot摆动/支撑来自原项目函数；Walk/单腿/CG轨迹没有改写，不能把此次参数调整表述为全部模式逐行复刻。

CG Walk实验仍为步长6mm、抬脚2mm、周期3.2s、额外名义余量2.5mm，轨迹本轮未改。前腿目标`stand_forward−margin`，后腿目标`stand_forward+step+margin`，实际命令范围包含手动body_x起点/回程端点。

Walk 每腿摆动占周期 20%，顺序腿1→2→3→4；相邻摆动间保留 5% 周期全支撑间隔，任意时刻最多一腿摆动。Trot 为腿1/3与腿2/4对角交替。所有模式在相位首尾保持目标与舵机命令连续。

## 启动、过渡与停止

1. 上电先尝试 PCA 全关；不会自动使能或运动。网页“使能并站立”使用 `arm_stand`：先完成 Stand 验证，再原子地 ARM+排队 Stand；失败不留下意外 ARM，命令处理本身输出 **0 路 PWM**。旧 `arm` 只保留给单路台架解锁。
2. 请求保存当前 IMU 样本 generation。后续新的约 10 ms IMU 采样到达后，才按固定通道顺序写八路当前标定中位，随后用 1.5 s smoothstep 过渡到目标首帧。该过渡时长和曲线保留，但不再按其命令角速度拒绝切换。
3. Walk/Trot/单腿测试每 20 ms 计算完整八路帧；整帧先校验再按八次 I²C 写入，**不是硬件原子提交**。Stand 到达后保持最后命令，不周期重写。
4. `motion_stop` 取消待启动/过渡/周期推进并保持最后一个完整命令帧；`off` 优先取消运动、尝试 PCA 全关并撤销 ARM。网页会在普通 POST/GET 阻塞时抢先、短时重试 Stop/Off，但软件全关不能替代舵机 V+ 物理断电。

首次运行必须悬空、限流并准备立即切断舵机电源。HXY可显式用于静态Stand反馈或移动中小幅body_x修正，但没有关节/足端/接触/实际COM反馈，也不是安全保护；步态轨迹和移身到位仍是命令侧模型。

网页足端 `forward/down` 与八路 `command_deg` 均为**命令遥测，不是反馈**。草稿、板上已应用 RAM 与 NVS 状态分开展示；编辑后轮询不覆盖，应用成功还需状态回读逐项一致。未应用草稿锁定运动启动。参数应用/预检允许 idle/OFF、稳定 Stand 或 Stop 后的 Hold，动作本身零 PWM且保持帧不变；运动、过渡、pending、单路或标定输出仍拒绝。启动/显式预检只检查所选模式的完整周期。

## 串口与网页操作

```text
arm                         # 仅单路调试解锁，不运动
arm_stand                   # 验证后使能并排队 Stand
stand | walk | trot | leg_test <1..4>
motion_stop | off
motion_set <step> <lift> <period_s> <height> <forward> <stride_bias>
motion_preview <stand|walk|trot|leg1|leg2|leg3|leg4>
motion_save | motion_load   # 全关/未使能时保存或恢复步态记录
body_set <pitch> <roll> <body_x> <cg_0_or_1> <cg_margin>
balance_roll_sign <-1|0|1>
balance_capture | balance_on | balance_off
moving_cg_on | moving_cg_off
drive_begin <epoch> <seq> <walk|trot> <forward[-1..1]> <turn[-1..1]> <step0|1>
drive_renew <epoch> <seq> | drive_end <epoch> <seq>
control_set <planner_slew> <wait_s> <angle_gain> <rate_gain> <tau> <max_shift> <feedback_slew> <pitch_kp> <pitch_kd> <roll_kp> <roll_kd>
profile_save | profile_load # 独立body/CG/control记录；全关无fault
trot_ff_set <enable0|1> <forward_gain> <backward_gain> <turn_gain> <max_abs_mm>
straight_trim_set <-0.30..+0.30> # RAM only；+纠正左偏，重启恢复默认0.25
trot_pitch_set <-0.5..0.0>       # 角度；负值压低前端；仅Trot/RAM，重启归零
```

推荐顺序：只读状态 → 选预设/应用其它步态参数 → 站立 → 实时调高度/Pitch/Roll → 点击锁定方向或开始Walk/Trot → 停止/关闭输出。需要跨重启保留时，先关闭输出，再分别保存motion/Profile。完整中文流程见 [WEB_UI_USER_GUIDE](../docs/WEB_UI_USER_GUIDE.md)。

## 严格回归命令

在本目录使用集中入口；脚本会以`-std=c++17 -Wall -Wextra -Werror`重编运行网页分段响应、host/gait/motion/body_balance/drive_control/IK/shank与Node UI回归：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File test\host\run_all_tests.ps1
```

`build-platformio`技能脚本detect/list/build均成功，未clean、未upload。0.8.14最终`firmware.bin`为1,040,512 bytes，2026-10-13 13:43:06，SHA-256 `8473C502941C254F6FFEC5B9A9D041C9C562AA6FBB84E81357E68C5766C827B7`。PlatformIO报告RAM 41,556/327,680 bytes（12.7%），Flash 1,003,376/1,310,720 bytes（76.6%）。尚未在真实手机/板卡验证实时body_x拖动或舵机连续跟随。
