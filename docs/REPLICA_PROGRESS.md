# Applepy_v1 · V10 当前进度

更新：2026-10-13。0.8.14把body_x加入height/Pitch/Roll原子实时重定位链；默认运动参数、范围与NVS schema不变。未上传、未触硬件、未写真实NVS。

传统控制里程碑已获用户实机体验认可并进入保存/发布收口。该状态涵盖Stand、Walk/Trot、点击方向/原地踏步、实时姿态与重心、标定、预检、保存及IMU补偿；仍按开环8DOF边界记录，不把缺少足压/关节/接触反馈的实体系统标为闭环完成。原地踏步轻微后退作为已知限制暂缓，当前`body_x`默认+8且范围±8，后续是否修正需新增机械或反馈证据。

| 子系统 | 当前状态与证据 | 剩余风险/下一步 |
|---|---|---|
| 传统控制里程碑0.8.14 | 用户实机体验认可；Stand、Walk/Trot、点击前后左右、原地踏步、实时高度/P/R/X、标定/预检/保存、静态/移动IMU补偿均已集成；严格测试与最终BIN证据完整 | 原地踏步轻微后退；无足压/关节/接触反馈，不宣称实体闭环完成；下一阶段可转向反馈、机械配重或更高层控制 |
| PCA/IMU/SoftAP/串口 | 主机假总线、网页 Node 回归；启动全关、ARM不动 | HXY约5.5m/s²历史异常、轴向与实物新版本未验 |
| 单路/q/八路中位 0.8.3 | L3H改Servo13/CH12；c12优先、仅缺失时读c8，保存只写c12；其余键与小腿q不变；真实Port stub回归exit0 | 必须断舵机电源后物理换插头并另行烧录；逐键NVS和八路I²C均非事务 |
| V10 IK与映射 | 协调顺序`0,1,2,3,12,9,10,11`，CH12正向帧/预检/单路/标定/offset测试通过，CH8从机械狗池拒绝 | 足底接触点、打印/装配误差、回差与负载误差待测 |
| Stand | 请求零PWM；新IMU样本后中位→1.5s平滑站姿；到达后保持 | 首帧中位无反馈，必须悬空；实际站高/干涉未验 |
| Walk | 默认关闭CG时保留0.6.1单腿顺序/stride bias；显式CG开时切换全支撑planner并忽略旧bias；可叠加受限移动Pitch反馈 | 无接触/实际COM反馈；命令到位不等于真实COM到位 |
| Trot | 两组对角腿半周期交替、周期连续 | 未上板、未落地，供电与冲击未知 |
| 状态机/安全 | pending generation、新样本门控、1.5s smoothstep、Stop/Off、IMU/gap/runtime/bus failure、模式互斥均有假件回归 | 不再按命令角速度拒绝；软件全关非硬急停，八路顺序写中途故障可能留下部分帧 |
| 增量预检0.8.4 | 普通/CG/移动包络逐帧cursor；每tick≤1帧，旧运动不饿死，commit重验；Drive provisional lease与完整取消矩阵有回归 | max slice仅host合成观测，0.8.4 ESP32-C3单帧实际耗时仍待测 |
| Web/API 0.8.2 | 保留0.8.1 Flash分段+token；控制/标定双视图、唯一步态选择、方向pad、四常用参数、紧凑标定流；高级能力收起但完整保留。严格Node覆盖零动作/dirty/readback/Stop/竞态，本地mock桌面/手机、Walk和标定流程通过 | 0.8.2未上传/未实机浏览器验收；void WebServer发送API不能证明真实网络无短写；命令遥测仍非反馈 |
| 手机姿态滑块0.8.12 | 高度100..115、Pitch/Roll±3°双向同步；沿用motion/body草稿、应用、NVS归属、disabled、迟到ACK和启动锁；Node零POST回归 | 尚未用真实手机浏览器/触控实测；高度是down而非外壳实测离地高度 |
| 实时姿态0.8.13 | 原子`pose_live_set`、150ms合并/单并发/latest-wins/失败回退；OFF零动作、Stand/Hold transition重定位；Pitch−7..+5、Roll±10硬范围+逐tuple预检 | 未实机验证拖动延迟/舵机连续跟随；硬范围不是组合可达保证 |
| 实时body_x 0.8.14 | 四字段原子命令；bX range-number双向、150ms合并/readback/回退；±8边界、±8.1原子拒绝、Profile保存/重启host回归 | 默认+8和范围未扩大；组合可达仍由Stand预检决定；手机/实体未验 |
| 步态参数持久化 | `applepygait/motion`版本化完整记录；缺失使用新Trot缺省，损坏/IO失败回退；有效旧记录仍覆盖；网页明确全关→RAM应用→NVS保存 | 只保存当前一套六参数，Walk/Trot共用；板上NVS尚未实测 |
| 普通预设0.8.7 | body_x=0时选定Trot 16/13.1/0.5/down114、Walk 6/7.1/0.5/down108；当时下一0.1mm均`shank_map`失败 | 0.8.8 +8下它们是保守预设，不再称当前绝对最大 |
| Trot轨迹0.8.5 | 原摆动/支撑kernel、50%对角错相；轨迹函数未改 | 实机跟随能力、落地幅度和冲击未验 |
| 协调速度拒绝0.8.6 | 配置、预检错误、峰值遥测、transition/runtime/balance/moving-CG拒绝链全部删除；原速度专属失败测试现通过 | 舵机实际速度由硬件决定，较快命令可能滞后或冲击，必须悬空限流验证 |
| 机身姿态0.7.0 | 142×92.8mm名义髋位纯变换；Pitch/Roll/body_x草稿/RAM/预检；零姿态兼容与符号测试 | 本机轴距未量；8DOF丢弃侧向分量，不是完整3D |
| 默认前移0.8.8 | 内置Profile `body_x=+8mm`；两组中位×五intent的Stand/Walk/Trot通过；重扫共同边界Trot≈13.4477、Walk≈7.49786mm，0.1mm档13.4/7.4通过、13.5/7.5拒绝 | 默认仍保守保持13.1/7.1；来自用户实机观察，不是压力/接触/COM测量；旧NVS可能仍显示0 |
| 最大步长0.8.9 | 修改前两组中位×五intent：Trot 17.9、Walk 25.3通过；下一0.1mm在限制性非踏步intent触发`hip_window`；30.1参数拒绝 | 仅软件命令模型，不是负载/防滑/舵机跟随安全认证；踏步不使用step幅值 |
| Trot down复核0.8.12 | lift12.8/body_x8/trim.25、两组中位×五intent×Pitch 0/-0.5°：down114共同18.5，down115共同21.0；单向前进两Pitch 19.8、Pitch0 24.1 | 默认仍17.9；100..113.9因12.8抬脚包络在step0即`shank_map`；不代表实物安全边界 |
| RAM直行修正0.8.11 | ±0.30纯函数、命令/API/UI、dirty/readback、普通/方向/preview/fake-PWM；默认0.25在两组中位×五intent×Pitch 0/-0.5°通过 | 手动值不持久化，重启恢复0.25；左偏根因及实体轨迹仍需实测 |
| Trot Pitch补偿0.8.10 | `-0.5..0°`仅Trot BodyPose前叠加；17.9/12.8在两组中位×五intent全过，12.9下一档`shank_map`；命令/API/UI/revision/迟到ACK/preview/start/direction/Stop/Off/Profile隔离回归 | 开环角度前馈，不是IMU闭环；默认0且不持久化；实体抬头改善程度待测 |
| 静态IMU自稳0.7.1 | Pitch默认−1不变；Roll已确认默认−1（左低右高时sensor正，body负），0关闭、+1覆盖；重启不恢复enable/reference | 未上板复核Roll轴与增益；只限Stand，不是移动自稳 |
| CG Walk 0.7.0 | 全支撑移身+腿1/2+换向+腿3/4+回程；实际模型足端XY支撑三角形、IK/映射全周期检查 | 名义COM仅体中心假设；无接触/实际COM反馈 |
| Signed drive 0.8.0 | 前后/左右差速/原地转向/踏步；Walk/Trot/CG实际假PWM多周期；旧step NVS兼容 | 方向pad是离散意图，不是连续模拟摇杆 |
| 点击锁定方向0.8.13 | click begin/renew、方向切换、原地踏步payload、活动态、Stop/Off/visibility/network与lease Node回归 | 手机浏览器菜单/触摸手感及实体转向待测；Hold仍保留力矩 |
| 移动CG 0.8.0 | body Pitch误差/导数、±0.5mm/2mm/s、planner 5..30mm/s、相位等待/超时、真实轨迹预检 | 无实际COM/接触反馈；参数仍需上板调试 |
| 安全会话0.8.0 | epoch/seq/500ms lease、迟到begin/renew隔离、松手/网络超时Hold、手动ARM规则不变 | Hold保留力矩且可能停在抬腿帧，不等于站稳 |
| Profile NVS 0.8.0 | 独立`applepyprof/profile`，body/CG/planner/movingCG/Pitch和Roll独立Kp/Kd；损坏/NaN/失败回退 | 不存enable/reference/drive/session；板上NVS未实测 |
| Trot intent前馈0.8.0 | 默认关闭；signed stride/10自动前/后/转向，独立f/b/turn/max，踏步0；Profile/预检/runtime测试 | 仍是命令前馈，不是实际COM反馈 |
| 迁移账本 | `MIGRATION_MATRIX.md`逐项映射原模块/API/硬件缺口 | 软件适配完成不等于整机实物完成 |
| 固件产物 | 0.8.14 detect/list/build成功；BIN 1,040,512 bytes，SHA-256 `8473C502941C254F6FFEC5B9A9D041C9C562AA6FBB84E81357E68C5766C827B7`；RAM 41,556/327,680（12.7%）、Flash 1,003,376/1,310,720（76.6%） | 未clean/upload；仍需用户授权后才可烧录/实测 |

下一步只能在用户明确授权且具备限流/物理断电时：上传后先核对版本和空载 PWM/OFF，再悬空 Stand，随后一次最慢 Walk/Trot；任何干涉、异响、电压跌落或方向错误立即切断舵机 V+。不直接进入地面闭环测试。
