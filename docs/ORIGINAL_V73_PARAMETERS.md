# Py-Apple Dynamics V7.3 原版参数与控制路径证据

本页记录只读参考源码 `third_party/py-apple-servo/software/py-apple-dynamics/Py Apple Dynamics V7.3 SRC/PA-Dynamics V7.3/` 的事实。上游根许可按GPL-3.0处理；原文件/启动打印又声明Apache-2.0，存在不一致。0.8.5移植Trot纯足端函数；0.8.6移除协调命令角速度拒绝；0.8.7调整抬脚预设；0.8.8把当前Profile静态body_x默认设为+8；0.8.9提高普通步长并增加RAM直行差动修正；0.8.10增加当前工程独有的RAM Trot Pitch角前馈；0.8.11把当前机器实测采用的直行trim启动默认固定为0.25；0.8.12只增加当前工程网页的高度/Pitch/Roll滑块和边界说明，其余控制仍是C++适配。

## 0.8.5移植的Trot kernel

`PA_GAIT.trot()`令腿1/3与2/4半周期错相，摆动和支撑各占50%。对`x0=z0=xv0=0`及归一化`u=t/Tf`：

```text
swing x/S = 0                              u < .25
            -16u³ + 24u² - 9u + 1         .25 ≤ u < .75
            1                              u ≥ .75
swing z/H = -16u³ + 12u²                   u < .5
            4u(1-u)                        u ≥ .5
support x/S = 1-u, z=0
```

原Python在`u=.75`使用严格`>`导致`xf`未赋值；0.8.5按左右连续极限补成1。原摆动touchdown垂直速度不为0，这是忠实行为而非raised-cosine。原x为0..S，V10接口只做`x_v10=S*(x/S−1/2)`坐标平移；机器人forward到IK仍只反号一次。

原`h=30/H=110`不能直接通过当前已标定模型：当前q表径向约100.8..118.3mm。host隔离证据显示step30/lift30/period10先在phase .087触发hip窗口；step=0后在phase .092触发shank映射；原理论0.1s还低于当前0.4s周期下限。0.8.9在`body_x=+8`、当前13.1/7.1抬脚与0.5s周期下测得Trot共同步长17.9、普通Walk25.3；下一0.1mm由限制性intent触发`hip_window`。这些是软件命令模型边界，不是实物机构或防滑安全极限。

原项目本身没有当前`straight_trim`这一独立诊断参数。正trim通过前进`turn=-trim`、倒退`turn=+trim`让物理右侧幅值减小，以补偿用户观察到的持续左偏；显式转向与踏步不变。0.8.11启动/重启默认0.25，手动值仍只在RAM中且不进入Profile或motion NVS；不能称为原项目逐值移植或自动Yaw闭环。

0.8.12滑块没有改变原项目或当前步态公式。当前down114、lift12.8、body_x8、trim.25在两组中位×五intent×Pitch 0/-0.5°的最新共同step上限为18.5mm，down115为21.0mm；默认仍保持17.9mm。这里的30mm是当前网页/参数输入硬上限，而非原项目或当前实物的已证最长步长。

原项目也没有0.8.10当前形式的`trot_pitch_offset_deg`独立控件。它只在Trot的BodyPose角度前馈中叠加`-0.5..0°`，默认0且重启归零；与原项目根据姿态修改`X_goal`的移动移身控制不是同一算法，不能称IMU闭环移植。

原`padog.mainloop()`在Trot时还通过`PA_ATTITUDE.cal_ges()`叠加姿态、`in_y=17`相关X基准，并按`spd`始终加入前/后/转4/4/2重心偏置。当前0.8.8的+8只使用`V10BodyPose`静态全局平移语义，来源是用户观察本机初始承重偏后；它不等于原`in_y=17`，也没有恢复原动态4/4/2或`X_S`状态。当前Trot前馈、IMU反馈仍默认关闭，Profile NVS schema/version未改。

## 配置值

`config_s.py`：

| 字段 | 原值 | 实际含义/使用情况 |
|---|---:|---|
| `l1` | 80 mm | 原腿部IK大腿长度 |
| `l2` | 69 mm | 原腿部IK小腿长度；V10改用STL名义候选61.357 mm，不是实机精密测量 |
| `l` | 142 mm | 原名义机身前后髋轴间距；0.7.0暂作可追溯参考，本机未量 |
| `b` | 92.8 mm | 原名义左右髋轴间距；0.7.0暂作可追溯参考，本机未量 |
| `w` | 108 mm | 原姿态函数的腿间距/偏置参数；不是0.7.0站高证明 |
| `speed` | 0.05 | 原相位每有效调用增量，不是秒周期 |
| `h` | 30 mm | 这份实际运行路径中Trot和Walk都把`h`传给步态函数作为抬腿目标；V10不可照搬，必须通过当前q表/IK预检 |
| `pit_Kp_G` / `pit_Kd_G` | 0.04 / 0.6 | 原静态自稳每调用累计项；不是带秒单位的现成PD |
| `rol_Kp_G` / `rol_Kd_G` | 0.04 / 0.35 | 同上 |
| `tran_mov_kp` | 0.1 | 原`X_S`每调用比例逼近；Walk使用一半 |
| `walk_h` / `walk_speed` | 50 / 0.02 | 在本次核对的`PA_GAIT.walk`执行路径只定义、未实际使用；Walk实际仍接收`padog.py`传入的`h` |
| `in_y` | 17 mm | 原网页/重心X基准 |
| `R_H` / `H_goal` / `Hgt` | 110 mm | `padog.py:221`运行高度状态/目标与`web_c.py:9`网页默认站高；网页滑杆允许90..120 |

## 时间与步幅

- `main.py`用`Timer(1)`每5 ms调用`padog.mainloop()`。
- Trot在`padog.py`中每有效调用`t += speed`，到1回绕；`speed=.05`时纯计数约20次、理论约0.1 s。这不等于真实舵机、网络和负载周期。
- Trot调用`PA_GAIT.trot(t, spd*10, h, ...)`。网页`web_c.py`先算`thr=int(f)*6/100`，再夹到±3，所以网页前后目标上限约30 mm，不是固定步长；启动时`spd=spd_goal=0`，不会上电自动使用30 mm。RC路径`thr`可到+6，对应60 mm，二者不同。
- Walk相位0..2.5；每个被允许推进的调用增加`speed/5=.01`。若完全没有重心等待，约250次×5 ms=1.25 s；实际还会等待`abs(X_S-X_goal)<1`，所以更长且不固定。

## 姿态与静态自稳

`PA_ATTITUDE.cal_ges(PIT,ROL,l,b,w,x,Hc)`用四个名义髋点做刚体姿态计算，随后只把每腿x/z交给8DOF IK，侧向y被丢弃；原Yaw固定0。这不是完整6DoF、足端反馈或质心测量。

`PA_STABLIZE.stab()`每调用执行：

```text
Sta_Pitch -= (Pitch - pitch_origin) * 0.04 + gyro_x_radps * 0.6
Sta_Roll  -= (Roll  - roll_origin)  * 0.04 + gyro_y_radps * 0.35
```

随后分别限幅。它依赖调用频率且持续累计。`padog.mainloop()`只在静止、速度接近0且无转向时调用；运动时把`PIT_goal/ROL_goal`置0。0.7.0因此采用显式真实dt、有界PD、输出/斜率限制和新参考门控，不把原每调用增益直接当SI参数。

## 原Walk真正移身

`PA_GAIT.foward_cg_stab()`在IMU有效时把全局目标设为：

```text
X_goal = in_y + gait_need
       + 110 * tan(pitch_error_rad * 1.1)
       + gyro_x_radps * 5
```

原Walk在腿1/2前用`gait_need=-30`，腿3/4前用`+40`，且只有`abs(X_S-X_goal)<1`才抬腿；最后四足支撑回程。它是全局机身X状态，不是0.6.1每腿`walk_body_shift_mm`局部bias。旧全套幅度、姿态与相位组合不可直接照搬V10，会触碰当前IK/q表/通道限制；当前只采用通过完整预检的小幅命令。

0.8.8静态+8与原`X_S/X_goal/tran_mov`的共同点仅是都通过全局前后机身命令改变足端相对机身的位置。原项目以`in_y=17`和步态阶段目标连续迁移；当前无有效Profile NVS时固定从+8开始，且没有足底压力或实际COM反馈，因此不能用17−8推导“少了9mm”，也不能宣称两套坐标数值可直接比较。

## 原网页范围

`control.html`滑杆：高度90..120 mm、Pitch/Roll各±10°、前后X±40。范围只是旧UI输入，不证明V10可达。当前手动姿态仍限制Pitch/Roll±3°、body_x±8mm，并由80/61.357mm IK、共享小腿表和独立中位决定是否接受；0.8.6不再按命令角速度拒绝。
