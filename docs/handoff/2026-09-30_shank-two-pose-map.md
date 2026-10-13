# 2026-09-30 小腿实际动作两点映射交接

用户希望简化映射并提供两张实际90°→51°舵机动作照片。以图序90、51理解；若顺序/通道有误须更正。照片固定H/K/F方向差估小腿相对大腿旋转21.31°，刚固在小腿的K/B摇臂变化减大腿变化为21.54°，两个独立方向相符；这不是精密3D测角，误差可为数度。以用户已调好的90°机械中位锚定，51°端点折转幅值约68.7°、膝内夹角约111°。

旧单照片 `V10PhotoLinkage` 在暂定符号下51°预测折转88.38°，明显与实际动作不符；复现器返回exit1。旧代码/测试已从活跃 src/include/test移除，归档 `.work/rejected-photo-fourbar-20260930.zip`，旧报告顶部明确标历史失败，不再把其非单调当真实机构。

新纯库 `Ctrl/V10ShankLinearMap` 显式接收本腿中位/端点两对，暂用 `(servo,bend)=(90,90),(51,68.55)`，斜率圆整0.55；`bend` 是 `abs(V10LegIk::knee_turn_deg)`，膝内夹角=180−bend，不要喂有符号角或内角。正向 `bend=90+0.55(s−90)`、逆向 `s=90+(bend−90)/0.55`，只在这些端点间插值，越界/NaN/退化参数失败且不改输出。没有默认镜像/四腿自动套用/实际PWM/NVS依赖，仍未接RobotApp或Web。

当前材料：`results/V10_shank_two_pose_calibration_20260930.md`、`.work/measure_shank_pair.py`、`.work/v10_shank_pair_measurement.json`、`.work/v10_shank_pair_angles.png`。原图未改。当前V10档案/README/状态/进度已同步；更多旧主文档版本存 `.work/pre-two-pose-docs-20260930.zip`。根项目无Git，只有磁盘文件和恢复ZIP，无提交号。

验证：板级 `host_tests`、V10 `v10_ik_tests`、新 `v10_shank_linear_map_tests` 三套严格g++运行exit0；Node UI烟雾exit0；PIO `esp32-c3-devkitc-02` **先clean再build**成功。没有刷板/串口命令/新PWM动作/改NVS。实物中间线性、机械端点裕量/负载安全、拍摄通道和其他腿方向未验；下一步用约70°无卡滞中间照片核对插值误差，才能逐步接单腿受限动作。
