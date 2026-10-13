# 2026-09-29 V10-only 精简版交接

## 用户目标与决定

用户明确要求活跃固件不再混用 V7.3 的 IK、连杆经验曲线和步态，精简并保存一次。以当前 Structure V10 装配中位为准，不改用户板上已有 NVS。项目根 `G:\PROJECT\Mechanical_dog` 不属于 Git 仓库，本次“保存”是落盘源码、状态文档与 ZIP，不是 Git 提交；未刷板。

## 改动

- 从 `mechanical_dog/include/src` 移除 `Applepy8Dof`、`ApplepyGaitPreview`、`MotionController`，移除网页步态预览/运动路由、RobotApp 八路运动入口和 ServoController 运动态；保留 PCA/HXY、单路手动保持、八路中位/全关与 IMU 调试。
- 新 `Ctrl/V10LegIk` 是显式长度、显式膝折叠方向的平面 FK/IK，不计算舵机角/PWM。V10 STL 上大腿髋—膝 80mm；小腿膝—圆脚端中心约61.357mm。没有直接采用 V7.3 的69mm或推断足底接触点。
- `PreferencesCalibrationStore` 的 `applepycal/c{channel}` 和 `ChannelCalibration` 已保存中位换算不变；NVS 未写入。页面仅保留单路、八路中位、IMU/姿态及优先全关。未配置 V10 连杆映射，所以固件中不存在站姿/WALK/TROT 执行 API。
- 当前文档入口已重写为 `docs/V10_KINEMATICS.md`、项目交接/技术文档、`REPLICA_STATUS/PROGRESS` 和接线表；旧研究仅在 `third_party/`、历史交接及清理前备份中追溯。

## 验证及产物

- `g++ -std=c++17 -Wall -Wextra -Werror`：`host_tests.cpp`（PCA/IMU/NVS/台架）、`v10_ik_tests.cpp`（V10两支FK/IK及非法参数）分别构建运行，退出0。
- `node test/host/web_ui_smoke.cjs` 退出0，核对无旧运动路由并验证忙时优先 OFF。
- PlatformIO `esp32-c3-devkitc-02` **先 clean 再 build** 成功；本地 `.pio/build/esp32-c3-devkitc-02/firmware.bin` 约849.4KB。没有上传、串口读取新版或运动真实舵机。
- 当前源码/主要文档快照：[results/applepy_v10_baseline_0.5.0_20260929.zip](../../results/applepy_v10_baseline_0.5.0_20260929.zip)，SHA-256 `FC0B2DF93EC94A5F3306FB62363F7D86E92ACD1AD6A090D6D83BE4B77F321AF8`；ZIP 内容与中文文件名已检查。删除前旧源码归档 `.work/pre-v10-cleanup-source-20260929.zip`，旧主文档归档 `.work/pre-v10-cleanup-docs-20260929.zip`。

## 风险与下一步

原板缺可控 OE#、硬急停/舵机电压采样，软件 OFF 不等于切电；手动台架仍保持到显式 OFF，八路回中非缓动。用户旧版八路NVS保持事实不验证新版或V10动力学。下一步固定足端参考点、其余腿镜像方向及一条腿连杆球头中心几何或少量安全小幅样本，在 PC 端建立膝折转角→小腿舵机脉宽的有界单调映射；只有单腿开环、限位、供电和物理切电都验收后才恢复多路动作。
