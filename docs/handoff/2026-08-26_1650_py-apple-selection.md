# Py-Apple 舵机版本筛选交接

## 当前目标

从 `ToanTech/py-apple-quadruped-robot` 及其三个子仓库筛出最新、非 BLDC、可形成整套舵机狗的复刻基线，并与现有 OQ12 目标对齐。

## 已冻结的上游来源

- 聚合仓库：`ToanTech/py-apple-quadruped-robot@d2bb34f82ff1d1c69c657c50e3af1608ebb6d1fb`
- 软件：`ToanTech/py-apple-dynamics@c69ab76df92a9a162b6608ea0ffb72d23c1e8990`（V7.3，8DOF）
- 控制器：`ToanTech/py-apple-controller@83d989b5d24b92cb699e61e1c798fc7b837c6df5`（V4.0 直插分立板）
- 结构：`ToanTech/py-apple-structure@84e3048eb545fca0ae44138a09eeb14e54feba5c`（V10.0 串联腿；实际 BOM/几何仍为 V7.0 8DOF）

## 已完成

- 核对聚合仓库只有 `master`、无 tags；证明聚合仓库内 V6.8 不是最新软件。
- 通过子仓库 README、GitHub 提交记录、V7.3 源码、Controller PDF/XLSX、Structure V10 STL/BOM 交叉核对版本。
- 确认当前舵机主线是 8DOF，不存在可直接提取的完整 12DOF 舵机结构/IK。
- 记录了 GPIO、I2C、PCA9685 通道、采样/控制周期、低压停机和网页命令路径。
- 记录了 `PA_SERVO.py` 缺 `math` 导入、通道 15 未释放、GPIO 复用、HTTP `exec()` 等复刻前必须修复的问题。
- 发现三个当前子仓库根 LICENSE 为 GPL-3.0，而旧源码头/教程写 Apache-2.0，后续按 GPL-3.0 做合规审查。

## 文档变更

- `docs/ORIGINAL_PROJECT.md`：加入版本筛选、提交冻结点、排除项和许可证边界。
- `docs/ORIGINAL_ARCHITECTURE.md`：加入 V7.3 软件运行流、代码地图、硬件文件地图和缺陷清单。
- `docs/PORTING_PLAN.md`：加入 8DOF 参考实现与 OQ12 12DOF 分叉、目标目录和分阶段验收路线。
- `docs/REPLICA_PROGRESS.md`：加入 Py-Apple 各子系统的 studying 状态。

## 已验证命令/证据

- `git ls-remote --heads --tags https://github.com/ToanTech/py-apple-quadruped-robot.git`：仅 `master`。
- GitHub API：三个子仓库最新提交分别为上面冻结的 commit。
- Controller BOM：ESP32、PCA9685、XL4005、AMS1117、MPU6050、800 mAh 2S 25C 电池。
- Structure V10 BOM：8×MG90S、4×6704ZZ、2 mm 球头、M2×L30 连杆；V10 STL 仅 9 个文件，未提供 STEP。
- Dynamics V7.3：PCA9685 `0x40`、SDA18/SCL19、IMU SDA21/SCL22、ADC33、SERVO_EN25、8 路通道 `2,3,13,12,10,11,5,4`、5 ms 主循环。

## 未完成/下一步

1. 选 2 个 MG90S 或同尺寸候选做堵转电流、回差、脉宽和温升实测。
2. 先用 ESP32 + PCA9685 + 外置电源完成单路/8 路舵机和 IMU 开环测试。
3. 复刻软件时先做 `IServo/IImu/ISafety/ITransport` 接口和主机 IK/轨迹测试，再接 ESP32。
4. 若继续 OQ12，另起 12DOF 机械/3D IK 分支，不要把 V10 8DOF 当成 12DOF 输入。

## 后续补充：上游资料已落盘

- 资料根目录：`third_party/py-apple-servo/`。
- 软件：V7.3 源码、开发工具、uPyCraft、USB 串口驱动。
- 硬件：Controller V4.0 BOM、Altium `SchDoc/pcbdoc/PrjPcb` 和库、三份 PDF。
- 机械：V10 BOM/ZIP/9 个 STL，V7 串联腿 RAR/BOM。
- 校验：V10 ZIP 10 个条目，解包 9 个 STL；V7.3 源码 12 个文件；三个上游仓库均在冻结提交且 `git status` 干净；未发现 BLDC/无刷文件。
- 哈希：见 `third_party/py-apple-servo/docs/DOWNLOAD_MANIFEST.md`。
