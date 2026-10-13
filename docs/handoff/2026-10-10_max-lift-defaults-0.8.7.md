# 0.8.7 最大已标定抬脚预设交接

日期：2026-10-10。范围仅为普通Walk/Trot缺省与网页预设、NVS操作说明、版本、测试和当前文档；未改项目结构、NVS schema、轨迹函数或安全检查。

## 边界证据

用修改前0.8.6真实`V10Gait`编译只读probe，覆盖中位`{90,100,90,90,90,97,90,102}`与`{95,100,85,80,100,80,85,100}`，以及forward/reverse/left/right/stepping五种intent：

- Trot：step16、period0.5、down114、forward0、bias2。10个场景连续边界均为13.17996mm；13.1mm通过，13.2mm均在phase 0.25、leg 0触发`shank_map`。
- 普通Walk（CG关闭）：step6、period0.5、down108、forward0、bias2。10个场景连续边界均为7.17996mm；7.1mm通过，7.2mm均在phase 0.10、leg 0触发`shank_map`。

因此内置缺省和Trot按钮为`16/13.1/0.5/114/0/2`，Walk按钮为`6/7.1/0.5/108/0/2`。慢速按钮及CG Walk 3.2s专用预设不变。30mm输入上限、IK、hip窗口、三点shank map、0..180°和support检查均未放宽。

## NVS语义

记录仍是`applepygait/motion`一份共享六参数记录，格式与版本不变。已有有效NVS仍覆盖内置Trot缺省。网页明确步骤：先关闭输出 → 应用参数（RAM） → 保存已应用值到NVS。保存当前一套六参数，Walk/Trot共用，另一按钮预设不会同时保存。

## 验证与产物

- `test\host\run_all_tests.ps1`：严格C++/Node全套exit 0；含两组中位×五intent边界、30mm失败、NVS缺失新缺省、有效旧记录覆盖、保存回读和网页标签/说明。
- `build-platformio`脚本：加入本机PlatformIO penv到本次PATH后，`--detect`、`--list-envs`、`--env esp32-c3-devkitc-02`均exit 0；未clean、未upload。
- RAM 41,524/327,680 bytes（12.7%）；Flash 983,596/1,310,720 bytes（75.0%）。
- `firmware.bin`：1,020,576 bytes，2026-10-10 21:07:07，SHA-256 `3E0E4B577C4158423B3F9D738C3BE528E61420AE896FB48D98F0B23D4F05EB4D`。

修改前备份：`.work/pre-max-lift-defaults-0.8.7-20261010-210131.zip`。未烧录、未接板、未访问串口、未发送真实PWM、未写真实NVS。0.5s最大映射抬脚命令的舵机跟随、冲击、供电和机构裕量仍需悬空限流实测。
