# 2026-10-12 · 手机高度/Pitch/Roll滑块 · 固件0.8.12

## 结果

- 站高目标/down新增100..115mm、0.1mm滑块；Pitch/Roll新增-3..+3°、0.1°滑块。三个滑块均保留原number输入并双向同步。
- 拖动只修改现有motion/body草稿、增加revision并锁定启动，不POST、不使能、不运动。预设、轮询、reset和load读回同步range；状态轮询与迟到ACK不会覆盖较新的草稿；range与number的disabled状态一致。
- 高度仍通过`motion_set.height`应用，关闭输出后由`motion_save`写当前六参数；Pitch/Roll仍通过`body_set`应用，由`profile_save`保存。没有新增后端API、NVS字段或运动算法。
- 版本升0.8.12；0.8.11的Trot/Walk参数、body_x、trim和Trot Pitch行为保持。

## 备份与边界证据

- 修改前定向备份：`.work/pre-mobile-pose-sliders-0.8.12-20261012-223557.zip`，1,076,828 bytes，SHA-256 `0E751FB746EE88A94EA0D8D0CD55ECEEA572FACA470B5F98BB646FBBAB9D5326`。包含相关源码、测试、docs及0.8.11 firmware.bin，不含完整`.pio`。
- `.work/mobile_pose_sliders_0_8_11_probe.cpp/.txt/.md`直接使用当前产品运动学。固定Trot lift12.8、period.5、body_x8、trim.25，并覆盖两组中位、五intent、Pitch offset 0/-0.5°。
- down114共同0.1mm step上限18.5；18.6在倒退/Pitch0/phase .340/腿1触发`hip_window`。仅前进且覆盖两Pitch为19.8，下一档`shank_map`；仅前进/Pitch0为24.1，下一档`hip_window`。
- down100.0..113.9在相同lift包络下step0即`shank_map`；down114.0..115.0共同上限由18.5单调升至21.0。因此默认17.9不是最新共同极限，down114也不是网页范围内最大步长的最优down；30mm只是输入硬上限。本轮未据此改参数。
- down数值是髋轴到足端向下目标；数值越大意味着腿更伸长/机身目标更低，不是外壳实测离地高度。

## 验证

- `test\host\run_all_tests.ps1`：严格C++、Node、Web全套exit 0。Node覆盖三组range存在/约束、双向同步、预设/轮询/reset同步、dirty锁、disabled一致、零自动POST，以及原方向pad/trim/Trot Pitch/迟到ACK回归。
- `build-platformio`技能：detect成功（PlatformIO Core 6.2.0），list-envs确认`esp32-c3-devkitc-02`，指定环境build成功；未clean、未upload。
- RAM 41,556/327,680 bytes（12.7%）；Flash 994,130/1,310,720 bytes（75.8%）。
- `firmware.bin` 1,031,200 bytes；SHA-256 `1676AE7AE7AA6A3253B1CD5E72F541F0AB73AD9B19213F1A90DBFA65227F9EC9`。

## 未验证

没有烧录、连接板卡/串口、发送PWM或写真实NVS；未在真实手机浏览器测试触控尺寸、滚动和拖动手感。软件预检通过不证明实物舵机跟随、摩擦、供电、机构干涉或地面稳定性。
