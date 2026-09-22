# fpga-inverted-pendulum
FPGA-based inverted pendulum control system for competition

## 项目资料

- [衡芯项目硬件信息汇总](docs/hardware-summary.md)：主控、驱动、采集、电源、安全与扩展模块的型号、接口及待确认项。
- [A 组旋转倒立摆采集工程](fpga/sensor/README.md)：ADS1115、霍尔编码器、Q16.16 状态、最终接线和下载文件。
- [Block A 实机验证版](fpga/block_a_verified/README.md)：2026-09-22 完成四状态 UART、方向、静止、运动及摆杆跨界验证的稳定检查点。

## FPGA 点灯基线

已建立适配 **Tang Primer 20K + Dock** 的 [Gowin 点灯工程](fpga/README.md)。目标器件为 GW2A-LV18PG256C8/I7（GW2A-18C），使用 H11 的 27 MHz 时钟驱动 L14 上的低有效 LED4，亮 1 秒、灭 1 秒。

2026-09-17：Gowin 综合、布局布线及位流生成通过，用户确认看到了预期闪烁效果。此验收仅覆盖点灯基线，不代表倒立摆外围或闭环控制已验证。

请完整下载/克隆仓库，再打开 `fpga/led_blink.gprj`；不能只下载单个工程索引文件。

## 机构更换进度（2026-09-17）

项目目标为旋转倒立摆。A 组已于 2026-09-22 在实物上完成 `theta / theta_dot / alpha /
alpha_dot` 联合验证：CCW 为正、CW 为负，静止速度回零，摆杆角度和角速度（含 ±π
跨界修正）工作正常。当前稳定版本保存在 `fpga/block_a_verified`，下一阶段进入不接电机的
LQR 输出验证。
