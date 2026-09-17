# fpga-inverted-pendulum
FPGA-based inverted pendulum control system for competition

## 项目资料

- [衡芯项目硬件信息汇总](docs/hardware-summary.md)：主控、驱动、采集、电源、安全与扩展模块的型号、接口及待确认项。

## FPGA 点灯基线

已建立适配 **Tang Primer 20K + Dock** 的 [Gowin 点灯工程](fpga/README.md)。目标器件为 GW2A-LV18PG256C8/I7（GW2A-18C），使用 H11 的 27 MHz 时钟驱动 L14 上的低有效 LED4，亮 1 秒、灭 1 秒。

2026-09-17：Gowin 综合、布局布线及位流生成通过，用户确认看到了预期闪烁效果。此验收仅覆盖点灯基线，不代表倒立摆外围或闭环控制已验证。

请完整下载/克隆仓库，再打开 `fpga/led_blink.gprj`；不能只下载单个工程索引文件。
