# fpga-inverted-pendulum
FPGA-based inverted pendulum control system for competition

## 项目资料

- [衡芯项目硬件信息汇总](docs/hardware-summary.md)：主控、驱动、采集、电源、安全与扩展模块的型号、接口及待确认项。

## FPGA 点灯基线

已建立适配 **Tang Primer 20K + Dock** 的 [Gowin 点灯工程](fpga/README.md)。目标器件为 GW2A-LV18PG256C8/I7（GW2A-18C），使用 H11 的 27 MHz 时钟驱动 L14 上的低有效 LED4，亮 1 秒、灭 1 秒。

2026-09-17：Gowin 综合、布局布线及位流生成通过，用户确认看到了预期闪烁效果。此验收仅覆盖点灯基线，不代表倒立摆外围或闭环控制已验证。

请完整下载/克隆仓库，再打开 `fpga/led_blink.gprj`；不能只下载单个工程索引文件。

## 机构更换进度（2026-09-17）

项目目标仍为旋转倒立摆。误购的直线倒立摆正在更换为旋转机构；据团队确认，两款共用说明书、接线图和使用资料，电机及摆杆角度传感器相同。现有 FPGA 点灯、电机驱动代码和离线验证结果保留。新机构尚未完成实机测试；机械运动范围、实际转向和传感器安装零点仍需到货核对。共用不代表厂家原始资料已全部归档。
