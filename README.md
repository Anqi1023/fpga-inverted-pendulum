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


## B 电机执行链路进展（2026-10-08）

当前采用 **BANK2 三引脚：RPWM=T11、LPWM=P11、共用使能 ibt_en=T12**。高云工程、C 正式执行链和接线说明见 [B 最新成果](fpga/motor/PROGRESS_2026-10-08.md)。三项仿真及编译通过，**新版尚未上板实测**；旧四脚版本的实际点动、保护及 CV 记录见 [历史成果](fpga/motor/PROGRESS_2026-09-27.md)。

给 C 使用 [正式三脚执行链](fpga/motor/experiments/motor_control_v1_bank2_3pin/integration/README.md)，B 单独测试使用 [200 ms 点动工程](fpga/motor/experiments/motor_control_v1_bank2_3pin/README.md)。两者用途不同；尚未验收三人联合闭环控制。
