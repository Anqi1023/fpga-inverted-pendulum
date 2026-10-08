> **当前 B 独立实板测试基线。** 新使能 T8/T9、就绪灯 N14；用户已确认点动后停下，电源 CV。见 [汇总](../../PROGRESS_2026-09-27.md)。

# B 保护点动：就绪灯改为 LED3 / N14

2026-09-27。用户指定调整 B 来解决 A 编码器灯和 B 就绪灯都占 N16 的冲突。A 的 LED2/N16 保留；B 的 led_ready_n 改到 LED3/N14，低电平点亮。只需更换程序，不需移动实体接线。

## 当前引脚分配

| 功能 | FPGA 引脚 | 板上名称 |
| --- | --- | --- |
| B 就绪指示 led_ready_n | N14 | LED3 |
| B 输出使能指示 led_run_n | L14 | LED4 |
| B 锁定指示 led_locked_n | L16 | LED5 |
| A 编码器指示灯（本 B 单独工程不驱动） | N16 | LED2 |
| RPWM / LPWM | T6 / P6 | 转换板 B1/B2 → A1/A2 → IBT-2 |
| R_EN / L_EN | T8 / T9 | 转换板 B3/B4 → A3/A4 → IBT-2 |

已对照 A/C 当前仓库约束检查：除共同 H11 时钟输入外，A 与此 B 工程没有重复引脚。N14 与 LED3 的对应来自 Dock 3713 原理图第 1 页及仓库硬件核对记录。T9 仍占用 RGB 数据功能，外部 MIC 阵列保持不接。

## 下载文件

```text
D:\FPGA\hengxin\motor-protection-jog-ab-led3\impl\pnr\motor_protection_jog_ab_led3.fs
```

仓库内可直接选择本目录的 `motor_protection_jog_ab_led3.fs`，与原 D 盘已实测文件逐字节一致。完整工程为 `motor_protection_jog_ab_led3.gprj`。顶层 dock_motor_protection。

工作区交付包 motor_protection_jog_ab_led3.zip 包含完整相对路径源码工程、约束、构建脚本、验证报告和预编译 motor_protection_jog_ab_led3.fs。直接试转时，在 Gowin Programmer 选择 SRAM Program，FS File 选择上述 D 盘文件或解压后目录根部的同名 .fs，无需重新编译。

查看源码时在 Gowin IDE 打开 .gprj，不要只添加顶层 .v。重新生成位流应运行随附 build.tcl（gw_sh build.tcl），该脚本明确设置顶层、将 SSPI 引脚用于 GPIO，并保留 JTAG；仅打开 .gprj 不能代替脚本中的器件选项。解压后如需重新构建，使用不含中文的路径。

仍为固定正向 1 kHz / 10%、最长 200 ms 点动、1 秒冷却；S0 复位锁定、S1 清锁/再次按下点动、S2 急停请求、S3 模拟故障。不是 A/B/C 闭环整合程序。

## 最小下载检查

1. 关闭台式电源 CH1 电机输出，保留 FPGA USB 和逻辑 5 V；接线保持。
2. SRAM Program 下载本文件。等待 2 秒，应只有 LED5 亮。
3. 按一下 S1，松开，再等 2 秒，应 LED3 亮、LED4/LED5 灭。此时只清除锁定，CH1 仍关闭。

旧说明中 B 的“LED2 就绪”在此版本全部改为“LED3 就绪”。旧 ab_pins 文件仍驱动 N16，不作为消除 LED 冲突后的整合基线。A 的 LED2 编码器功能要在加载包含 A 的联合程序后才会出现，本 B 单独文件不会采集编码器。

## 软件核验

- 与上一版 motor_protection_jog_ab_pins 比较：唯一功能配置改动是 CST 中 led_ready_n 从 N16 改为 N14；顶层只更新一行 LED 注释，其余 RTL 和 SDC 相同。
- 复用此前已经通过的保护逻辑/完整 200 ms 仿真及旧引脚灯版的用户实测；本次未重复运行相同逻辑仿真。
- Gowin 2026-09-27 16:10:49 完成综合、布局布线、位流生成，无 WARNING/ERROR；建立/保持时序违规 0/0，27 MHz 约束下 Fmax 167.970 MHz。
- 实际引脚报告确认：N14=led_ready_n，N16 为未使用输入，L14/L16 及 T6/P6/T8/T9 按表分配。
- 位流 SHA256：C21BDBE15BDD4390EEAAC5EBF7F17B8E28F790EF4B115FAC7A4B41D03FEE2871。
- 用户随后确认初始 LED5 锁定和 S1 清锁后 LED3 就绪检查通过；重新下载本版本后再次确认通过。
- 本版本实际点动已完成：CH1 按 12 V、0.5 A 限流设定开启后，用户确认电机保持不动、LED3 就绪；再次短按 S1，明确反馈“转动后停下，CV”。200 ms 来自固件和仿真，未独立计时；本次未报告电流数值或机械制动时间。
- 测试后已指导关闭 CH1、保持接线；当前准备与 C 确定 control_u 接口，尚未接入连续闭环控制。

## 再次进行 200 ms 点动

完成上述 LED3 就绪检查后，CH1 设定沿用 12 V、0.5 A 限流。机构保持此前检查状态，手离开旋转臂。打开 CH1，旋转臂应保持不动；如自行转动，立即关闭 CH1。随后短按一次 S1 并松开，应正向转动约 200 ms 后停下，LED4 熄灭，冷却后 LED3 恢复；记录电源 CV/CC 状态。S2 为逻辑急停请求，S3 为模拟故障，不是换向按钮。每次重新下载会回到 LED5 锁定，首次 S1 只清锁，下一次 S1 才点动。

参考原理图：D:\FPGA\hengxin\reference\Tang_Primer_20K_Dock-3713_Schematics.pdf。
