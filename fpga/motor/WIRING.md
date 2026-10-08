# B 当前接线：BANK2 三引脚

更新：2026-10-08。配套程序为 [motor_control_v1_bank2_3pin](experiments/motor_control_v1_bank2_3pin/README.md)，旧四脚位流不适用。

## FPGA → B 电平转换器 → IBT-2

| FPGA 输出 | BANK2 球位 | 转换器 3.3 V 输入 | 转换器 5 V 输出 | IBT-2 |
|---|---|---|---|---|
| rpwm | T11 | B1 | A1 | RPWM |
| lpwm | P11 | B2 | A2 | LPWM |
| ibt_en | T12 | B3、B4 共接这个 FPGA 输出 | A3、A4 分别输出 | R_EN、L_EN 分别连接 |

**只在输入侧将 T12 分接 B3/B4；A3 与 A4 不能短接。** 原 T6/P6/T8/T9 控制连接移走；临时把 R_EN/L_EN 接 5 V 的跳线不得保留。
IBT-2 仍接四个控制端；FPGA 只用三个输出，因为两个使能共用一个信号。无外部 DIR 引脚。
T11/P11/T12 按板上球位标识识别，属于 PMOD1/DVP 复用信号；本方案不连接摄像头或占用这三脚的其他外设。
C 联合顶层使用 rpwm/lpwm/ibt_en，并合并 [三脚 CST](experiments/motor_control_v1_bank2_3pin/integration/B_motor_bank2_3pin.cst)。
不能将两个顶层输出 r_en/l_en 都约束到 T12。motor_enable 是 C 的内部协议输入，不是物理 ibt_en 输出。

## 逻辑电源与转换方向

沿用 B 已使用的 CJMCU-245 / SN74LVC4245A，B 和 A 各用自己的转换器。

| 端子 | 接法 |
|---|---|
| VCB | 3.3 V |
| VCA、IBT-2 VCC | 稳压 5 V |
| DIR | GND，固定 B→A；不是电机转向控制 |
| GND | FPGA、转换器、逻辑电源、IBT-2 共地 |
| OE（低有效） | 保留原 10 kΩ 上拉至 VCA，以及 OE→GND 的手动使能连接 |
| B1–B4 | 保留各自 10 kΩ 下拉；B3/B4 同接使能后两只下拉并联 |
| IBT-2 四个控制输入 | 保留各自 10 kΩ 下拉 |
| B5–B8 / A5–A8 | 未用输入接地；未用输出悬空 |

OE 为高时输出高阻，由驱动端下拉保持禁止；OE 为低才允许转换。OE 节点可能为 5 V，不能直连 FPGA GPIO。
新 CST 的三个电机输出为 LVCMOS33、4 mA、内部下拉；编译设置不改变实际 BANK 供电，外部下拉仍保留。

## 功率接线

- 稳压电源 CH1 正极 → B+，CH1 负极 → B−。
- M+、M− → 电机两根动力线。保留已更换的 0.75 mm² 动力线和可靠端接。
- 编码器线不接 M+/M−；R_IS/L_IS 当前不接 FPGA。
- 新版首次点动沿用 12 V、0.5 A 限流设置，按配套 README 的先零输出、后短时点动顺序操作。

换线时：关闭 CH1 输出，断开 FPGA USB 和给降压模块供电的 12 V 适配器。
旧四脚版本的实测记录保留；**本三脚版本目前只有仿真/编译通过，尚未实物验收**。

## 信号含义和其他引脚

| 状态 | RPWM | LPWM | R_EN、L_EN |
|---|---|---|---|
| 复位、锁定、停止、等待或禁止 | 0 | 0 | 0、0 |
| 逻辑正向 | PWM | 0 | 1、1 |
| 逻辑反向 | 0 | PWM | 1、1 |

C 接口的 control_u=0 或 abs(control_u)<10 均转换为停止。禁止输出不等于机构瞬间静止。
机械正方向与 theta 增减仍待 ABC 联调确认。
H11/27 MHz、S0/T10、S1/T3、S2/T2、S3/D7、LED3/N14、LED4/L14、LED5/L16 保持原分配。
A 的 P9/P8/R8/T7/N16 不被三个新电机输出占用；联合工程仍须统一检查全量约束。
