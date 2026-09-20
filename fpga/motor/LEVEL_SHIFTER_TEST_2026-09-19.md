# SN74LVC8T245 电平转换实板测试记录（2026-09-19）

## 测试目的

验证 Tang Primer 20K Dock 的 3.3 V 控制信号经 CJMCU-245（SN74LVC8T245）转换后，可在 5 V 侧得到与 FPGA 状态一致的四路输出，并验证 OE 关闭时 B 侧被外接下拉电阻保持为低电平。

本记录只覆盖 FPGA、按键、指示灯、电平转换模块和示波器组成的台架。未连接 IBT-2、直流电机或机械机构。

## 测试配置

- FPGA：Tang Primer 20K + Dock，GW2A-LV18PG256C8/I7
- 下载文件：`fpga/motor/board/impl/pnr/dock_motor_test.fs`
- 下载方式：Gowin Programmer，`SRAM Program`
- 电平转换模块：CJMCU-245（SN74LVC8T245）
- A 侧电源：FPGA 3.3 V
- B 侧电源：LM2596 降压模块输出 5.0 V
- 降压模块输入：12 V 适配器，实测输入约 11.8 V
- 示波器：RIGOL DHO4804，探头与通道均设为 10X，DC 耦合，1 MΩ
- PWM 观察设置：约 10 µs/div、1 V/div，CH1 上升沿触发，触发电平约 2.5 V

## 接线

| FPGA / 电源 | CJMCU-245 | 5 V 侧用途 |
| --- | --- | --- |
| 3.3 V | VCA | A 侧供电 |
| 公共 GND | A 侧 GND、B 侧 GND | 两侧共地 |
| 5.0 V | VCB | B 侧供电 |
| PMOD0 T6 / RPWM | A1 | B1 / RPWM |
| PMOD0 P6 / LPWM | A2 | B2 / LPWM |
| PMOD0 T7 / R_EN | A3 | B3 / R_EN |
| PMOD0 R8 / L_EN | A4 | B4 / L_EN |
| 3.3 V | DIR | A→B |
| 10 kΩ 上拉至 3.3 V，并用跳线接 GND | OE | 低电平使能 |

B1～B4 各使用 10 kΩ 下拉至 GND。A5～A8 接 GND，B5～B8 未使用。

## 上电前检查

- 12 V 适配器极性确认：中心正极；母座转接线红线为正、黑线为负。
- LM2596 输出经台式万用表确认约 5.0 V。
- 公共地各连接点导通。
- 断电电阻检查结果：VCA–GND 约 0.555 kΩ，VCB–GND 约 0.83 kΩ，VCA–VCB 约 0.74 kΩ；未见近似 0 Ω 的直接短路。
- 上电后确认：VCA 约 3.3 V，VCB 约 5.0 V，DIR 约 3.3 V。

## 测试结果

以下结果均由现场逐项操作并确认。

| 场景 | LED 状态 | B1 RPWM | B2 LPWM | B3 R_EN | B4 L_EN | 结果 |
| --- | --- | --- | --- | --- | --- | --- |
| 下载后待机 | LED2 | 低 | 低 | 低 | 低 | 通过 |
| 正转运行 | LED4 | 约 20 kHz、10% 占空比脉冲 | 低 | 约 5 V | 约 5 V | 通过 |
| 正转停止 | LED2 | 低 | 低 | 低 | 低 | 通过 |
| 选择反转并停止 | LED2、LED5 | 低 | 低 | 低 | 低 | 通过 |
| 反转运行 | LED4、LED5 | 低 | 约 20 kHz、10% 占空比脉冲 | 约 5 V | 约 5 V | 通过 |
| 反转停止 | LED2、LED5 | 低 | 低 | 低 | 低 | 通过 |
| 按住 S0 复位 | 全灭 | 低 | 低 | 低 | 低 | 通过 |
| 松开 S0 | LED2 | 低 | 低 | 低 | 低 | 通过；不会自动启动 |

示波器初次观察时波形明显跳动。重新夹紧示波器地夹后，B1、B2 波形稳定，说明先前异常与测量接地接触不良一致。照片因拍摄角度产生倾斜，周期和占空比按屏幕网格作近似确认。

## OE 关闭保护检查

1. 断开 12 V 和 FPGA USB。
2. 移除 OE→GND 跳线，保留 OE→VCA 的 10 kΩ 上拉。
3. 重新上电、重新下载测试文件并启动正转。
4. FPGA 运行指示正常，B3 仍约 0 V；B1 仍约 0 V且无脉冲。
5. 断电后恢复 OE→GND 跳线，再次上电和下载；B1 脉冲恢复。

结果：OE 高电平时转换器输出关闭，B 侧由外接 10 kΩ 下拉保持低电平；恢复 OE 低电平后输出恢复。

## 证据照片

- [CJMCU-245 模块正面](evidence/2026-09-19/level-shifter-front.png)
- [完整台架接线](evidence/2026-09-19/level-shifter-wiring.png)
- [B1 正转 PWM](evidence/2026-09-19/level-shifter-b1-forward.png)
- [B2 反转 PWM](evidence/2026-09-19/level-shifter-b2-reverse.png)

## 结论与边界

四路 FPGA 输出经过 SN74LVC8T245 后，在 5 V 侧的静态电平、方向选择、PWM 通道切换、停止、复位和 OE 禁用行为均符合当前台架预期。

本次没有接入 IBT-2 和电机，因此不能据此确认驱动器大电流输出、死区/换向瞬态、机械限位、急停链路或闭环控制性能。S2 是普通逻辑停止键，不应被视为独立硬件急停。
