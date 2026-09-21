# 电平转换台架记录（2026-09-19；2026-09-21 更正）

> **接线更正：**此前版本把 CJMCU-245 的 A/B 供电和信号方向写反。板上芯片顶标 `LJ245A` 对应 TI **SN74LVC4245A**，不是 SN74LVC8T245。按[器件数据手册](https://www.ti.com/lit/ds/symlink/sn74lvc4245a.pdf)，本项目应将 **VCA 接 5 V、VCB 接 3.3 V、DIR 接 GND（B→A）、OE 接 GND 使能**。先前版本的 VCA=3.3 V、VCB=5 V 接法不可复用；5 V 超过该器件 VCCB 的绝对最大额定值 4.6 V。后续单通道功能检查正常，不能据此判断器件的长期可靠性。

## 原台架观察的范围

2026-09-19 使用 Tang Primer 20K + Dock、12 V→5 V 降压模块、CJMCU-245 和 RIGOL DHO4804 做了不接 IBT-2、电机和机械机构的信号台架观察。原照片保留如下，作为历史观察资料，**不作为原错误供电接法的验收依据**：

- [转换模块正面](evidence/2026-09-19/level-shifter-front.png)
- [当时的接线](evidence/2026-09-19/level-shifter-wiring.png)
- [当时正转波形](evidence/2026-09-19/level-shifter-b1-forward.png)
- [当时反转波形](evidence/2026-09-19/level-shifter-b2-reverse.png)

曾观察到待机控制线为低、运行时约 20 kHz 的窄脉冲，以及 OE 停用后输出被外部下拉保持为低；这些观察与当时的供电错误不能混同为可靠的器件验证。旧记录中的信号 A1～A4→B1～B4、DIR=3.3 V、B 侧 10 kΩ 下拉以及原电源电阻读数均不适用于更正后的接法。

## 更正后的连接与复测

实际连线：FPGA T6/P6/T7/R8 分别到 **B1/B2/B3/B4**；对应的 **A1/A2/A3/A4** 到 IBT-2 的 RPWM/LPWM/R_EN/L_EN。VCA=5 V、VCB=3.3 V，两侧 GND 与 FPGA、降压模块及 IBT-2 逻辑地共地；OE 以 10 kΩ 上拉到 VCA 作为默认禁用，台架使能时可靠接 GND；DIR 接 GND。B1～B4、A1～A4 分别各有 10 kΩ 下拉至 GND；未用的 B5～B8 接低，A5～A8 不接负载。

OE 曾处于悬空状态，万用表读约 0 V 但 A 侧输出仍为 0 V。断电导通检查确认 OE 未接地，重新可靠接地后，独立 B5 输入约 3.3 V、A5 输出约 5 V。此故障不能仅凭 OE 的万用表电压判断使能状态。

正确供电并连接 IBT-2 逻辑侧后的复测结果，以及尚未完成的电机试转范围，见[2026-09-21 执行链路台架记录](BENCH_TEST_2026-09-21.md)。
