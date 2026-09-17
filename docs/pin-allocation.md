# A/B/C 初版引脚分配（v1）

日期：2026-09-17。用户授权统一分配；用于三组同时开发的资源预留。
本文件定义项目计划，不表示队友已接线或已通过实机验收。
适用目标：Tang Primer 20K + Dock，GW2A-LV18PG256C8/I7。
实物照片确认 Dock 型号及以下 PMOD 丝印；具体 PCB 修订号尚未辨明。

## 分工

| 资源 | 负责人 | 本轮安排 |
|---|---|---|
| PMOD0 | A | 两路编码器 + 采集扩展预留；不接麦克风阵列 |
| PMOD1 的 M14/M15/J14/J16 | B | IBT-2 四路数字控制；不接 DVP 摄像头 |
| PMOD1 的 T11/P11/T12/R11 | 暂不使用 | 与板载 USB PHY 数据线共用，不能视为空闲 GPIO |
| PMOD2、PMOD3、RGB LCD 接口 | C | 显示扩展资源预留，不另分给 A/B；不是已完成的屏幕接线方案 |
| 板载时钟、按键、LED、JTAG/UART | C 统一管理 | A/B 通过顶层共享 clk/rst，不各自占不同系统时钟 |

C 的控制算法通过 FPGA 内部信号连接 A/B，不需要额外外部排针。
本轮不接摄像头、麦克风阵列和 USB-OTG 外设。屏幕也须等 C 完成引脚复用检查后再接。

## A：传感器（FPGA 侧资源确定，传感器侧接法待型号确认）

| RTL/预留信号 | FPGA 球号（背面丝印） | 接口 | 方向（相对 FPGA） | 用途 |
|---|---|---|---|---|
| position_enc_a | T6 | PMOD0 | 输入 | 旋转臂 A 相 |
| position_enc_b | P6 | PMOD0 | 输入 | 旋转臂 B 相 |
| angle_enc_a | T7 | PMOD0 | 输入 | 摆杆 A 相 |
| angle_enc_b | R8 | PMOD0 | 输入 | 摆杆 B 相 |
| sensor_aux0 / 候选 i2c_scl | T8 | PMOD0 | 待接口确定 | 采集扩展 |
| sensor_aux1 / 候选 i2c_sda | P8 | PMOD0 | 待接口确定 | 采集扩展 |
| 暂不使用 | P9 | PMOD0 | — | 留空；麦克风 LED 时钟共用线 |
| 板载 RGB LED 预留 | T9 | PMOD0 | — | 不作为传感器输入，已连接 WS2812 数据输入 |

这四个输入与已提交 sensor_state 端口名一致。若实物不是 A/B 编码器，
由 A 在 PMOD0 范围内重新定义接口，并更新此表，不擅自占用 B/C 的资源。
候选 I²C 尚未实现；需开漏驱动和已确认的上拉电压，不得直接接带 5 V 上拉的模块。

## B：电机驱动数字控制

| 建议顶层信号名 | FPGA 球号 | 接口 | 方向 | IBT-2 模块信号 |
|---|---|---|---|---|
| motor_rpwm | M14 | PMOD1 | 输出 | RPWM |
| motor_lpwm | M15 | PMOD1 | 输出 | LPWM |
| motor_r_en | J14 | PMOD1 | 输出 | R_EN |
| motor_l_en | J16 | PMOD1 | 输出 | L_EN |

这些是新增的建议顶层名字，未修改 B 的 RTL。B 可在集成时做命名映射。
该四脚同时连接 DVP 数据位，故摄像头接口必须空置，不导入摄像头示例约束。
驱动侧输入电平与电平转换模块的供电、DIR/OE 仍按实际 IBT-2 电路核对。
R_IS/L_IS 不在本轮数字输出分配内，不能直接接 FPGA 数字脚作为模拟采集。
电机电源、逻辑电源、急停回路均不由这个引脚表定义。
复位/未使能时 PWM 与使能的安全状态、配置期间硬件下拉由 B 的驱动方案落实。

## C：预留与复用约束

PMOD2 的 D14/E15/B14/A15/B13/A14/B12/C12 和
PMOD3 的 A11/B11/N6/D11/N9/N7/L9/N8 全部留给 C。
不能同时把这些位置作为独立扩展 IO 又驱动 RGB LCD。
其中 D14/E15 还与 USB PHY 的 DATA0/1 共用；N6 与以太网 PHY CRS 共用。
因此“留给 C”不是保证能直接启用全部显示信号，C 必须先处理板载 PHY 的禁用/复位、
确认相关输出不会争用，再编制完整 LCD 约束。不要只靠 USB/网线未插入判断无争用。

板载 UART 官方示例使用 FPGA TX=M11、RX=T13；仅供 C 调试资源预留。
官方 UART 示例 clk=H11；时钟频率/时序约束仍由对应核心板资料和实际工程确定。
本轮不直接复制示例 rst_n=T5：该脚与拨码开关相关，不能当作任意复位按键。
板卡原理图中 T10 对应 SiliconeKey_1，可由 C 核实按键电路后实现统一复位。

## 如何找到实物接点

球号 T6、M14 等是 FPGA 封装管脚标识，不是 PMOD 连接器的数字脚序。
按用户上传背面照片的文字朝向，PMOD0 信号区从上到下的左右两列为：

```text
T6   P6     旋转臂 A / B
T7   R8     摆杆 A / B
T8   P8     采集扩展
P9   T9     本轮不接传感器
GND  GND
3V3  3V3
```

PMOD1 为：

```text
T11  P11    暂不使用（USB PHY 共用）
T12  R11    暂不使用（USB PHY 共用）
M14  M15    RPWM / LPWM
J14  J16    R_EN / L_EN
GND  GND
3V3  3V3
```

翻到正面时左右会镜像，不能把这张背面图直接当正面插线方向。
先按丝印定位并核对连通性；PMOD 上的 3V3 标记不是传感器供电规格。
本轮只确定信号资源归属，传感器/驱动的供电与电平适配未确认前不按此表直接上电。

## 核对依据与范围

1. 用户提供的 Dock 正反面照片：核对上述 PMOD 球号标注。
2. [Sipeed 官方原理图目录及版本说明](https://dl.sipeed.com/shareURL/TANG/Primer_20K/02_Schematic)。
   目录说明 3710/3711/3712/3713 参考 3713 图，另有 3714 与旧版本；
   不能据此认定用户实物就是 3713。
3. 官方 Tang_Primer_20K_Dock-3713_Schematics.pdf 首页网络总览：
   MIC、DVP、USB PHY、LCD 和以太网共用网络核对。本表仍需在接线时匹配实物版本。
4. [官方摄像头约束](https://github.com/sipeed/TangPrimer-20K-example/blob/e469df4c0c9c41824f405a8515decf24ef1e8e6f/Cam2lcd/OV5640_LCD480_FIFO/src/top.cst)、
   [WS2812 约束](https://github.com/sipeed/TangPrimer-20K-example/blob/e469df4c0c9c41824f405a8515decf24ef1e8e6f/WS2812/src/top.cst)、
   [UART 约束](https://github.com/sipeed/TangPrimer-20K-example/blob/e469df4c0c9c41824f405a8515decf24ef1e8e6f/UART/UART_HELLO/src/uart_Hello.cst)。

本轮提交文档，不生成可直接烧录的全系统 CST，不修改 B/C 分支或 top。
后续约束必须与顶层端口、电压、板卡版本匹配；所有资源变更统一更新本文件。
