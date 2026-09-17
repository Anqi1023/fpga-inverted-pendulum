# 衡芯：主板与老师工程检查记录

检查日期：2026-09-17。依据：用户上传的两张照片、demo.gprj，以及下列 Sipeed 官方资料。后续已核对项目仓库 docs/hardware-summary.md，与本次板型和器件选择一致。

## 主板识别与证据边界

照片正面可读到 SIPEED TANG PRIMER 20K，背面可读到 TANG PRIMER 20K Dock；核心板为插入式模块。这是 Primer 20K + Dock，不能与 Nano 20K 或 Primer Lite 混用。

| 项目 | 结果 | 证据与限制 |
| --- | --- | --- |
| FPGA 完整订货型号 | GW2A-LV18PG256C8/I7 | 官方产品资料；照片芯片小字不足以独立逐字核准全部后缀 |
| Gowin Device | GW2A-18C | 官方点灯教程；本机工具也提供该配置 |
| 时钟 | 27 MHz，H11 | 官方 Primer 20K 点灯教程；不是从照片上猜晶振频率 |
| 本工程目标 LED | 底板数字标号 4，L14，低电平亮 | 照片背面 LED 表 + 官方亮灯教程交叉确认 |
| LED 0～5 | C13、A13、N16、N14、L14、L16 | 照片丝印；本工程只使用 LED4 |
| 五个用户按键 | S0→T10、S1→T3、S2→T2、S3→D7、S4→C7 | 正反面位置对应和背面丝印；按键有效电平、上拉及防抖尚未核验，不接入本工程 |
| 拨码区可读标记 | B10、E9、E8、T4、T5 | 只记录丝印；不据此推断每一开关的功能或有效电平 |
| RGB LED 区标记 | T9 | 只记录丝印，不当作普通单色 LED 控制 |

背面 PMOD 信号丝印抄录如下，每一对表示照片表格同一行两列，不代表已完成连接器编号和电气核验：

| 接口 | 四行信号 |
| --- | --- |
| PMOD0 | T6/P6，T7/R8，T8/P8，P9/T9 |
| PMOD1 | T11/P11，T12/R11，M14/M15，J14/J16 |
| PMOD2 | D14/E15，B14/A15，B13/A14，B12/C12 |
| PMOD3 | A11/B11，N6/D11，N9/N7，L9/N8 |

这些接口可能与底板外设复用。连接编码器或电机驱动前仍须核对原理图、连接器方向、Bank 电压和电平兼容性；照片上的 3V3 电源标记不能证明所有信号 Bank 都是 3.3 V。本基线不使用这些接口。

## demo.gprj 实际内容

实际附件是一个文本工程索引，不是含源码的压缩包。文件头为 gowin-fpga-project，Template=FPGA，Version=5（工程格式版本，不是 IDE 版本）。

- Device name：GW1N-9C。
- pn：GW1N-LV9EQ144C6/I5。
- 内部器件标识：gw1n9c-022。
- 未显式记录顶层模块、时钟频率、引脚位置、时序约束或综合选项。
- 20 个 Verilog 引用均 enable=1；8 个 CST 引用只有 freq_meter_demo.cst 为 enable=1。
- 因此不能把 running_led_demo.v 的文件名当作实际顶层，也不能认定老师工程当前选择的是点灯例程。
- 当前附件目录没有 src/，下面全部 28 个引用文件缺失；无法检查其实现、引脚与顶层依赖。
- XML 声明写成 version="1"，严格 XML 解析器会拒绝；本检查直接读取文本，未修改老师文件。

Verilog 引用（均位于 src/）：

```text
basic_io_demo.v  counter.v  dec_counter.v  edge_detect.v
freq_div.v  freq_meter_demo.v  key_db.v  key_db_demo.v
mux_4x4.v  pwm_ctrl.v  pwm_demo.v  running_led_demo.v
seg_ctrl.v  seg_decode.v  seg_decode_demo.v  seg_scan_demo.v
shift_reg.v  uart_demo.v  uart_rx.v  uart_tx.v
```

CST 引用（均位于 src/）：

```text
basic_io_demo.cst       disabled
freq_meter_demo.cst     enabled
key_db_demo.cst         disabled
pwm_demo.cst            disabled
running_led_demo.cst    disabled
seg_decode_demo.cst     disabled
seg_scan_demo.cst       disabled
uart_demo.cst           disabled
```

结论：老师工程的器件与本板不同，不能沿用器件或约束。本项目使用 Gowin，不应按之前对话中的 Quartus / QSF 路线建立工程。昨天的不规则亮灯不能证明这份 gprj 就是当时烧录文件的来源，也不能证明整板所有功能正常。

## 尚缺的信息

不妨碍本次最小 LED 工程：老师完整 src/、主板具体 PCB 版本、按键电气细节、系统外围接线。仓库硬件汇总已另行记录完整芯片型号的实物核对结果。

后续验收结果：2026-09-17 用户下载后确认看到了预期闪烁效果，点灯基线实板验收通过。未提供扫描截图、仪器测量或完整 10 周期记录；不将用户目视反馈扩展为其他外设已验证。

## 官方来源

1. 板型和完整器件：[Sipeed 产品页](https://wiki.sipeed.com/hardware/en/tang/tang-primer-20k/primer-20k.html)。
2. 27 MHz、H11、Device 选择与下载流程：[官方闪灯教程](https://wiki.sipeed.com/hardware/en/tang/tang-primer-20k/examples/led.html)。
3. LED4=L14、低有效：[官方亮灯教程](https://wiki.sipeed.com/hardware/zh/tang/tang-primer-20k/examples/assign_led.html)。
4. 其他接口资料入口：[Sipeed 官方例程仓库](https://github.com/sipeed/TangPrimer-20K-example)。这不是用户的 GitHub 仓库。

工程内部器件标识 gw2a18c-011 另与本机 Gowin V1.9.12.03 自带 can 示例的 GW2A-18C / GW2A-LV18PG256C8/I7 配置核对，并已通过实际编译。
