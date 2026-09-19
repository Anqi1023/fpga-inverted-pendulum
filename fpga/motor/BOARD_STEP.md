# 第四步：Dock 板级测试工程

2026-09-18。独立工程 `board/dock_motor_test.gprj`，保留原点灯和motor_offline工程。本工程只用于FPGA独立信号/LED测试，驱动板、电平转换器和电机均不连接。初始建档时尚未下载到硬件；后续实板结果见下方记录。

## 已确认的资料与边界

复用用户已有正反面照片，无需重复拍照。照片背面S0–S4对应T10、T3、T2、D7、C7。
从[Sipeed官方目录](https://dl.sipeed.com/shareURL/TANG/Primer_20K/02_Schematic)下载并查看3708、3709、3713、3714原理图的按键页。旧版按键页元件S1–S5对应板面S0–S4，不能把原理图元件编号直接当作板面编号。

- [3713原理图](https://dl.sipeed.com/fileList/TANG/Primer_20K/02_Schematic/Tang_Primer_20K_Dock-3713_Schematics.pdf)：第1页连接映射，第9页按键/LED电路。
- [3708原理图](https://dl.sipeed.com/fileList/TANG/Primer_20K/02_Schematic/old/Tang_Primer_20K_Dock_3708_Schematic.pdf)、[3709原理图](https://dl.sipeed.com/fileList/TANG/Primer_20K/02_Schematic/old/Tang_Primer_20K_Dock_3709_Schematics.pdf)：按键页与上述所用S0–S3电路一致。
- [3714原理图](https://dl.sipeed.com/fileList/TANG/Primer_20K/02_Schematic/Tang_Primer_20K_Dock_3714_Schematics.pdf)：版本记录包含SW0与KEY4管脚交换。本步骤不使用S4或拨码开关；不宣称实物版本号已识别。
- 所用按键均按下接地、低有效，有10kΩ外部上拉和100nF电容。S0上拉3.3V；S1–S3上拉1.5V，因此对应CST使用LVCMOS15，而非LVCMOS33。CST关闭内部上拉下拉。
- LED低有效，沿用板面丝印与官方映射。PMOD电机四路保留用户确认分配。

## 端口表

| 功能 | 板上标记 | FPGA管脚 | 电平标准 |
|---|---|---|---|
| 时钟27MHz | 核心板时钟 | H11 | LVCMOS33 |
| 复位 | S0 | T10 | LVCMOS33 |
| 启动 | S1 | T3 | LVCMOS15 |
| 普通停止 | S2 | T2 | LVCMOS15 |
| 选择方向 | S3 | D7 | LVCMOS15 |
| 运行指示 | LED4 | L14 | LVCMOS33 |
| 反向选择指示 | LED5 | L16 | LVCMOS33 |
| 等待结束/可启动指示 | LED2 | N16 | LVCMOS33 |
| RPWM/LPWM/R_EN/L_EN | PMOD0 | T6/P6/T7/R8 | LVCMOS33 |

T10是SSPI复用管脚。构建脚本设`use_sspi_as_gpio=1`、`use_jtag_as_gpio=0`；依据本机Gowin SUG1220-2.0E第45–46页。首次布局布线因SSPI专用功能报错，加入该选项后通过；不能通过修改成其他未确认管脚绕过错误。按本目录脚本构建，不能假定只打开gprj已加载所有Tcl选项。

## 行为

复位/上电后四路关闭；约10ms释放复位，再等待约1秒进入可启动状态。S1产生新按下事件才启动。S2为带消抖的普通停止，不是急停。S3切换逻辑方向且停止，等待结束后必须重新按S1。S0触发复位后不自动恢复运行。

测试顶层permit固定1，只表示无电机测试时允许普通按键控制；没有外部许可/急停输入，不能作为可直接接电机的最终顶层。OE及外部下拉仍按独立接线计划，配置前安全不由RTL保证。

## 第四步原始验证结果（PR #10）

第五步已进一步修正复位分发、加入同步链约束并重新验证；当前结果见[CDC_RESET_REVIEW.md](CDC_RESET_REVIEW.md)。

- ModelSim：`HENGXIN_DOCK_MOTOR_TEST_PASS`，0错误、0警告。用户于2026-09-18确认已复现通过。
- 覆盖完整复位链、上电按住启动、两方向10% PWM（135/1350拍）、停止、LED指示、运行中异步复位及复位后按住启动。
- Gowin V1.9.12.03：综合、布局布线、位流生成通过；147个寄存器，0锁存器。
- 27MHz约束下已分析路径setup/hold违例数均0；最小setup裕量31.634ns、hold裕量0.425ns。
- SDC目前只定义27MHz时钟。以上是内部受约束路径结果，不代表异步输入、复位恢复/移除、CDC可靠性或外部PWM电气时序已完整验收。尚需同步器布局/约束复核、外部测量和真实按键测试。
- `.fs`仅本地生成，未下载或烧录；不上传impl产物。

## 现在只复现仿真

```powershell
cd D:\FPGA\hengxin\motor-dev\fpga\motor
vsim -c -l board_simulation.log -do run_board.do
```

成功标记`HENGXIN_DOCK_MOTOR_TEST_PASS`。若失败，发终端最后一屏。

构建复现（不执行下载）：

```powershell
cd D:\FPGA\hengxin\motor-dev\fpga\motor\board
gw_sh build.tcl
```

同步器/复位约束复核已完成，记录见CDC_RESET_REVIEW.md。下一步单独安排FPGA按键和LED实板验证；急停和电机联调另行完成。

## 实板验证进展（2026-09-19 更新）

- [2026-09-18 按键与 LED 实板测试](BOARD_TEST_2026-09-18.md)：用户逐项确认，含复位期间持续按启动键的重测记录。
- [2026-09-19 四路输出实测](OUTPUT_TEST_2026-09-19.md)：RPWM/LPWM 分别在对应方向输出 PWM；停止及相反方向为低电平；R_EN/L_EN 在两方向运行时为高，停止和按住 S0 时为低，松开 S0 后不自动使能。附两张 PWM 照片。
- 以上为逐路稳态检查；未验证双通道瞬态重叠、毛刺、保护响应时间或配置期间输出安全。电平转换器、外部急停、驱动和电机联调尚未完成。
- 下一阶段：核对电平转换板焊接、供电、OE 和下拉接线，再验证转换后的信号。

上面的 PR #10 原始验证段落保留其当时结论，不代表最新进展。
