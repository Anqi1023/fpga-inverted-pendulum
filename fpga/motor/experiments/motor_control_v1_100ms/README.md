# C→B V1.0 补充版：100 ms 超时、整数百分比

本包替代交接用的 motor_control_v1_20ms。C 的正常更新周期仍为 **20 ms**；100 ms 是丢失更新后的停止期限。旧包及其已实测位流保留，文件名不同，避免混用。

## 使用哪个文件

- **交给 C 集成**：rtl/motor_control_v1.v，以及 rtl/pwm_tenths.v、rtl/protection.v、rtl/ibt2_adapter.v。motor_control_v1 直接接收三个 C 接口，不需要再套旧 motor_protected 或重复除以 10。
- **高云独立上板测试工程**：motor_control_v1_100ms.gprj，顶层 dock_motor_control_v1。模拟 C 每 20 ms 发 +100，每次 S1 启动仅允许实际 EN 输出 200 ms。
- **下载文件**：impl/pnr/motor_control_v1_100ms.fs，发布包根目录另附同内容副本。
- **仅核心综合**：motor_control_v1_core.gprj 没有物理引脚分配，不用作板级下载工程。

板级程序没有接入 A 的传感器或 C 的真实控制器；核心没有 200 ms 自动停止限制。

## C 接口与占空比

输入均与同一个 27 MHz 时钟同步。

| 端口 | 约定 |
|---|---|
| control_u | signed [15:0]；单位 0.1 个占空比百分点；范围 -300～+300 |
| control_valid | 正常每 540000 拍（20 ms）拉高 1 拍，该上升时钟沿锁存 control_u |
| motor_enable | 0 时立即禁止四路驱动，并将有效 command/duty 清零；1 时等待合法新命令 |

PWM=1 kHz。范围检查先于绝对值计算，绝对值先扩展到 17 位再除以 10，整数截断；-32768、±301 等越界命令触发锁定，不做溢出或饱和输出。

| control_u | 有效 command | duty_pct | PWM 高电平拍数/27000 |
|---|---|---|---|
| +100 | 01 | 10 | 2700 |
| -200 | 10 | 20 | 5400 |
| +85 / -85 | 01 / 10 | 8 | 2160 |
| +101 / -101 | 01 / 10 | 10 | 2700 |
| +299 / -299 | 01 / 10 | 29 | 7830 |
| +300 / -300 | 01 / 10 | 30 | 8100 |
| -9～+9（含 0） | 00 | 0 | 0；四路驱动关闭 |

绝对值小于 10 时按停止处理，是 B 对截断为零的命令的补充规则；不执行最小占空比补偿。机械正方向仍由 A+B+C 联调确认。

为保持现有模块连接，PWM 子模块和 duty_tenths_o 状态口继续使用“十分之一百分点”计数，但本版只提供 10 的整数倍。例如 +85 的 duty_pct=8、duty_tenths_o=80，实际输出 8%，不是 8.5%。保护或禁用时此状态口立即为 0。有效 command/duty 是核心内部信号，不新增外部 GPIO。

## 超时及保护规则

- 默认 COMMAND_TIMEOUT_CYCLES=2700000，即 **100 ms**。motor_enable=1 且保护已解除时，从使能或最近一次接受的合法命令起计时；第一条命令缺失也检查超时。
- 0 命令和截断为 0 的合法命令仍刷新计时。motor_enable=0 时不累计超时，清掉旧命令；重新使能需要新的 control_valid。
- 期限同拍收到合法命令，优先刷新。超时已发生后，后来的命令不能自动恢复驱动。
- **保留 B 已有的超时锁定策略**：100 ms 超时立即 command=00、duty_pct=0，并将 RPWM/LPWM/R_EN/L_EN 全部撤销；锁存原因，须人工清除。这一恢复规则属于 B 的补充，交接 C 时一并说明。
- fault、estop、protection_locked 优先于 C 命令。上电/复位锁定；控制范围错误或 valid 连续两拍为高也锁定。
- 清除条件：motor_enable=0、control_valid=0、故障解除，再给 clear_fault 一个新的上升沿。清除只解锁，不执行旧命令。正常禁用不锁定。
- fault/estop/clear_fault 为同步核心输入，外部按键需要同步和相应消抖。逻辑保护不是独立硬件断电急停，也不监测 FPGA 时钟失效。

## 尚未改动的联调边界

**STOP_CYCLES=27000000，即 1 秒停止/换向等待仍保留。** 核心在等待期间仍接收更新并检查超时，等待结束后可按最新命令运行。高速连续换向尚未实测，进入倒立平衡闭环前需与 C 确定并验证新的策略。相同方向的幅度更新在 PWM 周期边界生效，停机不等 PWM 边界。

其余核心输入：clk、rst_n、fault、estop、clear_fault。状态输出：motor_dir、ready、waiting、protection_locked、fault_latched、estop_latched、clear_accepted、timeout_latched、command_error_latched、duty_tenths_o。ready 表示解锁且未运行、等待已结束；不能简单用它门控 C 运行中的 motor_enable。

## 接线、按键不变

| FPGA | 电平转换器 B 侧 | IBT-2 |
|---|---|---|
| T6 | B1 → A1 | RPWM |
| P6 | B2 → A2 | LPWM |
| T8 | B3 → A3 | R_EN |
| T9 | B4 → A4 | L_EN |

LED3=N14 就绪；LED4=L14 使能；LED5=L16 锁定。A 保留 P9/P8、R8/T7、N16；时钟 H11 共用。FPGA 3V3→VCB、GND 共地；转换器 VCA 保留 5 V，既有 DIR/OE 设置不变。按键和 LED 使用板上现有约束。

S0 复位锁定；S1 锁定时只清除，再次新按下才启动 10% 正向点动；S2 急停、S3 故障注入。200 ms 点动后等待约 1 秒，长按 S1 不重复启动。

## 构建与验证

推荐构建位置：D:\FPGA\hengxin\motor-control-v1-100ms。

- `gw_sh build_core.tcl`：通用有符号命令核心综合。
- `gw_sh build.tcl`：板级位流，显式设置 use_sspi_as_gpio=1、use_jtag_as_gpio=0。
- `run_tests.ps1 -Simulator <vsim.exe>`：4 项 ModelSim 回归，含整数截断与真实 100 ms 超时。
- 首次下载前关闭台式电源 CH1 电机输出，保留 FPGA USB 和逻辑 5 V；SRAM Program 选择新的 motor_control_v1_100ms.fs。等待两秒应仅 LED5 亮，第一次 S1 只解锁到 LED3。

构建结果及本版用户“符合预期，CV”的点动复测见 TEST_RESULTS.md；旧版保护/长按实测记录见 BENCH_BASELINE.md。本版的 100 ms 超时和非整数输入截断仍以仿真为证据；固定 +100 点动本身不能证明这两项。
