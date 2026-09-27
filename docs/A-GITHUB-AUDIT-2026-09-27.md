# A 传感器资料 GitHub 核对（2026-09-27）

## 核对来源

本次通过 GitHub CLI 实际读取远端分支，并下载独立检查副本。未修改、推送或下载任何 FPGA 程序；未运行硬件测试。本次结论来自源代码、仓库记录和用户补充，不把仓库的实测记录描述为本次亲自复测。

- 仓库：https://github.com/Anqi1023/fpga-inverted-pendulum
- main：623c2886686761f4c267f626b5cc49fb0c6d76f4，2026-09-21；没有下面的新 A 稳定工程。
- feature/sensor：f53d9c8e6c9299a4514080da2c99f52ca0982f45，2026-09-22；实板检查点在 fpga/block_a_verified。
- feature/control：ebf1cc127b0bc761914266abd03cd96a99931c44，2026-09-24；README 指定 A 正式工程为 gowin_project/inverted_pendulum，含 calibration.v。
- A 工程正确入口为 gowin_project/inverted_pendulum/inverted_pendulum.gprj。其上一级 gowin_project/inverted_pendulum.gprj 是另一套工程，引用 C:/Users/21889/... 的绝对路径和 rtl/top.v，不可混用。

已比较两份 A 工程：ads1115_i2c.v、encoder.v、angle_sensor.v、theta_velocity.v、alpha_velocity.v 和引脚约束内容的 SHA256 均一致。带 calibration 的工程使用修改过的 top.v。

## 已由代码确定，不需要再让 A 重答

| 项目 | 结果与依据 |
| --- | --- |
| ADC 配置 | ads1115_i2c.v：0xC283，AIN0 对 GND，连续模式，±4.096 V，128 SPS，比较器禁用；地址 0x48。旧 docs 的 860 SPS 不适用于这份工程。 |
| I2C | DIVIDER=68，27 MHz；常规位传输四相，位时钟约 99.3 kHz，字节间有额外状态间隔。SDA/SCL 都只拉低或高阻释放。 |
| 编码器 | encoder.v 对每个有效 AB 状态转换加减一，为四倍频正交解码；theta_calc 使用 count×6283/978，单位整数 mrad。978 的实物精确标定仍属暂定。 |
| θ 方向 | 9 月 22 日 README、TEST_RECORD 记载 CCW 为正、CW 为负；观察视角没有图示。 |
| α 标定 | angle_sensor.v：竖直 ADC=22005，左水平=15317，右水平=1797；低于 0x2000 时加 26896 展开，再减 22005，乘 1571/6688。注释定义向右为正，观察视角没有图示。 |
| 角速度 | 两路均每 540000 个 27 MHz 时钟更新，即 20 ms；角度差×50，无额外低通/滑动平均。alpha_velocity 对差值跨 π 做 ±6283 修正，alpha_valid_i 当前未参与计算。 |
| 自动零点 | feature/control 的 calibration.v 在复位后 13500000 拍（0.5 s）捕获当前 θ/α 为零偏，输出 calibrated_o。README 要求下载/复位时机构保持参考位置、摆杆竖直向上约 1 s。 |
| 状态类型 | 顶层内部四个量为 signed[31:0]，整数 mrad、mrad/s，无额外 Q 格式缩放。 |
| 引脚 | enc_a=P9，enc_b=P8，SDA=R8，SCL=T7；clk=H11，UART TX=M11，调试 LED=N16，均按 A 的 CST。 |

用户已补充：ADS 模块输入为 +5 V，SDA/SCL 各约 4.69 kΩ 上拉至模块自身约 3.3 V 轨。该信息不应再次询问，也不应被旧文档中到 +5 V 的 73.64 kΩ 等效电阻覆盖。

## 代码中已可判断的集成工作（不是要求 A 再提供参数）

- 四个状态量目前是 A 顶层内部连线，顶层对外提供 UART/LED，还需整理为可供 C 实例化的状态接口。
- 两路速度每 20 ms 更新，θ 随计数变化、α 随 ADC 数据更新，尚无面向控制器的一组同步状态快照和 state_valid。
- 存在内部 ads_ack_error 和 calibrated，但未从 A 顶层输出，也未接到 B 保护逻辑。该 A 版本没有统一的传感器失效输出和数据过期保护。
- ads1115_i2c 在 SAMPLE_WAIT 期间反复置 data_valid=1，并持续约 10 ms，不能直接把它当“每个新样本一个系统时钟宽的脉冲”。ACK 错误置位后仍继续流程，未抑制样本发布。
- calibration 按固定时间捕获，并未等待有效 ADC/无 ACK 错误；闭环集成时需要用有效数据和标定状态限制启动。
- 两份 A 的 N16 是编码器调试灯，原 B 的 ready LED2 也占 N16。用户随后选择调整 B，就绪灯现改为 LED3/N14，A 保留 LED2/N16；新 B 工程 motor_protection_jog_ab_led3 构建/实际引脚核对通过，用户已确认锁定/清除及实际点动“转动后停下，CV”。GPIO 数据通道已按 B=T6/P6/T8/T9、A=P9/P8/R8/T7 分开。
- A 正式 .gprj 没有列入 SDC，集成工程需要保留并完善时序约束。以上是静态检查发现，尚未修改源代码或复跑仿真。

## 仍不能从现有仓库和用户描述确定的实物信息

1. 用户随后提供截图确认 ADS1115 芯片 VDD≈5 V、模块输入 5 V、SDA/SCL 各经约 4.69 kΩ 上拉至约 3.3 V。VDD 已不再缺失。尚需确定模块内部是否有 I2C 电平转换；若 3.3 V 总线直接接到 5 V VDD 的 ADS1115，低于 TI 手册保证的 0.7×VDD=3.5 V 高电平门限，需处理该电平兼容性，不能仅凭能通信认定裕量合格。
2. 此项随后已由用户确认：A/B 各有一块相同的 CJMCU-245 / SN74LVC4245A，不共用；型号、数量和归属不再待确认。A 与 B 的转换方向相反。
3. 978 对应输出轴一整圈的最终实测记录。代码证明四倍频和换算公式，但不能证明实物整圈标定精度。用户已经说明该值暂定，可留到最终标定时确认。
4. CCW/向右的实物观察视角和与电机正转的对应关系，需用一张方向图或后续联合方向测试统一；方向符号本身已经在代码/记录中给出。

完整源代码已经找到，不应再向用户索要同一份工程。只有 A 本地存在未上传的新版本时才需要另行同步。


当前 B 交付入口：[成果汇总](../fpga/motor/PROGRESS_2026-09-27.md)。ADS1115 门限依据：[TI ADS1115 数据手册](https://www.ti.com/lit/ds/symlink/ads1115.pdf)，VIH 最低保证值 0.7×VDD；该值不能证明模块内部有无转换电路。
