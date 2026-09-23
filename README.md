# fpga-inverted-pendulum
FPGA-based inverted pendulum control system for competition

## 项目资料

- [衡芯项目硬件信息汇总](docs/hardware-summary.md)：主控、驱动、采集、电源、安全与扩展模块的型号、接口及待确认项。

## Stage 1 - Sensor Chain

第一阶段的正式 Gowin 工程位于
[`gowin_project/inverted_pendulum`](gowin_project/inverted_pendulum)，目标器件为
Tang Primer 20K（GW2A-LV18PG256C8/I7）。当前顶层通过 UART 输出四个有符号
32 位状态量，角度单位为 mrad，角速度单位为 mrad/s：

- `theta`：旋转臂角度；`theta = 0` 为旋转臂启动参考位置。
- `theta_dot`：旋转臂角速度。
- `alpha`：摆杆角度；`alpha = 0` 为摆杆竖直向上的倒立平衡位置。
- `alpha_dot`：摆杆角速度。

硬件信号链与通信参数：

- Motor Hall encoder → CJMCU level shifter → FPGA `P9/P8`。
- WDD35D4 → ADS1115 → FPGA `R8/T7`。
- ADS1115 I2C 地址：`0x48`。
- FPGA 时钟：27 MHz（`H11`）。
- UART：115200 baud，8N1（TX：`M11`）。
- 板载验证 LED：`N16`。
- 当前编码器标定：约 978 counts/revolution。

`calibration.v` 会在 FPGA reset/programming 后约 0.5 秒捕获 `theta` 与 `alpha`
零点。测试时应先把旋转臂放在参考位置，并让摆杆保持竖直向上至少约 1 秒。

已完成的实物验证：

- `theta` 随旋转臂转动连续变化。
- `theta_dot` 随运动产生正负值，停止后回到接近 0。
- `alpha` 随摆杆运动变化并可跨越正负零点。
- `alpha_dot` 随摆杆运动方向改变正负，停止后接近 0。
- 启动标定后 `theta ≈ 0`、`alpha ≈ 0`。
- 第一阶段 Sensor Chain 已完成硬件验证。

LQR、电机 PWM 和 IBT-2 控制属于下一阶段，尚未加入本次硬件验证工程。
