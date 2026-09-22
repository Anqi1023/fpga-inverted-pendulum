# Block A：旋转倒立摆四状态采集实机验证版

这是 2026-09-22 在 Tang Primer 20K + Dock 实物上完成联合验证的稳定检查点。
工程读取旋转臂霍尔编码器和 WDD35D4/ADS1115，输出后续 LQR 所需的四个状态量：

- `theta`：旋转臂角度，mrad
- `theta_dot`：旋转臂角速度，mrad/s
- `alpha`：摆杆角度，mrad
- `alpha_dot`：摆杆角速度，mrad/s

UART 使用 115200、8N1，每 0.5 s 输出一行 32 位十六进制数据：

```text
T=XXXXXXXX TD=XXXXXXXX A=XXXXXXXX AD=XXXXXXXX
```

负数使用 32 位二进制补码表示。

## 已验证行为

- 旋转臂 CCW：`theta` 增大，`theta_dot` 为正。
- 旋转臂 CW：`theta` 减小，`theta_dot` 为负。
- 旋转臂静止：`theta` 保持，`theta_dot` 回到 0。
- 单独拨动摆杆：`alpha` 明显变化，`alpha_dot` 在运动时变化、停止后回到 0 附近。
- `alpha_dot` 已验证跨越正负 π 边界时的回绕修正。
- 四状态 UART 联合输出已通过实物观察。

详细记录见 [TEST_RECORD.md](TEST_RECORD.md)。

## 实际管脚

| 信号 | FPGA 管脚 |
|---|---|
| 27 MHz 时钟 | H11 |
| 编码器 A/B | P9 / P8 |
| 编码器指示 LED | N16 |
| UART TX | M11 |
| ADS1115 SDA/SCL | R8 / T7 |

完整约束位于 `src/inverted_pendulum.cst`。

## 构建

器件为 `GW2A-LV18PG256C8/I7`，Gowin 工程为 `inverted_pendulum.gprj`。在本目录运行：

```text
gw_sh build.tcl
```

2026-09-22 的原始综合和布局布线记录保存在 `evidence/`。最终实机使用的位流为
`output/inverted_pendulum.fs`，SHA-256：

```text
9313FDD749AC42D969B443E7D363B96C37E46F1B4DA08017E00C9A6D3251649A
```

`src/adc_uart.v`、`alpha_uart.v`、`count_uart.v` 和 `encoder_uart.v` 是分阶段调试模块；
最终顶层 `top.v` 使用 `state_uart.v` 统一输出四个状态量。文件名 `theta_calc.v.v` 沿用
实机验证工程，未在此稳定节点重命名。
