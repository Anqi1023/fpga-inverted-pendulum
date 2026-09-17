# A/B/C 最终接口分配（旋转倒立摆）

更新：2026-09-17。目标板为 Tang Primer 20K + Dock（GW2A-LV18PG256C8/I7）。

| 组别 | Dock 资源 | FPGA 管脚 | 信号 |
|---|---|---|---|
| B 电机 | PMOD0 | T6、P6、T7、R8 | RPWM、LPWM、R_EN、L_EN |
| A 传感器 | PMOD1 | M14、M15 | 编码器 A、B（必须先降到 3.3 V） |
| A 传感器 | PMOD1 | J14、J16 | ADS1115 SCL、SDA（开漏） |
| A 调试 | 板载 USB-UART | M11 | UART TX，115200 8N1 |
| A 复位 | 板载 1.5 V 按键 | C7 | 低有效复位 |
| 公共 | 板载晶振 | H11 | 27 MHz |
| C 控制/显示 | PMOD2、PMOD3、LCD | 由 C 组最终约束 | 控制和显示预留 |

PMOD1 背面丝印排列：

```text
T11  P11    保留（与 USB PHY 共用）
T12  R11    保留（与 USB PHY 共用）
M14  M15    encoder_a / encoder_b
J14  J16    ads_scl / ads_sda
GND  GND
3V3  3V3
```

PMOD1 同时复用 DVP 摄像头网络，本方案使用时不能连接摄像头。PMOD0 已由 B 组占用，A
组旧版 T6/P6/T8/P8 分配全部作废。M11 是官方板载串口 TX 网络。

T10 是 FPGA 专用配置/复位脚。Gowin 布局布线会拒绝把它作为普通用户复位输入；A 工程
因此使用 C7。Dock 3713 原理图显示 C7 所在按键网络由 1.5 V 上拉、按下接地，故约束为
`LVCMOS15`、低有效。若实物 Dock 不是 3713，应先测 C7 按键电压再下载。

约束文件：`fpga/sensor/constraints/sensor.cst`。该约束已在 GW2A-18C 上完成布局布线和
比特流生成，并避开 READY、DONE、JTAG 与配置专用脚。
