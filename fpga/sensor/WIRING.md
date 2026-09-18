# A 组实际接线

## 1. WDD35D4 与 ADS1115

| 端点 | 接到 |
|---|---|
| WDD35D4 绿色标记端（滑动端） | ADS1115 AIN0 |
| WDD35D4 中间端 | 3.3 V |
| WDD35D4 另一端 | GND |
| ADS1115 ADDR | GND（地址 0x48） |
| ADS1115 A/R | 不接 |
| ADS1115 SCL | Dock PMOD1 J14 |
| ADS1115 SDA | Dock PMOD1 J16 |
| ADS1115 GND | Dock GND，与传感器共地 |
| ADS1115 模块 `+5V` 输入 | 5 V（实测工作电流约 3 mA） |

2026-09-18 实测：模块输入 5.00 V 时，SDA/SCL 空闲均为 3.296 V，工作电流约
3 mA，因此两根 I2C 线的高电平对 FPGA 3.3 V IO 安全，可以接 J14/J16。断电测得
SDA→`+5V` 和 SCL→`+5V` 均为 73.64 kΩ；该值包含模块内部供电路径，不作为实际
上拉电阻值。连接时 ADS GND 必须先与 Dock GND 共地。

首次连接后以 FPGA 是否收到地址 `0x48` 的 ACK 为功能验收：UART 中
`sensor_valid` 在两次状态更新后应变为 1。若一直为 0，再测芯片实际 VDD 或板上
3.3 V 稳压节点，无需继续从 `+5V` 输入端推算上拉阻值。

## 2. 霍尔编码器

编码器端子：1 电机负、2 编码器 5 V、3 A、4 B、5 编码器地、6 电机正。
1/6 由 B 组接电机驱动。2/3/4/5 属于 A 组。

每一路用两个电阻降压：

```text
编码器 A(5 V逻辑) -- 12 kΩ --+-- M14
                              |
                            22 kΩ
                              |
                             GND

编码器 B(5 V逻辑) -- 12 kΩ --+-- M15
                              |
                            22 kΩ
                              |
                             GND
```

FPGA 接入前，分别转动电机并测两个分压节点：低电平接近 0 V，高电平应为 3.0-3.3 V；
超出范围先停下检查。编码器 GND、ADS GND、Dock GND 必须共地。

## 3. 下载和串口

下载文件：`output/sensor_acquisition.fs`。USB-JTAG 连接电脑后，UART TX 走板载 M11；
找到对应 COM 口，运行 `python uart_monitor.py COMx`。按下 C7 对应的板载按键复位。
