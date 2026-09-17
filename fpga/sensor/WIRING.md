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
| ADS1115 电源 | 测量确认后接 3.3 V |

先不要连接 J14/J16。ADS 模块只接 3.3 V/GND 时，用万用表确认 VDD、SDA 和 SCL；
SDA/SCL 空闲电压必须约为 3.3 V。模块丝印 `+5V` 不能证明 I2C 电平安全。

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
