# A 组旋转倒立摆采集与实机验收

本工程采集两种传感器：电机输出轴霍尔 A/B 编码器用于旋转臂角 `theta`，WDD35D4
经 ADS1115 用于摆杆角 `alpha`。四个状态均为 signed 32-bit Q16.16，单位 rad/rad/s，
50 Hz 更新；`sensor_valid` 表示 ADC、编码器和换算链路均已建立且没有锁存故障。

## 软件组成

| 文件 | 作用 |
|---|---|
| `rtl/sensor/i2c_byte_master.v` | 100 kHz 开漏 I2C 字节操作 |
| `rtl/sensor/ads1115_reader.v` | 配置并读取 ADS1115 AIN0 |
| `rtl/sensor/quadrature_decoder.v` | 编码器同步、四倍频和非法跳变检测 |
| `rtl/sensor/sensor_math.v` | 1040 count/rev 与厂家 WDD35D4 标定换算 |
| `rtl/sensor/sensor_state.v` | 16 点平均、20 ms 更新、故障锁存 |
| `rtl/sensor/sensor_uart_packet.v` | 20 字节 UART 调试帧 |
| `fpga/sensor/uart_monitor.py` | 电脑端显示 rad/rad/s |

## 仿真与构建

仓库根目录运行：

```powershell
./sim/run_iverilog.ps1
```

ModelSim GUI 可运行 `do C:/Users/hp/Documents/ChatGPT/FPGA/sim/run_sensor.do`。测试覆盖
正交解码、速度、物理量换算、ADS1115 正常应答/采样、NACK 故障和完整状态路径。

在 `fpga/sensor` 运行 `gw_sh build.tcl` 会执行综合、布局布线、时序和比特流生成。

## UART 帧

每帧 20 字节：`A5 5A`、theta、theta_dot、alpha、alpha_dot、valid、异或校验。
四个数均按 little-endian signed int32 发送。串口为 115200、8N1。

## 实机顺序

1. 按 `fpga/sensor/WIRING.md` 先做断电电阻和上电电压测量。
2. 电平合格后接 PMOD1，下载 `fpga/sensor/output/sensor_acquisition.fs`。
3. 启动 `python fpga/sensor/uart_monitor.py COMx`。
4. 静止两次更新后 valid 应变为 1；转旋转臂观察 theta，转摆杆观察 alpha。
5. 编码器整圈应约 2π rad。方向相反时交换 A/B，不改比例。
6. 记录摆杆竖直向上的 alpha 及两个已知角度，用于最后修正零点和比例。

I2C NACK 或编码器非法跳变会让 valid 锁存为 0，排除接线问题后按复位键恢复。这种
锁存避免控制器在已经可能丢计数时自动恢复为“有效”。
