# A 组旋转倒立摆状态采集工程

目标器件：Tang Primer 20K + Dock，GW2A-LV18PG256C8/I7，27 MHz。

工程实现：MG513 霍尔 A/B 四倍频、ADS1115 AIN0 读取、16 点平均、原厂 WDD35D4
标定关系、20 ms 状态更新、Q16.16 rad/rad/s 输出和 UART 调试包。原厂 STM32 源码明确
使用 1040 count/rev、20 ms 和 `angle=(3100-adc12)*0.0879°`；FPGA 版本把 12 位
3.3 V ADC 标定换算为 ADS1115 ±4.096 V 档的等效码值。零点与比例仍需实物复标。

引脚：M14/M15=编码器 A/B，J14/J16=ADS1115 SCL/SDA，M11=UART TX，C7=低有效复位。
B 组已占用 PMOD0，因此 A 组不得继续使用旧的 T6/P6/T8/P8 分配。

打开 `sensor_acquisition.gprj` 可综合。命令行在本目录运行：

```text
gw_sh build_synth.tcl
```

执行完整构建和生成比特流用 `gw_sh build.tcl`；已生成的下载文件位于
`output/sensor_acquisition.fs`。实际接线及上电前测量见 `WIRING.md`。

仿真从仓库根目录运行 `sim/run_iverilog.ps1`，包括正交解码、换算、ADS1115 ACK/读取、
NACK 故障和完整状态路径。UART 为 115200、8N1；安装 pyserial 后运行
`python uart_monitor.py COMx`。
