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


## 原始数据调试（2026-09-18）

默认正常模式保持A5 5A、20字节协议不变。`gw_sh build_raw.tcl`选择sensor_raw_debug_top，输出A5 5B调试帧。两种构建写入同一个impl目录，最后构建的模式覆盖前者；下载前务必检查顶层和构建日志。本次没有执行下载。

调试帧20字节：0–1=A5 5B；2–5=有符号32位编码器计数；6–7=有符号16位ADC末次样本；8–9=有符号16位ADC平均值；10–17保留0；18=valid；19=从FF开始异或前19字节。多字节均小端。

三个值在状态计算更新时锁存，ADC末次样本属于刚完成的16样本窗口，平均值为该窗口算术右移4位的结果。不是每个ADC样本都输出；采样停止时不会持续生成调试帧。没有收到帧不能解释为原始值为0。

同一`python uart_monitor.py COMx`自动识别两种帧。live串口需安装pyserial；离线解析测试无需该依赖。调试示例：`encoder=-12345 count adc=-1234 raw average=19985 raw valid=1`。

当前验证与原始报告见[测试记录](../../docs/sensor-test-results.md)。2026-09-18 已按最新代码
重新完成正常模式构建，`output/sensor_acquisition.fs` 是可直接下载的正常状态输出版本；
2026-09-20 已在 Tang Primer 20K + Dock 实机完成 SRAM 下载，最终状态为
`0x00002020`（DONE=1）。本板应选择 `USB Debugger A`、Location `609`，JTAG 频率使用
`1 MHz`；默认 `2.5 MHz` 会传输到 100% 但 DONE 不置位。当前文件 SHA-256 为
`1ED343867287610327AF0F2F19261C833CAFC3794F53E0F810207A8D1C1C3CE3`。

详细操作和成功判据见 [PROGRAMMING.md](PROGRAMMING.md)。SRAM 下载断电后会丢失；需要
上电自动运行时，再按板卡官方流程写入外部 Flash。
