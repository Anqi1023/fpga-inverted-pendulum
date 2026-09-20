# Tang Primer 20K 下载记录

2026-09-20 已在项目实物板完成 `sensor_acquisition.fs` 的 SRAM 下载。

## 已验证配置

- Device：`GW2A-18C`，ID `0x0000081B`
- Cable：`USB Debugger A`
- Location：`609`
- JTAG：`1 MHz`
- Operation：`SRAM Program`
- FS File：`output/sensor_acquisition.fs`
- User Code：`0x00000000`
- 下载后 Status Code：`0x00002020`

`0x00002020` 的 bit 13 为 1，即 DONE=1，表示 SRAM 配置成功。默认的 2.5 MHz 在这块
实物板上虽然进度能到 100%，但只返回 `0x00000020`，DONE 没有置位，程序不会启动。
400 kHz 会由当前 BL702 下载器直接返回错误，因此固定使用 1 MHz。

## Gowin Programmer 图形界面

1. 将 Dock 红色拨码开关 1 拨到启用核心板的位置。
2. 扫描并选择 `USB Debugger A`，确认器件为 `GW2A-18C (0x0000081B)`。
3. 将 Cable Setting 的频率改为 `1 MHz`。
4. Access Mode 选择 `SRAM Mode`，Operation 选择 `SRAM Program`。
5. FS File 选择本目录下的 `output/sensor_acquisition.fs`。
6. 下载结束后读取 Status Register；只有 DONE 位为 1 才算成功，本次实测值为
   `0x00002020`。

## 本次使用的命令

```powershell
& 'C:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe' `
  --cable-index 4 --location 609 --frequency 1MHz `
  --device GW2A-18C --operation_index 2 `
  --fsFile 'C:\Users\hp\Documents\ChatGPT\FPGA\fpga\sensor\output\sensor_acquisition.fs'
```

SRAM 配置断电后会丢失。外部 Flash 写入是另一种操作，不能用 SRAM 成功日志替代。
