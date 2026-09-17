# 衡芯——Tang Primer 20K Dock 点灯基线

状态：源码、物理约束、时序约束和工程已建立；Gowin V1.9.12.03 综合、布局布线、位流生成通过。**2026-09-17 用户确认看到了预期 LED 闪烁效果，点灯基线实板验收通过。**该结果为用户观察反馈，未提供仪器测量记录。

目标：底板数字标号 4 的普通 LED（L14）灭 1 秒、亮 1 秒循环，完整周期 2 秒（0.5 Hz）。使用 27 MHz 板载时钟 H11，低电平亮。初始化时熄灭；计数器从 0 数到 26,999,999 后清零并翻转灯状态。参数 HALF_PERIOD_CYCLES 必须大于等于 1。

## 文件

- `rtl/led_blink.v`：唯一顶层 led_blink，配置时初始化，无外部复位或按键依赖。
- `constraints/tang_primer_20k_dock.cst`：仅 clk→H11、led_n→L14，LVCMOS33。
- `constraints/timing.sdc`：27 MHz 时钟；LED 输出不作为外部同步接口计时。
- `led_blink.gprj`：GW2A-18C / GW2A-LV18PG256C8/I7。
- `build.tcl`：可重复编译入口。
- `docs/hardware_review.md`：照片和老师工程检查、证据来源、缺失信息。

## 编译

1. 用 Gowin IDE 打开 `led_blink.gprj`，确认器件为 GW2A-18C，完整型号 GW2A-LV18PG256C8/I7，顶层 led_blink。
2. 确认一个 RTL、一个 CST、一个 SDC 都已启用；不添加老师工程文件。
3. 运行 Synthesize，再运行 Place & Route；检查报告无错误且时序通过。
4. 输出位流为 `impl/pnr/led_blink.fs`。

也可在本目录用已加入 PATH 的 Gowin 命令行执行：

```powershell
gw_sh build.tcl
```

如果工程移到别处，整个 fpga 文件夹一起复制；推荐使用无中文路径。

## 首次下载与验收

1. 本次仅连接开发板，暂不接倒立摆电机驱动等外围。
2. 使用底板标注 USB-JTAG 的接口连接电脑，确认核心板供电/使能；拨码 1 的使能位置参照下方官方教程图片。
3. 打开 Gowin Programmer，Scan Device。核对扫描结果与工程器件系列匹配；若扫描到其他器件，先记录结果并复核，不强行套用。
4. 选择 SRAM Mode / SRAM Program，载入本次编译的 `impl/pnr/led_blink.fs`，执行下载并确认成功。
5. 观察底板数字标号 4 的 LED：持续至少 20 秒，应完成约 10 个完整周期，每次亮/灭各约 1 秒。其他灯不是此工程的验收对象。
6. 记录 IDE 版本、扫描器件、下载结果、实际周期。SRAM 下载断电不保留，重新上电可能运行原 Flash 程序；需要持久化时，在本轮验收通过后再按官方流程写外部 Flash。

下载参考：[Sipeed 官方点灯流程](https://wiki.sipeed.com/hardware/en/tang/tang-primer-20k/examples/led.html)。

## 本次验证记录

2026-09-17，Windows，Gowin V1.9.12.03：

- 综合、布局布线、位流生成成功。
- 报告器件 GW2A-LV18PG256C8/I7 / GW2A-18，Device Version C。
- 引脚报告确认 clk=H11、led_n=L14，两端口均已约束。
- 27 MHz 时钟约束被读取；建立时间违例端点 0，保持时间违例端点 0。
- 使用 26 个寄存器、35 个逻辑资源、2 个 I/O。
- 未运行独立 RTL 仿真。用户已在主板下载并确认看到了预期闪烁效果；未提供仪器周期测量记录。

构建报告保存在 `impl/`。Git 仅保存源码、约束、工程和文档；构建报告和位流通过本地编译生成，已由 .gitignore 排除。
