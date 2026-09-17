# A 组仿真验证记录

日期：2026-09-17；分支 feature/sensor。

## 实际运行

- 仿真器：Icarus Verilog 10.1 stable（FPGAwars 发布的便携 Windows 构建）。
- 语言开关：`-g2001 -Wall`，没有启用 SystemVerilog。
- 复现命令：`./sim/run_iverilog.ps1 -ToolBin 'C:\Users\hp\.cache\fpga-sensor-tools\bin'`。
  其他电脑按实际安装目录替换 ToolBin，或将工具加入 PATH 后省略该参数。
- 结果：三个自检 Testbench 的 errors 均为 0，脚本成功退出。

```text
PASS tb_quadrature_decoder
PASS tb_velocity_estimator
PASS tb_sensor_state
PASS all sensor tests
```

解码器测试枚举全部 16 种转换，验证正向累计 +40、反向回到 -40，
以及 8 bit 计数 127/-128 两侧回绕。整体测试验证旋转臂 +24、摆杆 -24，
随后反转返回 0，速度方向随之反转；停止后两路速度归零。
还验证两路分别单独运动，任一路非法跳变均使 sensor_valid 保持低直到复位。

该版本 Icarus 在严格 2001 + Wall 下报告标量端口的
`Port declaration ... inherits dimensions from var/net` 警告；无编译错误。
RTL 的标量端口没有总线维度，仿真端口连接及上述断言通过。
详细运行输出和 VCD 在 sim 目录重新运行可生成，未把生成物加入仓库。

## 未通过或尚未执行的项目

后续补充：用户在 ModelSim GUI 执行 run_sensor.do 后反馈“通过了”。
据用户反馈 GUI 仿真已通过；以下命令行许可证进程报错保留为原始运行记录，
不代表 ModelSim GUI 无法使用。尚未收集该次 GUI 的日志作为归档证据。

- 本机 ModelSim 2020.1 执行 `vsim -c -do sim/run_sensor.do` 时出现
  `Fatal: Exiting VSIM license process.`，未进入测试。保留 .do 脚本，但没有宣称
  ModelSim 已通过，也未生成可交付的 ModelSim WLF。
- 没有执行 Gowin 综合、布局布线、时序收敛或下载。
- 没有实机传感器、电气接线、最大边沿率或机械计数验证。
- Icarus 的数字仿真不验证真实亚稳态、抖动、电压适配及板级时序。

下一步硬件工作与接线待确认表见 [sensor-bringup.md](sensor-bringup.md)。
