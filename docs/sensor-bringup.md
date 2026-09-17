# A 组：编码器采集与仿真交付

日期：2026-09-17。目标分支：`feature/sensor`。

## 当前结论

项目已有资料只有 README 与 hardware-summary，没有可复用 RTL 或 A/C 位宽约定。
主控已确认 Tang Primer 20K + Dock、GW2A-LV18PG256C8/I7。
**摆杆与旋转臂的传感器型号、输出形式仍未确定，不能声称已完成实物接入。**
本次实现双路增量式 A/B 编码器的候选采集方案，支持独立仿真。
若实物输出是模拟电压、SPI、I²C 或差分信号，应增加相应采集/电气适配，
不能将它们直接作为 A/B 输入。ADS1115 是 ADC，清单中有 ADC 不代表角度传感器已经确定。

## 资料核对与接线表

更新：用户已授权统一分配 A/B/C 资源，FPGA 侧的初版分配见
[pin-allocation.md](pin-allocation.md)。下表保留原始待确认接线项；其中 FPGA 侧
四路信号现已预留为 T6/P6/T7/R8，传感器端子、电平及上电验证仍未完成。

来源：[项目硬件汇总](hardware-summary.md)、
[Sipeed 官方板卡介绍与硬件资料入口](https://wiki.sipeed.com/hardware/zh/tang/tang-primer-20k/primer-20k.html)、
[官方示例仓库](https://github.com/sipeed/TangPrimer-20K-example)。
官方介绍提供规格书、原理图和点位图入口，并说明 bank 0/1/7 默认供电为 3.3 V；
这不代表所有引脚都允许直接输入传感器输出电压。仍需匹配实际 PCB 版本和所选 bank。

以下是**逻辑接线规划，不是可直接照接的物理接线表**。暂不生成含猜测管脚的 CST。

| 测量对象/信号 | RTL 端口 | 传感器端子 | Dock 排针位置 | FPGA 封装管脚 | 电气适配 |
|---|---|---|---|---|---|
| 旋转臂 A 相（若有） | position_enc_a | 待型号确认 | 待确认 | 待确认 | 待输出电平/类型确认 |
| 旋转臂 B 相（若有） | position_enc_b | 待型号确认 | 待确认 | 待确认 | 同上 |
| 摆杆 A 相（若有） | angle_enc_a | 待型号确认 | 待确认 | 待确认 | 同上 |
| 摆杆 B 相（若有） | angle_enc_b | 待型号确认 | 待确认 | 待确认 | 同上 |
| 传感器电源 | 无 | VCC 定义待确认 | 不预先指定 | 不使用普通 IO 供电 | 电压/电流待确认 |
| 信号参考地 | 无 | GND 定义待确认 | GND 位置待核对 | 不分配逻辑 IO | 单端接口需公共参考地 |
| 系统时钟 | clk | 板载时钟 | 按板卡原理图 | 待版本核对 | 频率与时序约束待确认 |
| 系统复位 | rst_n | 按键/复位电路待确认 | 待确认 | 待确认 | 低有效，板级异步置位、同步释放 |

补齐步骤：取得两只传感器标签/规格书及端子图，确认推挽/开漏/差分输出和电压；
核对 Dock 版本、排针方向与封装球号；与 B/C 排除管脚冲突；再填写 CST 的 IO_LOC/IO_PORT。
不要把排针序号直接当 FPGA 球号，不要将 5 V 信号直接接到未核实耐压的 FPGA IO。
已有 SN74LVC8T245 模块的供电、DIR/OE 和具体接法须结合实际模块确认。

还需确认：PPR/CPR 是否包含四倍频、最高转速/边沿率、减速比及编码器安装轴、
零点与正方向、是否有 Z 相、机械范围、时钟频率、采样周期、最终单位/定点格式。
旋转臂的 position 是旋转位置，不是小车位移。

## RTL 与 A/C 接口

全部 RTL 为 Verilog-2001，不使用 real。WIDTH/COUNT_WIDTH 默认 32，允许 >=2；
这只是新模块默认值，不是团队已确认的接口。SAMPLE_CYCLES 为正整数，默认 100000
仅是可覆盖的时钟周期数，不代表已确定的控制周期。

| 模块 | 职责 |
|---|---|
| quadrature_decoder | 双级同步、四倍频计数、启动基线、非法跳变脉冲 |
| velocity_estimator | 固定间隔差分，可用于任意同宽有符号采集值 |
| sensor_state | 双路采集，实例化两次速度估计，输出五个统一状态信号 |

向 C 提供 angle、angle_vel、position、position_vel（signed [WIDTH-1:0]），以及 sensor_valid。
angle/position 目前就是相对上电/复位的原始 count，**不是绝对角度，也不是 rad/degree/mm**。
angle_vel/position_vel 是每 SAMPLE_CYCLES 个 clk 内的 count 增量；单位 count/sample。
物理速度换算需已确认的时钟和标定参数。没有实现速度滤波与精确标定。

{A,B} 的正序为 00→01→11→10→00，每个合法跳变 +1，一轮 +4；反序 -1。
计数按固定位宽自然回绕。差分采用模运算，要求每个采样窗口真实位移绝对值小于半个计数范围。
同步器不是去抖滤波器，也不保证捕获任意窄脉冲；每个相位需要稳定足够多个 clk。
测试以至少 5 个 clk 的相位保持时间输入，不代表已经测得硬件最高速率。

复位清零；第三个 clk 建立真实相位基线，避免静止在 01/10/11 时产生假计数。
初始化期间运动不保证被计入，应保持机构静止。
速度第一次采样只建基线，第二次才输出有效差分，因此启动不会把已有偏移当速度。
position/angle 每拍可能更新，速度在采样点更新并保持，C 必须在 clk 域读取。
sensor_valid 表示两路初始化和速度启动完成，**不表示完成物理标定或传感器在线检测**。

两个比特同时变化的非法转换不计数，重建相位并输出一个 clk 错误脉冲。
sensor_state 遇任一路错误便锁存无效，直到复位。错误后 count 可能已丢步，
不自动恢复有效；复位后还需由应用重新建立机械零点。尚不能检测断线、超时或全部漏采情况。

底层解码与差分各自独立；更换传感器时替换采集部分，并保留速度模块。
当前没有加入系统 top、驱动/控制代码、ADC 驱动、UART 或实际管脚约束。

## 仿真与观察

ModelSim 中执行（把路径替换为实际仓库）：

```tcl
do C:/Users/hp/Documents/ChatGPT/FPGA/sim/run_sensor.do
```

命令行：`vsim -c -do sim/run_sensor.do`。
脚本按 Verilog-2001 编译，运行三个自检 Testbench；任一 errors 非零返回失败。
生成 sim/sensor-tests.log 与每个 Testbench 的 WLF，输出不会提交到 Git。
GUI 中可用 `vsim -view tb_sensor_state.wlf` 打开保存的波形，再 `add wave -r /*`、`wave zoom full`。
将 position、angle 及速度设置为 signed decimal。

整体波形包括 clk、rst_n、position_ab[1:0]、angle_ab[1:0]、position、position_vel、
angle、angle_vel、sensor_valid。AB[1] 为 A，AB[0] 为 B。所有驱动在非采样沿改变，
仿真只能验证逻辑，不模拟真实亚稳态。

也可使用 Icarus Verilog：在仓库根目录执行 `./sim/run_iverilog.ps1`；工具不在 PATH 时，
传入 `-ToolBin '包含iverilog.exe和vvp.exe的目录'`。脚本强制 `-g2001`，
检查编译退出码和所有 PASS/FAIL，生成 VCD 与 sim/icarus-tests.log。
VCD 可在兼容的波形查看器中打开，ModelSim WLF 则需要实际运行上述 ModelSim 脚本生成。

| 测试 | 主要验收 |
|---|---|
| tb_quadrature_decoder | 全 16 种状态转换、各初始相位、静止、正反转、非法跳变恢复、8 bit 回绕 |
| tb_velocity_estimator | SAMPLE_CYCLES=1/3、逐周期比较、非零基线、正负差分、停止与回绕、复位 |
| tb_sensor_state | 双路相反方向运动、正反速度、停止、启动有效性、两路非法跳变锁存、复位恢复 |

## 实机验收记录（待执行）

取得传感器型号与接线资料后再制作板级测试工程，复用 quadrature_decoder；
将 count 接入 Gowin 在线逻辑分析仪，或经确认的 LED/UART 接口显示。
本次用仿真完成软件验证，不将其写作“手转实物已经成功”。

1. 复位且机构静止，确认 count=0，启动不误计数。
2. 手动正转/反转旋转臂，分别记录 count 增加/减少及实际方向。
3. 独立手动转动摆杆，确认另一通道不会同步误变。
4. 静止时确认 count 稳定；连续转动检查非法跳变和漏计。
5. 记录板卡/传感器型号、接线照片、比特流版本、波形/视频及实际计数。

以上硬件结果目前均未执行；图片中的“手转机构，FPGA 计数变化”成功标准仍待实机完成。
