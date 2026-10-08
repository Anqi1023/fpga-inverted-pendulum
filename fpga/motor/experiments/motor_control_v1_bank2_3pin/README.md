# B 第一阶段：BANK2 三引脚版

更新：2026-10-08。RPWM=T11，LPWM=P11，共用使能 ibt_en=T12；均 LVCMOS33 / 3.3 V。
本目录是新版本，旧实测工程和四脚冲突审查版本均未覆盖；使用本版新生成的 FS。

## 你打开哪个工程

- B 独立上板测试：motor_control_v1_bank2_3pin.gprj；顶层 dock_motor_control_v1。
- 推荐本地目录：D:/FPGA/hengxin/motor-control-v1-bank2-3pin。
- 位流：motor_control_v1_bank2_3pin.fs（编译通过后附在根目录），与 impl/pnr 内的同名文件一致。
- 给 C：本包 integration 目录，包含正式三引脚执行链、全部依赖及 CST 片段。

独立测试仍是原来的模拟 C 点动程序：1 kHz、+100 即 10%、每 20 ms 更新、EN 最长 200 ms、
100 ms 看门狗、停止/换向等待 1 s。它没有接入 A 或 C 的真实联合程序。
第一次 ABC 安全上电由 C 的联合程序保持 control_u=0、motor_enable=0；不能把本点动程序
当作联合 TEST_U=0 程序。C 可使用 integration 内正式模块搭建联合工程。

## 新接线

换线前：关闭 CH1 输出，断开 FPGA USB 和给降压模块供电的 12 V 适配器。
不要接 DVP 摄像头，也不要在这些引脚上接其他 PMOD 外设。

| FPGA | B 电平转换器输入 | 转换器输出 → IBT-2 |
|---|---|---|
| T11 | B1 | A1 → RPWM |
| P11 | B2 | A2 → LPWM |
| T12 | 同时接 B3、B4 | A3 → R_EN；A4 → L_EN |

T12 是同一个输出分到两个输入，可用可靠的分线端子；**A3 与 A4 不短接**。
原 T6/P6/T8/T9 到转换器的四根控制连接应移走；不再使用 R12/R13/T13 的旧审查方案。
转换器 VCB=3.3 V、VCA=5 V、DIR/OE 及电阻设置沿用原实测接法。
IBT-2 VCC=5 V，各设备共地；B+/B− 接电机电源，M+/M− 接电机，保持原接法。
旧的 R_EN/L_EN 临时直连 5 V 跳线不能保留。

PMOD1 背面丝印中的 T11、P11、T12 为本方案三脚，按字母编号识别，不猜正反面的左右位置。
T11=DVP_D1、P11=DVP_D2、T12=DVP_D0；在 Dock 3713 上未接摄像头时不存在摄像头输出争用。
这些引脚均属 BANK2；原理图注明 BANK2=3.3 V。编译设置不改变实际供电电压。
原理图核对来源：D:/FPGA/hengxin/reference/Tang_Primer_20K_Dock-3713_Schematics.pdf。

## 软件改动

独立顶层物理输出从 rpwm/lpwm/r_en/l_en 改为 rpwm/lpwm/ibt_en。
原核心两个使能保留为内部信号，ibt_en = r_en && l_en；两路都允许才输出使能。
200 ms 计时和 LED4 改为观察实际共用使能。原始驱动、PWM、保护与 C 协议核心不改。
GPRJ/CST/build.tcl、板级仿真接口同步修改；SDC 路径继续适用。

S0/T10 复位；S1/T3 启动/清锁；S2/T2 急停；S3/D7 故障注入，保持原样。
LED3/N14 就绪；LED4/L14 使能；LED5/L16 锁定，保持原样。
A 的 P9/P8/R8/T7/N16 及共享时钟 H11 不受三脚分配影响。

## 首次换脚复测

1. 接好线后，先保持 CH1 输出关闭；接 FPGA USB 和逻辑电源。
2. 下载新三引脚 FS。等待 2 秒，上电锁定 LED5 亮，IBT-2 四个控制端均应约 0 V。
3. 确认零输出后，CH1 设置 12 V、0.5 A 限流并打开；旋转臂应保持不动。
4. 单击 S1，只清锁到 LED3，电机仍不转。再次单击 S1 才进行 10% 的 200 ms 点动。
5. 点动后自动停止，等待约 1 秒恢复就绪；新接线和本次位流尚需用户实测。

仿真、布局布线与校验记录见 TEST_RESULTS.md。GitHub 交接入口为本目录；尚未下载到实板验证。
