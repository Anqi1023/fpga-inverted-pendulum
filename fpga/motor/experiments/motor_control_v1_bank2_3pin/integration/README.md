# 给 C：BANK2 三引脚执行链

选择顶层可实例化模块 rtl/motor_control_u_chain_3pin.v，添加本目录 rtl 下全部依赖。
这是正式执行链，没有按键和 200 ms 点动限制；不要再并行实例化另一条驱动链去占同一引脚。

变化只有物理输出接口：rpwm、lpwm、ibt_en（一个输出控制两个 IBT-2 使能输入）。
原 motor_control_u_chain.v 及 control_u_adapter.v 和五个原始驱动模块保留，内容未改。
新模块保留原参数、C 输入、保护输入与状态输出，在模块内合并两个驱动使能。

- control_u：signed[15:0]，单位 0.1%，范围 ±300，除 10 截断；零占空比停止。
- control_valid：同一 27 MHz 时钟下 1 拍，正常间隔 20 ms。
- motor_enable：C 的允许输入，0 时停止；不是实际物理使能输出。
- ibt_en：B 的实际 3.3 V 物理使能输出，经转换器的两个独立通道到 R_EN/L_EN。
- fault/estop/permit_in/clear_request 和原反馈保护链保持原约定；不可用 motor_enable 代替 permit_in。
- 上电锁定；100 ms 无有效控制更新锁定；停止状态人工 clear_request 清除后仍需新命令。
- STOP_CYCLES 默认 1 s；还没有验证高速连续换向或倒立平衡闭环。

合并 B_motor_bank2_3pin.cst 到联合 CST，顶层三个输出端口名须匹配；
删除旧 B 的 T6/P6/T8/T9 或 R12/R13/T12/T13 约束，以及旧顶层 r_en/l_en 物理输出。
不要将 r_en 和 l_en 两个端口都约束到 T12，也不要再把 ibt_en 命名成 motor_enable。
原理图层面要求：摄像头不连接。团队层面要求：C 的其他逻辑没有占用 T11/P11/T12。
联合顶层和全量 CST 不在本包内，必须在联合工程重新检查全部引脚及 BANK 电压。

连接形态：C 的 control_u/control_valid/motor_enable → motor_control_u_chain_3pin →
rpwm(T11)、lpwm(P11)、ibt_en(T12)。T12 同时接转换器 B3/B4；A3/A4 分别接 R_EN/L_EN。
初次安全上电请保持 control_u=0、motor_enable=0，先确认零输出。

motor_control_u_chain_3pin.gprj 只供核心综合，不含联合顶层物理约束，不用于直接下载。
