# C→B 接口 V1.0：最新补充约定

来源：用户与 C 确认的 V1.0 约定。2026-10-08 更新交接入口和物理引脚，命令单位、20 ms 更新、100 ms 超时不变。
当前正式模块为 [motor_control_u_chain_3pin](../fpga/motor/experiments/motor_control_v1_bank2_3pin/integration/README.md)；旧 motor_control_v1_100ms 留作四脚历史基线。

## 已确认并实现的参数

| 项目 | 最新定义 |
|---|---|
| 时钟 | 27 MHz，同步时钟域 |
| control_u | signed [15:0]，单位 0.1% PWM，范围 -300～+300 |
| control_valid | 正常每 20 ms（540000 拍）产生一次、宽度 1 拍；在有效时钟沿采纳命令 |
| duty 转换 | abs(control_u)/10 整数截断，例如 +85→8%，-200→20% |
| 逻辑方向 | 正数正向（01），负数反向（10），0 停止（00）；截断为 0 的小命令也按停止处理 |
| motor_enable=0 | 立即撤销四路驱动，有效 command/duty 清零，不执行保存的旧命令 |
| watchdog | 100 ms（2700000 拍）未收到有效命令，停止 |
| 优先级 | fault / estop / protection_locked 优先于 C 命令 |
| PWM | 1 kHz；最大 30%，不自动补偿启动死区 |

机械正方向与 A 的 theta 增减尚未定义，由 A+B+C 实机联调确认。

## B 的恢复与边界规则，需随包告知 C

1. 沿用现有超时锁定：100 ms 超时除了清零，还锁存保护；迟到命令不会自动解锁。人工清除后才能重新启动。
2. 普通禁用不锁定，但清掉旧命令；重新使能必须有新的 control_valid。使能与新命令可在同一拍到达。
3. 使能且保护解除后，第一条命令缺失也计时；有效 0 命令刷新计时；禁用期间不计时。
4. abs(control_u)<10 截断为 0，按 command=00 且四路驱动关闭处理。
5. 清除要求 motor_enable=0、control_valid=0、故障解除，permit_in 恢复允许，然后给 clear_request 新上升沿（适配器产生 clear_fault）；清除本身不启动。
6. 越界值先拦截后取绝对值，±301 和 -32768 等触发保护；valid 连续两拍为高视为协议错误。
7. **1 秒停止/换向等待仍保留**，尚未验证高速正反向闭环。进入平衡控制前须与 C 确定并验证新的换向策略。

## 程序交接（2026-10-08）

- C 实例化 integration/rtl/motor_control_u_chain_3pin.v，并加入该目录全部依赖。链路为 control_u_adapter → 原 motor_protected → 三个物理输出；原五个驱动模块未改。
- C 输入仍为 control_u/control_valid/motor_enable；permit_in/fault/estop/clear_request 按正式模块接口连接，保护反馈链已包含在包装模块中。不能用 motor_enable 代替独立 permit_in。
- command/duty_pct/arm/clear_fault 由适配器生成；C 不需要在外部重复转换或绕开保护。
- 物理输出：rpwm=T11、lpwm=P11、ibt_en=T12，BANK2 / LVCMOS33。T12 分到转换器 B3/B4，A3/A4 分别接 R_EN/L_EN，不短接输出。
- 联合顶层合并 integration/B_motor_bank2_3pin.cst，删除旧 B 四输出约束；摄像头及占用三脚的其他外设不接。
- B 独立 dock_motor_control_v1 仍是模拟 C 的 10%/200 ms 点动顶层，不能作为 ABC 联合 TEST_U=0 顶层。正式集成模块没有 200 ms 运行限制。
- 时钟 H11、LED3/N14 及原按键分配不变；其他联合资源由统一顶层和 CST 管理。

本次三项仿真和构建结果见 [TEST_RESULTS](../fpga/motor/experiments/motor_control_v1_bank2_3pin/TEST_RESULTS.md)。**新三脚版本尚未上板**；旧四脚点动“符合预期，CV”是历史实物证据，不能转算为新引脚验收。1 秒停止/换向等待仍保留。
