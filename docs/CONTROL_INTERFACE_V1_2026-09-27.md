# C→B 接口 V1.0：最新补充约定

来源：用户转交的 C 接口截图、20 ms 周期确认，以及随后给出的六条 V1.0 补充约定。最新交接目录为 **motor_control_v1_100ms**；旧 motor_control_v1_20ms 留作刚完成用户实测的基线，不再作为最新接口参数。

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
5. 清除要求 motor_enable=0、control_valid=0、故障解除，然后 clear_fault 给新上升沿；清除本身不启动。
6. 越界值先拦截后取绝对值，±301 和 -32768 等触发保护；valid 连续两拍为高视为协议错误。
7. **1 秒停止/换向等待仍保留**，尚未验证高速正反向闭环。进入平衡控制前须与 C 确定并验证新的换向策略。

## 程序交接

- C 实例化 rtl/motor_control_v1.v，依赖 pwm_tenths.v、protection.v、ibt2_adapter.v。新核心直接接受三个 C 端口；不要在 C 端先截断一次再把整数百分比当 control_u 输入。
- command/duty_pct 是新版核心内部的有效请求；PWM 与 duty_tenths_o 保留十分之一百分点计数接口，但本补充版只输出 10 的倍数。
- 原 motor_protected 的 command/arm/permit_in/duty_pct 是旧接口，不等于已直接支持 C 的新接口。
- dock_motor_control_v1 是独立模拟 C 的 10%/200 ms 点动顶层，没有实例化真实 A/C；不能作为三人联合控制顶层。
- 引脚不变：RPWM=T6、LPWM=P6、R_EN=T8、L_EN=T9；LED3=N14；时钟 H11。A 的 P9/P8、R8/T7、N16 不被 B 占用。

本次修改的仿真、构建结果以 motor_control_v1_100ms/TEST_RESULTS.md 为准。旧版用户反馈单独记录于该包 BENCH_BASELINE.md，不转算为新补充版的硬件验证。实物下载和电源操作由用户完成。最新点动复测反馈为“符合预期，CV”；100 ms 失联和非整数输入截断以仿真为证据。
