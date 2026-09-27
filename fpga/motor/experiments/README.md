# B 实验程序归档与当前入口

完整保留上次 PR #15 后的独立工程、位流、测试平台和现有报告。
**当前接线只使用 [motor_protection_jog_ab_led3](motor_protection_jog_ab_led3/README.md)。**
各子目录包含自己的源码和约束，不可混用工程或只替换一个 .v 文件。

| 工程目录 | 用途及状态 | 引脚版本 |
|---|---|---|
| motor_jog_20khz_20pct_20260923 | 历史 20 kHz、20%、200 ms；仿真/构建通过，未据此认定实机通过 | 旧 EN T7/R8、就绪 N16 |
| motor_pwm_1khz_test | 1 kHz/10% 连续输出；空载 1.2 V DC/3.6 V AC，正反转、停止通过 | 同上 |
| motor_jog_1khz_20pct | 1 kHz、20%、200 ms；用户合并反馈通过 | 同上 |
| motor_jog_1khz_30pct | 1 kHz、30%、200 ms；用户合并反馈通过 | 同上 |
| motor_jog_1khz_5pct | 正向 0/3 启动 | 同上 |
| motor_jog_1khz_7pct | 正向 1/3 明显转动、2/3 抖动；反向 0/3 | 同上 |
| motor_jog_1khz_8pct | 正反向各 3/3 明显转动 | 同上 |
| motor_protection | protection + motor_protected 离线验证/综合快照 | 无板级引脚 |
| motor_protection_board | 最长 5 秒信号测试，CH1 关闭；S2/S3 锁定和四路归零通过 | 旧 EN T7/R8、就绪 N16 |
| motor_protection_jog_200ms | 带保护 1 kHz/10%、200 ms，实际点动及禁止启动通过 | 同上 |
| motor_protection_jog_ab_pins | 新 EN T8/T9；点动及 S2/S3 禁止启动复测通过 | EN T8/T9、旧就绪 N16 |
| **motor_protection_jog_ab_led3** | **当前基线；LED3 就绪、实际转动后停下、CV** | **EN T8/T9、就绪 N14** |

S3 在早期无保护程序中是换向，在保护程序中是模拟故障；不能混用操作。
5 秒信号版仅用于 CH1 关闭时的逻辑测试，不是当前带载点动程序。
逐项实物反馈、条件和限制见 [测试记录](../TEST_RECORD_2026-09-26_27.md)。

每个 .fs 均保留原始字节并列于 [位流清单](../BITSTREAMS_2026-09-27.json)。
verification 中的日志属于当时构建，可能包含开发机绝对路径；工程源码引用为相对路径。
重建建议复制整个所选工程到不含中文的路径，使用 Gowin V1.9.12.03 的 gw_sh build.tcl；脚本中的 SSPI/JTAG 设置不可省略。
