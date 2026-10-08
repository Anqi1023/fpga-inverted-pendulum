# B 电机执行链路

当前版本：**2026-10-08 BANK2 三引脚版**。RPWM=T11、LPWM=P11、共用使能 ibt_en=T12。

## 当前入口

- [最新成果与验证状态](PROGRESS_2026-10-08.md)
- [当前接线](WIRING.md) · [团队引脚交接](../../docs/B_PINOUT_2026-10-08.md)
- [B 独立点动工程、位流及操作](experiments/motor_control_v1_bank2_3pin/README.md)
- [交给 C 的正式执行链和 CST](experiments/motor_control_v1_bank2_3pin/integration/README.md)
- [C→B V1.0 约定](../../docs/CONTROL_INTERFACE_V1_2026-09-27.md)

三项仿真及高云构建通过，**新引脚版尚未上板**。独立工程模拟 C 做 1 kHz、10%、200 ms 点动；联合工程由 C 实例化正式执行链，首次上电保持零命令和禁止电机。
完整下载所选目录再打开 GPRJ，不能只替换 CST 或将新源码与旧位流混用。

## 历史代码和记录

本目录直属 rtl、constraints、tb 及早期工程是历史离线基线，保留原接口和配套约束，**不用于本次三脚接线**。
历史 20 kHz 参数、T6/P6/T7/R8 或 T6/P6/T8/T9 分配不再是当前板级入口。

- [独立实验归档](experiments/README.md)
- [原实物测试记录](TEST_RECORD_2026-09-26_27.md)
- [2026-09-27 成果快照](PROGRESS_2026-09-27.md)
- [当前位流清单](BITSTREAMS_2026-10-08.json)；旧日期清单保留为历史快照。

原 motor_protected.v、protection.v、motor_driver.v、pwm_gen.v、ibt2_adapter.v 保持不变。
当前 C 包内增加 control_u_adapter 和 motor_control_u_chain_3pin；不要让旧顶层和新顶层同时占用电机输出。
