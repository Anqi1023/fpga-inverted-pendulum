# IBT-2 四路接口与接线计划

2026-09-17 更新。以下 FPGA 分配来自用户本轮确认；尚未进行实物接线或电机测试。
采用 IBT-2 的 RPWM/LPWM/R_EN/L_EN，不存在外接 DIR 端子。用户本轮将功率芯片型号确认为 BTS7960B ×2，缓冲为 74HC244D；此记录替代此前 BTN7960B 识别记录。仓库早期硬件汇总属于历史记录，应以本更新为准。

## GPIO 与电平转换

| FPGA 信号 | FPGA 球位 | Dock 接口 | 中间路径 | IBT-2 端子 | 方向/有效电平 |
|---|---|---|---|---|---|
| rpwm | T6 | PMOD0 | 3.3V → 非反相 HCT/AHCT 通道 → 5V | RPWM | 输出，高有效 PWM |
| lpwm | P6 | PMOD0 | 同上，独立通道 | LPWM | 输出，高有效 PWM |
| r_en | T7 | PMOD0 | 同上，独立通道 | R_EN | 输出，1 使能，0 禁止 |
| l_en | R8 | PMOD0 | 同上，独立通道 | L_EN | 输出，1 使能，0 禁止 |
| GND | 地端，无 GPIO 编号 | PMOD0 GND | 与转换器及逻辑电源共地 | GND | 参考地 |

表中球位不是 PMOD 连接器物理针序。插头方向/针序按实际板和原理图对应，不依赖照片左右方向。

FPGA 管脚使用 LVCMOS33、4mA、内部下拉；IBT-2 VCC 计划接稳压 5V。PMOD 的 3V3 不给 IBT-2 的 5V VCC 供电。5V 不得反向接入 FPGA GPIO。

可选转换器：74HCT125、74AHCT125、74HCT244；最终具体型号、封装及通道脚号仍为 TODO。必须选 HCT/AHCT 的 TTL 输入版本，不能用同名 HC/AHC 版本替代。转换器电源为 5V，四路均从 FPGA 指向驱动，使用非反相通道。

## 四路真值表

| 控制状态 | RPWM | LPWM | R_EN | L_EN |
|---|---|---|---|---|
| 上电初始化/复位/未解锁/STOP/等待/失去许可 | 0 | 0 | 0 | 0 |
| 已使能逻辑正向 | PWM | 0 | 1 | 1 |
| 已使能逻辑反向 | 0 | PWM | 1 | 1 |
| 接口层检测到方向变化的空白周期 | 0 | 0 | 0 | 0 |

占空比 0% 而命令仍为运行时，两 PWM 为 0、两 EN 仍为 1；因此它不是 STOP，不应作为释放驱动的办法。BTS7960 的 INH=0 禁止桥驱动；INH=1、IN=0 为低侧导通。此处 STOP 采用禁用，不承诺机械立即停稳。

方向只表示 RPWM 分支或 LPWM 分支。实际电机运动方向待低占空比实测，不标为机械左/右。

## 安全边界

- RTL 复位/撤销许可使四路输出为零。配置完成后的初始化是零，但 FPGA 未配置、重新下载、掉电时不能只依靠 RTL 保证输出。
- 转换器 /OE、输入默认下拉，以及驱动侧尤其 R_EN/L_EN 的外部下拉须设计并核实。/OE 关闭通常为高阻而不是输出零，必须确保高阻时驱动也保持禁止；电阻值、供电顺序及断电回灌需匹配实际器件后确定。
- 四路均需默认确定状态，不能让转换器输入悬空。最终 /OE 与急停方案不得依赖未初始化 FPGA 信号。
- 状态机换向先禁止，默认等待 1 秒并要求 STOP 状态重新解锁；接口层另在方向变化时至少保留一个完整时钟的空白周期。后者是数字互锁，不代表已经计算了功率器件死区或电机停稳时间。
- 控制输入目前必须同步。复位、按键/拨码、许可/急停输入管脚、同步释放和防抖方案尚未完成，不直接连接原始按键。
- R_IS/L_IS 不使用、不接 FPGA，未创建任何对应端口或约束。
- B+/B− 接电机电源、M+/M− 接电机。具体动力线识别、保护和接线仍待实物；今天不连接功率侧。

## 必须等实物确认

电机动力线、联轴器能否脱开、机械行程、实际转向、停稳时间、启动占空比、电流、发热、异响以及失电/下载时的实际驱动状态。电源保护和急停需先确认再安排实测。

## 资料核对

- [Infineon BTS7960 数据手册](https://www.infineon.com/assets/row/public/documents/10/57/infineon-bts7960-ds-en.pdf)：INH/IN 真值表。
- [Nexperia 74HC125/74HCT125](https://www.nexperia.com/products/analog-logic-ics/logic/buffers-inverters-transceivers/series/74HC125-74HCT125.html)：HCT 为 TTL 输入，/OE 控制三态输出。
- [Nexperia 74AHC125/74AHCT125](https://www.nexperia.com/products/analog-logic-ics/logic/buffers-inverters-transceivers/buffers/serie/74ahc125-74ahct125)：AHCT 为 TTL 输入版本。
