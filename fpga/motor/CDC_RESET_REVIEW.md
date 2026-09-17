# 第五步：同步器与复位约束复核

2026-09-18；基于已合并PR #10，未下载、未实机测试。用户已于2026-09-18复现HENGXIN_CDC_RESET_REVIEW_PASS。

## 发现与修正

原reset_guard虽然等待两级同步信号，但原始复位按键仍直接异步复位计数器及reset_n寄存器。现改为原始复位只接两级release_sync；其第二级异步断言、同步撤销后，再释放延时计数器和reset_n。释放周期保持N+2，复位断言无需等待时钟（实际传播有延迟）。

button_input的两个同步寄存器、reset_guard的两级同步寄存器使用Gowin支持的syn_preserve属性防止优化。该属性是保留寄存器，不等于自动证明亚稳态可靠性或强制相邻放置。

## 约束范围

- 三个普通按键：仅输入端到各自第一级D输入的异步到达路径设false path。
- 原始复位：仅输入端到两级复位同步器CLEAR引脚设false path。
- 四组同步器第一级到第二级：5 ns最大延时，这是本项目布局实现预算，不是外部器件参数。保留hold检查。
- 第二级复位同步器到计数器/reset_n的恢复、移除检查保留，不能整体屏蔽reset域。
- 没有添加虚构的外部输入/输出延时；普通按钮不等于急停。

## 验证证据

Gowin V1.9.12.03，27MHz，2026-09-18 01:26完整编译通过。

| 检查 | 结果 |
|---|---|
| 复位、按键、板级ModelSim回归 | 三套均PASS，0错误0警告 |
| 综合、布局布线、位流生成 | 通过，未下载 |
| 综合后四组同步链 | 各两级，首级仅驱动次级D，无额外扇出 |
| 原始复位扇出 | 仅两级同步器CLEAR；后续20寄存器使用同步后复位 |
| SDC约束 | 1时钟+5条精确false path+4条max delay全部Actived |
| 四组5ns级间约束裕量 | start 4.193ns，stop 4.193ns，direction 3.950ns，reset 4.193ns |
| 内部setup/hold违例 | 均0；最差常规setup 32.198ns，hold 0.425ns |
| 下游复位recovery | 20端点，最小裕量36.080ns |
| 下游复位removal | 20端点，最小裕量0.326ns |

新增board/check_cdc.py读取真实综合网表及布局布线报告：检查链结构、约束生效状态、级间预算和下游复位时序，报告比源代码旧时拒绝通过。它是针对当前工程及Gowin报告格式的复核脚本，不是通用CDC工具，也不计算MTBF。

## 一步复现

```powershell
cd D:\FPGA\hengxin\motor-dev\fpga\motor\board
python check_cdc.py
```

成功标记：HENGXIN_CDC_RESET_REVIEW_PASS。报错时提供最后一屏；若提示Stale build，先运行`gw_sh build.tcl`，再运行检查脚本。修改SDC对象命名或升级工具后必须重新复核，不应删掉检查以绕过失败。

## 边界与下一步

本次完成数字逻辑结构和已约束路径复核。仿真不能模拟真实亚稳态概率；未量化MTBF，未测量按键抖动、外部复位最小脉宽或配置前输出。异步复位短脉冲仿真通过不代表任意短硬件脉冲都可靠。

下一步可以安排FPGA单板按键和LED验证，电平转换器、IBT-2、电机仍断开。驱动接入前还需硬件OE/下拉验证、外部许可/急停策略与真实电气测量，不能据本次PASS进行电机测试。

## 依据

本机随工具附带SUG550-2.4E第5.13节（syn_preserve），SUG940-2.0.4E第5.4/5.5节（精确路径例外、最大延时、恢复/移除报告）。[高云官方文档目录](https://www.gowinsemi.com/en/document/main/database/1871/?order=DESC&page=1&support_search=&type=version)；[官方时序约束手册](https://cdn.gowinsemi.com.cn/SUG940E.pdf)。版本以本机构建工具配套文档为准。
