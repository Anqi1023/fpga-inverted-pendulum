# A 组当前验证记录

2026-09-18，基于feature/sensor，新增可选原始数据串口模式。原有早期双编码器/三项测试记录已归档至[历史记录](history/sensor-tests-initial-2026-09-17.md)。9月17日的六项测试构建记录保留为历史快照；以下为本次实际重新运行结果。

## 本次执行结果

- ModelSim Intel FPGA Edition 2020.1：原有六个Testbench全部PASS，另以RAW_DEBUG_TEST执行原始串口帧测试PASS；0错误、0警告。
- sensor_state测试增加编码器计数、ADC末次样本与16次平均值快照检查，使用变化样本区分原始值和平均值。
- 串口测试逐字节验证正常/调试帧、有符号负值及校验；Python两项测试覆盖分片接收、正常帧兼容、坏校验后重新同步。
- Gowin V1.9.12.03：正常及raw两种顶层均完成综合、布局布线及位流生成。
- 正常模式1572条、raw模式1169条已分析路径；setup/hold违例均0。SDC只有时钟约束，不代表外部电气/CDC已验收。

原始文本证据：[evidence/2026-09-18](../fpga/sensor/evidence/2026-09-18)。包含ModelSim记录、两种构建控制台/布局布线/完整时序文本报告、Python测试记录。报告中的本机路径仅标识运行环境。

## 复现

从仓库sim目录运行：

```powershell
vsim -c -l review-tests.log -do run_review.do
```

从fpga/sensor目录运行：

```powershell
python -m unittest test_uart_monitor.py
gw_sh build.tcl
gw_sh build_raw.tcl
```

完整PASS还需检查测试输出标记。原run_sensor.do首次批处理尝试遇到许可证进程退出；新增run_review.do从sim目录成功执行，不能将前次失败算作通过。原GUI脚本保留。

## 未完成的验收

未下载、未接传感器或电机。原始值只是读取链路的调试入口，不证明实际接线、比例、方向或零点正确。已知待办仍包括角度回绕求导、真实更新间隔、板卡版本/C7电平核对及复位/CDC复核。本PR不修改这些策略。
