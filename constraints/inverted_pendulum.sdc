// Tang Primer 20K 板载 27 MHz 晶振（H11）
// 没有这一行，PnR 日志里只有一条 WARN (TA1132) 'clk' was determined to be
// a clock but was not created，等于完全没有做时序分析。
create_clock -name clk -period 37.037 [get_ports {clk}]

// D34 UART RX is asynchronous. It drives only the FIRST synchronizer stage.
// Keep rx_meta -> rx_sync and all following paths timed by clk.
set_false_path -from [get_ports {uart_rx_i}]
