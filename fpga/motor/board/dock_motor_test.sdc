create_clock -name sys_clk -period 37.037037 [get_ports {clk}]
// Async arrival is excluded ONLY up to each first-stage D pin.
set_false_path -from [get_ports {start_key_n}] -to [get_pins {control_i/start_i/sync_meta_s0/D}]
set_false_path -from [get_ports {stop_key_n}] -to [get_pins {control_i/stop_i/sync_meta_s0/D}]
set_false_path -from [get_ports {direction_key_n}] -to [get_pins {control_i/direction_i/sync_meta_s0/D}]
// Raw reset only clears the two reset synchronizer flops.
set_false_path -from [get_ports {reset_key_n}] -to [get_pins {reset_i/release_sync_0_s0/CLEAR}]
set_false_path -from [get_ports {reset_key_n}] -to [get_pins {reset_i/release_sync_1_s0/CLEAR}]
// 5 ns is an implementation budget, not an external device requirement.
// Keep all interstage hold checks and downstream recovery/removal checks.
set_max_delay -from [get_regs {control_i/start_i/sync_meta_s0}] -to [get_regs {control_i/start_i/sync_value_s0}] 5
set_max_delay -from [get_regs {control_i/stop_i/sync_meta_s0}] -to [get_regs {control_i/stop_i/sync_value_s0}] 5
set_max_delay -from [get_regs {control_i/direction_i/sync_meta_s0}] -to [get_regs {control_i/direction_i/sync_value_s0}] 5
set_max_delay -from [get_regs {reset_i/release_sync_0_s0}] -to [get_regs {reset_i/release_sync_1_s0}] 5
// No invented external input/output delay. Outputs are standalone test signals.
