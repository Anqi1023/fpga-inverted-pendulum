create_clock -name sys_clk -period 37.037037 [get_ports {clk}]
set_false_path -from [get_ports {start_key_n}] -to [get_pins {start_i/sync_meta_s0/D}]
set_false_path -from [get_ports {direction_key_n}] -to [get_pins {direction_i/sync_meta_s0/D}]
set_false_path -from [get_ports {stop_key_n}] -to [get_pins {stop_meta_s0/D}]
set_false_path -from [get_ports {reset_key_n}] -to [get_pins {reset_i/release_sync_0_s0/CLEAR}]
set_false_path -from [get_ports {reset_key_n}] -to [get_pins {reset_i/release_sync_1_s0/CLEAR}]
set_max_delay -from [get_regs {start_i/sync_meta_s0}] -to [get_regs {start_i/sync_value_s0}] 5
set_max_delay -from [get_regs {direction_i/sync_meta_s0}] -to [get_regs {direction_i/sync_value_s0}] 5
set_max_delay -from [get_regs {stop_meta_s0}] -to [get_regs {stop_sync_s0}] 5
set_max_delay -from [get_regs {reset_i/release_sync_0_s0}] -to [get_regs {reset_i/release_sync_1_s0}] 5
