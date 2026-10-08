create_clock -name sys_clk -period 37.037037 [get_ports {clk}]
set_false_path -from [get_ports {start_key_n}] -to [get_pins {start_i/sync_meta_s0/D}]
set_false_path -from [get_ports {estop_key_n}] -to [get_pins {estop_meta_s0/D}]
set_false_path -from [get_ports {fault_key_n}] -to [get_pins {fault_meta_s0/D}]
set_false_path -from [get_ports {reset_key_n}] -to [get_pins {reset_i/release_sync_0_s0/CLEAR}]
set_false_path -from [get_ports {reset_key_n}] -to [get_pins {reset_i/release_sync_1_s0/CLEAR}]
set_max_delay -from [get_regs {start_i/sync_meta_s0}] -to [get_regs {start_i/sync_value_s0}] 5
set_max_delay -from [get_regs {estop_meta_s0}] -to [get_regs {estop_sync_s0}] 5
set_max_delay -from [get_regs {fault_meta_s0}] -to [get_regs {fault_sync_s0}] 5
set_max_delay -from [get_regs {reset_i/release_sync_0_s0}] -to [get_regs {reset_i/release_sync_1_s0}] 5
