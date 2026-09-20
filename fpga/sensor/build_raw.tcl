cd [file dirname [file normalize [info script]]]
open_project sensor_acquisition.gprj
set_option -top_module sensor_raw_debug_top
set_option -bit_security 0
set_option -user_code 00000000
run all
