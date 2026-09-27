cd [file dirname [file normalize [info script]]]
open_project motor_jog_1khz_20pct.gprj
set_option -top_module dock_motor_jog
set_option -use_sspi_as_gpio 1
set_option -use_jtag_as_gpio 0
run all
