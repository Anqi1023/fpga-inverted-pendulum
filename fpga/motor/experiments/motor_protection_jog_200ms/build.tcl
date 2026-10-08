cd [file dirname [file normalize [info script]]]
open_project motor_protection_jog_200ms.gprj
set_option -top_module dock_motor_protection
set_option -use_sspi_as_gpio 1
set_option -use_jtag_as_gpio 0
run all
