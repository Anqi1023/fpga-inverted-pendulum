cd [file dirname [file normalize [info script]]]
open_project dock_motor_test.gprj
set_option -top_module dock_motor_test
# T10 is a dual-purpose SSPI pin. Preserve JTAG for programming.
set_option -use_sspi_as_gpio 1
set_option -use_jtag_as_gpio 0
run all
