cd [file dirname [file normalize [info script]]]
open_project motor_offline.gprj
set_option -top_module motor_test_top
# Synthesis only: no pin-complete board design or bitstream is authorized here.
run syn
