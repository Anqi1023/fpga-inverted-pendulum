cd [file dirname [file normalize [info script]]]
open_project led_blink.gprj
set_option -top_module led_blink
set_option -bit_security 0
set_option -user_code 00000000
run all
