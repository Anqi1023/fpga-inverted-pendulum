cd [file dirname [file normalize [info script]]]
open_project motor_protected.gprj
set_option -top_module motor_protected
# Reusable logical top. Fault/clear board pins are not assigned: synthesis only.
run syn
