onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/pwm_tenths.v rtl/protection.v rtl/ibt2_adapter.v rtl/motor_control_v1.v rtl/button_input.v rtl/reset_guard.v rtl/dock_motor_control_v1.v tb/tb_motor_control_v1.v tb/tb_control_watchdog.v tb/tb_control_board.v tb/tb_control_board_pwm.v
if {![info exists test_top]} {set test_top tb_motor_control_v1}
vsim work.$test_top
run -all
quit -code 0 -force
