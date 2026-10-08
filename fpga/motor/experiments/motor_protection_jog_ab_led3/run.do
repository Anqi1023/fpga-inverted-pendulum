onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/pwm_gen.v rtl/motor_driver.v rtl/ibt2_adapter.v rtl/protection.v rtl/motor_protected.v rtl/button_input.v rtl/reset_guard.v rtl/dock_motor_protection.v tb/tb_protection_board.v tb/tb_protection_board_pwm.v
if {![info exists test_top]} {set test_top tb_protection_board}
vsim work.$test_top
run -all
quit -code 0 -force
