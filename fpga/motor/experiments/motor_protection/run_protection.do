onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/pwm_gen.v rtl/motor_driver.v rtl/ibt2_adapter.v rtl/protection.v rtl/motor_protected.v tb/tb_protection.v tb/tb_motor_protected.v tb/tb_motor.v tb/tb_ibt2_adapter.v
if {![info exists test_top]} {set test_top tb_motor_protected}
vsim work.$test_top
run -all
quit -code 0 -force
