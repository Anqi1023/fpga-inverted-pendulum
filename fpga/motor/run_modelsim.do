onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/pwm_gen.v rtl/motor_driver.v rtl/ibt2_adapter.v rtl/motor_test_top.v tb/tb_motor.v
vsim work.tb_motor
log -r /*
run -all
quit -code 0 -force
