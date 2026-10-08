onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/pwm_gen.v rtl/motor_driver.v rtl/ibt2_adapter.v rtl/button_input.v rtl/motor_buttons_top.v rtl/reset_guard.v rtl/dock_motor_test.v tb/tb_motor_1khz.v
vsim work.tb_motor_1khz
run -all
quit -code 0 -force
