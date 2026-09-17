onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/button_input.v rtl/pwm_gen.v rtl/motor_driver.v rtl/ibt2_adapter.v rtl/motor_buttons_top.v tb/tb_motor_buttons.v
vsim work.tb_motor_buttons
run -all
quit -code 0 -force
