onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv pwm_gen.v button_input.v reset_guard.v dock_motor_jog.v tb_jog.v
vsim work.tb_jog
run -all
quit -code 0 -force
