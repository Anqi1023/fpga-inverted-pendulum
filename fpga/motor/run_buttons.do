onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/button_input.v tb/tb_button_input.v
vsim work.tb_button_input
run -all
quit -code 0 -force
