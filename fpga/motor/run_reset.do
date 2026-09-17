onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -sv rtl/reset_guard.v tb/tb_reset_guard.v
vsim work.tb_reset_guard
run -all
quit -code 0 -force
