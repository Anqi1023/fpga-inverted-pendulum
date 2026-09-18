onerror {quit -code 1 -force}
onbreak {resume}
if {![file isdirectory work]} {vlib work}
vmap work work
vlog -vlog01compat ../rtl/sensor/*.v tb_*.v
foreach bench {tb_quadrature_decoder tb_velocity_estimator tb_sensor_math tb_ads1115_reader tb_sensor_state tb_sensor_uart_packet} {
    vsim -onfinish stop work.$bench
    run -all
    set errors [examine -radix decimal /$bench/errors]
    if {$errors != 0} {quit -code 1 -force}
    quit -sim
}
vlog -vlog01compat +define+RAW_DEBUG_TEST tb_sensor_uart_packet.v
vsim -onfinish stop work.tb_sensor_uart_packet
run -all
set errors [examine -radix decimal /tb_sensor_uart_packet/errors]
if {$errors != 0} {quit -code 1 -force}
puts "PASS raw debug UART test"
puts "PASS all sensor tests"
quit -code 0 -force
