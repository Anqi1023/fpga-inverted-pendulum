# Run from ModelSim: do <repository>/sim/run_sensor.do
# Generates a waveform database for each test and a combined transcript.
set sim_dir [file dirname [file normalize [info script]]]
cd $sim_dir
onerror {quit -code 1 -force}
onbreak {resume}
transcript file sensor-tests.log
if {![file isdirectory work]} {vlib work}
vmap work work
vlog -vlog01compat ../rtl/sensor/*.v tb_quadrature_decoder.v tb_velocity_estimator.v tb_sensor_math.v tb_ads1115_reader.v tb_sensor_state.v tb_sensor_uart_packet.v
foreach bench {tb_quadrature_decoder tb_velocity_estimator tb_sensor_math tb_ads1115_reader tb_sensor_state tb_sensor_uart_packet} {
    vsim -onfinish stop -voptargs=+acc -wlf ${bench}.wlf work.$bench
    log -r /*
    if {![batch_mode]} {add wave -r /*}
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
quit -sim
puts "PASS raw debug UART test"
puts "PASS all sensor tests"
if {[batch_mode]} {quit -code 0 -force}
