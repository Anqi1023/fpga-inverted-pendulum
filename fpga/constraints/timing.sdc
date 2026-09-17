create_clock -name clk27 -period 37.037037 [get_ports {clk}]
# LED is a human-visible indicator, not a synchronous external interface.
set_false_path -to [get_ports {led_n}]
