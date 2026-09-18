`timescale 1ns/1ps
// Build-time raw debug mode; identical board ports and no motor outputs.
module sensor_raw_debug_top(input wire clk,rst_n,encoder_a,encoder_b,
    inout wire ads_scl,ads_sda,output wire uart_tx_o);
    sensor_hw_top #(.RAW_DEBUG(1)) core(.clk(clk),.rst_n(rst_n),.encoder_a(encoder_a),.encoder_b(encoder_b),
        .ads_scl(ads_scl),.ads_sda(ads_sda),.uart_tx_o(uart_tx_o));
endmodule
