`timescale 1ns/1ps
// Standalone board top used by sensor_acquisition.gprj.
module sensor_hw_top #(parameter integer RAW_DEBUG=0)(
    input wire clk,input wire rst_n,input wire encoder_a,input wire encoder_b,
    inout wire ads_scl,inout wire ads_sda,output wire uart_tx_o
);
    wire signed [31:0] theta,theta_dot,alpha,alpha_dot;wire sensor_valid;
    sensor_top #(.RAW_DEBUG(RAW_DEBUG)) core(.clk(clk),.rst_n(rst_n),.encoder_a(encoder_a),.encoder_b(encoder_b),
        .ads_scl(ads_scl),.ads_sda(ads_sda),.uart_tx_o(uart_tx_o),.theta(theta),
        .theta_dot(theta_dot),.alpha(alpha),.alpha_dot(alpha_dot),.sensor_valid(sensor_valid));
endmodule
