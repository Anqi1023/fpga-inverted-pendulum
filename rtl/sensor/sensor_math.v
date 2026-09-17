`timescale 1ns/1ps
// Q16.16 rad/rad/s. Reproduces vendor mapping after ADS1115 scaling.
module sensor_math #(
    parameter signed [31:0] ALPHA_ZERO_RAW=32'sd19985,
    parameter signed [31:0] ALPHA_WRAP_RAW=32'sd6782,
    parameter signed [31:0] ALPHA_GAIN_Q24=32'sd3992,
    parameter signed [31:0] TWO_PI_Q16=32'sd411775,
    // round((2*pi/1040)*65536*256) and round((2*pi/1040)*50*65536)
    parameter signed [31:0] THETA_GAIN_Q24=32'sd101360,
    parameter signed [31:0] SPEED_GAIN_Q16=32'sd19797,
    parameter integer UPDATE_HZ=50
) (
    input wire clk,input wire rst_n,input wire signed [31:0] encoder_count,
    input wire signed [15:0] adc_average,input wire update,
    output reg signed [31:0] theta,theta_dot,alpha,alpha_dot,output reg valid
);
    reg signed [31:0] last_encoder,last_alpha;reg baseline;
    reg signed [63:0] theta_calc,speed_calc,alpha_calc;reg signed [31:0] alpha_next;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin theta<=0;theta_dot<=0;alpha<=0;alpha_dot<=0;last_encoder<=0;last_alpha<=0;
            baseline<=0;valid<=0;theta_calc<=0;speed_calc<=0;alpha_calc<=0;alpha_next<=0;end
        else if(update)begin
            theta_calc=$signed(encoder_count)*$signed(THETA_GAIN_Q24);
            speed_calc=$signed(encoder_count-last_encoder)*$signed(SPEED_GAIN_Q16);
            alpha_calc=($signed({{48{adc_average[15]}},adc_average})-ALPHA_ZERO_RAW)*ALPHA_GAIN_Q24;
            alpha_next=alpha_calc[39:8];
            if($signed(adc_average)<=ALPHA_WRAP_RAW)alpha_next=alpha_next+TWO_PI_Q16;
            theta<=theta_calc[39:8];theta_dot<=baseline?speed_calc[31:0]:0;
            alpha<=alpha_next;alpha_dot<=baseline?(alpha_next-last_alpha)*UPDATE_HZ:0;
            last_encoder<=encoder_count;last_alpha<=alpha_next;baseline<=1;valid<=baseline;
        end
    end
endmodule
