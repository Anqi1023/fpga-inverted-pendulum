`timescale 1ns/1ps
module uart_tx #(parameter integer CLKS_PER_BIT=234)(input wire clk,input wire rst_n,
    input wire start,input wire [7:0] data,output reg tx,output reg busy,output reg done);
    reg [15:0] timer;reg [3:0] bit_index;reg [9:0] frame;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin tx<=1;busy<=0;done<=0;timer<=0;bit_index<=0;frame<=0;end
        else begin done<=0;if(!busy&&start)begin frame<={1'b1,data,1'b0};bit_index<=0;timer<=0;tx<=0;busy<=1;end
            else if(busy)begin if(timer==CLKS_PER_BIT-1)begin timer<=0;if(bit_index==9)begin tx<=1;busy<=0;done<=1;end
                else begin bit_index<=bit_index+1'b1;tx<=frame[bit_index+1'b1];end end else timer<=timer+1'b1;end end
    end
endmodule
