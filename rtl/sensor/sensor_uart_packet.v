`timescale 1ns/1ps
// A5 5A + four little-endian Q16.16 words + valid + XOR checksum.
module sensor_uart_packet #(parameter integer CLKS_PER_BIT=234, parameter integer RAW_DEBUG=0)(input wire clk,input wire rst_n,input wire send,
    input wire [31:0] theta,theta_dot,alpha,alpha_dot,input wire sensor_valid,input wire [31:0] raw_encoder,input wire [15:0] raw_adc,raw_adc_average,output wire tx,output reg busy);
    reg [7:0] bytes[0:18];reg [4:0] index;reg tx_start;reg [7:0] tx_data,checksum;wire tx_busy,tx_done;
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u(.clk(clk),.rst_n(rst_n),.start(tx_start),.data(tx_data),.tx(tx),.busy(tx_busy),.done(tx_done));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin busy<=0;index<=0;tx_start<=0;tx_data<=0;checksum<=0;end else begin tx_start<=0;
            if(!busy&&send)begin bytes[0]<=8'hA5;bytes[1]<=8'h5A;
                bytes[2]<=theta[7:0];bytes[3]<=theta[15:8];bytes[4]<=theta[23:16];bytes[5]<=theta[31:24];
                bytes[6]<=theta_dot[7:0];bytes[7]<=theta_dot[15:8];bytes[8]<=theta_dot[23:16];bytes[9]<=theta_dot[31:24];
                bytes[10]<=alpha[7:0];bytes[11]<=alpha[15:8];bytes[12]<=alpha[23:16];bytes[13]<=alpha[31:24];
                bytes[14]<=alpha_dot[7:0];bytes[15]<=alpha_dot[15:8];bytes[16]<=alpha_dot[23:16];bytes[17]<=alpha_dot[31:24];
                // Alternate debug frame keeps 20 bytes and uses a distinct header.
                if(RAW_DEBUG!=0) begin
                    bytes[1]<=8'h5B;
                    bytes[2]<=raw_encoder[7:0];bytes[3]<=raw_encoder[15:8];bytes[4]<=raw_encoder[23:16];bytes[5]<=raw_encoder[31:24];
                    bytes[6]<=raw_adc[7:0];bytes[7]<=raw_adc[15:8];
                    bytes[8]<=raw_adc_average[7:0];bytes[9]<=raw_adc_average[15:8];
                    bytes[10]<=0;bytes[11]<=0;bytes[12]<=0;bytes[13]<=0;
                    bytes[14]<=0;bytes[15]<=0;bytes[16]<=0;bytes[17]<=0;
                end
                bytes[18]<={7'b0,sensor_valid};checksum<=8'hFF;index<=0;busy<=1;end
            else if(busy&&!tx_busy&&!tx_start)begin if(index<=18)begin tx_data<=bytes[index];tx_start<=1;checksum<=checksum^bytes[index];end
                else if(index==19)begin tx_data<=checksum;tx_start<=1;end else busy<=0;index<=index+1'b1;end end
    end
endmodule
