`timescale 1ns/1ps
`default_nettype none
// 8N1; two-flop synchronizer, start midpoint validation, stop-bit validation.
module uart_rx #(parameter integer CLK_HZ=27000000, BAUD=460800)(
    input wire clk, rst_n, rx_i,
    output reg [7:0] data_o,
    output reg valid_o, error_o
);
    localparam integer CPB=CLK_HZ/BAUD;
    localparam integer CW=(CPB<2)?1:$clog2(CPB);
    (* syn_preserve=1 *) reg rx_meta, rx_sync;
    reg [CW-1:0] count;
    reg [1:0] state;
    reg [2:0] bit_index;
    reg [7:0] shift;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin rx_meta<=1; rx_sync<=1; end
        else begin rx_meta<=rx_i; rx_sync<=rx_meta; end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=0; count<=0; bit_index<=0; shift<=0;
            data_o<=0; valid_o<=0; error_o<=0;
        end else begin
            valid_o<=0; error_o<=0;
            case(state)
                0: if (!rx_sync) begin count<=(CPB/2)-1; state<=1; end
                1: if (count!=0) count<=count-1'b1;
                   else if (rx_sync) state<=0;
                   else begin state<=2; count<=CPB-1; bit_index<=0; end
                2: if (count!=0) count<=count-1'b1;
                   else begin
                       shift[bit_index]<=rx_sync; count<=CPB-1;
                       if (bit_index==7) state<=3; else bit_index<=bit_index+1'b1;
                   end
                3: if (count!=0) count<=count-1'b1;
                   else begin
                       state<=0;
                       if (rx_sync) begin data_o<=shift; valid_o<=1; end
                       else error_o<=1;
                   end
            endcase
        end
    end
endmodule
`default_nettype wire
