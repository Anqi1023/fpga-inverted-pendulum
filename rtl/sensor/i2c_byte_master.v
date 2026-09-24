`timescale 1ns/1ps
// Byte-oriented open-drain I2C engine. The bus must have external pull-ups.
// Commands: 1 START, 2 STOP, 3 WRITE byte, 4 READ byte.
module i2c_byte_master #(
    parameter integer HALF_PERIOD_CYCLES = 135
) (
    input wire clk, input wire rst_n,
    input wire cmd_valid, input wire [2:0] cmd,
    input wire [7:0] write_data, input wire read_send_ack,
    output wire busy, output reg done, output reg ack_received,
    output reg [7:0] read_data, inout wire scl, inout wire sda
);
    localparam [2:0] CMD_START=3'd1, CMD_STOP=3'd2, CMD_WRITE=3'd3, CMD_READ=3'd4;
    localparam [4:0] IDLE=0, START_A=1, START_B=2, START_C=3,
        STOP_A=4, STOP_B=5, STOP_C=6, W_SETUP=7, W_HIGH=8, W_LOW=9,
        W_ACK_HIGH=10, W_ACK_LOW=11, R_HIGH=12, R_LOW=13,
        R_ACK_SETUP=14, R_ACK_HIGH=15, R_ACK_LOW=16, DONE_STATE=17;
    reg [4:0] state; reg [15:0] timer; reg [2:0] bit_index;
    reg [7:0] shift; reg scl_low, sda_low, read_ack_latched;
    assign scl=scl_low?1'b0:1'bz; assign sda=sda_low?1'b0:1'bz;
    assign busy=(state!=IDLE);
    function tick_done; input [15:0] count; begin tick_done=(count==HALF_PERIOD_CYCLES-1); end endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin state<=IDLE;timer<=0;bit_index<=0;shift<=0;scl_low<=0;sda_low<=0;
            done<=0;ack_received<=0;read_data<=0;read_ack_latched<=0; end
        else begin
            done<=0;
            case(state)
                IDLE: begin timer<=0; if(cmd_valid) case(cmd)
                    CMD_START: begin scl_low<=0;sda_low<=0;state<=START_A;end
                    CMD_STOP: begin scl_low<=1;sda_low<=1;state<=STOP_A;end
                    CMD_WRITE: begin scl_low<=1;shift<=write_data;bit_index<=7;sda_low<=~write_data[7];state<=W_SETUP;end
                    CMD_READ: begin scl_low<=1;sda_low<=0;bit_index<=7;shift<=0;read_ack_latched<=read_send_ack;state<=R_HIGH;end
                    default:; endcase end
                START_A: if(tick_done(timer))begin timer<=0;sda_low<=1;state<=START_B;end else timer<=timer+1'b1;
                START_B: if(tick_done(timer))begin timer<=0;scl_low<=1;state<=START_C;end else timer<=timer+1'b1;
                START_C: if(tick_done(timer))begin timer<=0;done<=1;state<=DONE_STATE;end else timer<=timer+1'b1;
                STOP_A: if(tick_done(timer))begin timer<=0;scl_low<=0;state<=STOP_B;end else timer<=timer+1'b1;
                STOP_B: if(tick_done(timer))begin timer<=0;sda_low<=0;state<=STOP_C;end else timer<=timer+1'b1;
                STOP_C: if(tick_done(timer))begin timer<=0;done<=1;state<=DONE_STATE;end else timer<=timer+1'b1;
                W_SETUP: if(tick_done(timer))begin timer<=0;scl_low<=0;state<=W_HIGH;end else timer<=timer+1'b1;
                W_HIGH: if(tick_done(timer))begin timer<=0;scl_low<=1;state<=W_LOW;end else timer<=timer+1'b1;
                W_LOW: if(tick_done(timer))begin timer<=0;if(bit_index==0)begin sda_low<=0;state<=W_ACK_HIGH;end
                    else begin bit_index<=bit_index-1'b1;sda_low<=~shift[bit_index-1'b1];state<=W_SETUP;end end else timer<=timer+1'b1;
                W_ACK_HIGH: if(tick_done(timer))begin timer<=0;scl_low<=0;state<=W_ACK_LOW;end else timer<=timer+1'b1;
                W_ACK_LOW: if(tick_done(timer))begin timer<=0;ack_received<=~sda;scl_low<=1;done<=1;state<=DONE_STATE;end else timer<=timer+1'b1;
                R_HIGH: if(tick_done(timer))begin timer<=0;scl_low<=0;state<=R_LOW;end else timer<=timer+1'b1;
                R_LOW: if(tick_done(timer))begin timer<=0;shift[bit_index]<=sda;scl_low<=1;
                    if(bit_index==0)state<=R_ACK_SETUP;else begin bit_index<=bit_index-1'b1;state<=R_HIGH;end end else timer<=timer+1'b1;
                R_ACK_SETUP: if(tick_done(timer))begin timer<=0;sda_low<=read_ack_latched;read_data<=shift;state<=R_ACK_HIGH;end else timer<=timer+1'b1;
                R_ACK_HIGH: if(tick_done(timer))begin timer<=0;scl_low<=0;state<=R_ACK_LOW;end else timer<=timer+1'b1;
                R_ACK_LOW: if(tick_done(timer))begin timer<=0;scl_low<=1;sda_low<=0;read_data<=shift;done<=1;state<=DONE_STATE;end else timer<=timer+1'b1;
                DONE_STATE: begin done<=0;state<=IDLE;end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
