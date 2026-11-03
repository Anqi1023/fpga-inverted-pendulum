`timescale 1ns/1ps
`default_nettype none
// ADS1115 address 0x48.
// Config 0xC2E3: AIN0-GND, +/-4.096V, continuous conversion,
// 860 SPS, comparator disabled.
module ads1115_reader #(
    parameter integer SAMPLE_INTERVAL_CYCLES = 33750,
    parameter integer I2C_HALF_PERIOD_CYCLES = 135
) (
    input wire clk,
    input wire rst_n,
inout wire scl,
    inout wire sda,
output reg signed [15:0] sample,
    output reg sample_valid,
    output reg ready,
    output reg fault
);
localparam [4:0]
        INIT_START   = 0,
        INIT_ADDR    = 1,
        INIT_PTR     = 2,
        INIT_HI      = 3,
        INIT_LO      = 4,
        INIT_STOP    = 5,
        WAIT_SAMPLE  = 6,
        READ_START   = 7,
        READ_ADDR_W  = 8,
        READ_PTR     = 9,
        READ_RESTART = 10,
        READ_ADDR_R  = 11,
        READ_HI      = 12,
        READ_LO      = 13,
        READ_STOP    = 14,
        ERROR_STOP   = 15,
        RETRY_WAIT   = 16;
localparam [2:0]
        CMD_START = 3'd1,
        CMD_STOP  = 3'd2,
        CMD_WRITE = 3'd3,
        CMD_READ  = 3'd4;
reg [4:0] state;
    reg [31:0] sample_timer;
    reg [7:0] high_byte;
wire ll_busy;
    wire ll_done;
    wire ll_error;
    wire ll_ack;
    wire [7:0] ll_read;
reg [2:0] ll_cmd;
    reg [7:0] ll_write;
    reg ll_read_ack;
    reg ll_cmd_valid;
i2c_byte_master #(
        .HALF_PERIOD_CYCLES(I2C_HALF_PERIOD_CYCLES),
        .COMMAND_TIMEOUT_CYCLES(
            I2C_HALF_PERIOD_CYCLES * 64
        )
    ) engine (
        .clk(clk),
        .rst_n(rst_n),
.cmd_valid(ll_cmd_valid),
        .cmd(ll_cmd),
        .write_data(ll_write),
        .read_send_ack(ll_read_ack),
.busy(ll_busy),
        .done(ll_done),
        .error(ll_error),
        .ack_received(ll_ack),
        .read_data(ll_read),
.scl(scl),
        .sda(sda)
    );
// =====================================================
    // Command decoder
    // =====================================================
always @* begin
        ll_cmd_valid = 1'b0;
        ll_cmd = CMD_START;
        ll_write = 8'h00;
        ll_read_ack = 1'b0;
        case (state)
INIT_START,
            READ_START,
            READ_RESTART: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_START;
            end
INIT_STOP,
            READ_STOP,
            ERROR_STOP: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_STOP;
            end
INIT_ADDR,
            READ_ADDR_W: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_WRITE;
                ll_write = 8'h90;
            end
INIT_PTR: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_WRITE;
                ll_write = 8'h01;
            end
INIT_HI: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_WRITE;
                ll_write = 8'hC2;
            end
INIT_LO: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_WRITE;
                ll_write = 8'hE3;
            end
READ_PTR: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_WRITE;
                ll_write = 8'h00;
            end
READ_ADDR_R: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_WRITE;
                ll_write = 8'h91;
            end
READ_HI: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_READ;
                ll_read_ack = 1'b1;
            end
READ_LO: begin
                ll_cmd_valid = 1'b1;
                ll_cmd = CMD_READ;
                ll_read_ack = 1'b0;
            end
default: begin
                ll_cmd_valid = 1'b0;
            end
endcase
    end
// =====================================================
    // Identify states that transmit bytes and require ACK
    // =====================================================
function is_write_state;
        input [4:0] s;
        begin
            is_write_state =
                (s == INIT_ADDR)   ||
                (s == INIT_PTR)    ||
                (s == INIT_HI)     ||
                (s == INIT_LO)     ||
                (s == READ_ADDR_W) ||
                (s == READ_PTR)    ||
                (s == READ_ADDR_R);
        end
    endfunction
// =====================================================
    // ADC state machine
    // =====================================================
always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= INIT_START;
sample_timer <= 0;
            high_byte <= 0;
sample <= 0;
            sample_valid <= 0;
ready <= 0;
            fault <= 0;
        end
        else begin
            sample_valid <= 0;
if (state == WAIT_SAMPLE) begin
                // Normal sample interval.
                if (sample_timer >= SAMPLE_INTERVAL_CYCLES - 1) begin
                    sample_timer <= 0;
                    state <= READ_START;
                end
                else begin
                    sample_timer <= sample_timer + 1'b1;
                end
            end
            else if (state == RETRY_WAIT) begin
                // Delay before retrying initialization.
                if (sample_timer >= SAMPLE_INTERVAL_CYCLES - 1) begin
                    sample_timer <= 0;
                    state <= INIT_START;
                end
                else begin
                    sample_timer <= sample_timer + 1'b1;
                end
            end
            else if (ll_done) begin
                sample_timer <= 0;
// A failed STOP still ends the failed transaction.
                // The engine has already released the bus on timeout.
                if (state == ERROR_STOP) begin
                    state <= RETRY_WAIT;
                end
// Timeout or NACK in any other transaction.
                else if (
                    ll_error ||
                    (is_write_state(state) && !ll_ack)
                ) begin
                    fault <= 1'b1;
                    ready <= 1'b0;
                    state <= ERROR_STOP;
                end
                else begin
                    case (state)
INIT_START:
                            state <= INIT_ADDR;
INIT_ADDR:
                            state <= INIT_PTR;
INIT_PTR:
                            state <= INIT_HI;
INIT_HI:
                            state <= INIT_LO;
INIT_LO:
                            state <= INIT_STOP;
INIT_STOP: begin
                            ready <= 1'b1;
                            fault <= 1'b0;
                            sample_timer <= 0;
                            state <= WAIT_SAMPLE;
                        end
READ_START:
                            state <= READ_ADDR_W;
READ_ADDR_W:
                            state <= READ_PTR;
READ_PTR:
                            state <= READ_RESTART;
READ_RESTART:
                            state <= READ_ADDR_R;
READ_ADDR_R:
                            state <= READ_HI;
READ_HI: begin
                            high_byte <= ll_read;
                            state <= READ_LO;
                        end
READ_LO:
                            state <= READ_STOP;
READ_STOP: begin
                            sample <= {high_byte, ll_read};
                            sample_valid <= 1'b1;
                            state <= WAIT_SAMPLE;
                        end
default: begin
                            fault <= 1'b1;
                            ready <= 1'b0;
                            state <= ERROR_STOP;
                        end
endcase
                end
            end
            else begin
                sample_timer <= 0;
            end
        end
    end
// synthesis translate_off
    initial begin
        if (SAMPLE_INTERVAL_CYCLES < 1)
            $error("SAMPLE_INTERVAL_CYCLES must be >= 1");
    end
    // synthesis translate_on
endmodule
`default_nettype wire