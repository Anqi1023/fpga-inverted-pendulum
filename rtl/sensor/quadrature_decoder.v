`timescale 1ns/1ps
// Verilog-2001: asynchronous A/B input, x4 quadrature count.
// Positive sequence {A,B}: 00 -> 01 -> 11 -> 10 -> 00.
// COUNT_WIDTH >= 2. Count wraps modulo 2**COUNT_WIDTH.
// rst_n: asynchronous assertion; release synchronously to clk at board level.
// Hold inputs still during startup. Two-flop CDC is not a debounce filter;
// every quadrature state must remain long enough to be sampled reliably.
module quadrature_decoder #(
    parameter integer COUNT_WIDTH = 32
) (
    input wire clk, input wire rst_n,
    input wire enc_a, input wire enc_b,
    output reg signed [COUNT_WIDTH-1:0] position_count,
    output reg ready,
    output reg illegal_transition
);
    reg [1:0] ab_meta, ab_sync, ab_previous;
    reg [1:0] startup;
    localparam [COUNT_WIDTH-1:0] ONE = {{(COUNT_WIDTH-1){1'b0}}, 1'b1};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_meta <= 0;
            ab_sync <= 0;
            ab_previous <= 0;
            startup <= 0;
            position_count <= 0;
            ready <= 0;
            illegal_transition <= 0;
        end else begin
            ab_meta <= {enc_a, enc_b};
            ab_sync <= ab_meta;
            illegal_transition <= 0;
            if (!ready) begin
                // Capture actual initial phase; do not count reset's artificial 00.
                if (startup == 2) begin
                    ab_previous <= ab_sync;
                    ready <= 1;
                end else startup <= startup + 1'b1;
            end else begin
                ab_previous <= ab_sync;
                case ({ab_previous, ab_sync})
                    4'b0001, 4'b0111, 4'b1110, 4'b1000:
                        position_count <= position_count + ONE;
                    4'b0010, 4'b1011, 4'b1101, 4'b0100:
                        position_count <= position_count - ONE;
                    4'b0011, 4'b1100, 4'b0110, 4'b1001:
                        // Both bits changed: discard delta, resynchronize phase.
                        illegal_transition <= 1;
                    default: ; // Unchanged phase: hold count.
                endcase
            end
        end
    end
endmodule
