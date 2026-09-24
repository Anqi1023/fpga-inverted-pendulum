`timescale 1ns/1ps
// Verilog-2001: delta per SAMPLE_CYCLES clocks, NOT counts/second.
// WIDTH >= 2, SAMPLE_CYCLES >= 1. First sample establishes a baseline.
// Modular subtraction handles counter wrap if |delta| < 2**(WIDTH-1).
// rst_n may be held low until the upstream acquisition is initialized.
module velocity_estimator #(
    parameter integer WIDTH = 32,
    parameter integer SAMPLE_CYCLES = 100000
) (
    input wire clk, input wire rst_n,
    input wire signed [WIDTH-1:0] position,
    output reg signed [WIDTH-1:0] velocity,
    output reg valid,
    output reg sample_tick
);
    function integer bits_needed;
        input integer value;
        integer tmp;
        begin
            tmp = value - 1;
            bits_needed = 0;
            while (tmp > 0) begin
                bits_needed = bits_needed + 1;
                tmp = tmp >> 1;
            end
            if (bits_needed < 1) bits_needed = 1;
        end
    endfunction
    localparam integer TIMER_WIDTH = bits_needed(SAMPLE_CYCLES);
    reg [TIMER_WIDTH-1:0] timer;
    reg signed [WIDTH-1:0] previous_position;
    reg baseline_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer <= 0;
            previous_position <= 0;
            baseline_valid <= 0;
            velocity <= 0;
            valid <= 0;
            sample_tick <= 0;
        end else begin
            sample_tick <= 0;
            if (timer == SAMPLE_CYCLES - 1) begin
                timer <= 0;
                previous_position <= position;
                baseline_valid <= 1;
                if (baseline_valid) begin
                    velocity <= position - previous_position;
                    valid <= 1;
                    sample_tick <= 1;
                end
            end else timer <= timer + 1'b1;
        end
    end
endmodule
