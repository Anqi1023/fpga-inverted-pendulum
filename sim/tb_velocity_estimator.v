`timescale 1ns/1ps
module tb_velocity_estimator;
    reg clk = 0, rst_n = 0;
    reg signed [7:0] position = 0;
    wire signed [7:0] velocity, velocity_fast;
    wire valid, sample_tick, valid_fast, tick_fast;
    integer errors = 0, k, previous, previous_fast, expected, expected_fast;
    initial if ($test$plusargs("vcd")) begin
        $dumpfile("tb_velocity_estimator.vcd"); $dumpvars(0, tb_velocity_estimator);
    end
    always #5 clk = ~clk;
    velocity_estimator #(.WIDTH(8), .SAMPLE_CYCLES(3)) dut (
        .clk(clk), .rst_n(rst_n), .position(position), .velocity(velocity),
        .valid(valid), .sample_tick(sample_tick));
    velocity_estimator #(.WIDTH(8), .SAMPLE_CYCLES(1)) fast (
        .clk(clk), .rst_n(rst_n), .position(position), .velocity(velocity_fast),
        .valid(valid_fast), .sample_tick(tick_fast));
    function integer wrap;
        input integer value;
        begin wrap = value & 255; if (wrap >= 128) wrap = wrap - 256; end
    endfunction
    initial begin
        repeat (2) @(negedge clk);
        if (velocity !== 0 || valid !== 0 || sample_tick !== 0) errors = errors + 1;
        rst_n = 1; previous = 0; previous_fast = 0; expected = 0; expected_fast = 0;
        for (k = 1; k <= 36; k = k + 1) begin
            // Nonzero baseline, positive/negative motion, wrap, then stop.
            if (k <= 12) position = 110 + k*4;
            else if (k <= 24) position = 158 - (k-12)*5;
            else position = 98;
            if (k % 3 == 0) begin
                if (k > 3) expected = wrap(position - previous);
                previous = position;
            end
            if (k > 1) expected_fast = wrap(position - previous_fast);
            previous_fast = position;
            @(negedge clk);
            if (velocity !== expected || valid !== (k >= 6) ||
                sample_tick !== ((k >= 6) && (k % 3 == 0)) ||
                velocity_fast !== expected_fast || valid_fast !== (k >= 2) ||
                tick_fast !== (k >= 2)) begin
                $display("FAIL velocity cycle=%0d got=%0d expected=%0d", k, velocity, expected);
                errors = errors + 1;
            end
        end
        rst_n = 0; #1;
        if (velocity !== 0 || valid !== 0 || velocity_fast !== 0) errors = errors + 1;
        if (errors == 0) $display("PASS tb_velocity_estimator");
        else $display("FAIL tb_velocity_estimator errors=%0d", errors);
        $finish;
    end
endmodule
