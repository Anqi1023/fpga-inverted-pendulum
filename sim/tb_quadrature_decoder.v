`timescale 1ns/1ps
module tb_quadrature_decoder;
    reg clk = 0, rst_n = 0;
    reg [1:0] ab = 0;
    wire signed [7:0] position_count;
    wire ready, illegal_transition;
    integer errors = 0, illegal_seen = 0, i, j;
    initial if ($test$plusargs("vcd")) begin
        $dumpfile("tb_quadrature_decoder.vcd"); $dumpvars(0, tb_quadrature_decoder);
    end
    always #5 clk = ~clk;
    quadrature_decoder #(.COUNT_WIDTH(8)) dut (
        .clk(clk), .rst_n(rst_n), .enc_a(ab[1]), .enc_b(ab[0]),
        .position_count(position_count), .ready(ready),
        .illegal_transition(illegal_transition));
    always @(negedge clk) if (illegal_transition) illegal_seen = illegal_seen + 1;
    task drive;
        input [1:0] next_ab;
        begin @(negedge clk); #2 ab = next_ab; repeat (5) @(negedge clk); end
    endtask
    task check;
        input integer expected;
        begin
            if (position_count !== expected) begin
                $display("FAIL count=%0d expected=%0d", position_count, expected);
                errors = errors + 1;
            end
        end
    endtask
    task reset_at;
        input [1:0] phase;
        begin
            @(negedge clk); rst_n = 0; ab = phase;
            repeat (3) @(negedge clk);
            check(0);
            if (ready !== 0) errors = errors + 1;
            rst_n = 1;
            repeat (6) @(negedge clk);
            check(0);
            if (ready !== 1) errors = errors + 1;
        end
    endtask
    initial begin
        // All 16 old/new combinations, including all four illegal jumps.
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                reset_at(i);
                illegal_seen = 0;
                drive(j);
                case (i*4+j)
                    1, 7, 14, 8: check(1);
                    2, 11, 13, 4: check(-1);
                    default: check(0);
                endcase
                if ((i ^ j) == 3) begin
                    if (illegal_seen != 1) errors = errors + 1;
                end else if (illegal_seen != 0) errors = errors + 1;
            end
        end
        reset_at(0);
        repeat (10) begin drive(1); drive(3); drive(2); drive(0); end
        check(40);
        repeat (20) begin drive(2); drive(3); drive(1); drive(0); end
        check(-40);
        drive(0); check(-40);
        reset_at(0);
        repeat (32) begin drive(1); drive(3); drive(2); drive(0); end
        check(-128); // 8-bit positive overflow wraps.
        drive(2); check(127); // Reverse across wrap.
        reset_at(0);
        drive(3); check(0); drive(2); check(1); // Recover phase after illegal jump.
        if (errors == 0) $display("PASS tb_quadrature_decoder");
        else $display("FAIL tb_quadrature_decoder errors=%0d", errors);
        $finish;
    end
endmodule
