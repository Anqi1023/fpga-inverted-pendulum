`timescale 1ns/1ps
module tb_sensor_state;
    reg clk = 0, rst_n = 0;
    reg [1:0] position_ab = 0, angle_ab = 0;
    wire signed [15:0] position, position_vel, angle, angle_vel;
    wire sensor_valid;
    integer errors = 0, i;
    initial if ($test$plusargs("vcd")) begin
        $dumpfile("tb_sensor_state.vcd"); $dumpvars(0, tb_sensor_state);
    end
    always #5 clk = ~clk;
    sensor_state #(.WIDTH(16), .SAMPLE_CYCLES(24)) dut (
        .clk(clk), .rst_n(rst_n),
        .position_enc_a(position_ab[1]), .position_enc_b(position_ab[0]),
        .angle_enc_a(angle_ab[1]), .angle_enc_b(angle_ab[0]),
        .position(position), .position_vel(position_vel),
        .angle(angle), .angle_vel(angle_vel), .sensor_valid(sensor_valid));
    function [1:0] next_phase;
        input [1:0] phase;
        input reverse;
        begin
            case (phase)
                0: next_phase = reverse ? 2 : 1;
                1: next_phase = reverse ? 0 : 3;
                3: next_phase = reverse ? 1 : 2;
                2: next_phase = reverse ? 3 : 0;
            endcase
        end
    endfunction
    task move;
        input reverse;
        begin
            for (i=0; i<24; i=i+1) begin
                @(negedge clk); #2;
                position_ab = next_phase(position_ab, reverse);
                angle_ab = next_phase(angle_ab, !reverse);
                repeat (5) @(negedge clk);
            end
        end
    endtask
    task reset_check;
        begin
            @(negedge clk); rst_n = 0;
            repeat (3) @(negedge clk);
            if (position !== 0 || angle !== 0 || position_vel !== 0 ||
                angle_vel !== 0 || sensor_valid !== 0) errors = errors + 1;
            rst_n = 1;
            repeat (10) @(negedge clk);
            if (sensor_valid !== 0) errors = errors + 1;
            repeat (50) @(negedge clk);
            if (sensor_valid !== 1) errors = errors + 1;
        end
    endtask
    initial begin
        reset_check;
        if (position_vel !== 0 || angle_vel !== 0) errors = errors + 1;
        move(0);
        if (position !== 24 || angle !== -24 || (position_vel > 0) !== 1'b1 ||
            (angle_vel < 0) !== 1'b1 || sensor_valid !== 1) errors = errors + 1;
        move(1);
        if (position !== 0 || angle !== 0 || (position_vel < 0) !== 1'b1 ||
            (angle_vel > 0) !== 1'b1 || sensor_valid !== 1) errors = errors + 1;
        repeat (50) @(negedge clk);
        if (position_vel !== 0 || angle_vel !== 0) errors = errors + 1;
        position_ab = 3; // Illegal 00 -> 11, position must hold.
        repeat (6) @(negedge clk);
        if (position !== 0 || sensor_valid !== 0) errors = errors + 1;
        repeat (50) @(negedge clk);
        if (sensor_valid !== 0) errors = errors + 1; // Fault is sticky.
        reset_check; // Nonzero phase startup must not invent a count.
        if (position !== 0 || angle !== 0) errors = errors + 1;
        angle_ab = 3;
        repeat (6) @(negedge clk);
        if (angle !== 0 || sensor_valid !== 0) errors = errors + 1;
        reset_check;
        // Independent channels: move only the arm, then only the pendulum.
        for (i=0; i<4; i=i+1) begin
            position_ab = next_phase(position_ab, 0);
            repeat (6) @(negedge clk);
        end
        if (position !== 4 || angle !== 0) errors = errors + 1;
        for (i=0; i<4; i=i+1) begin
            angle_ab = next_phase(angle_ab, 1);
            repeat (6) @(negedge clk);
        end
        if (position !== 4 || angle !== -4) errors = errors + 1;
        if (errors == 0) $display("PASS tb_sensor_state");
        else $display("FAIL tb_sensor_state errors=%0d", errors);
        $finish;
    end
endmodule
