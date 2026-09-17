`timescale 1ns/1ps
module button_case #(parameter N=4, parameter ACTIVE=0)(output reg done=0);
    reg clk=0, rst_n=0;
    reg raw=ACTIVE; // Held at power-up must not generate a start event.
    wire pressed, pulse;
    integer events=0;
    reg last_pulse=0;
    always #5 clk=~clk;
    button_input #(.STABLE_CYCLES(N), .ACTIVE_LEVEL(ACTIVE)) dut
        (.clk(clk), .rst_n(rst_n), .button_raw(raw), .pressed(pressed), .press_pulse(pulse));
    always @(negedge clk) begin
        if (pulse && last_pulse) $fatal(1,"Pulse longer than one cycle");
        if (pulse) events=events+1;
        last_pulse=pulse;
    end
    task settle; begin repeat(N+6) @(negedge clk); #1; end endtask
    task drive(input bit down); begin
        @(negedge clk); #2; raw=down ? ACTIVE : 1-ACTIVE;
    end endtask
    task expect_state(input integer total, input bit down); begin
        if(events!=total || pressed!==down)
            $fatal(1,"N=%0d active=%0d events=%0d expected=%0d pressed=%b",N,ACTIVE,events,total,pressed);
    end endtask
    initial begin
        settle; rst_n=1; settle; expect_state(0,1);
        drive(0); settle; expect_state(0,0);
        if(N>1) begin
            // One-clock pulses and bouncing must not become a press.
            repeat(5) begin drive(1); drive(0); end
            settle; expect_state(0,0);
        end
        drive(1); settle; expect_state(1,1);
        settle; expect_state(1,1); // Long hold does not repeat.
        if(N>1) begin
            repeat(5) begin drive(0); drive(1); end
            settle; expect_state(1,1);
        end
        drive(0); settle; expect_state(1,0);
        drive(1); settle; expect_state(2,1);
        rst_n=0; settle; expect_state(2,0);
        rst_n=1; settle; expect_state(2,1); // Still held across reset.
        drive(0); settle; drive(1); settle; expect_state(3,1);
        $display("BUTTON_CASE_PASS N=%0d ACTIVE=%0d",N,ACTIVE);
        done=1;
    end
endmodule

module tb_button_input;
    wire [3:0] done;
    button_case #(.N(4),.ACTIVE(0)) a(done[0]);
    button_case #(.N(4),.ACTIVE(1)) b(done[1]);
    button_case #(.N(1),.ACTIVE(0)) c(done[2]);
    button_case #(.N(1),.ACTIVE(1)) d(done[3]);
    initial begin wait(&done); $display("HENGXIN_BUTTON_INPUT_PASS"); $finish; end
    initial begin #100000; $fatal(1,"Timeout"); end
endmodule
