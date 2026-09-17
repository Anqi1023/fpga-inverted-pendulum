`timescale 1ns/1ps
module tb_ibt2_adapter(output reg done=0);
    reg clk=0,rst_n=0,pwm=1,dir=0,en=1;
    wire rp,lp,re,le;
    ibt2_adapter dut(.clk(clk),.rst_n(rst_n),.pwm(pwm),.dir(dir),.en(en),
        .rpwm(rp),.lpwm(lp),.r_en(re),.l_en(le));
    always #5 clk=~clk;
    task check; input [3:0] expected; begin
        #1; if({rp,lp,re,le} !== expected) $fatal(1,"IBT pins mismatch");
    end endtask
    initial begin
        check(0); @(negedge clk); rst_n=1;
        @(posedge clk); check(4'b1011);
        // Deliberately violate upstream protocol: change direction under PWM.
        @(negedge clk); dir=1; check(0);
        @(posedge clk); check(0); // full blank interval on capture
        @(posedge clk); check(4'b0111);
        @(negedge clk); en=0; check(0);
        @(posedge clk); check(0);
        @(negedge clk); en=1;
        @(posedge clk); check(4'b0111);
        @(negedge clk); rst_n=0; check(0);
        @(posedge clk); check(0);
        $display("HENGXIN_IBT2_ADAPTER_PASS"); done=1;
    end
    initial begin #1000; if(!done) $fatal(1,"Adapter timeout"); end
endmodule
