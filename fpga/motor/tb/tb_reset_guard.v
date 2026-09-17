`timescale 1ns/1ps
module reset_case #(parameter N=4)(output reg done=0);
    reg clk=0, clock_on=1, ext_n=1;
    wire rst_n;
    always #5 if(clock_on) clk=~clk;
    reset_guard #(.RELEASE_CYCLES(N)) dut(.clk(clk),.external_reset_n(ext_n),.reset_n(rst_n));
    task check_release; integer i; begin
        for(i=1;i<=N+2;i=i+1) begin
            @(posedge clk); #1;
            if(rst_n !== (i==N+2)) $fatal(1,"Release timing N=%0d cycle=%0d",N,i);
        end
    end endtask
    task assert_reset; begin
        @(negedge clk); #2; ext_n=0; #1;
        if(rst_n!==0) $fatal(1,"Reset did not assert between clocks");
    end endtask
    initial begin
        #1; if(rst_n!==0) $fatal(1,"Power-up reset missing");
        check_release;
        assert_reset; #1; ext_n=1; check_release;
        // A short release followed by a new reset must restart the whole delay.
        assert_reset; #1; ext_n=1;
        @(posedge clk); #1; ext_n=0; #1;
        if(rst_n!==0) $fatal(1,"Bounce escaped reset");
        @(negedge clk); #2; ext_n=1; check_release;
        // Assertion does not need a running clock; release does.
        @(negedge clk); #1; clock_on=0; ext_n=0; #1;
        if(rst_n!==0) $fatal(1,"Stopped clock assertion failed");
        ext_n=1; #30;
        if(rst_n!==0) $fatal(1,"Released without clock");
        clock_on=1; check_release;
        $display("RESET_CASE_PASS N=%0d",N); done=1;
    end
endmodule
module tb_reset_guard;
    wire a,b;
    reset_case #(.N(1)) c1(a);
    reset_case #(.N(4)) c4(b);
    initial begin wait(a&&b); $display("HENGXIN_RESET_GUARD_PASS"); $finish; end
    initial begin #10000; $fatal(1,"Timeout"); end
endmodule
