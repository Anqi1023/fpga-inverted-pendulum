`timescale 1ns/1ps
module tb_dock_motor_test;
    reg clk=0, reset_n=1, start_n=0, stop_n=1, dir_n=1;
    wire rp,lp,re,le,run_n,rev_n,ready_n;
    integer highs;
    always #18.5185185 clk=~clk;
    dock_motor_test #(.RELEASE_CYCLES(4),.DEBOUNCE_CYCLES(4),.STOP_CYCLES(80)) dut
        (.clk(clk),.reset_key_n(reset_n),.start_key_n(start_n),.stop_key_n(stop_n),
         .direction_key_n(dir_n),.rpwm(rp),.lpwm(lp),.r_en(re),.l_en(le),
         .led_run_n(run_n),.led_reverse_n(rev_n),.led_ready_n(ready_n));
    task step(input integer n); begin repeat(n) @(negedge clk); #1; end endtask
    task quiet; begin if({rp,lp,re,le}!==0 || run_n!==1) $fatal(1,"Not stopped"); end endtask
    task press_start; begin start_n=0; step(15); start_n=1; step(15); end endtask
    task measure(input bit reverse); begin
        step(2800); highs=0;
        repeat(1350) begin step(1); if(rp||lp) highs=highs+1; end
        if(highs!=135 || run_n!==0 || rev_n!==!reverse || ready_n!==1)
            $fatal(1,"PWM/LED mismatch");
    end endtask
    always @(negedge clk) begin
        if(rp&&lp || re!==le) $fatal(1,"Output interlock");
        if(!reset_n && {rp,lp,re,le}!==0) $fatal(1,"Reset output gate");
    end
    initial begin
        step(120); quiet;
        if(ready_n!==0) $fatal(1,"Not ready");
        start_n=1; step(15); press_start; measure(0);
        stop_n=0; step(15); quiet; stop_n=1; step(110);
        dir_n=0; step(15); quiet; dir_n=1; step(110);
        press_start; measure(1);
        // Sub-clock reset assertion must propagate to all four pins.
        #3; reset_n=0; #1; quiet;
        step(3); start_n=0; reset_n=1; step(120); quiet;
        start_n=1; step(15); press_start; measure(0);
        stop_n=0; step(15); quiet;
        $display("HENGXIN_DOCK_MOTOR_TEST_PASS"); $finish;
    end
    initial begin #5000000; $fatal(1,"Timeout"); end
endmodule
