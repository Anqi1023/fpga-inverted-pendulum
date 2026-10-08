`timescale 1ns/1ps
module tb_jog_actual;
    reg clk=0, reset_n=0, start_n=1, stop_n=1, direction_n=1;
    always #18.5185185 clk=~clk;
    wire rp,lp,re,le,run_n,rev_n,ready_n;
    dock_motor_jog #(.RELEASE_CYCLES(3),.DEBOUNCE_CYCLES(3)) dut
        (clk,reset_n,start_n,stop_n,direction_n,rp,lp,re,le,run_n,rev_n,ready_n);
    integer ticks, highs, rises, last_rise, width;
    reg last_pwm;
    task step(input integer n); repeat(n) begin @(posedge clk); #1; end endtask
    initial begin #250000000; $fatal(1,"Actual timing test timeout"); end
    initial begin
        if(dut.CLK_HZ!=27000000 || dut.PWM_HZ!=1000 || dut.DUTY_PERCENT!=8 ||
           dut.JOG_CYCLES!=5400000 || dut.LOCK_CYCLES!=27000000)
            $fatal(1,"Unexpected hardware defaults");
        step(5); reset_n=1; step(30);
        @(negedge clk); start_n=0;
        wait(re); #1;
        ticks=0; highs=0; rises=0; last_rise=-1; width=0; last_pwm=0;
        while(re) begin
            if(lp || !le || run_n || !ready_n) $fatal(1,"Wrong active output/LED");
            if(rp) highs=highs+1;
            if(rp && !last_pwm) begin
                if(last_rise>=0 && ticks-last_rise!=27000) $fatal(1,"Wrong PWM period");
                rises=rises+1; last_rise=ticks; width=0;
            end
            if(rp) width=width+1;
            if(!rp && last_pwm && width!=2160) $fatal(1,"Wrong PWM width");
            last_pwm=rp; ticks=ticks+1; step(1);
        end
        if(ticks!=5400000 || highs!=429840 || rises!=199)
            $fatal(1,"Jog count mismatch ticks=%0d highs=%0d rises=%0d",ticks,highs,rises);
        if(rp || lp || le || !run_n || !ready_n || dut.cooldown!=27000000)
            $fatal(1,"Auto-stop/cooldown failed");
        step(100);
        if(re || rp || lp) $fatal(1,"Held start restarted");
        $display("JOG_ACTUAL_PASS: 1 kHz, 8%%, 5400000-clock/200-ms run, automatic stop, 1-s cooldown loaded");
        $finish;
    end
endmodule
