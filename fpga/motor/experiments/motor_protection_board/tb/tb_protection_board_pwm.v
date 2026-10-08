`timescale 1ns/1ps
module tb_protection_board_pwm;
    reg clk=0, reset_n=0, start_n=1, estop_n=1, fault_n=1;
    wire rp,lp,re,le,run_n,locked_n,ready_n;
    integer ticks, highs, rises, width, last_rise;
    reg last_pwm;
    always #18.5185185 clk=~clk;
    // Actual 27 MHz / 1 kHz; shorten run timeout to 3 ms and release/STOP waits.
    dock_motor_protection #(.RELEASE_CYCLES(3),.DEBOUNCE_CYCLES(3),
        .STOP_CYCLES(20),.RUN_CYCLES(81000)) dut
        (clk,reset_n,start_n,estop_n,fault_n,rp,lp,re,le,run_n,locked_n,ready_n);
    // Check shipped defaults separately, kept in reset (no 5-second simulation).
    dock_motor_protection defaults (.clk(clk),.reset_key_n(1'b0),.start_key_n(1'b1),
        .estop_key_n(1'b1),.fault_key_n(1'b1),.rpwm(),.lpwm(),.r_en(),.l_en(),
        .led_run_n(),.led_locked_n(),.led_ready_n());
    task step(input integer n); repeat(n) @(negedge clk); endtask
    initial begin
        if(defaults.CLK_HZ!=27000000 || defaults.PWM_HZ!=1000 ||
           defaults.RUN_CYCLES!=135000000 || defaults.STOP_CYCLES!=27000000)
            $fatal(1,"Unexpected shipped defaults");
        step(5); reset_n=1; step(50);
        start_n=0; step(15); start_n=1; step(50);
        if(ready_n || !locked_n || re) $fatal(1,"Failed to clear to ready");
        start_n=0; wait(re); @(negedge clk); #1;
        ticks=0; highs=0; rises=0; width=0; last_rise=-1; last_pwm=0;
        while(re) begin
            if(lp || !le || run_n) $fatal(1,"Wrong running outputs");
            if(rp) highs=highs+1;
            if(rp && !last_pwm) begin
                if(last_rise>=0 && ticks-last_rise!=27000) $fatal(1,"Wrong period");
                rises=rises+1; last_rise=ticks; width=0;
            end
            if(rp) width=width+1;
            if(!rp && last_pwm && width!=2700) $fatal(1,"Wrong 10-percent pulse width");
            last_pwm=rp; ticks=ticks+1; step(1);
        end
        if(ticks!=81000 || rises!=2 || highs!=5400)
            $fatal(1,"Duration/PWM count: %0d %0d %0d",ticks,rises,highs);
        step(100);
        if(rp || lp || re || le || ready_n || !locked_n) $fatal(1,"Timeout/held-start failed");
        $display("PROTECTION_BOARD_1KHZ_PWM_PASS");
        $finish;
    end
    initial begin #10000000; $fatal(1,"Board PWM timeout"); end
endmodule
