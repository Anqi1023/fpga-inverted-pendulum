`timescale 1ns/1ps
module tb_control_board_pwm;
    reg clk=0,reset_n=0,start_n=1;
    wire rp,lp,ibt_en,run_n,locked_n,ready_n;
    wire re=ibt_en;
    wire le=dut.l_en;
    integer ticks,highs,rises,width,last_rise,valids,cycles,last_valid;
    reg last_pwm,last_v;
    always #18.5185185 clk=~clk;
    dock_motor_control_v1 #(.RELEASE_CYCLES(3),.DEBOUNCE_CYCLES(3),.STOP_CYCLES(20)) dut
        (clk,reset_n,start_n,1'b1,1'b1,rp,lp,ibt_en,run_n,locked_n,ready_n);
    dock_motor_control_v1 defaults (.clk(clk),.reset_key_n(1'b0),.start_key_n(1'b1),
        .estop_key_n(1'b1),.fault_key_n(1'b1),.rpwm(),.lpwm(),.ibt_en(),
        .led_run_n(),.led_locked_n(),.led_ready_n());
    task step(input integer n); begin repeat(n) @(negedge clk); #1; end endtask
    always @(negedge clk) begin
        #0.5;
        if(ibt_en !== (dut.r_en && dut.l_en) || dut.r_en !== dut.l_en)
            $fatal(1,"Shared enable lost a core safety signal");
        #2;
        if(reset_n) begin
            cycles=cycles+1;
            if(dut.control_valid) begin
                if(last_v || dut.control_u!=100) $fatal(1,"Wrong mock C command");
                if(last_valid>=0 && cycles-last_valid!=540000) $fatal(1,"Wrong 20-ms update interval");
                last_valid=cycles; valids=valids+1;
            end
            last_v=dut.control_valid;
        end
    end
    initial begin
        cycles=0; last_valid=-1; valids=0; last_v=0;
        if(defaults.RUN_CYCLES!=5400000 || defaults.CONTROL_CYCLES!=540000 ||
           defaults.TIMEOUT_CYCLES!=2700000 || defaults.STOP_CYCLES!=27000000 ||
           defaults.RELEASE_CYCLES!=270000 || defaults.DEBOUNCE_CYCLES!=270000)
            $fatal(1,"Unexpected board defaults");
        step(5); reset_n=1; step(60);
        start_n=0; step(15); start_n=1; step(30);
        if(ready_n || !locked_n || re) $fatal(1,"Clear should not start motor");
        start_n=0; wait(re); @(negedge clk); #1;
        ticks=0; highs=0; rises=0; width=0; last_rise=-1; last_pwm=0;
        while(re) begin
            if(lp || !le || run_n) $fatal(1,"Bad board outputs");
            if(rp) highs=highs+1;
            if(rp && !last_pwm) begin
                if(last_rise>=0 && ticks-last_rise!=27000) $fatal(1,"Bad PWM period");
                rises=rises+1; last_rise=ticks; width=0;
            end
            if(rp) width=width+1;
            if(!rp && last_pwm && width!=2700) $fatal(1,"Bad PWM width");
            last_pwm=rp; ticks=ticks+1; step(1);
        end
        if(ticks!=5400000 || rises!=199 || highs!=537300)
            $fatal(1,"Duration/pulses: %0d %0d %0d",ticks,rises,highs);
        step(30000);
        if(rp || lp || re || le || ready_n || !locked_n || valids<10)
            $fatal(1,"Timeout, heartbeat or held-S1 failure");
        $display("CONTROL_BOARD_200MS_PASS ticks=%0d pulses=%0d highs=%0d",ticks,rises,highs);
        $finish;
    end
    initial begin #220000000; $fatal(1,"Board PWM test timed out"); end
endmodule
