`timescale 1ns/1ps
module tb_protection_board;
    reg clk=0, reset_n=0, start_n=0, estop_n=1, fault_n=1;
    wire rp,lp,re,le,run_n,locked_n,ready_n;
    integer ticks, starts=0, before_count;
    reg last_en=0;
    always #5 clk=~clk;
    dock_motor_protection #(.CLK_HZ(100000),.PWM_HZ(1000),.RELEASE_CYCLES(3),
        .DEBOUNCE_CYCLES(3),.STOP_CYCLES(50),.RUN_CYCLES(1000)) dut
        (clk,reset_n,start_n,estop_n,fault_n,rp,lp,re,le,run_n,locked_n,ready_n);
    task step(input integer n); repeat(n) @(negedge clk); endtask
    task quiet; #1; if({rp,lp,re,le}!==0 || !run_n) $fatal(1,"Outputs not quiet"); endtask
    task press_s1;
        start_n=1; step(15); start_n=0; step(12); start_n=1; step(15);
    endtask
    task clear_ready;
        start_n=1; step(15); press_s1; step(70); quiet;
        if(!locked_n || ready_n) $fatal(1,"Clear did not reach ready without running");
    endtask
    task wait_high; while(!rp) @(negedge clk); endtask
    task locked_quiet;
        quiet;
        if(locked_n || !ready_n) $fatal(1,"Wrong lock LEDs");
    endtask
    always @(negedge clk) begin
        #2;
        if(lp || re!==le || (rp && !re)) $fatal(1,"Output interlock failed");
        if(re && !last_en) starts=starts+1;
        last_en=re;
    end
    initial begin
        step(3); reset_n=1; step(100); locked_quiet;
        if(starts!=0) $fatal(1,"Held S1 at boot activated motor");
        clear_ready;
        // Hold S1: one bounded run, followed by ready, with no repeat.
        start_n=0; wait(re); @(negedge clk); #1; ticks=0;
        while(re) begin ticks=ticks+1; step(1); end
        if(ticks!=1000) $fatal(1,"Wrong bounded duration: %0d",ticks);
        step(1200); quiet;
        if(starts!=1 || ready_n || !locked_n) $fatal(1,"Held start repeated or timeout locked");
        start_n=1; step(15);
        $display("BOARD_BOOT_CLEAR_TIMEOUT_HELD_START_PASS");

        press_s1; wait_high;
        estop_n=0; step(2); quiet; step(2); locked_quiet;
        before_count=starts; press_s1; step(20); locked_quiet;
        if(starts!=before_count) $fatal(1,"S1 defeated held S2");
        // Hold S1 across emergency release: cannot clear or restart later.
        start_n=0; step(15); estop_n=1; step(1200); locked_quiet;
        start_n=1; step(15); locked_quiet;
        clear_ready;
        if(starts!=before_count) $fatal(1,"Clear also started motor");
        press_s1; wait_high;
        $display("BOARD_S2_ESTOP_LATCH_NO_RESTART_PASS");

        fault_n=0; step(2); quiet; step(2); locked_quiet;
        press_s1; fault_n=1; step(100); locked_quiet;
        clear_ready; press_s1; wait_high;
        // Simultaneous requests are remembered after both keys are released.
        fault_n=0; estop_n=0; step(4); locked_quiet;
        fault_n=1; step(10); press_s1; locked_quiet;
        estop_n=1; step(100); locked_quiet;
        clear_ready; press_s1; wait_high;
        $display("BOARD_S3_FAULT_AND_SIMULTANEOUS_TRIP_PASS");

        reset_n=0; step(1); quiet; step(5);
        start_n=0; reset_n=1; step(150); locked_quiet;
        start_n=1; step(20); locked_quiet;
        clear_ready; press_s1; wait_high;
        // A human-key glitch long enough to cross the synchronizer is latched.
        estop_n=0; step(4); estop_n=1; step(100); locked_quiet;
        $display("PROTECTION_BOARD_CONTROLS_PASS");
        $finish;
    end
    initial begin #200000; $fatal(1,"Board controls timeout"); end
endmodule
