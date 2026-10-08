`timescale 1ns/1ps
module tb_motor_protected;
    reg clk=0, rst_n=0, arm=1, permit_in=1, fault=0, estop=0, clear_fault=1;
    reg [1:0] command=1;
    reg [6:0] duty=10;
    wire rp,lp,re,le,dir,permit,locked,fl,el,accepted;
    wire [2:0] state;
    integer high_count, i;
    always #18.5185185 clk=~clk;
    // Use real 27 MHz / 1 kHz PWM, shorten only the mechanical stop wait.
    motor_protected #(.STOP_CYCLES(20)) dut (
        .clk(clk),.rst_n(rst_n),.command(command),.arm(arm),
        .permit_in(permit_in),.duty_pct(duty),.fault(fault),.estop(estop),
        .clear_fault(clear_fault),.rpwm(rp),.lpwm(lp),.r_en(re),.l_en(le),
        .motor_dir(dir),.state_o(state),.permit_o(permit),
        .protection_locked(locked),.fault_latched(fl),.estop_latched(el),
        .clear_accepted(accepted)
    );
    task step(input integer n);
        repeat(n) @(negedge clk);
    endtask
    task quiet;
        #1;
        if({rp,lp,re,le} !== 4'b0000) $fatal(1,"Physical four-wire outputs not zero");
    endtask
    task clear_new;
        clear_fault=0; step(2); clear_fault=1; step(1);
    endtask
    task recover;
        command=0; arm=0; fault=0; estop=0; permit_in=1; duty=10;
        clear_new;
        if(!accepted || locked || !permit) $fatal(1,"Recovery clear rejected");
        clear_fault=0; step(30); quiet;
        if(state!==0) $fatal(1,"Not unarmed after recovery");
    endtask
    task start_motor(input [1:0] request);
        command=0; arm=0; step(30);
        arm=1; step(1); arm=0; step(1);
        if(state!==1) $fatal(1,"Fresh start did not arm");
        command=request; step(5);
        if(!re || !le || dir!==(request==2)) $fatal(1,"Direction start failed");
    endtask
    task measure(input integer pct);
        duty=pct; step(54010);
        high_count=0;
        repeat(27000) begin @(negedge clk); if(rp || lp) high_count=high_count+1; end
        if(high_count!=27000*pct/100) $fatal(1,"Duty changed through protection: %0d",high_count);
    endtask
    task wait_high;
        while(!(rp || lp)) @(negedge clk);
    endtask
    always @(negedge clk) begin
        #2;
        if(rp && lp) $fatal(1,"Two PWM directions active together");
        if(re!==le) $fatal(1,"Enable mismatch");
        if((rp || lp) && (!re || !le)) $fatal(1,"PWM outside enable");
        if((rp && dir) || (lp && !dir)) $fatal(1,"PWM on wrong leg");
        if((!rst_n || !permit || fault || estop || duty>30) && {rp,lp,re,le}!==0)
            $fatal(1,"Output escaped protection");
    end
    initial begin
        step(3); rst_n=1; step(50); quiet;
        if(!locked) $fatal(1,"Boot was not locked");
        command=0; arm=0; step(20); quiet;
        if(!locked) $fatal(1,"Held clear armed when command became STOP");
        recover;
        command=1; step(30000); quiet; // clear is not an arm event
        start_motor(1); measure(10); measure(20); measure(30);
        $display("PROTECTED_FORWARD_1KHZ_DUTY_PASS");

        // Assert fault during an ACTUAL high PWM pulse, not an idle interval.
        wait_high; fault=1; quiet; step(1);
        if(!locked || !fl) $fatal(1,"Fault latch missing");
        fault=0; step(30000); quiet;
        clear_new; step(3); quiet;
        if(!locked || accepted) $fatal(1,"Running command allowed clear");
        command=0; step(3); quiet;
        if(!locked) $fatal(1,"Held clear was deferred until STOP");
        recover;
        arm=1; step(1); arm=0; step(1); command=2; step(5);
        measure(10); measure(20); measure(30);
        $display("PROTECTED_FAULT_ZERO_AND_REVERSE_PASS");

        wait_high; estop=1; quiet; step(1);
        if(!el || !locked) $fatal(1,"Emergency request not latched");
        // Start held during a trip cannot become a restart when it is cleared.
        arm=1; estop=0; command=0; clear_new; step(30); quiet;
        if(!locked || accepted) $fatal(1,"Clear with held arm accepted");
        arm=0; step(30); quiet;
        if(!locked) $fatal(1,"Held clear was accepted on arm release");
        recover; step(30000); quiet;
        start_motor(1); wait_high;
        $display("PROTECTED_ESTOP_RELEASE_NO_RESTART_PASS");

        permit_in=0; quiet; step(1);
        permit_in=1; step(30000); quiet;
        if(!locked) $fatal(1,"Permit restore auto-unlocked");
        recover; start_motor(2); wait_high;
        duty=31; quiet; step(1);
        duty=10; step(30000); quiet;
        if(!locked || !fl) $fatal(1,"Over-limit duty not latched");
        recover; start_motor(1); wait_high;
        $display("PROTECTED_PERMISSION_AND_DUTY_TRIP_PASS");

        // Trip coincident with clear always wins at the four physical outputs.
        clear_fault=0; step(2); fault=1; estop=1; clear_fault=1; quiet; step(1);
        if(!fl || !el || accepted) $fatal(1,"Simultaneous trip/clear priority");
        fault=0; estop=0; command=0; step(30); quiet;
        if(!locked) $fatal(1,"Held clear auto-released simultaneous trip");
        recover; start_motor(2); wait_high;
        rst_n=0; quiet; step(3); rst_n=1; step(30000); quiet;
        if(!locked) $fatal(1,"Reset re-enabled output");

        recover; start_motor(1); wait_high;
        command=3; quiet; step(30); quiet; // preserve normal STOP behavior
        if(locked) $fatal(1,"Normal STOP incorrectly latched as a fault");
        start_motor(2); measure(10);
        command=0; quiet; step(30); quiet;
        $display("PROTECTED_RESET_STOP_AND_REARM_PASS");
        $display("MOTOR_PROTECTED_INTEGRATION_PASS");
        $finish;
    end
    initial begin #100000000; $fatal(1,"Protected integration timeout"); end
endmodule
