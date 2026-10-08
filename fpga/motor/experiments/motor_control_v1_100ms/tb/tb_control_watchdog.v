`timescale 1ns/1ps
module tb_control_watchdog;
    reg clk=0,rst_n=0,valid=0,en=0,clear=0;
    reg signed [15:0] u=100;
    wire rp,lp,re,le,locked,to,ca;
    integer i;
    always #18.5185185 clk=~clk;
    // Real 27-MHz clock/PWM, real 100-ms watchdog. Stop wait alone is shortened.
    motor_control_v1 #(.STOP_CYCLES(8)) dut (
        .clk(clk),.rst_n(rst_n),.control_u(u),.control_valid(valid),.motor_enable(en),
        .fault(1'b0),.estop(1'b0),.clear_fault(clear),.rpwm(rp),.lpwm(lp),.r_en(re),.l_en(le),
        .motor_dir(),.ready(),.waiting(),.protection_locked(locked),.fault_latched(),
        .estop_latched(),.clear_accepted(ca),.timeout_latched(to),.command_error_latched(),.duty_tenths_o()
    );
    motor_control_v1 defaults (.clk(clk),.rst_n(1'b0),.control_u(16'sd0),
        .control_valid(1'b0),.motor_enable(1'b0),.fault(1'b0),.estop(1'b0),.clear_fault(1'b0),
        .rpwm(),.lpwm(),.r_en(),.l_en(),.motor_dir(),.ready(),.waiting(),.protection_locked(),
        .fault_latched(),.estop_latched(),.clear_accepted(),.timeout_latched(),.command_error_latched(),.duty_tenths_o());
    task step(input integer n); begin repeat(n) @(negedge clk); #1; end endtask
    task sample; begin valid=1; step(1); valid=0; end endtask
    task clear_lock;
        begin en=0; valid=0; clear=0; step(12); clear=1; step(1); clear=0; step(3); end
    endtask
    task quiet;
        begin
            if({rp,lp,re,le}!==0 || dut.command!==0 || dut.duty_pct!==0 || dut.duty_tenths_o!==0)
                $fatal(1,"Watchdog failed to stop outputs/command/duty");
        end
    endtask
    initial begin
        if(defaults.CLK_HZ!=27000000 || defaults.PWM_HZ!=1000 ||
           defaults.COMMAND_TIMEOUT_CYCLES!=2700000 || defaults.STOP_CYCLES!=27000000)
            $fatal(1,"Wrong shipped core defaults");
        step(5); rst_n=1; step(12); clear_lock; en=1; sample;
        for(i=0;i<4;i=i+1) begin
            step(539999); if(locked || to || !re) $fatal(1,"20-ms heartbeat failed");
            sample;
        end
        $display("CONTROL_20MS_HEARTBEAT_PASS cycles=540000");
        // The watchdog expires on the 2,700,000th missing-sample clock.
        // Input changes without valid neither replace the command nor refresh age.
        u=-200;
        step(2699999); if(to || locked || !re || dut.command!=1 || dut.duty_pct!=10)
            $fatal(1,"Watchdog expired early or unsampled input changed command");
        u=100;
        sample; if(to || locked) $fatal(1,"Fresh deadline sample rejected");
        step(2699999); if(to || locked || !re) $fatal(1,"Watchdog expired early after refresh");
        step(1); quiet; if(!to) $fatal(1,"100-ms watchdog missing");
        // Late input must not revoke the already issued timeout request.
        sample; step(2); quiet; if(!locked) $fatal(1,"Late command restarted output");
        clear_lock; if(to || locked) $fatal(1,"Timeout clear failed");
        step(2700002); quiet;
        if(to || locked) $fatal(1,"Disabled motor must not accumulate timeout");
        en=1; step(30); quiet; // no stale replay after clear
        step(2699970); quiet; if(!to) $fatal(1,"No-first-command timeout missing");
        step(2); if(!locked) $fatal(1,"No-first-command timeout not latched");
        // Zero commands still count as fresh heartbeats while enabled.
        clear_lock; u=0; en=1; sample;
        for(i=0;i<6;i=i+1) begin
            step(539999); quiet;
            if(to || locked) $fatal(1,"Fresh zero commands must feed watchdog");
            sample;
        end
        step(2699999); quiet; if(to || locked) $fatal(1,"Zero-command timeout early");
        step(1); quiet; if(!to) $fatal(1,"Missing timeout after zero-command stream ends");
        step(2); if(!locked) $fatal(1,"Zero-command timeout not latched");
        $display("CONTROL_100MS_WATCHDOG_PASS cycles=2700000"); $finish;
    end
    initial begin #800000000; $fatal(1,"Watchdog test timed out"); end
endmodule
