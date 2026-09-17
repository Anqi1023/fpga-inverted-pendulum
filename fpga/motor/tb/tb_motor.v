`timescale 1ns/1ps
module tb_motor;
    reg clk=0, rst_n=0, arm=1, permit=1;
    reg [1:0] command=1;
    reg [6:0] duty=10;
    wire pwm, dir, en;
    wire [2:0] state;
    wire rp,lp,re,le,locked_rp,locked_lp,locked_re,locked_le;
    integer highs, cycles;
    reg previous_dir=0, previous_en=0;
    // Keep real PWM ratio (27 MHz / 20 kHz = 1350); shorten STOP timer only.
    motor_driver #(.STOP_CYCLES(20)) dut (
        .clk(clk),.rst_n(rst_n),.command(command),.arm(arm),.permit(permit),
        .duty_pct(duty),.motor_pwm(pwm),.motor_dir(dir),.motor_en(en),.state_o(state));
    ibt2_adapter #(.INTERFACE_CONFIRMED(1)) model (
        .pwm(pwm),.dir(dir),.en(en),.rpwm(rp),.lpwm(lp),.r_en(re),.l_en(le));
    ibt2_adapter locked (
        .pwm(pwm),.dir(dir),.en(en),.rpwm(locked_rp),.lpwm(locked_lp),
        .r_en(locked_re),.l_en(locked_le));
    always #18.5185185 clk=~clk;
    task step; input integer n; begin repeat(n) @(negedge clk); end endtask
    task quiet; begin
        if (en !== 0 || pwm !== 0) $fatal(1,"Outputs not stopped");
    end endtask
    task unlock; begin
        command=0; arm=0; step(30); arm=1; step(1); arm=0; step(1);
        if(state !== 1) $fatal(1,"Failed to arm in STOP");
    end endtask
    task measure; input integer pct; begin
        duty=pct;
        // Allow previous cycle to end and new duty to latch.
        step(2800);
        highs=0;
        repeat(1350) begin @(negedge clk); if(pwm) highs=highs+1; end
        if(highs != 1350*pct/100) $fatal(1,"Duty %0d: got %0d high ticks",pct,highs);
        if(pct > 0) begin
            while(pwm) @(negedge clk);
            while(!pwm) @(negedge clk);
            cycles=0;
            while(pwm) begin @(negedge clk); cycles=cycles+1; end
            while(!pwm) begin @(negedge clk); cycles=cycles+1; end
            if(cycles != 1350) $fatal(1,"PWM period mismatch: %0d",cycles);
        end
        $display("DUTY_PASS %0d percent: %0d/1350",pct,highs);
    end endtask
    always @(negedge clk) begin
        if((rp && lp) || ((locked_rp|locked_lp|locked_re|locked_le) !== 0))
            $fatal(1,"Adapter interlock failure");
        if(pwm && !en) $fatal(1,"PWM outside enable");
        if(dir != previous_dir && previous_en) $fatal(1,"DIR changed without disabled interval");
        previous_dir=dir; previous_en=en;
    end
    initial begin
        step(3); rst_n=1; step(50); quiet;
        $display("BOOT_HELD_COMMAND_ARM_PASS");
        unlock; command=1; step(3);
        if(!en || dir !== 0) $fatal(1,"Forward failed");
        measure(10); measure(20); measure(30); measure(0);
        measure(10);
        command=0; #1; quiet; step(2);
        command=2; step(50); quiet; // cannot auto restart after STOP
        unlock; command=2; step(3);
        if(!en || dir !== 1) $fatal(1,"Reverse failed");
        measure(10); measure(20); measure(30);
        command=0; #1; quiet; step(30);
        $display("STOP_FORWARD_STOP_REVERSE_STOP_PASS");
        unlock; command=1; step(5);
        command=2; #1; quiet; step(2);
        command=0; arm=1; step(1); arm=0; step(30); quiet;
        if(state !== 0) $fatal(1,"Early arm bypassed dead time");
        $display("DIRECT_REVERSAL_EARLY_ARM_PASS");
        unlock; command=1; step(10);
        duty=31; #1; quiet; step(2); duty=10; step(40); quiet;
        unlock; command=1; step(10);
        permit=0; #1; quiet; step(3); permit=1; step(40); quiet;
        unlock; command=1; step(10);
        command=3; #1; quiet; step(30);
        unlock; command=2; step(10);
        rst_n=0; #1; quiet; step(3); rst_n=1; step(40); quiet;
        $display("INVALID_DUTY_PERMIT_COMMAND_RESET_PASS");
        $display("HENGXIN_MOTOR_OFFLINE_PASS"); $finish;
    end
    initial begin #5000000; $fatal(1,"TIMEOUT"); end
endmodule
