`timescale 1ns/1ps
module tb_shared_chain;
    reg clk=0, rst_n=0;
    reg signed [15:0] control_u=0;
    reg control_valid=0, motor_enable=0;
    reg permit_in=1, fault=0, estop=0, clear_request=0;
    wire rpwm,lpwm,ibt_en,locked,timeout_latched,command_error_latched;
    wire ref_rpwm,ref_lpwm,ref_r_en,ref_l_en,ref_locked,ref_timeout,ref_error;
    wire [2:0] state_o,ref_state;
    integer forward_highs=0,reverse_highs=0;
    always #5 clk=~clk;
    motor_control_u_chain_3pin #(.CLK_HZ(1000000),.PWM_HZ(10000),
        .STOP_CYCLES(8),.TIMEOUT_CYCLES(1000)) dut (
        .clk(clk),.rst_n(rst_n),.control_u(control_u),.control_valid(control_valid),
        .motor_enable(motor_enable),.permit_in(permit_in),.fault(fault),.estop(estop),
        .clear_request(clear_request),.rpwm(rpwm),.lpwm(lpwm),.ibt_en(ibt_en),
        .protection_locked(locked),.state_o(state_o),.timeout_latched(timeout_latched),
        .command_error_latched(command_error_latched),.motor_dir(),.permit_o(),
        .fault_latched(),.estop_latched(),.clear_accepted(),.command(),.duty_pct(),
        .arm(),.clear_fault());
    // Independent, unchanged four-output production chain is the reference.
    motor_control_u_chain #(.CLK_HZ(1000000),.PWM_HZ(10000),
        .STOP_CYCLES(8),.TIMEOUT_CYCLES(1000)) reference_i (
        .clk(clk),.rst_n(rst_n),.control_u(control_u),.control_valid(control_valid),
        .motor_enable(motor_enable),.permit_in(permit_in),.fault(fault),.estop(estop),
        .clear_request(clear_request),.rpwm(ref_rpwm),.lpwm(ref_lpwm),
        .r_en(ref_r_en),.l_en(ref_l_en),.protection_locked(ref_locked),.state_o(ref_state),
        .timeout_latched(ref_timeout),.command_error_latched(ref_error),
        .motor_dir(),.permit_o(),.fault_latched(),.estop_latched(),.clear_accepted(),
        .command(),.duty_pct(),.arm(),.clear_fault());
    task step(input integer n); begin repeat(n) @(negedge clk); #1; end endtask
    task quiet;
        if({rpwm,lpwm,ibt_en}!==3'b000) $fatal(1,"Three-pin outputs did not stop");
    endtask
    task sample(input integer value);
        begin control_u=value; control_valid=1; step(1); control_valid=0; step(1); end
    endtask
    task clear;
        begin
            motor_enable=0; control_valid=0; clear_request=0;
            fault=0; estop=0; permit_in=1; step(12);
            clear_request=1; step(1); clear_request=0; step(20);
            quiet;
            if(locked || timeout_latched || command_error_latched)
                $fatal(1,"Manual clear did not safely clear both stages");
        end
    endtask
    task start_forward;
        begin motor_enable=1; sample(100); step(150); end
    endtask
    task wait_pwm;
        begin
            while(!rpwm) step(1);
        end
    endtask
    // Check observable equivalence after both clock edges, including stop edges.
    always @(posedge clk or negedge clk) begin
        #2;
        if({rpwm,lpwm,ibt_en,locked,state_o,timeout_latched,command_error_latched} !==
           {ref_rpwm,ref_lpwm,ref_r_en,ref_locked,ref_state,ref_timeout,ref_error})
            $fatal(1,"Three-pin chain differs from original production chain");
        if(ref_r_en!==ref_l_en || dut.r_en!==dut.l_en)
            $fatal(1,"Original enables no longer identical");
        if((rpwm && lpwm) || ((rpwm || lpwm) && !ibt_en))
            $fatal(1,"PWM / enable interlock violated");
        if(rpwm) forward_highs=forward_highs+1;
        if(lpwm) reverse_highs=reverse_highs+1;
    end
    initial begin
        step(5); rst_n=1; step(20); quiet;
        if(!locked) $fatal(1,"Boot must lock");
        clear;
        start_forward; step(100);
        if(forward_highs==0 || !ibt_en) $fatal(1,"No forward PWM");
        sample(-200); step(250);
        if(reverse_highs==0 || !ibt_en) $fatal(1,"No reverse PWM after waiting");
        sample(0); step(15); quiet;
        sample(9); step(15); quiet;

        sample(100); wait_pwm;
        motor_enable=0; #1; quiet; step(5);
        motor_enable=1; step(30); quiet; // old command must not restart
        sample(100); wait_pwm;
        fault=1; #1; quiet; step(4);
        if(!locked) $fatal(1,"Fault must lock");
        fault=0; sample(100); step(30); quiet;

        clear; start_forward; wait_pwm;
        estop=1; #1; quiet; step(4);
        if(!locked) $fatal(1,"Estop must lock");
        clear; start_forward; wait_pwm;
        permit_in=0; #1; quiet; step(4);
        if(!locked) $fatal(1,"Permission loss must lock");

        clear; start_forward; step(1100); quiet;
        if(!timeout_latched || !locked) $fatal(1,"Watchdog must stop and lock");
        sample(100); step(30); quiet;
        if(!timeout_latched || !locked) $fatal(1,"Late valid cleared watchdog");

        clear; motor_enable=1; sample(301); step(5); quiet;
        if(!command_error_latched || !locked) $fatal(1,"Range error must lock");
        clear; start_forward; wait_pwm;
        rst_n=0; #1; quiet; step(5); rst_n=1; step(30); quiet;
        if(!locked) $fatal(1,"Reset must relock");
        $display("SHARED_CHAIN_EQUIVALENCE_PASS forward=%0d reverse=%0d",forward_highs,reverse_highs);
        $finish;
    end
    initial begin #1000000; $fatal(1,"Shared chain test timed out"); end
endmodule
