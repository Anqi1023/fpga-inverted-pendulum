`timescale 1ns/1ps
module tb_motor_control_v1;
    reg clk=0,rst_n=0,valid=0,en=0,fault=0,estop=0,clear=0;
    reg signed [15:0] u=0;
    wire rp,lp,re,le,dir,ready,waiting,locked,fl,el,ca,to,ce;
    wire [8:0] duty;
    integer width,period,k;
    always #18.5185185 clk=~clk;
    motor_control_v1 #(.COMMAND_TIMEOUT_CYCLES(270000),.STOP_CYCLES(8)) dut
        (clk,rst_n,u,valid,en,fault,estop,clear,rp,lp,re,le,dir,ready,waiting,locked,fl,el,ca,to,ce,duty);
    task step(input integer n); begin repeat(n) @(negedge clk); #1; end endtask
    task quiet; if({rp,lp,re,le}!==4'b0) $fatal(1,"Outputs not zero"); endtask
    task request_stopped;
        begin
            quiet;
            if(dut.command!==2'b00 || dut.duty_pct!==0 || duty!==0)
                $fatal(1,"Effective command/duty must be zero");
        end
    endtask
    task clear_lock;
        begin
            en=0; valid=0; u=0; fault=0; estop=0; clear=0; step(12);
            clear=1; step(1); clear=0; step(3);
            if(locked || !ready || to || ce) $fatal(1,"Manual clear failed");
            quiet;
        end
    endtask
    task send(input integer value);
        begin u=value; valid=1; step(1); valid=0; step(1); end
    endtask
    function pulse(input integer value); pulse=value<0?lp:rp; endfunction
    task quantized_stop(input integer value);
        begin
            en=1; send(value); request_stopped; step(12); request_stopped;
            if(locked || !ready) $fatal(1,"Quantized zero must stop without locking: %0d",value);
            $display("CONTROL_QUANTIZED_STOP_PASS u=%0d",value);
        end
    endtask
    task wave(input integer value);
        begin
            en=1; send(value);
            if(dut.command!==(value<0?2'b10:2'b01) ||
               dut.duty_pct!=(value<0?-value:value)/10 ||
               duty!=((value<0?-value:value)/10)*10)
                $fatal(1,"Bad sign mapping/integer truncation for %0d",value);
            // Discard a possible old pulse; measure a complete new period.
            step(27005);
            while(pulse(value)) step(1);
            while(!pulse(value)) step(1);
            width=0; period=0;
            while(pulse(value)) begin width=width+1; period=period+1; step(1); end
            while(!pulse(value)) begin period=period+1; step(1); end
            if(width!=270*((value<0?-value:value)/10) || period!=27000)
                $fatal(1,"Bad PWM u=%0d width=%0d period=%0d",value,width,period);
            $display("CONTROL_PWM_PASS u=%0d ticks=%0d/27000",value,width);
        end
    endtask
    task expect_command_trip(input integer value);
        begin
            en=1; send(value); step(2); quiet;
            if(!locked || !fl || !ce) $fatal(1,"Invalid command did not latch: %0d",value);
            u=100; valid=0; step(5); quiet; clear_lock;
        end
    endtask
    always @(negedge clk) begin
        #2;
        if(rp && lp || re!==le || ((rp || lp) && !re)) $fatal(1,"Output interlock");
        if(!rst_n || !en || fault || estop || locked) request_stopped;
    end
    initial begin
        step(5); rst_n=1; en=1; send(100); step(20); quiet;
        if(!locked) $fatal(1,"Boot must lock");
        clear_lock; en=1; step(20); quiet;
        quantized_stop(1); quantized_stop(7); quantized_stop(9);
        wave(10); wave(19); wave(81); wave(85); wave(99); wave(101); wave(200); wave(299); wave(300);
        // Invalid input changes without valid must not change the held command.
        u=-32768; step(15);
        if(duty!=300 || locked) $fatal(1,"Changed command without valid");
        // A zero-rounded input also stops a motor which was already running.
        quantized_stop(-1); quantized_stop(-7); quantized_stop(-9);
        wave(-10); wave(-19); wave(-81); wave(-85); wave(-99); wave(-101); wave(-200); wave(-299); wave(-300);
        quantized_stop(0);
        wave(100);
        // Disable beats output combinationally, without a valid command.
        en=0; #1; request_stopped; step(12); en=1; step(30); request_stopped;
        if(locked) $fatal(1,"Ordinary disable locked protection");
        wave(-100);
        // Direct sign reversal must blank before opposite output is enabled.
        send(100); quiet;
        for(k=0;k<7;k=k+1) begin step(1); quiet; end
        wave(100);
        // Fault on a high pulse suppresses immediately and latches.
        while(!rp) step(1);
        fault=1; #1; quiet; step(2); fault=0; step(10); quiet;
        if(!locked || !fl) $fatal(1,"Fault did not remain locked");
        clear_lock; wave(-100);
        while(!lp) step(1);
        estop=1; #1; quiet; step(2); estop=0; step(10); quiet;
        if(!locked || !el) $fatal(1,"Estop did not remain locked");
        clear_lock;
        expect_command_trip(301); expect_command_trip(-301);
        expect_command_trip(32767); expect_command_trip(-32768);
        en=1; u=100; valid=1; step(3); valid=0; step(2); quiet;
        if(!locked || !ce) $fatal(1,"Stuck-high valid not detected");
        // Clear while enabled is rejected, then a held clear cannot unlock
        // automatically when enable later falls.
        clear=1; step(2); en=0; step(4); quiet;
        if(!locked) $fatal(1,"Unsafe clear accepted");
        clear=0; step(2); clear_lock;
        wave(100); rst_n=0; #1; quiet; step(3); rst_n=1; step(15); quiet;
        if(!locked) $fatal(1,"Reset caused restart");
        $display("MOTOR_CONTROL_V1_FUNCTIONAL_PASS"); $finish;
    end
    initial begin #200000000; $fatal(1,"Functional test timed out"); end
endmodule
