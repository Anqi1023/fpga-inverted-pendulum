`timescale 1ns/1ps
module tb_motor_1khz;
    reg clk=0, reset_n=1, start_n=0, stop_n=1, dir_n=1;
    wire rp,lp,re,le,run_n,rev_n,ready_n;
    integer highs, ticks, rises, last_rise, high_width;
    reg previous_pwm;
    always #18.5185185 clk=~clk;
    dock_motor_test #(.RELEASE_CYCLES(4),.DEBOUNCE_CYCLES(4),.STOP_CYCLES(80)) dut
        (.clk(clk),.reset_key_n(reset_n),.start_key_n(start_n),.stop_key_n(stop_n),
         .direction_key_n(dir_n),.rpwm(rp),.lpwm(lp),.r_en(re),.l_en(le),
         .led_run_n(run_n),.led_reverse_n(rev_n),.led_ready_n(ready_n));
    task step(input integer n); begin repeat(n) @(negedge clk); #1; end endtask
    task quiet; begin
        if({rp,lp,re,le}!==4'b0000 || run_n!==1'b1) $fatal(1,"Not stopped");
    end endtask
    task press_start; begin start_n=0; step(15); start_n=1; step(15); end endtask
    task measure(input bit reverse); begin
        // Use the hardware's actual 27 MHz / 1 kHz counts, including the adapter.
        step(54000);
        highs=0; rises=0; last_rise=-1; high_width=0; previous_pwm=rp||lp;
        for(ticks=0;ticks<81000;ticks=ticks+1) begin
            step(1);
            if(re!==1'b1 || le!==1'b1) $fatal(1,"Enable missing");
            if(reverse ? rp!==1'b0 : lp!==1'b0) $fatal(1,"Wrong PWM channel");
            if(rp||lp) highs=highs+1;
            if((rp||lp) && !previous_pwm) begin
                if(last_rise>=0 && ticks-last_rise!=27000)
                    $fatal(1,"Period is not 27000 clocks / 1 ms");
                last_rise=ticks; rises=rises+1; high_width=0;
            end
            if((rp||lp) && last_rise>=0) high_width=high_width+1;
            if(!(rp||lp) && previous_pwm && last_rise>=0 && high_width!=2700)
                $fatal(1,"Pulse width is not 2700 clocks / 100 us");
            previous_pwm=rp||lp;
        end
        if(highs!=8100 || rises!=3 || run_n!==0 || rev_n!==!reverse || ready_n!==1)
            $fatal(1,"Duty, edge count, or LED mismatch");
        $display("PASS: direction=%0d, period=27000 clocks, width=2700 clocks, duty=10%%",reverse);
    end endtask
    always @(negedge clk) begin
        if((rp&&lp) || re!==le) $fatal(1,"Output interlock");
    end
    initial begin
        // A start held during power-up must not start output.
        step(120); quiet;
        if(ready_n!==0) $fatal(1,"Not ready");
        start_n=1; step(15); press_start; measure(0);
        stop_n=0; step(15); quiet;
        stop_n=1; step(110); quiet;
        press_start; step(54000);
        // Changing direction during a run must stop, without automatic restart.
        dir_n=0; step(15); quiet; dir_n=1; step(110); quiet;
        press_start; measure(1);
        // Reset assertion suppresses all four physical control outputs.
        #3; reset_n=0; #1; quiet;
        step(3); start_n=0; reset_n=1; step(120); quiet;
        start_n=1; step(15); press_start; measure(0);
        stop_n=0; step(15); quiet;
        $display("MOTOR_1KHZ_10PCT_TEST_PASS"); $finish;
    end
    initial begin #50000000; $fatal(1,"Timeout"); end
endmodule
