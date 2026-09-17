`timescale 1ns/1ps
module tb_motor_buttons;
    reg clk=0, rst_n=0, permit=1;
    reg start_raw=0, stop_raw=1, direction_raw=1;
    wire rp,lp,re,le,reverse,running;
    wire [2:0] state;
    integer highs;
    always #18.5185185 clk=~clk;
    motor_buttons_top #(.STOP_CYCLES(80),.DEBOUNCE_CYCLES(4)) dut
        (.clk(clk),.rst_n(rst_n),.permit(permit),.start_raw(start_raw),.stop_raw(stop_raw),
         .direction_raw(direction_raw),.rpwm(rp),.lpwm(lp),.r_en(re),.l_en(le),
         .selected_reverse(reverse),.running(running),.state_o(state));
    task step(input integer n); begin repeat(n) @(negedge clk); #1; end endtask
    task quiet; begin if({rp,lp,re,le}!==0 || running!==0) $fatal(1,"Not stopped"); end endtask
    task start_press; begin start_raw=0; step(15); start_raw=1; step(15); end endtask
    task check_run(input bit rev); begin
        step(2800);
        if(!running || reverse!==rev) $fatal(1,"Wrong running direction");
        highs=0;
        repeat(1350) begin step(1); if(rp||lp) highs=highs+1; end
        if(highs!=135) $fatal(1,"Wrong duty %0d",highs);
    end endtask
    always @(negedge clk) begin
        if(rp&&lp) $fatal(1,"Both PWM active");
        if(re!==le) $fatal(1,"EN mismatch");
        if((rp||lp)&&!(re&&le)) $fatal(1,"PWM without enable");
        if(rp&&reverse || lp&&!reverse) $fatal(1,"Wrong PWM channel");
        if((!rst_n || !permit) && {rp,lp,re,le}!==0) $fatal(1,"Reset/permit gate");
    end
    initial begin
        step(5); rst_n=1; step(110); quiet; // Boot held START.
        start_raw=1; step(15);
        repeat(4) begin start_raw=0; step(1); start_raw=1; step(1); end
        step(15); quiet; // Bounce ignored.
        start_press; check_run(0);
        stop_raw=0; step(15); quiet;
        start_press; quiet; // STOP held wins.
        stop_raw=1; step(110); quiet;
        start_press; check_run(0);
        direction_raw=0; step(15); quiet;
        if(!reverse) $fatal(1,"Direction not selected");
        direction_raw=1; start_press; quiet; // Lockout START ignored.
        step(110); quiet; // No queued restart.
        start_press; check_run(1);
        permit=0; step(2); quiet; permit=1; step(110); quiet;
        // Simultaneous STOP and START must not start.
        stop_raw=0; start_raw=0; step(20); quiet;
        stop_raw=1; start_raw=1; step(110); quiet;
        // Direction and START together: select, but do not run.
        direction_raw=0; start_raw=0; step(20); quiet;
        direction_raw=1; start_raw=1; step(110); quiet;
        start_press; check_run(0);
        rst_n=0; start_raw=0; step(5); quiet;
        rst_n=1; step(110); quiet;
        start_raw=1; step(15); start_press; check_run(0);
        $display("HENGXIN_MOTOR_BUTTONS_PASS"); $finish;
    end
    initial begin #5000000; $fatal(1,"Timeout"); end
endmodule
