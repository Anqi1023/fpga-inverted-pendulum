`timescale 1ns/1ps
module tb_jog;
    reg clk=0, reset_n=0, start_n=0, stop_n=1, direction_n=1;
    always #5 clk=~clk;
    wire rp,lp,re,le,run_n,rev_n,ready_n;
    dock_motor_jog #(.CLK_HZ(200000),.PWM_HZ(20000),.RELEASE_CYCLES(3),
        .DEBOUNCE_CYCLES(3),.JOG_CYCLES(100),.LOCK_CYCLES(50)) dut
        (clk,reset_n,start_n,stop_n,direction_n,rp,lp,re,le,run_n,rev_n,ready_n);
    integer starts=0, ticks, highs, before_count;
    reg last_en=0;
    task step(input integer n);
        repeat(n) begin @(posedge clk); #1; end
    endtask
    task start_press;
        @(negedge clk); start_n=0;
        step(10);
        @(negedge clk); start_n=1;
    endtask
    always @(posedge clk) begin
        #2;
        if(re && !last_en) starts=starts+1;
        last_en=re;
        if(rp && lp) $fatal(1,"Both PWM legs active");
        if(re!==le) $fatal(1,"Enable mismatch");
        if(!re && (rp || lp)) $fatal(1,"PWM active without enable");
    end
    initial begin
        #100000; $fatal(1,"Timeout");
    end
    initial begin
        step(5); reset_n=1;
        step(30);
        if(starts!=0) $fatal(1,"Held start at boot triggered motion");
        start_n=1; step(15);
        @(negedge clk); start_n=0;
        wait(re); #1;
        ticks=0; highs=0;
        while(re) begin
            ticks=ticks+1;
            if(rp) highs=highs+1;
            if(lp) $fatal(1,"Wrong forward output");
            step(1);
        end
        if(ticks!=100 || highs!=27) $fatal(1,"Jog/PWM mismatch: ticks=%0d highs=%0d",ticks,highs);
        // 10 cycles startup latency + nine full 30-percent PWM periods.
        step(100);
        if(starts!=1 || re) $fatal(1,"Held start repeated");
        start_n=1; step(15);
        start_press(); wait(re); step(4);
        stop_n=0; step(3);
        if(re || rp || lp) $fatal(1,"Stop latency exceeded");
        before_count=starts;
        step(60); start_press(); step(15);
        if(starts!=before_count) $fatal(1,"Start defeated held stop");
        stop_n=1; step(15);
        direction_n=0; step(12); direction_n=1; step(12);
        if(rev_n) $fatal(1,"Idle direction selection failed");
        start_press(); wait(lp);
        if(rp) $fatal(1,"Wrong reverse output");
        direction_n=0; step(12);
        if(re || rev_n) $fatal(1,"Direction during jog did not stop/retain direction");
        direction_n=1; step(12);
        before_count=starts; start_press(); step(10);
        if(starts!=before_count) $fatal(1,"Cooldown accepted start");
        step(60); start_press(); wait(re);
        reset_n=0; step(3);
        if(re || rp || lp) $fatal(1,"Reset failed to stop");
        reset_n=1; step(80);
        if(re || !rev_n) $fatal(1,"Reset restarted or retained direction");
        $display("JOG_TEST_PASS: timeout, PWM duty, stop, reset, direction, held start, cooldown");
        $finish;
    end
endmodule
