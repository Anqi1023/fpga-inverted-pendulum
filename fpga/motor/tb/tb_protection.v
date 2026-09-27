`timescale 1ns/1ps
module tb_protection;
    reg clk=0, rst_n=0, permit_in=1, fault=0, estop=0;
    reg stopped=1, clear_fault=1;
    wire permit, locked, fault_latched, estop_latched, accepted;
    always #5 clk=~clk;
    protection dut (
        .clk(clk),.rst_n(rst_n),.permit_in(permit_in),.fault(fault),
        .estop(estop),.stop_request(stopped),.clear_fault(clear_fault),
        .permit(permit),.locked(locked),.fault_latched(fault_latched),
        .estop_latched(estop_latched),.clear_accepted(accepted)
    );
    task step(input integer n);
        repeat(n) @(negedge clk);
    endtask
    task blocked;
        #1;
        if(permit !== 0) $fatal(1,"Permission not blocked");
    endtask
    task clear_new;
        clear_fault=0; step(2);
        clear_fault=1; step(1);
    endtask
    task unlocked;
        #1;
        if(!permit || locked || fault_latched || estop_latched || !accepted)
            $fatal(1,"Valid fresh clear failed");
        clear_fault=0; step(1);
        if(accepted) $fatal(1,"Clear acknowledgement lasted more than one clock");
    endtask
    initial begin
        step(3); rst_n=1; step(5); blocked;
        if(!locked || accepted) $fatal(1,"Held clear unlocked at boot");
        clear_new; unlocked;
        $display("PROTECTION_BOOT_MANUAL_CLEAR_PASS");

        // Observe combinational suppression BEFORE another rising clock edge.
        fault=1; blocked; step(1);
        if(!locked || !fault_latched || estop_latched) $fatal(1,"Fault not latched");
        clear_new; blocked;
        if(accepted) $fatal(1,"Clear won over active fault");
        fault=0; step(5); blocked;
        if(!locked) $fatal(1,"Held clear auto-released a vanished fault");
        stopped=0; clear_new; blocked;
        stopped=1; step(3); blocked;
        clear_new; unlocked;
        $display("PROTECTION_FAULT_CLEAR_PRIORITY_PASS");

        estop=1; blocked; step(1);
        if(!estop_latched || fault_latched) $fatal(1,"ESTOP not latched separately");
        estop=0; step(2); blocked;
        fault=1; step(1); fault=0; step(1);
        if(!fault_latched || !estop_latched) $fatal(1,"Trip reasons not accumulated");
        clear_new; unlocked;
        $display("PROTECTION_ESTOP_REASON_ACCUMULATION_PASS");

        permit_in=0; blocked; step(1);
        clear_new; blocked;
        permit_in=1; step(5); blocked;
        if(fault_latched || estop_latched) $fatal(1,"Permission loss invented a cause");
        clear_new; unlocked;
        $display("PROTECTION_PERMIT_RESTORE_NO_RESTART_PASS");

        // Reset itself cannot arm; a fault held across reset still inhibits.
        clear_fault=1; fault=1; rst_n=0; blocked; step(3);
        rst_n=1; step(3); blocked;
        if(!fault_latched) $fatal(1,"Fault held through reset lost");
        fault=0; step(5); blocked;
        clear_new; unlocked;

        // Simultaneous two causes and clear, then clear while still running.
        clear_fault=0; step(2);
        fault=1; estop=1; clear_fault=1; blocked; step(1);
        if(!fault_latched || !estop_latched || accepted) $fatal(1,"Trip priority failed");
        fault=0; estop=0; stopped=0; clear_new; blocked;
        stopped=1; step(5); blocked;
        clear_new; unlocked;
        $display("PROTECTION_UNIT_PASS");
        $finish;
    end
    initial begin #10000; $fatal(1,"Protection unit timeout"); end
endmodule
