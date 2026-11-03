`timescale 1ns/1ps
module tb_motion_lean;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,en=1,moving=1,rev=0;reg signed [14:0] lean=0;
 reg signed [31:0] x=0,alpha=0,ad=0;
 wire signed [31:0] target,refangle,legacytarget;wire recovery;wire [3:0] ist;
 arm_balance_coordinator_r15 #(.MOTION_CONTROL(1),.MOTION_LEAN_CONTROL(1)) dut(
 .clk(clk),.rst_n(rst),.update_i(1'b1),.ready_i(en),.speed_known_i(1'b1),.stopped_i(1'b0),
 .catch_direction_known_i(1'b1),.motion_active_i(moving),.motion_lean_i(lean),.output_saturated_i(1'b0),
 .goal_error_i(32'sd411775),.integral_config_i(16'd0),.integral_status_o(ist),
 .point_config_i(64'd0),.point_target_i(19'd0),.position_error_i(x),.speed_i(32'd0),
 .alpha_i(alpha),.alpha_dot_i(ad),.position_gain_i(15'sd983),.reverse_i(rev),
 .angle_limit_i(13'd2288),.angle_step_i(10'd128),.hold_band_i(15'd11438),
 .alpha_gate_i(14'd3431),.alpha_dot_gate_i(18'd32768),
 .target_angle_o(target),.reference_angle_o(refangle),.recovery_o(recovery));
 integer checks=0;
 task check(input bit ok,input string msg);begin checks++;if(!ok)$fatal(1,"%s target=%0d ref=%0d",msg,target,refangle);end endtask
 task settle;begin repeat(50)@(negedge clk);end endtask
 initial begin
 repeat(3)@(negedge clk);rst=1;settle;
 check(target==0,"zero acceleration and tracking error preserve original reference");
 lean=-840;settle;check(target==-840 && refangle==-840,"positive planned acceleration creates negative lean even at zero tracking error");
 alpha=-840;ad=-100;settle;check(!recovery,"tracking a small nonzero task lean is not a fall");
 lean=840;alpha=0;ad=0;settle;check(target==840 && refangle==840,"planned braking reverses task lean");
 lean=8191;settle;check(target==2288 && refangle==2288,"combined request retains existing two-degree cap");
 lean=-8192;settle;check(target==-2288,"negative limit is symmetric");
 lean=-840;rev=1;settle;check(target==840,"original outer direction mapping applies once to total lean");
 rev=0;lean=-840;moving=0;settle;check(target==0,"trajectory completion removes task acceleration lean");
 moving=1;lean=-840;settle;en=0;settle;check(refangle==0,"permission loss clears task reference");
 $display("PASS motion lean %0d assertions: acceleration, braking, cap, reverse, recovery and permit",checks);$finish;
 end
endmodule
