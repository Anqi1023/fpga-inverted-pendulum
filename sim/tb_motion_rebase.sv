`timescale 1ns/1ps
module tb_motion_rebase;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,update=0,rebase=0;reg signed [31:0] actual=0,goal=0;
 wire signed [31:0] pos,vel;wire active,paused;
 arm_motion_reference_r15l dut(.clk(clk),.rst_n(rst),.update_i(update),.ready_i(1'b1),.enable_i(1'b1),
 .pause_i(1'b0),.rebase_i(rebase),.actual_i(actual),.goal_i(goal),
 .max_velocity_i(18'd29952),.acceleration_i(11'd210),.following_band_i(16'd11438),
 .position_o(pos),.velocity_o(vel),.active_o(active),.paused_o(paused));
 integer checks=0,j;reg signed [31:0] old;
 task check(input bit ok,input string msg);begin checks++;if(!ok)$fatal(1,"%s pos=%0d vel=%0d",msg,pos,vel);end endtask
 task step;begin @(negedge clk);update=1;@(negedge clk);update=0;repeat(254)@(negedge clk);end endtask
 initial begin
 repeat(3)@(negedge clk);rst=1;repeat(3)step();actual=57190;repeat(3)step();
 check(pos==0 && vel==0,"stationary reference fifty degrees behind actual remains eligible for qualified rebase");
 goal=411775;
 @(negedge clk);rebase=1;@(negedge clk);rebase=0;check(pos==actual && vel==0 && goal==411775,"qualified stationary rebase preserves final cumulative goal");
 for(j=0;j<100;j++)begin actual=pos;step();end
 check(vel>0 && pos>57190,"new trajectory starts continuously from current physical position");
 actual=pos+50000;old=pos;@(negedge clk);rebase=1;@(negedge clk);rebase=0;
 check(pos==old,"moving trajectory never teleports on a rebase request");
 $display("PASS motion restart %0d assertions: stationary offset, qualified restart, cumulative goal, moving rejection",checks);$finish;
 end
endmodule
