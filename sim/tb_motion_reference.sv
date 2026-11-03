`timescale 1ns/1ps
module tb_motion_reference;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,ready=1,en=1,pause=0,follow=1;
 reg [7:0] divider=0;wire update=divider==255;
 always @(posedge clk)if(!rst)divider<=0;else divider<=update ? 0:divider+1'b1;
 reg signed [31:0] goal=0,actual=0;wire signed [31:0] position,speed;
 wire active,paused,calculating;
 arm_motion_reference_r15l dut(.clk(clk),.rst_n(rst),.update_i(update),.ready_i(ready),.enable_i(en),.rebase_i(1'b0),.pause_i(pause),
  .actual_i(actual),.goal_i(goal),.max_velocity_i(18'd29952),.acceleration_i(11'd210),.following_band_i(16'd11438),
  .position_o(position),.velocity_o(speed),.active_o(active),.paused_o(paused),.calculating_o(calculating));
 integer checks=0,updates=0,n,j,quiet_steps;reg signed [31:0] old_position,old_speed;
 reg signed [18:0] old_velocity;reg signed [11:0] old_acceleration;
 reg [35:0] square_oracle;reg [40:0] distance_oracle;
 task check(input bit ok,input string msg);begin checks=checks+1;if(!ok)$fatal(1,"%s: goal=%0d pos=%0d v=%0d a=%0d",msg,goal,position,speed,dut.acceleration_q24);end endtask
 task step;begin @(negedge clk);while(!update)@(negedge clk);old_position=position;old_velocity=dut.velocity_q24;old_acceleration=dut.acceleration_q24;@(negedge clk);#1;
  updates=updates+1;
  check(speed<=23400 && speed>=-23400,"planned speed stays within configured ceiling");
  check(dut.acceleration_q24<=210 && dut.acceleration_q24>=-210,"planned acceleration stays bounded");
  if(position!=goal || dut.velocity_q24!=0)check(position-old_position<=118 && position-old_position>=-118,"motion reference advances continuously across wraps and retargets");
  if(follow)actual=position;
 end endtask
 task reach(input signed [31:0] target,input integer limit);begin
  goal=target;n=0;quiet_steps=0;
  while((position!=goal || speed!=0) && n<limit)begin
   old_speed=speed;step();n=n+1;
   if(speed==0 && position!=goal)quiet_steps=quiet_steps+1;
  end
  check(position==goal && speed==0,"trajectory reaches cumulative target with exact zero terminal speed");
  check(quiet_steps<50,"motion does not stop at intermediate commanded waypoints");
  $display("REACHED %0d after %0d updates, intermediate zero samples=%0d",goal,n,quiet_steps);
 end endtask
 // Prediction must finish before the next control sample at these settings.
 always @(posedge clk)if(rst && update && dut.seeded)
  check(!dut.calc_busy,"stop prediction completes inside its control period");
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(3)step();
  reach(411775,6500);check(position>205887,"one revolution is never wrapped to zero or shortest path");
  reach(-823550,17000);check(position< -411775,"negative two-turn target is cumulative");
  reach(0,11000);
  goal=411775;repeat(300)step();goal=-411775;
  reach(-411775,11000); // retarget while moving: reference remains continuous
  goal=823550;repeat(300)step();pause=1;repeat(300)step();
  check(paused && speed==0 && position!=goal,"recovery pause decelerates without losing cumulative target");
  pause=0;reach(823550,15000);
  ready=0;step();check(speed==0 && !active,"permission loss revokes trajectory motion");
  actual=32'sd2147483000;goal=32'sd2147483647;ready=1;repeat(4)step();reach(32'sd2147483647,1000);
  follow=0;ready=0;step();actual=-32'sd2147483000;goal=-32'sd2147483647;ready=1;repeat(4)step();follow=1;reach(-32'sd2147483647,1000);
  $display("PASS motion reference %0d assertions, %0d updates; cumulative turns, smooth retarget, braking and rails",checks,updates);$finish;
 end
 initial begin #500000000;$fatal(1,"motion reference timeout");end
endmodule
