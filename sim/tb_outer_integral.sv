`timescale 1ns/1ps
module tb_outer_integral;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,ready=1,known=1,moving=0,saturated=0,stopped=1;
 reg [5:0] counter=0;wire update=counter==39;
 always @(posedge clk)if(!rst)counter<=0;else counter<=update?0:counter+1'b1;
 reg signed [31:0] x=5719,v=0,a=0,ad=0;reg signed [31:0] goal=0;
 reg [15:0] icfg={11'd286,4'd6,1'b1};
 wire signed [13:0] integral;wire [3:0] status;wire signed [31:0] target,reference;
 wire [7:0] ps;integer checks=0,j;reg signed [13:0] saved;
 arm_balance_coordinator_r15 #(.POINT_CONTROL(1),.TARGET_BITS(32),.MOTION_CONTROL(1)) dut(
  .clk(clk),.rst_n(rst),.update_i(update),.ready_i(ready),.speed_known_i(known),.stopped_i(stopped),
  .catch_direction_known_i(1'b1),.motion_active_i(moving),.output_saturated_i(saturated),
  .goal_error_i(x),.integral_config_i(icfg),.integral_angle_o(integral),.integral_status_o(status),
  .point_config_i({21'd0,8'd160,17'd23400,17'd22876,1'b1}),.point_target_i(goal),
  .position_error_i(x),.speed_i(v),.alpha_i(a),.alpha_dot_i(ad),.position_gain_i(15'sd983),
  .reverse_i(1'b0),.angle_limit_i(13'd2288),.angle_step_i(10'd128),.hold_band_i(15'd11438),
  .alpha_gate_i(14'd3431),.alpha_dot_gate_i(18'd32768),.target_angle_o(target),.reference_angle_o(reference),.point_status_o(ps));
 task check(input bit ok,input string msg);begin checks=checks+1;if(!ok)$fatal(1,"%s I=%0d status=%b",msg,integral,status);end endtask
 task tick;begin @(negedge clk);while(!update)@(negedge clk);@(negedge clk);#1;
  check(integral<=286 && integral>=-286 && reference<=2288 && reference>=-2288,"integral and total angle reference stay bounded");
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(40)tick();
  check(integral>0 && status[3],"holding error creates real angle-domain integral");
  saved=integral;moving=1;repeat(40)tick();check(integral==saved && !ps[7],"motion freezes I and never claims arrived");
  moving=0;known=0;repeat(40)tick();check(integral==saved,"provisional/unknown speed cannot authorize integration");
  known=1;saturated=1;repeat(40)tick();check(integral==saved,"same-direction actuator saturation prevents windup");
  saturated=0;repeat(400)tick();check(integral==286,"integral reaches configured angle cap exactly");
  saturated=1;x=-5719;repeat(40)tick();check(integral<286,"saturation still permits unwinding opposite the saturated contribution");
  saturated=0;a=5000;ad=50000;saved=integral;repeat(40)tick();check(integral==saved,"pendulum recovery freezes integral");
  a=0;ad=0;x=-50000;saved=integral;repeat(40)tick();check(integral==saved,"far-position error cannot wind integral during travel");
  x=-5719;repeat(500)tick();check(integral==-286,"negative holding correction has symmetric integral bound");
  icfg[0]=0;tick();check(integral==0,"serial disable removes and clears integral");
  icfg[0]=1;repeat(10)tick();ready=0;tick();check(integral==0,"STOP/sensor readiness loss clears integral state");
  $display("PASS outer integral %0d assertions; real effect, holding gates, anti-windup, symmetry and clear",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"outer I timeout");end
endmodule
