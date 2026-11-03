`timescale 1ns/1ps
module tb_motion_budget;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,update=0;reg [10:0] amax=16;
 wire signed [31:0] position,velocity;
 arm_motion_reference_r15l dut(.clk(clk),.rst_n(rst),.update_i(update),.ready_i(1'b1),.enable_i(1'b1),.rebase_i(1'b0),.pause_i(1'b0),
  .actual_i(32'sd0),.goal_i(32'sd1000000),.max_velocity_i(18'd167772),.acceleration_i(amax),
  .following_band_i(16'd11438),.position_o(position),.velocity_o(velocity));
 integer cycles,checks=0,k;reg signed [18:0] injected_v;reg signed [11:0] injected_a;
 reg [63:0] square,minimum_distance;
 task check(input bit ok,input string msg);begin checks=checks+1;if(!ok)$fatal(1,"%s cycles=%0d",msg,cycles);end endtask
 initial begin
  for(k=0;k<4;k=k+1)begin
   rst=0;repeat(3)@(negedge clk);amax=k<2?16:1024;rst=1;repeat(3)@(negedge clk);
   // Reachable legal cruise/acceleration states, injected only to isolate the
   // worst prediction latency rather than simulate minutes of acceleration.
   injected_v=k[0]?-19'sd167772:19'sd167772;
   injected_a=k[0]?-$signed({1'b0,amax}):$signed({1'b0,amax});
   force dut.velocity_q24=injected_v;force dut.acceleration_q24=injected_a;
   update=1;@(negedge clk);update=0;release dut.velocity_q24;release dut.acceleration_q24;
   cycles=0;while(dut.calc_busy && cycles<13000)begin @(negedge clk);cycles=cycles+1;end
   check(cycles<12000 && dut.calculation_valid,"legal worst-case brake prediction fits well inside 135000-clock/5ms period");
   square=64'd167772*167772;minimum_distance=square/(2*amax);
   check(dut.brake_distance>=minimum_distance,"prediction includes positive-acceleration reversal before full braking");
  end
  $display("PASS brake prediction budget %0d assertions, min/max acceleration and both velocity directions",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"brake prediction budget timeout");end
endmodule
