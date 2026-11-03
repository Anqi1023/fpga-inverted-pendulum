`timescale 1ns/1ps
module tb_compact_position;
 reg signed [14:0] error=0;reg near_on=0;
 wire signed [63:0] actual,expected;
 cruise_position_feedback dut(.error_i(error),.near_i(near_on),.request_o(actual));
 position_original_theta_position_tenths baseline(.position_i({{17{error[14]}},error}),
 .gain_x10_i(11'sd80),.near_enabled_i(near_on),.near_gain_i(7'd4),.range_i(16'd11438),.p_q32_o(expected));
 integer x,n,checks=0;
 initial begin
 for(n=0;n<2;n=n+1)begin near_on=n;
  for(x=-11438;x<=11438;x=x+1)begin error=x;#1;
   if(actual!==expected)$fatal(1,"position mismatch x=%0d near=%0d actual=%0d expected=%0d",x,n,actual,expected);
   checks++;
  end
 end
 $display("PASS compact position %0d exhaustive exact source comparisons, all bounded errors and both near states",checks);$finish;
 end
endmodule
