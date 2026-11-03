`timescale 1ns/1ps
module tb_compact_integral;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,update=0,ready=1;
 reg [95:0] cfg={32'd11438,32'd20,32'd500};
 reg signed [31:0] alpha=0;
 reg signed [47:0] total=0;
 wire signed [31:0] actual,expected;
 balance_pid_integrator #(.COMPACT(1)) dut(.clk(clk),.rst_n(rst),.update_i(update),.ready_i(ready),.config_i(cfg),.alpha_i(alpha),.limit_u_i(32'sd1000),.total_q16_i(total),.integral_q16_o(actual));
 cruise_original_balance_pid_integrator baseline(.clk(clk),.rst_n(rst),.update_i(update),.ready_i(ready),.config_i(cfg),.alpha_i(alpha),.limit_u_i(32'sd1000),.total_q16_i(total),.integral_q16_o(expected));
 integer a,g,checks=0;
 task step;begin @(negedge clk);update=1;@(negedge clk);update=0;#1;
 if(actual!==expected)$fatal(1,"integral mismatch a=%0d gain=%0d actual=%0d expected=%0d",alpha,cfg[31:0],actual,expected);
 checks++;end endtask
 initial begin
 repeat(3)@(negedge clk);rst=1;
 for(g=0;g<=500;g=g+1)begin
  cfg[31:0]=g;
  for(a=-11438;a<=11438;a=a+199)begin alpha=a;step();end
 end
 for(a=0;a<5000;a=a+1)begin
  alpha=$random;cfg[31:0]=$unsigned($random)%501;
  cfg[63:32]=$unsigned($random)%101;
  cfg[95:64]=1144+($unsigned($random)%10295);
  total=$signed($random)*16'sd100;ready=(a%11)!=0;step();
 end
 $display("PASS compact integral %0d exact reference comparisons, signed truncation, legal gains/gates, leak, windup and ready loss",checks);$finish;
 end
endmodule
