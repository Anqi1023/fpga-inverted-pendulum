`timescale 1ns/1ps
module tb_dual_bank;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,cv=0;reg [3:0] cmd=0;reg [7:0] id=0;reg signed [31:0] value=0;
 wire [7:0] tx;wire tx_start;reg busy=0;integer wait_count=0;
 wire run;wire [31:0] version;
 serial_parameter_bank #(.EXT_POSITION(37)) dut(.clk(clk),.rst_n(rst),.tick_i(1'b1),
  .command_i(cmd),.command_valid_i(cv),.id_i(id),.value_i(value),.stopped_i(!run),.ready_i(1'b1),
  .position_ready_i(1'b0),.position_actual_i(32'sd0),.position_target_i(32'sd0),
  .phase_i(3'd5),.reason_i(4'd0),.tx_busy_i(busy),.tx_data_o(tx),.tx_start_o(tx_start),
  .run_o(run),.version_o(version));
 always @(posedge clk)begin
  if(tx_start)begin $write("%c",tx);busy<=1;wait_count<=2;end
  else if(wait_count>0)begin wait_count<=wait_count-1;if(wait_count==1)busy<=0;end
 end
 integer checks=0,k;
 task check(input bit ok,input string message);begin checks=checks+1;if(!ok)$fatal(1,"%s",message);end endtask
 task send(input [3:0] c,input [7:0] i,input signed [31:0] v);begin
  @(negedge clk);cmd=c;id=i;value=v;cv=1;@(negedge clk);cv=0;repeat(650)@(negedge clk);
 end endtask
 initial begin
 #12;@(negedge clk);rst=1;repeat(5)@(negedge clk);
 for(k=0;k<110;k=k+1)if(k!=40)$display("PARAM %0d %0d %0d %0d",k,$signed(dut.active[k]),dut.minimum_value(k),dut.maximum_value(k));
 send(0,0,0);send(1,0,0);repeat(90000)@(negedge clk);
 send(2,102,-23400);check($signed(dut.pending[102])==-23400 && dut.active[102]==23400,"signed cruise pending");
 send(2,80,0);send(3,0,0);check(dut.active[80]==0 && $signed(dut.active[102])==-23400,"atomic mode and config");
 send(2,54,41);send(3,0,0);check(dut.active[54]==41,"mode0 integral writable");
 send(2,80,2);check(dut.response_code==8'h82,"invalid mode rejected");
 send(2,109,-501);check(dut.response_code==8'h82,"gain range enforced");
 send(2,102,-65537);check(dut.response_code==8'h82,"speed range enforced");
 send(5,0,0);check(run,"start mode0");send(2,80,1);send(3,0,0);
 check(dut.active[80]==0 && run && dut.response_code==8'h83,"running mode cannot commit");
 send(4,0,0);send(3,0,0);check(!run && dut.active[80]==1,"stopped atomic return mode1");
 $display("PASS dual parameter bank %0d assertions",checks);$finish;
 end
 initial begin #4000000;$fatal(1,"timeout");end
endmodule
