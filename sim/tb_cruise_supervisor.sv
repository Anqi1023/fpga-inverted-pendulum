`timescale 1ns/1ps
module tb_cruise_supervisor;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,update=0,ready=1,known=1;
 reg signed [31:0] x=0,a=0,ad=0;
 reg [223:0] cfg={32'd100,32'd20,32'd65536,32'd3431,32'd128,32'd23400,32'd1};
 wire selected,correction,recovering;wire signed[31:0] v,e,p;wire[2:0] state;
 cruise_recovery dut(clk,rst,update,ready,known,cfg,x,a,ad,selected,correction,recovering,v,e,p,state);
 integer checks=0,i,k;reg signed[31:0] before_v;reg signed[63:0] distance_sum;
 task check(input bit ok,input string message);begin checks++;if(!ok)$fatal(1,"%s state=%0d v=%0d x=%0d p=%0d",message,state,v,x,p);end endtask
 task step;begin @(negedge clk);update=1;@(negedge clk);update=0;repeat(24)@(negedge clk);#1;end endtask
 task boot;begin rst=0;repeat(3)@(negedge clk);rst=1;end endtask
 initial begin
 boot();repeat(19)begin step();check(v==0 && recovering,"initial quiet confirmation");end
 step();check(!recovering,"exact quiet interval releases");
 distance_sum=0;
 for(i=0;i<500;i++)begin before_v=v;step();check(v-before_v>=0 && v-before_v<=128,"bounded acceleration");distance_sum=distance_sum+v;x=p;end
 check(v==23400 && state==3,"positive reference reaches setpoint");
 check(p==distance_sum/200,"fractional position integration");
 // Immediate entry before the next update, both signs and angle/speed causes.
 for(k=0;k<4;k++)begin
  a=k==0?3431:k==1?-3431:0;ad=k==2?65536:k==3?-65536:0;#1;
  check(recovering && !correction && v==0,"immediate catch withdraws cruise reference");
  x=x+9000;step();check(p==x && e==0,"catch rebases moving reference without sensor zero changes");
  a=2000;ad=0;repeat(25)step();check(recovering,"hysteresis prevents early release");
  a=0;repeat(19)step();check(recovering,"quiet duration required after disturbance");step();
  check(!recovering && cfg[63:32]==23400,"original target preserved");
  repeat(200)begin step();x=p;end
  check(v==23400,"automatic return to original speed");
 end
 known=0;#1;check(v==0 && !correction && state==5,"unknown velocity not zero");step();
 known=1;repeat(20)step();repeat(200)begin step();x=p;end;check(v==23400,"estimate recovery resumes original command");
 ready=0;#1;check(v==0 && state==0,"permission loss immediate reference withdrawal");step();ready=1;
 cfg[63:32]=-32'sd23400;repeat(20)step();
 for(i=0;i<200;i++)begin before_v=v;step();check(v-before_v<=0 && v-before_v>=-128,"negative acceleration symmetric");x=p;end
 check(v== -23400,"negative reference reaches setpoint");
 // No accumulated unbounded task when actual arm cannot follow.
 repeat(2000)step();check(e>=-11438 && e<=11438,"moving position error bounded on stalled input");
 cfg[63:32]=0;repeat(200)begin before_v=v;step();check(v-before_v<=128,"zero speed bounded approach");x=p;end
 check(v==0,"zero command holds stationary reference");
 cfg[0]=0;step();check(!selected && v==0 && state==0,"disabled path");
 cfg[0]=1;cfg[95:64]=0;step();check(!selected,"invalid config disables reference");
 cfg[95:64]=128;a=32'sh80000000;step();check(recovering,"INT_MIN angle cannot appear settled");
 a=0;ad=32'sh80000000;step();check(recovering,"INT_MIN derivative cannot appear settled");
 $display("PASS cruise supervisor %0d assertions: signed ramps, exact integration, disturbance hysteresis, resume, invalid speed, STOP, bounds",checks);$finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
