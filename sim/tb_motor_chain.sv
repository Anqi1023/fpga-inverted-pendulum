`timescale 1ns/1ps
module tb_motor_chain;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,en=0,valid=0,clear=0,boost=0,fault=0,estop=0;
 reg signed [15:0] u=0;reg signed [31:0] theta=0;
 wire rpwm,lpwm,re,le,locked,permit,timeout_error,command_error;
 wire [2:0] state;wire [1:0] command;wire [9:0] duty;
 // Only the watchdog is shortened for this test. PWM/dead-time counters use
 // the real design's 27MHz/10kHz/2700-clock settings.
 motor_control_u_chain #(.CLK_HZ(27000000),.PWM_HZ(10000),.STOP_CYCLES(2700),
 .TIMEOUT_CYCLES(12000),.LIVE_FINE_DRIVE(1),.LIVE_START_BOOST(1)) dut(
 .clk(clk),.rst_n(rst),.control_u(u),.motion_theta(theta),.control_valid(valid),.motor_enable(en),
 .fine_drive_enable(1'b1),.boost_enable(boost),.boost_min_u(8'd150),.boost_ticks(4'd4),
 .permit_in(1'b1),.fault(fault),.estop(estop),.clear_request(clear),
 .rpwm(rpwm),.lpwm(lpwm),.r_en(re),.l_en(le),.state_o(state),.permit_o(permit),
 .protection_locked(locked),.timeout_latched(timeout_error),.command_error_latched(command_error),
 .command(command),.duty_permille(duty));
 integer checks=0,j,latency;
 task check(input bit ok,input string msg);begin checks=checks+1;if(!ok)$fatal(1,"%s",msg);end endtask
 task request(input signed [15:0] value);begin
  @(negedge clk);u=value;valid=1;@(negedge clk);valid=0;repeat(3)@(negedge clk);
 end endtask
 task clear_trip;begin
  en=0;u=0;valid=0;fault=0;estop=0;clear=0;repeat(4)@(negedge clk);
  clear=1;repeat(8)@(negedge clk);clear=0;repeat(8)@(negedge clk);
  check(!locked,"clear in STOP releases protection only");
 end endtask
 always @(negedge clk)if(rst)begin
  if(rpwm && lpwm)$fatal(1,"overlapping IBT outputs");
  if((fault||estop||locked)&& (rpwm||lpwm||re||le))$fatal(1,"drive during protection");
 end
 initial begin
  #12;@(negedge clk);rst=1;clear_trip();en=1;request(200);
  latency=0;while(!rpwm && latency<6000)begin @(negedge clk);latency=latency+1;end
  check(rpwm && !lpwm && state==2 && duty==200,"positive U drives RPWM");
  $display("START_FIRST_PWM_CLOCKS %0d",latency);
  request(-200);latency=0;
  while(!lpwm && latency<6000)begin @(negedge clk);latency=latency+1;end
  check(lpwm && !rpwm && state==3 && duty==200,"negative U drives LPWM after bounded blanking");
  check(latency>=2700 && latency<5600,"reversal latency is about dead-time plus one PWM period");
  $display("REVERSE_FIRST_PWM_CLOCKS %0d",latency);
  request(5);repeat(3000)@(negedge clk);
  check(duty==5 && command==1,"fine mode passes 0.5pct without raising to 8/10pct");
  request(0);repeat(4)@(negedge clk);check(!rpwm && !lpwm && !re && !le,"zero command withdraws drive");
  boost=1;request(20);
  for(j=0;j<11;j=j+1)request(20);
  check(dut.adapter_i.held_duty==150,"bounded stationary-start pulse reaches 15pct");
  for(j=0;j<7;j=j+1)request(20);
  check(dut.adapter_i.held_duty==20,"boost expires and does not become a permanent duty floor");
  boost=0;request(1001);repeat(5)@(negedge clk);
  check(command_error && !rpwm && !lpwm,"invalid U trips execution chain");
  clear_trip();en=1;request(200);repeat(12010)@(negedge clk);
  check(timeout_error && !rpwm && !lpwm,"command watchdog removes drive");
  clear_trip();en=1;request(200);repeat(6000)@(negedge clk);
  estop=1;#1;check(!rpwm && !lpwm && !re && !le,"estop blocks outputs without waiting for PWM boundary");
  $display("PASS motor_chain %0d assertions",checks);$finish;
 end
 initial begin #2000000;$fatal(1,"motor chain test timeout");end
endmodule
