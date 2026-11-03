`timescale 1ns/1ps
module tb_target_uart_hold;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;wire cv;wire [3:0] cmd;wire [7:0] id;wire signed [31:0] value;
 reg [7:0] rxbyte=0;reg rxvalid=0;
 uart_command_parser parser(.clk(clk),.rst_n(rst),.data_i(rxbyte),.valid_i(rxvalid),.error_i(1'b0),
 .command_o(cmd),.id_o(id),.value_o(value),.command_valid_o(cv));
 reg [5:0] divider=0;wire tick=divider==49;
 always @(posedge clk)if(!rst)divider<=0;else if(tick)divider<=0;else divider<=divider+1'b1;
 wire run,set_position,position_ready;wire signed [31:0] target_command,target_actual;
 wire signed [31:0] actual;
 wire [1279:0] cfg;wire [127:0] loop_cfg;
 wire [95:0] near_cfg,return_cfg,coord_cfg,pid_cfg;
 wire [223:0] motion_cfg;wire [191:0] pos_i_cfg;wire [255:0] settle_cfg;
 wire [63:0] point_cfg,mp_cfg;wire signed [31:0] rp,rv,iv;wire [63:0] sd;wire bypass;wire [6:0] return_cap;wire signed [10:0] ktheta;
 wire tx_start;reg tx_busy=0;integer tx_wait=0;
 always @(posedge clk)begin
  if(tx_start)begin tx_busy<=1;tx_wait<=2;end
  else if(tx_wait>0)begin tx_wait<=tx_wait-1;if(tx_wait==1)tx_busy<=0;end
 end
 serial_parameter_bank #(.EXT_POSITION(36)) bank(.clk(clk),.rst_n(rst),.tick_i(1'b1),
  .command_i(cmd),.command_valid_i(cv),.id_i(id),.value_i(value),.stopped_i(!run),.ready_i(1'b1),
  .position_ready_i(position_ready),.position_actual_i(actual),.position_target_i(target_actual),
  .position_set_o(set_position),.position_value_o(target_command),.phase_i(3'd5),.reason_i(4'd0),.tx_busy_i(tx_busy),.tx_start_o(tx_start),
  .run_o(run),.config_o(cfg),.near_config_o(near_cfg),.return_config_o(return_cfg),.coordinate_config_o(coord_cfg),
  .pid_config_o(pid_cfg),.motion_config_o(motion_cfg),.position_i_config_o(pos_i_cfg),.settle_config_o(settle_cfg),
  .arm_loop_config_o(loop_cfg),.return_position_cap_o(return_cap),.k_theta_x10_o(ktheta),
  .point_config_o(point_cfg),.motion_pi_config_o(mp_cfg),.velocity_slew_bypass_o(bypass));
 reg signed [31:0] theta=0,speed=0,alpha=0,ad=0;
 reg sensor_valid=1,speed_valid=1;reg [36:0] speed_status=37'd131072;reg follow=0;
 wire signed [15:0] u;wire valid;
 wire signed [31:0] ar,vr,aq,goal,ct;wire signed [32:0] error;
 wire [31:0] hc,ss;wire signed [31:0] pa,pd;
 controller #(.MOTION_LEAN_CONTROL(1),.MOTION_CONTROL(1),.MULTITURN_CONTROL(1),.SPEED_HANDOFF_CONTROL(1),.POINT_CONTROL(1),.COORDINATED_CONTROL(1),.LIVE_VELOCITY_BYPASS(1),.R14M_ENABLE(1),
   .SPEED_LOOP_CONTROL(1),.LIVE_BRAKE_CONFIG(1),.LIVE_CONFIG(1),.POSITION_CONTROL(1),.FIXED_POSITION(0),
   .BALANCE_SPEED_INPUT(1),.RETURN_CONTROL(1),.INDEPENDENT_RETURN_CAP(1),.K_THETA_TENTHS(1),
   .NEAR_CONTROL(1),.SPEED_FILTER_CONTROL(1),.RETURN_TRIM_ENABLE(0),.ARM_SLEW_U_PER_UPDATE(10),
   .UPDATE_DIV(5),.AUTO_SWINGUP(0)) dut(
   .clk(clk),.rst_n(rst),.control_tick(tick),.system_state(3'd3),.sensor_valid(sensor_valid),
   .theta(theta),.theta_dot(speed),.alpha(alpha),.alpha_dot(ad),.balance_speed_i(speed),
   .balance_speed_valid_i(speed_valid),.balance_speed_select_i(1'b1),.speed_status_i(speed_status),
   .run_i(run),.rearm_i(1'b0),.config_i(cfg),.coordinate_config_i(coord_cfg),.arm_loop_config_i(loop_cfg),
   .point_config_i(point_cfg),.motion_pi_config_i(mp_cfg),
   .dbg_motion_position(rp),.dbg_motion_velocity(rv),.dbg_outer_i_angle(iv),.dbg_speed_status(sd),.velocity_slew_bypass_i(bypass),
   .brake_config_i(64'd0),.return_position_cap_i(return_cap),.k_theta_x10_i(ktheta),.near_config_i(near_cfg),
   .speed_filter_mode_i(2'd0),.settle_config_i(settle_cfg),.motion_config_i(motion_cfg),
   .position_i_config_i(pos_i_cfg),.return_config_i(return_cfg),.pid_config_i(pid_cfg),.pid_permit_i(1'b1),
   .position_set_i(set_position),.position_target_i(target_command),.manual_start_i(1'b0),
   .position_ready_o(position_ready),.position_actual_o(actual),.position_target_o(target_actual),
   .control_u(u),.control_valid(valid),.dbg_aref(ar),.dbg_vref(vr),.dbg_hold_control(hc),
   .dbg_arm_request_q16(aq),.dbg_loop_reference(goal),.dbg_velocity_error(error),
   .dbg_capture_travel(ct),.dbg_settle_status(ss),.dbg_pa(pa),.dbg_pd(pd));

 integer checks=0,j,n,expected;reg signed [63:0] numerator;reg signed [31:0] last_rp;
 task check(input bit ok,input string msg);begin checks=checks+1;if(!ok)$fatal(1,"%s goal=%0d actual=%0d rp=%0d rv=%0d SD=%h MPC=%h motion=%b",msg,goal,ct,rp,rv,sd,mp_cfg,dut.motion_enabled);end endtask
 task update;begin @(negedge clk);while(!valid)@(negedge clk);#1;end endtask
 task byte_rx(input [7:0] v);begin @(negedge clk);rxbyte=v;rxvalid=1;@(negedge clk);rxvalid=0;repeat(5)@(negedge clk);end endtask
 task send(input [3:0] c,input [7:0] i,input signed [31:0] v);
 string line;integer k;begin
 case(c) 0:line="GET INFO";1:line="GET ALL";2:line=$sformatf("SET %0d %0d",i,v);
 3:line="APPLY";4:line="STOP";5:line="START";8:line="GET POS";default:line="INVALID";endcase
 for(k=0;k<line.len();k++)byte_rx(line[k]);byte_rx(13);byte_rx(10);repeat(1000)@(negedge clk);
 end endtask
 function signed [31:0] wrap(input signed [31:0] v);reg signed [31:0] m;begin
  m=v%411775;if(m>205887)m=m-411775;if(m< -205887)m=m+411775;wrap=m;
 end endfunction
 always @(negedge clk)if(rst && valid && follow)begin #2;theta=wrap(rp);speed=rv;end
 task reach(input signed [31:0] target,input integer timeout);begin
  send(2,40,target);check(target_actual==target && bank.response_code==9,"full-width target accepted without relocking captured origin");n=0;
  while(n<timeout && (sd[29] || (ct-target>2 || ct-target< -2)))begin
   update();n=n+1;
   numerator=-($signed(alpha)-$signed(ar))*3300-$signed(ad)*160+$signed(aq);
   expected=numerator/65536;if(expected>1000)expected=1000;if(expected< -1000)expected=-1000;
   check(u==expected,"moving reference and integral stay inside one exact inner PD plus one speed contribution");
  end
  repeat(10)update();
  check(!sd[29] && ct>=target-2 && ct<=target+2 && goal==target,"cumulative position reaches commanded turn count under ideal reference following");
 end endtask
 initial begin
 repeat(3)@(negedge clk);rst=1;repeat(10)@(negedge clk);
 send(2,0,3300);send(2,1,160);send(2,4,1295);send(2,81,983);send(2,82,200);send(2,90,1);
 send(3,0,0);send(5,0,0);repeat(12)update();follow=0;theta=57190;speed=0;repeat(25)update();
 send(2,40,411775);check(target_actual==411775,"ASCII one-turn command latches exactly");
 check(rp>=57190 && rp<60000,"qualified real controller restart begins near actual fifty-degree position without changing cumulative goal");follow=1;
 repeat(12)begin send(8,0,0);check(target_actual==411775,"GET POS never changes fixed final target");end
 send(1,0,0);repeat(150000)@(negedge clk);check(target_actual==411775,"complete GET ALL preserves commanded target while planner moves");
 send(0,0,0);check(target_actual==411775,"GET INFO cannot rewrite target");
 send(2,40,-823550);check(target_actual==-823550,"explicit negative multi-turn command is the only normal rewrite");
 repeat(6)begin send(8,0,0);check(target_actual==-823550,"negative final target holds through repeated polls");end
 send(4,0,0);repeat(3)update();check(target_actual==0 && u==0,"STOP intentionally clears target and drive");
 $display("PASS ASCII UART target persistence %0d assertions: SET, GET POS, GET ALL, GET INFO, retarget, STOP",checks);$finish;
 end
 initial begin #300000000;$fatal(1,"UART target integration timeout");end
endmodule
