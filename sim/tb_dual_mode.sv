`timescale 1ns/1ps
module tb_dual_mode;
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
 wire [223:0] cruise_cfg;wire [63:0] cruise_gains;wire [63:0] point_cfg,mp_cfg;wire signed [31:0] rp,rv,iv;wire [63:0] sd;wire bypass;wire [6:0] return_cap;wire signed [10:0] ktheta;
 wire tx_start;reg tx_busy=0;integer tx_wait=0;
 always @(posedge clk)begin
  if(tx_start)begin tx_busy<=1;tx_wait<=2;end
  else if(tx_wait>0)begin tx_wait<=tx_wait-1;if(tx_wait==1)tx_busy<=0;end
 end
 serial_parameter_bank #(.EXT_POSITION(37)) bank(.clk(clk),.rst_n(rst),.tick_i(1'b1),
  .command_i(cmd),.command_valid_i(cv),.id_i(id),.value_i(value),.stopped_i(!run),.ready_i(1'b1),
  .position_ready_i(position_ready),.position_actual_i(actual),.position_target_i(target_actual),
  .position_set_o(set_position),.position_value_o(target_command),.phase_i(3'd5),.reason_i(4'd0),.tx_busy_i(tx_busy),.tx_start_o(tx_start),
  .run_o(run),.config_o(cfg),.near_config_o(near_cfg),.return_config_o(return_cfg),.coordinate_config_o(coord_cfg),
  .pid_config_o(pid_cfg),.motion_config_o(motion_cfg),.position_i_config_o(pos_i_cfg),.settle_config_o(settle_cfg),
  .cruise_gains_o(cruise_gains),.cruise_config_o(cruise_cfg),.arm_loop_config_o(loop_cfg),.return_position_cap_o(return_cap),.k_theta_x10_o(ktheta),
  .point_config_o(point_cfg),.motion_pi_config_o(mp_cfg),.velocity_slew_bypass_o(bypass));
 reg signed [31:0] theta=0,speed=0,alpha=0,ad=0;
 reg sensor_valid=1,speed_valid=1;reg [36:0] speed_status=37'd131072;reg follow=0;
 wire signed [15:0] u;wire valid;
 wire signed [31:0] ar,vr,aq,goal,ct;wire signed [32:0] error;
 wire [31:0] hc,ss;wire signed [31:0] pa,pd;
 controller #(.DUAL_MODE(1),.PID_CONTROL(1),.MOTION_LEAN_CONTROL(1),.MOTION_CONTROL(1),.MULTITURN_CONTROL(1),.SPEED_HANDOFF_CONTROL(1),.POINT_CONTROL(1),.COORDINATED_CONTROL(1),.LIVE_VELOCITY_BYPASS(1),.R14M_ENABLE(1),
   .SPEED_LOOP_CONTROL(1),.LIVE_BRAKE_CONFIG(1),.LIVE_CONFIG(1),.POSITION_CONTROL(1),.FIXED_POSITION(0),
   .BALANCE_SPEED_INPUT(1),.RETURN_CONTROL(1),.INDEPENDENT_RETURN_CAP(1),.K_THETA_TENTHS(1),
   .NEAR_CONTROL(1),.SPEED_FILTER_CONTROL(1),.RETURN_TRIM_ENABLE(0),.ARM_SLEW_U_PER_UPDATE(10),
   .UPDATE_DIV(5),.AUTO_SWINGUP(0)) dut(
   .clk(clk),.rst_n(rst),.control_tick(tick),.system_state(3'd3),.sensor_valid(sensor_valid),
   .theta(theta),.theta_dot(speed),.alpha(alpha),.alpha_dot(ad),.balance_speed_i(speed),
   .balance_speed_valid_i(speed_valid),.balance_speed_select_i(1'b1),.speed_status_i(speed_status),
   .cruise_gains_i(cruise_gains),.cruise_config_i(cruise_cfg),.run_i(run),.rearm_i(1'b0),.config_i(cfg),.coordinate_config_i(coord_cfg),.arm_loop_config_i(loop_cfg),
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


 wire signed [15:0] baseline_u;
 position_original_controller #(.MOTION_LEAN_CONTROL(1),.MOTION_CONTROL(1),.MULTITURN_CONTROL(1),.SPEED_HANDOFF_CONTROL(1),.POINT_CONTROL(1),.COORDINATED_CONTROL(1),.LIVE_VELOCITY_BYPASS(1),.R14M_ENABLE(1),
   .SPEED_LOOP_CONTROL(1),.LIVE_BRAKE_CONFIG(1),.LIVE_CONFIG(1),.POSITION_CONTROL(1),.FIXED_POSITION(0),
   .BALANCE_SPEED_INPUT(1),.RETURN_CONTROL(1),.INDEPENDENT_RETURN_CAP(1),.K_THETA_TENTHS(1),
   .NEAR_CONTROL(1),.SPEED_FILTER_CONTROL(1),.RETURN_TRIM_ENABLE(0),.ARM_SLEW_U_PER_UPDATE(10),
   .UPDATE_DIV(5),.AUTO_SWINGUP(0)) baseline(
   .clk(clk),.rst_n(rst),.control_tick(tick),.system_state(3'd3),.sensor_valid(sensor_valid),
   .theta(theta),.theta_dot(speed),.alpha(alpha),.alpha_dot(ad),.balance_speed_i(speed),
   .balance_speed_valid_i(speed_valid),.balance_speed_select_i(1'b1),.speed_status_i(speed_status),
   .run_i(run),.rearm_i(1'b0),.config_i(cfg),.coordinate_config_i(coord_cfg),.arm_loop_config_i(loop_cfg),
   .point_config_i(point_cfg),.motion_pi_config_i(mp_cfg),
   .dbg_motion_position(),.dbg_motion_velocity(),.dbg_outer_i_angle(),.dbg_speed_status(),.velocity_slew_bypass_i(bypass),
   .brake_config_i(64'd0),.return_position_cap_i(return_cap),.k_theta_x10_i(ktheta),.near_config_i(near_cfg),
   .speed_filter_mode_i(2'd0),.settle_config_i(settle_cfg),.motion_config_i(motion_cfg),
   .position_i_config_i(pos_i_cfg),.return_config_i(return_cfg),.pid_config_i(pid_cfg),.pid_permit_i(1'b1),
   .position_set_i(set_position),.position_target_i(target_command),.manual_start_i(1'b0),
   .position_ready_o(),.position_actual_o(),.position_target_o(),
   .control_u(baseline_u),.control_valid(),.dbg_aref(),.dbg_vref(),.dbg_hold_control(),
   .dbg_arm_request_q16(),.dbg_loop_reference(),.dbg_velocity_error(),
   .dbg_capture_travel(),.dbg_settle_status(),.dbg_pa(),.dbg_pd());
 wire signed [15:0] cruise_u;wire [1279:0] cruise_cfg_common={cfg[1279:64],cruise_gains};
 cruise_original_controller #(.CRUISE_CONTROL(1),.UPDATE_DIV(5),.LIVE_CONFIG(1),.POSITION_CONTROL(1),
 .FIXED_POSITION(1),.PID_CONTROL(1),.POSITION_I_CONTROL(0),.BALANCE_SPEED_INPUT(1),.RETURN_CONTROL(1),
 .SPEED_FILTER_CONTROL(1),.NEAR_CONTROL(1),.RETURN_TRIM_ENABLE(0)) cruise_baseline(
 .clk(clk),.rst_n(rst),.control_tick(tick),.system_state(3'd3),.sensor_valid(sensor_valid),
 .theta(theta),.theta_dot(speed),.alpha(alpha),.alpha_dot(ad),.balance_speed_i(speed),
 .balance_speed_valid_i(speed_valid),.balance_speed_select_i(1'b1),
 .run_i(run),.rearm_i(1'b0),.config_i(cruise_cfg_common),.cruise_config_i(cruise_cfg),
 .position_set_i(1'b0),.position_target_i(19'd0),.pid_permit_i(1'b1),.pid_config_i(pid_cfg),
 .position_i_config_i(pos_i_cfg),.near_config_i(near_cfg),.return_config_i(return_cfg),
 .speed_filter_mode_i(2'd0),.settle_config_i(settle_cfg),.motion_config_i(motion_cfg),.control_u(cruise_u));
 integer checks=0,i;reg signed[15:0] saved_u;
 task check(input bit ok,input string msg);begin checks++;if(!ok)$fatal(1,"%s u=%0d pos=%0d cruise=%0d mode=%0d vr=%0d",msg,u,baseline_u,cruise_u,loop_cfg[31:0],dut.cruise_vref);end endtask
 task update;begin @(negedge clk);while(!valid)@(negedge clk);#1;end endtask
 task byte_rx(input [7:0] v);begin @(negedge clk);rxbyte=v;rxvalid=1;@(negedge clk);rxvalid=0;repeat(5)@(negedge clk);end endtask
 task send(input [3:0] c,input [7:0] i,input signed [31:0] v);
 string line;integer k;begin
 case(c) 0:line="GET INFO";1:line="GET ALL";2:line=$sformatf("SET %0d %0d",i,v);
 3:line="APPLY";4:line="STOP";5:line="START";8:line="GET POS";default:line="INVALID";endcase
 for(k=0;k<line.len();k++)byte_rx(line[k]);byte_rx(13);byte_rx(10);repeat(1200)@(negedge clk);
 end endtask
 initial begin
 repeat(8)@(negedge clk);rst=1;repeat(50)@(negedge clk);
$display("CHECKPOINT reset t=%0t",$time); check(loop_cfg[31:0]==1,"default positioning mode");
 send(2,81,131);send(2,98,1);send(3,0,0);send(5,0,0);repeat(60)update();
$display("CHECKPOINT start t=%0t",$time); check(position_ready,"position ready");send(2,40,411775);
 for(i=0;i<500;i++)begin
 theta=(i*31)%411775;speed=(i%5)*1000;alpha=(i%11)*31;ad=(i%7)*97;update();
 check(u===baseline_u,"mode1 exactly equals supplied R15N motor output");
 check(dut.integral_q16==0,"mode1 posture I disabled");
 end
$display("CHECKPOINT position compared t=%0t",$time); check(dut.coordinated_outer_loop.coordinate_i.catch_direction_known_i===1'b1,"mode1 pure-PD catch direction remains enabled despite shared PID hardware");
 for(i=0;i<50;i++)begin
  alpha=i%2 ? -4000:4000;ad=i%2 ? -70000:70000;update();
  check(u===baseline_u,"mode1 disturbed catch equals original R15N");
 end
 alpha=0;ad=0;
 send(2,80,0);check(loop_cfg[31:0]==1,"SET changes pending only during run");
 send(3,0,0);check(loop_cfg[31:0]==1 && bank.response_code==8'h83,"running APPLY rejected");
 send(4,0,0);repeat(3)update();check(u==0,"STOP zero output");
 send(3,0,0);check(loop_cfg[31:0]==0,"STOP APPLY switches mode0");
 check(cruise_gains=={32'd100,32'd2400},"mode0 original inner gain profile");
 theta=0;speed=0;alpha=0;ad=0;send(5,0,0);repeat(300)update();
 check(dut.cruise_vref==23400,"cruise reaches reference speed");
 check(!dut.motion_enabled && dut.outer_i_angle==0,"mode0 motion and outer integral reset");
 check(!position_ready,"mode0 rejects positioning commands");
 for(i=0;i<500;i++)begin
 theta=(i*73)%411775;speed=(i%19)*400;alpha=(i%13)*23;ad=(i%9)*67;update();
 check(u===cruise_u,"mode0 exactly equals supplied R14E cruise motor output");
 end
 alpha=4000;ad=70000;update();check(dut.cruise_recovering && dut.cruise_vref==0,"disturbance withdraws cruise");
 check(dut.arm_request==0 || dut.arm_request[63]==dut.pendulum_request[63],"catch priority");
 alpha=0;ad=0;repeat(350)update();check(dut.cruise_vref==23400,"automatic recovery resumes cruise");
 speed_valid=0;update();check(dut.cruise_state==5 && dut.cruise_vref==0,"unknown speed withdraws cruise");
 speed_valid=1;repeat(350)update();
 send(4,0,0);send(2,80,1);send(3,0,0);check(loop_cfg[31:0]==1,"switch back positioning");
 check(dut.cruise_vref==0,"old cruise reference cleared");
 send(5,0,0);repeat(60)update();check(target_actual==0,"old positioning task cleared");
 check(dut.integral_q16==0,"old posture integral cleared");
 sensor_valid=0;repeat(2)update();check(u==0,"sensor protection");
 $display("PASS dual-mode %0d assertions: exact source outputs, stop-only switch, profiles, reset, recovery, protections",checks);$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
