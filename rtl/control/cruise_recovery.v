`timescale 1ns/1ps
`default_nettype none
// 200Hz reference supervisor. No motor output: original R14E PD/PID remains live.
// Inputs: Q16 rad, Q16 rad/s. Config words77..83, STOP-only atomic application.
module cruise_recovery(
 input wire clk,rst_n,update_i,ready_i,speed_known_i,
 input wire [223:0] config_i,
 input wire signed [31:0] position_i,alpha_i,alpha_dot_i,
 output wire selected_o,correction_o,recovering_o,
 output wire signed [31:0] velocity_ref_o,position_error_o,position_ref_o,
 output wire [2:0] state_o
);
 wire enabled=config_i[31:0]==1;
 wire signed [31:0] target=$signed(config_i[63:32]);
 wire [31:0] accel=config_i[95:64],ag=config_i[127:96],dg=config_i[159:128],quiet=config_i[191:160];
 wire config_ok=target>=-65536 && target<=65536 && accel>=16 && accel<=1024 &&
     ag>=1144 && ag<=11438 && dg>=6554 && dg<=131072 && quiet>=20 && quiet<=400 &&
     config_i[223:192]>=10 && config_i[223:192]<=200;
 wire [32:0] aa=alpha_i[31]?-$signed({alpha_i[31],alpha_i}):{1'b0,alpha_i};
 wire [32:0] ad=alpha_dot_i[31]?-$signed({alpha_dot_i[31],alpha_dot_i}):{1'b0,alpha_dot_i};
 wire bad=aa>=ag || ad>=dg;
 wire settled=aa<(ag>>1) && ad<(dg>>1);
 reg recovering;reg [8:0] stable_count;
 reg signed [17:0] velocity;
 reg signed [31:0] reference;
 reg signed [8:0] remainder;
 assign selected_o=enabled && config_ok;
 // Enter immediately; release only on an update after a full quiet interval.
 assign recovering_o=selected_o && ready_i && (recovering || bad || !speed_known_i);
 assign correction_o=selected_o && ready_i && !recovering && !bad && speed_known_i;
 assign velocity_ref_o=correction_o ? {{14{velocity[17]}},velocity}:32'sd0;
 wire signed [32:0] error=$signed({position_i[31],position_i})-$signed({reference[31],reference});
 // Bound moving-reference error to +/-10deg. Prevent a catch episode from
 // storing a distant positional task which would pull the arm back afterwards.
 assign position_error_o=!correction_o ? 32'sd0:error>11438 ? 32'sd11438:
     error< -11438 ? -32'sd11438:error[31:0];
 assign position_ref_o=reference;
 assign state_o=!selected_o || !ready_i ? 3'd0:!speed_known_i ? 3'd5:
     recovering_o ? (bad ? 3'd4:3'd1):$signed(velocity)!=target ? 3'd2:3'd3;
 wire signed [32:0] delta=$signed({target[31],target})-$signed({{15{velocity[17]}},velocity});
 wire signed [32:0] nv=delta>$signed({1'b0,accel}) ? $signed(velocity)+$signed({1'b0,accel}):
     delta< -$signed({1'b0,accel}) ? $signed(velocity)-$signed({1'b0,accel}):$signed(target);
 wire signed [19:0] travel=$signed({{2{nv[17]}},nv[17:0]})+$signed(remainder);
 // Exact signed fraction: preserve sub-Q16 displacement across all updates.
 // Shared restoring divide by 200, 20 FPGA clocks per 5ms update.
 // Avoid a large combinational constant divider in the 20K device.
 reg div_busy;reg [4:0] div_count;reg [19:0] quotient;
 reg [7:0] div_remainder;reg div_negative;reg signed[31:0] held_position;
 wire [8:0] trial={div_remainder,quotient[19]};
 wire take=trial>=9'd200;
 wire [8:0] rem_next=take ? trial-9'd200:trial;
 wire [19:0] quotient_next={quotient[18:0],take};
 wire signed [20:0] increment=div_negative ? -$signed({1'b0,quotient_next}):$signed({1'b0,quotient_next});
 wire signed [32:0] next_reference=$signed({reference[31],reference})+$signed(increment);
 wire signed [32:0] follow=$signed({held_position[31],held_position})-next_reference;
 wire signed [32:0] bounded_reference=follow>11438 ? $signed({held_position[31],held_position})-33'sd11438:
     follow< -11438 ? $signed({held_position[31],held_position})+33'sd11438:next_reference;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin recovering<=1;stable_count<=0;velocity<=0;reference<=0;remainder<=0;div_busy<=0;div_count<=0;quotient<=0;div_remainder<=0;div_negative<=0;held_position<=0;end
  else if(!selected_o || !ready_i)begin
   recovering<=1;stable_count<=0;velocity<=0;reference<=position_i;remainder<=0;div_busy<=0;
  end else if(update_i)begin
   div_busy<=0;
   if(bad || !speed_known_i)begin
    recovering<=1;stable_count<=0;velocity<=0;reference<=position_i;remainder<=0;
   end else if(recovering)begin
    velocity<=0;reference<=position_i;remainder<=0;
    if(!settled)stable_count<=0;
    else if(stable_count>=quiet-1)begin recovering<=0;stable_count<=0;end
    else stable_count<=stable_count+1'b1;
   end else begin
    velocity<=nv[17:0];held_position<=position_i;
    quotient<=travel[19] ? -travel:travel;div_negative<=travel[19];
    div_remainder<=0;div_count<=20;div_busy<=1;
   end
  end else if(div_busy)begin
   quotient<=quotient_next;div_remainder<=rem_next[7:0];div_count<=div_count-1'b1;
   if(div_count==1)begin
    div_busy<=0;
    remainder<=div_negative ? -$signed({1'b0,rem_next[7:0]}):$signed({1'b0,rem_next[7:0]});
    reference<=bounded_reference>2147483647 ? 32'sh7fffffff:
       bounded_reference< -33'sd2147483648 ? 32'sh80000000:bounded_reference[31:0];
   end
  end
 end
endmodule
`default_nettype wire
