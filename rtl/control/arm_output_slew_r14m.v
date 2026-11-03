`timescale 1ns/1ps
`default_nettype none
module arm_output_slew_r14m #(
 parameter integer BRAKE_STEP_U=20,parameter integer LIVE_BRAKE_CONFIG=0,
 parameter integer STEP_U=10,parameter integer VELOCITY_TRANSITION=0,
 parameter integer RELEASE_STEP_U=50
)(
 input wire [5:0] brake_step_i,
 input wire [9:0] release_rate_i,
 input wire release_enable_i,
 input wire brake_i,
 input wire clk,rst_n,update_i,ready_i,
 input wire stage_i, // actual balance stage, independent of temporary permission loss
 // R15C: only runtime mode1 uses this policy. Legacy callers keep the old path.
 input wire transition_i,speed_valid_i,catch_priority_i,
 input wire signed [63:0] pendulum_request_i,
 input wire signed [63:0] request_i,
 output wire signed [63:0] request_o,
 output wire transition_releasing_o,transition_catch_o
);
 localparam signed [64:0] CAP_Q32=65'sd4294967296000; // 1000 U
 localparam signed [48:0] STEP_Q32=$signed(STEP_U)*49'sd4294967296;
 wire [5:0] safe_brake_step=LIVE_BRAKE_CONFIG?((brake_step_i>=10 && brake_step_i<=40)?brake_step_i:6'd20):BRAKE_STEP_U;
 wire signed [48:0] BRAKE_STEP_Q32=LIVE_BRAKE_CONFIG?$signed({11'd0,safe_brake_step,32'd0}):((49'sd1*BRAKE_STEP_U)<<<32);
 wire signed [48:0] selected_step=brake_i?BRAKE_STEP_Q32:STEP_Q32;
 wire [9:0] release_rate=(release_rate_i>=10 && release_rate_i<=1000)?release_rate_i:10'd1000;
 wire signed [48:0] release_step=$signed({7'd0,release_rate,32'd0});
 wire use_release=LIVE_BRAKE_CONFIG && release_enable_i;
 localparam signed [48:0] WITHDRAW_Q32=$signed(RELEASE_STEP_U)*49'sd4294967296;
 wire transition_mode=VELOCITY_TRANSITION && transition_i;
 reg signed [47:0] previous;
 reg entry_pending;
 reg signed [64:0] bounded_input;
 reg signed [48:0] target_w,previous_w,next_w,delta_w;
 reg signed [63:0] safe_output;
 reg releasing,catch_release;
 wire signed [64:0] input_w=$signed({request_i[63],request_i});
 wire state_bad=($signed(previous)>48'sd4294967296000) ||
                ($signed(previous)< -48'sd4294967296000);
 always @* begin
  // Clamp BEFORE narrowing. Do not negate an unbounded input (INT64_MIN).
  if(input_w>CAP_Q32) bounded_input=CAP_Q32;
  else if(input_w< -CAP_Q32) bounded_input=-CAP_Q32;
  else bounded_input=input_w;
  target_w=$signed(bounded_input[48:0]);
  // Unknown speed means a zero target, never a held/expanded stale demand.
  if(transition_mode && !speed_valid_i)target_w=0;
  previous_w=$signed({previous[47],previous});
  delta_w=target_w-previous_w;
  next_w=49'sd0;
  releasing=1'b0;catch_release=1'b0;
  if(rst_n && ready_i && stage_i) begin
   if(state_bad) next_w=49'sd0; // corrupt history must never reach the motor
   else if(entry_pending) next_w=target_w; // capture entry: seed current bounded request
   else if(transition_mode)begin
    // A stale contribution must not oppose urgent pendulum capture. Direction
    // comparisons use sign bits, not extra multipliers. This exception may jump.
    if((!speed_valid_i || (target_w!=0 && previous_w!=0 && target_w[48]!=previous_w[48])) &&
       catch_priority_i && previous_w!=0 && pendulum_request_i!=0 &&
       previous_w[48]!=pendulum_request_i[63])begin
     next_w=0;releasing=1'b1;catch_release=1'b1;
    end else if(target_w!=0 && previous_w!=0 && target_w[48]!=previous_w[48])begin
     // Withdraw the old direction to exact zero; start the other direction only
     // on a later control update. PWM reversal blanking stays in the motor chain.
     releasing=1'b1;
     if(previous_w>WITHDRAW_Q32)next_w=previous_w-WITHDRAW_Q32;
     else if(previous_w< -WITHDRAW_Q32)next_w=previous_w+WITHDRAW_Q32;
     else next_w=0;
    end else if((previous_w>0 && target_w<=previous_w) ||
                (previous_w<0 && target_w>=previous_w))begin
     releasing=(target_w!=previous_w);
     if(delta_w>WITHDRAW_Q32)next_w=previous_w+WITHDRAW_Q32;
     else if(delta_w< -WITHDRAW_Q32)next_w=previous_w-WITHDRAW_Q32;
     else next_w=target_w;
    end else if(delta_w>selected_step)next_w=previous_w+selected_step;
    else if(delta_w< -selected_step)next_w=previous_w-selected_step;
    else next_w=target_w;
   end
   else if((target_w>0 && previous_w<0) || (target_w<0 && previous_w>0))
    begin // bounded release of old direction; cannot cross zero in this update
     if(!use_release)next_w=0;
     else if(previous_w>release_step)next_w=previous_w-release_step;
     else if(previous_w< -release_step)next_w=previous_w+release_step;
     else next_w=0;
    end
   else if((previous_w>0 && target_w<=previous_w) ||
           (previous_w<0 && target_w>=previous_w))begin
     if(use_release && delta_w>release_step)next_w=previous_w+release_step;
     else if(use_release && delta_w< -release_step)next_w=previous_w-release_step;
     else next_w=target_w;
    end
   else if(delta_w>selected_step) next_w=previous_w+selected_step;
   else if(delta_w< -selected_step) next_w=previous_w-selected_step;
   else next_w=target_w;
  end
  // Independent post-arithmetic range guard, BEFORE slicing/sign extension.
  if(next_w>49'sd4294967296000 || next_w< -49'sd4294967296000)
   safe_output=64'sd0;
  else safe_output=$signed({{15{next_w[48]}},next_w});
 end
 assign request_o=safe_output;
 assign transition_releasing_o=releasing;
 assign transition_catch_o=catch_release;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin previous<=48'sd0;entry_pending<=1'b1;end
  else begin
   // Only leaving the balance stage arms a full-request capture seed.
   // Permission loss inside balance clears history but MUST NOT re-arm capture.
   if(!stage_i) entry_pending<=1'b1;
   else if(update_i && ready_i) entry_pending<=1'b0;
   if(!ready_i || !stage_i) previous<=48'sd0;
   else if(update_i) previous<=safe_output[47:0];
  end
 end
 // synthesis translate_off
 initial if(BRAKE_STEP_U<STEP_U || BRAKE_STEP_U>1000 || STEP_U<1 || STEP_U>1000 ||
            RELEASE_STEP_U<1 || RELEASE_STEP_U>1000)$fatal(1,"Invalid arm slew step");
 always @(posedge clk) if(rst_n && update_i && ready_i) begin
  if(request_o>64'sd4294967296000 || request_o< -64'sd4294967296000)
   $fatal(1,"Arm output outside independent legal envelope");
 end
 // synthesis translate_on
endmodule
`default_nettype wire
