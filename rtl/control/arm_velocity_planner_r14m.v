`default_nettype none
// All distances Q16 rad, velocities Q16 rad/s, acceleration Q16 rad/s^2.
// Shared shift/add multiplier and restoring divider: no parallel variable multiplier/divider.
module arm_velocity_planner_r14m #(
 parameter integer BRAKE_DISTANCE_GAIN_Q8=384, LIVE_BRAKE_CONFIG=0,
 parameter integer PLANNER_PERIOD_MS=5, PLANNER_V_MAX_Q16=65536,
 parameter integer PLANNER_A_ACCEL_Q16=32768, PLANNER_A_BRAKE_Q16=65536,
 parameter integer PLANNER_NEAR_GAIN_Q16=131072,
 parameter integer PLANNER_ENTER_Q16=1144, PLANNER_EXIT_Q16=2288
)(
 input wire [9:0] brake_gain_i,
 input wire brake_enable_i,
 input wire [18:0] brake_accel_i,
 input wire [16:0] approach_speed_i,
 input wire clk,rst_n,update_i,enabled_i,speed_valid_i,
 input wire signed [31:0] position_target_i,position_actual_i,actual_arm_speed_i,
 input wire [31:0] sample_id_i,
 output reg busy_o,valid_o,reference_valid_o,
 output reg [31:0] sample_id_o,
 output reg signed [32:0] position_error_o,
 output reg [32:0] remaining_distance_o,
 output reg [31:0] allowed_speed_o,
 output reg signed [31:0] envelope_reference_o,arm_velocity_reference_o,
 output reg signed [32:0] velocity_error_o,
 output reg [2:0] braking_state_o,
 output reg target_band_o,overrun_o,
 output reg [31:0] brake_distance_o,
 output reg brake_risk_o,brake_distance_valid_o
);
 wire signed [32:0] err=$signed({position_target_i[31],position_target_i})-$signed({position_actual_i[31],position_actual_i});
 wire [32:0] distance_now=err[32]?-err:err;
 wire [31:0] speed_abs=actual_arm_speed_i[31]?$unsigned(-actual_arm_speed_i):$unsigned(actual_arm_speed_i);
 wire [9:0] gain=LIVE_BRAKE_CONFIG?brake_gain_i:BRAKE_DISTANCE_GAIN_Q8;
 wire [18:0] accel=LIVE_BRAKE_CONFIG?brake_accel_i:PLANNER_A_BRAKE_Q16;
 wire [16:0] approach=LIVE_BRAKE_CONFIG?approach_speed_i:17'd23400;
 wire brake_en=LIVE_BRAKE_CONFIG?brake_enable_i:1'b1;
 reg signed [32:0] he;
 reg signed [31:0] hv;
 reg [32:0] hd,usable;
 reg [31:0] sid;
 reg [9:0] hg;
 reg [18:0] ha;
 reg [16:0] hs;
 reg hb,hen;
 reg [3:0] state,ret_state;
 reg [6:0] count;
 // Products: abs(signed32)^2 * gain <= 2^62*768 < 2^72.
 // Multiplier accumulator and divider dividend/quotient have disjoint lifetimes.
 // Keep the original 74 division clocks (two leading-zero clocks) for cycle equivalence.
 reg [71:0] work,shifted;
 reg [32:0] mult;
 reg [27:0] denom,rem;
 wire [28:0] trial={rem,work[71]};
 wire [28:0] div_difference=trial-{1'b0,denom};
 wire take=!div_difference[28]; // trial < 2^28; MSB is the subtraction borrow
 wire [28:0] rem_next=take?div_difference:trial;
 wire [71:0] product_next=work+(mult[0]?shifted:72'd0);
 reg [33:0] rad;
 reg [18:0] sr;
 reg [16:0] root;
 wire [20:0] strial={sr,rad[33:32]};
 wire [20:0] subtrial={2'd0,root,2'b01};
 wire [21:0] sqrt_difference={1'b0,strial}-{1'b0,subtrial};
 wire stake=!sqrt_difference[21];
 wire [16:0] root_next={root[15:0],stake};
 reg [31:0] bd;
 localparam integer LIMIT_W=$clog2(PLANNER_V_MAX_Q16+1);
 reg [LIMIT_W-1:0] limit;
 localparam integer STEP_W=$clog2((ACC_STEP>DEC_STEP?ACC_STEP:DEC_STEP)+1)+2;
 localparam integer REF_W=(LIMIT_W+2>STEP_W)?LIMIT_W+2:STEP_W;
 reg signed [REF_W-1:0] desired,prev,nxt,delta;
 reg [2:0] next_state;
 wire toward=(he>0 && hv>0)||(he<0 && hv<0);
 wire risk=hen && hv!=0 && ((toward && {1'b0,bd}>=hd) || !toward);
 localparam signed [32:0] ACC_STEP=PLANNER_A_ACCEL_Q16*33'sd5/1000;
 // Reference deceleration rate is unchanged. The runtime acceleration parameter
 // describes the stopping envelope, not a claimed measured actuator acceleration.
 localparam signed [32:0] DEC_STEP=PLANNER_A_BRAKE_Q16*33'sd5/1000;
 reg signed [REF_W-1:0] step;
 always @* begin
  limit=hen?{15'd0,root}:PLANNER_V_MAX_Q16;
  if(limit>PLANNER_V_MAX_Q16)limit=PLANNER_V_MAX_Q16;
  if(hen && (risk || hd<=33'd11438) && limit>{15'd0,hs})limit={15'd0,hs}; // 10 degrees
  if(usable<32768 && limit>{usable[15:0],1'b0})limit={usable[15:0],1'b0};
  desired=he[32]?-$signed({1'b0,limit}):$signed({1'b0,limit});
  if(hb)desired=0;
  prev=$signed({arm_velocity_reference_o[31],arm_velocity_reference_o});
  step=((prev>0 && desired<prev)||(prev<0 && desired>prev))?DEC_STEP:ACC_STEP;
  delta=desired-prev;
  nxt=delta>step?prev+step:delta< -step?prev-step:desired;
  next_state=limit<PLANNER_V_MAX_Q16?2:1;
  if(!toward && hv!=0)next_state=4;
  else if(toward && (hv[31]?$unsigned(-hv):$unsigned(hv))>limit)next_state=3;
  if(hb)next_state=5;
  if((nxt>0 && he<0)||(nxt<0 && he>0))next_state=6;
 end
 localparam IDLE=0,MUL=1,DIV=2,SQUARE_DONE=3,GAIN_DONE=4,DIST_DONE=5,ENV_PRODUCT=6,ENV_DIV=7,ROOT=8,DONE=9;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;ret_state<=0;count<=0;work<=0;shifted<=0;mult<=0;denom<=1;rem<=0;
   he<=0;hv<=0;hd<=0;usable<=0;sid<=0;hg<=384;ha<=65536;hs<=23400;hb<=0;hen<=0;
   rad<=0;sr<=0;root<=0;bd<=0;
   busy_o<=0;valid_o<=0;reference_valid_o<=0;sample_id_o<=0;position_error_o<=0;remaining_distance_o<=0;
   allowed_speed_o<=0;envelope_reference_o<=0;arm_velocity_reference_o<=0;velocity_error_o<=0;
   braking_state_o<=0;target_band_o<=0;overrun_o<=0;brake_distance_o<=0;brake_risk_o<=0;brake_distance_valid_o<=0;
  end else begin
   valid_o<=0;
   if(!enabled_i)begin
    state<=IDLE;busy_o<=0;reference_valid_o<=0;arm_velocity_reference_o<=0;envelope_reference_o<=0;
    allowed_speed_o<=0;velocity_error_o<=0;braking_state_o<=0;target_band_o<=0;overrun_o<=0;
    brake_risk_o<=0;brake_distance_valid_o<=0;
   end else if(!speed_valid_i)begin
    // Invalid is not zero speed. Discard any pending calculation, invalidate
    // the loop and distance, retain last measured/reference values for diagnosis.
    state<=IDLE;busy_o<=0;reference_valid_o<=0;braking_state_o<=7;brake_risk_o<=0;brake_distance_valid_o<=0;
   end else begin
    if(busy_o && update_i)overrun_o<=1;
    case(state)
    IDLE:if(update_i)begin
     he<=err;hv<=actual_arm_speed_i;hd<=distance_now;sid<=sample_id_i;
     usable<=distance_now>PLANNER_ENTER_Q16?distance_now-PLANNER_ENTER_Q16:0;
     hb<=target_band_o?distance_now<=PLANNER_EXIT_Q16:distance_now<=PLANNER_ENTER_Q16;
     hg<=(gain>=256 && gain<=768)?gain:384;ha<=(accel>=6554 && accel<=262144)?accel:65536;
     hs<=(approach>=3277 && approach<=65536)?approach:23400;hen<=brake_en;
     work<=0;shifted<={40'd0,speed_abs};mult<={1'b0,speed_abs};count<=32;ret_state<=SQUARE_DONE;state<=MUL;busy_o<=1;
    end
    MUL:begin
     work<=product_next;shifted<=shifted<<1;mult<=mult>>1;count<=count-1'b1;
     if(count==1)state<=ret_state;
    end
    DIV:begin
     if(count<=72)begin work<={work[70:0],take};rem<=rem_next[27:0];end
     count<=count-1'b1;
     if(count==1)state<=ret_state;
    end
    SQUARE_DONE:begin shifted<=work;mult<={23'd0,hg};work<=0;count<=10;ret_state<=GAIN_DONE;state<=MUL;end
    GAIN_DONE:begin denom<={ha,9'd0};rem<=0;count<=74;ret_state<=DIST_DONE;state<=DIV;end
    DIST_DONE:begin
     bd<=|work[71:32]?32'hffffffff:work[31:0];
     // 2*a*d*256 in Q32, then divide by Q8 gain.
     shifted<={44'd0,ha,9'd0};mult<=usable;work<=0;count<=33;ret_state<=ENV_PRODUCT;state<=MUL;
    end
    ENV_PRODUCT:begin denom<={18'd0,hg};rem<=0;count<=74;ret_state<=ENV_DIV;state<=DIV;end
    ENV_DIV:begin rad<=work>72'd4294967296 ?34'd4294967296:work[33:0];root<=0;sr<=0;count<=17;state<=ROOT;end
    ROOT:begin rad<=rad<<2;sr<=stake?sqrt_difference[18:0]:strial[18:0];root<=root_next;count<=count-1'b1;if(count==1)state<=DONE;end
    DONE:begin
     // A completed calculation recovers the transient deadline fault.
     // A coincident update while busy remains a fault for this completion.
     overrun_o<=update_i;
     state<=IDLE;busy_o<=0;valid_o<=1;reference_valid_o<=1;sample_id_o<=sid;
     position_error_o<=he;remaining_distance_o<=hd;target_band_o<=hb;allowed_speed_o<=limit;
     envelope_reference_o<=desired;arm_velocity_reference_o<=nxt;velocity_error_o<=nxt-$signed({hv[31],hv});
     braking_state_o<=next_state;brake_distance_o<=bd;brake_risk_o<=risk;brake_distance_valid_o<=1;
    end
    default:begin state<=IDLE;busy_o<=0;reference_valid_o<=0;end
    endcase
   end
  end
 end
endmodule
`default_nettype wire
