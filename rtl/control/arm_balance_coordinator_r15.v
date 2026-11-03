`timescale 1ns/1ps
`default_nettype none
// R15J: POSITION ONLY -> bounded pendulum-angle reference.
// Velocity damping is a separate U-domain path in controller.v.
// position_error = measured arm position - fixed target; all angles Q16 rad.
// Signed gains use the original machine feedback direction. No position I.
// Only reference growth is slewed; the inner pendulum P/D is never slewed.
module arm_balance_coordinator_r15 #(parameter integer POINT_CONTROL=0, TARGET_BITS=19, MOTION_CONTROL=0, MOTION_LEAN_CONTROL=0)(
    // R15G_TRACE_BEGIN: read-only probes; no signal below feeds control.
    output wire [3:0] trace_reference_reason_o,
    // R15G_TRACE_END
    input wire clk,rst_n,update_i,ready_i,speed_known_i,stopped_i,
    input wire catch_direction_known_i,
    input wire motion_active_i,output_saturated_i,
    input wire signed [14:0] motion_lean_i,
    input wire signed [31:0] goal_error_i,
    input wire [15:0] integral_config_i,
    output wire signed [13:0] integral_angle_o,
    output wire [3:0] integral_status_o,
    input wire [63:0] point_config_i,
    input wire signed [TARGET_BITS-1:0] point_target_i,
    output wire [7:0] point_status_o,
    output wire point_near_o,point_braking_o,
    input wire signed [31:0] position_error_i,speed_i,alpha_i,alpha_dot_i,
    input wire signed [14:0] position_gain_i,
    input wire reverse_i,
    input wire [12:0] angle_limit_i,
    input wire [9:0] angle_step_i,
    input wire [14:0] hold_band_i,
    input wire [13:0] alpha_gate_i,
    input wire [17:0] alpha_dot_gate_i,
    output wire signed [31:0] target_angle_o,reference_angle_o,
    output wire [2:0] state_o,
    output wire hold_o,recovery_o,limited_o,known_o
);
    wire point_ok=!POINT_CONTROL || !point_config_i[0] ||
        (point_config_i[17:1]>=2288 && point_config_i[17:1]<=102944 &&
         point_config_i[34:18]>=3277 && point_config_i[34:18]<=65536 && point_config_i[42:35]<=200);
    wire config_ok=point_ok && position_gain_i>=-8192 && position_gain_i<=8192 &&
        angle_limit_i>=572 && angle_limit_i<=5719 &&
        angle_step_i>=16 && angle_step_i<=512 &&
        hold_band_i>=2288 && hold_band_i<=22876 &&
        alpha_gate_i>=1144 && alpha_gate_i<=11438 &&
        alpha_dot_gate_i>=6554 && alpha_dot_gate_i<=131072;
    // Position validity comes from ready_i (sensor/run/position permission).
    // A missing velocity estimate must not clear a valid position correction.
    // Keep known_o's telemetry meaning: VELOCITY is confirmed, not position.
    wire known=ready_i && config_ok;
    assign known_o=known && speed_known_i;
    // These magnitudes feed only bounded threshold comparisons. Keep exact
    // values inside each comparison range; larger signed inputs saturate ABOVE
    // every possible threshold. INT32_MIN therefore remains out of range.
    wire signed [31:0] point_error=MOTION_CONTROL ? goal_error_i:position_error_i;
    wire [15:0] xp=point_error[31:15]!={17{point_error[31]}} ? 16'hffff:
        (point_error[31] ? -point_error[15:0]:point_error[15:0]);
    wire [13:0] av=speed_i[31:13]!={19{speed_i[31]}} ? 14'h3fff:
        (speed_i[31] ? -speed_i[13:0]:speed_i[13:0]);
    wire [14:0] aa=alpha_i[31:14]!={18{alpha_i[31]}} ? 15'h7fff:
        (alpha_i[31] ? -alpha_i[14:0]:alpha_i[14:0]);
    wire [18:0] ad=alpha_dot_i[31:18]!={14{alpha_dot_i[31]}} ? 19'h7ffff:
        (alpha_dot_i[31] ? -alpha_dot_i[18:0]:alpha_dot_i[18:0]);
    wire signed [32:0] recovery_error=MOTION_LEAN_CONTROL && motion_active_i ?
        $signed({alpha_i[31],alpha_i})-$signed({{19{previous[13]}},previous}):
        $signed({alpha_i[31],alpha_i});
    wire [32:0] recovery_abs=recovery_error[32] ? -recovery_error:recovery_error;
    wire worsening=(recovery_error>0 && alpha_dot_i>0)||(recovery_error<0 && alpha_dot_i<0);
    // Select entry/release thresholds before the existing comparisons: no
    // duplicate wide magnitude comparisons or threshold subtractors.
    reg recovery_hold;
    wire hysteresis_active=MOTION_LEAN_CONTROL && motion_active_i && recovery_hold;
    wire [13:0] recovery_angle_gate=hysteresis_active ? (alpha_gate_i>>1):alpha_gate_i;
    wire [17:0] recovery_speed_gate=hysteresis_active ? (alpha_dot_gate_i>>1):alpha_dot_gate_i;
    wire recovery=(worsening || hysteresis_active) &&
        (recovery_abs>={19'd0,recovery_angle_gate} || ad>={1'b0,recovery_speed_gate});
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)recovery_hold<=0;
        else if(!known || !MOTION_LEAN_CONTROL || !motion_active_i)recovery_hold<=0;
        else if(update_i)recovery_hold<=recovery;
    end
    assign recovery_o=known && recovery;
    wire point_toward,point_overspeed;
    wire point_enabled=POINT_CONTROL && point_config_i[0] && known;
    generate if(POINT_CONTROL)begin: point_approach
        arm_point_approach #(.TARGET_BITS(TARGET_BITS)) approach_i(.clk(clk),.rst_n(rst_n),.ready_i(point_enabled),
            .speed_known_i(speed_known_i),.error_i(point_error),.speed_i(speed_i),
            .target_i(point_target_i),.range_i(point_config_i[17:1]),.speed_limit_i(point_config_i[34:18]),
            .near_o(point_near_o),.toward_o(point_toward),.overspeed_o(point_overspeed));
    end else begin: point_disabled
        assign point_near_o=0;assign point_toward=0;assign point_overspeed=0;
    end endgenerate
    assign point_braking_o=point_overspeed && !recovery && !(MOTION_CONTROL && motion_active_i);
    wire point_arrived=point_enabled && !(MOTION_CONTROL && motion_active_i) && speed_known_i && xp<=1144 && av<=6554 && !recovery;
    wire [2:0] point_state=!point_enabled ? 3'd0:!speed_known_i ? 3'd6:
        recovery ? 3'd7:point_arrived ? 3'd5:point_braking_o ? 3'd3:
        point_near_o && !point_toward && av>6554 ? 3'd4:point_near_o ? 3'd2:3'd1;
    assign point_status_o={point_arrived,point_braking_o,point_toward,point_near_o,point_enabled,point_state};
    reg hold;
    reg [2:0] position_weight;
    // 50% entry / 100% exit hysteresis; enter only with low measured speed.
    wire [14:0] hold_enter=hold_band_i-(hold_band_i>>2);
    wire quiet=speed_known_i && av<=6554; // unknown speed cannot qualify as stopped
    wire signed [18:0] x=position_error_i>205887 ? 19'sd205887:
        position_error_i< -205887 ? -19'sd205887:position_error_i[18:0];
    // One position product in 15 clocks (<0.56us at 27MHz). No velocity
    // product or velocity reference is present in this module.
    wire [18:0] xmag=x[18] ? -x:x;
    wire [14:0] pgmag=position_gain_i[14] ? -position_gain_i:position_gain_i;
    reg calc_state;
    reg [3:0] remaining;
    // Right-shift serial multiply: one 20-bit adder, a held 19-bit
    // multiplicand and a combined 33-bit product/multiplier register.
    // Retain all 14 original calculation clocks and exact sign/truncation.
    reg [32:0] product;
    reg [18:0] multiplicand;
    reg negative,products_valid;
    reg signed [15:0] position_q16;
    wire [19:0] product_sum={1'b0,product[32:14]}+
        (product[0] ? {1'b0,multiplicand}:20'd0);
    wire [32:0] product_next={product_sum,product[13:1]};
    wire signed [15:0] product_angle=negative ? -$signed({1'b0,product_next[30:16]}):
        $signed({1'b0,product_next[30:16]});
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            calc_state<=0;remaining<=0;product<=0;multiplicand<=0;
            negative<=0;products_valid<=0;position_q16<=0;
        end else if(!known)begin calc_state<=0;products_valid<=0;end
        else if(!calc_state)begin
            product<={19'd0,pgmag[13:0]};multiplicand<=xmag;
            negative<=x[18]^position_gain_i[14];remaining<=14;calc_state<=1;
        end else begin
            product<=product_next;
            remaining<=remaining-1'b1;
            if(remaining==1)begin position_q16<=product_angle;products_valid<=1;calc_state<=0;end
        end
    end
    // Angle-domain holding integral; 5ms * 2^shift is its P-relative time constant.
    // The existing serial P product is reused; no integral multiplier is added.
    wire integral_enabled=MOTION_CONTROL && integral_config_i[0] &&
        integral_config_i[4:1]>=6 && integral_config_i[4:1]<=12 && integral_config_i[15:5]<=1144;
    reg signed [21:0] integral_accumulator;
    wire signed [21:0] integral_limit=$signed({3'd0,integral_config_i[15:5],8'd0});
    wire signed [23:0] integral_delta=($signed({{8{position_q16[15]}},position_q16})<<<8)>>>integral_config_i[4:1];
    wire signed [24:0] integral_proposed=$signed({{3{integral_accumulator[21]}},integral_accumulator})+
        $signed({integral_delta[23],integral_delta});
    wire signed [17:0] pi_unclipped=$signed({{2{position_q16[15]}},position_q16})+
        $signed({{4{integral_angle_o[13]}},integral_angle_o})+
        (MOTION_LEAN_CONTROL && motion_active_i ? $signed({{3{motion_lean_i[14]}},motion_lean_i}):18'sd0);
    wire same_integral_direction=(integral_delta>0 && pi_unclipped>0)||(integral_delta<0 && pi_unclipped<0);
    wire integral_held=motion_active_i || !speed_known_i || av>6554 || aa>={1'b0,alpha_gate_i} ||
        ad>={1'b0,alpha_dot_gate_i} || xp>{1'b0,hold_band_i} || recovery || point_braking_o;
    wire integral_saturated=(pi_unclipped>$signed({5'd0,angle_limit_i}) ||
        pi_unclipped < -$signed({5'd0,angle_limit_i}) || output_saturated_i || limited_o) && same_integral_direction;
    wire integral_updates=integral_enabled && known && products_valid && !integral_held && !integral_saturated;
    wire integral_limited=integral_proposed>$signed(integral_limit) || integral_proposed < -$signed(integral_limit);
    assign integral_angle_o=integral_enabled ? integral_accumulator[21:8]:14'sd0;
    assign integral_status_o={integral_enabled,integral_updates,
        integral_enabled && integral_limited,integral_enabled && (!integral_updates)};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)integral_accumulator<=0;
        else if(!known || !integral_enabled)integral_accumulator<=0;
        else if(update_i && integral_updates)begin
            integral_accumulator<=integral_proposed>$signed(integral_limit) ? integral_limit:
                integral_proposed < -$signed(integral_limit) ? -integral_limit:integral_proposed[21:0];
        end
    end
    wire signed [15:0] pos_cap=$signed({3'd0,angle_limit_i});
    // Configured cap <=5719 fits signed14; no angle/control limit changes.
    wire signed [13:0] position_clipped=pi_unclipped>pos_cap ? $signed({1'b0,angle_limit_i}):
        pi_unclipped< -pos_cap ? -$signed({1'b0,angle_limit_i}):pi_unclipped[13:0];
    // During recovery alpha and alpha_dot have the same sign. With Ka>0,
    // Kd>=0 and no balance integrator, the pure catch PD has the opposite
    // sign to alpha. Retain position ONLY when its unweighted, mapped
    // contribution has that same catch sign. Use the unweighted value so
    // a previous zero weight cannot prevent recovery of a helping request.
    // Other gain/integrator configurations retain the original withdrawal.
    // No new multiplier, motor request, gain, timer or reference cap.
    wire position_helps_catch=catch_direction_known_i && products_valid &&
        position_clipped!=0 && ((position_clipped[13]^reverse_i)!=recovery_error[32]);
    wire position_withdraw=recovery && !position_helps_catch;
    reg signed [13:0] position_weighted;
    always @* begin
        case(position_weight)
            0:position_weighted=0;
            1:position_weighted=position_clipped>>>2;
            2:position_weighted=position_clipped>>>1;
            3:position_weighted=position_clipped-(position_clipped>>>2);
            default:position_weighted=position_clipped;
        endcase
        // HOLD is a near-target state only. Position P continues correcting
        // the original fixed target, including while inside the hold band.
        if(POINT_CONTROL && point_braking_o)position_weighted=0;
    end
    // The first position clip already bounds magnitude by angle_limit_i.
    // All weights reduce magnitude and reverse only changes sign; repeating
    // the same cap after weighting is redundant. Values fit signed14.
    wire signed [13:0] target=reverse_i ? -$signed(position_weighted[13:0]):
        $signed(position_weighted[13:0]);
    reg signed [13:0] previous;
    wire signed [14:0] target_w={target[13],target};
    wire signed [14:0] previous_w={previous[13],previous};
    wire signed [14:0] step=$signed({5'd0,angle_step_i});
    wire signed [14:0] delta=target_w-previous_w;
    // R15G_TRACE_BEGIN
    // Reason priority describes the live reference, not a mechanical diagnosis.
    // In R15H, reasons 3/5 are observations; neither withdraws position P.
    // 0 tracking,1 not ready,2 config invalid,3 speed unknown,4 product pending,
    // 5 hold,6 recovery zero,7 direction reset,8 zero gain,9 zero target,
    // 10 recovery reduced,11 limit/step. Controller gates use 12..14.
    assign trace_reference_reason_o=!ready_i ? 4'd1:!config_ok ? 4'd2:
        !products_valid ? 4'd4:
        (position_withdraw && position_weight==0) ? 4'd6:
        ((target_w>0 && previous_w<0)||(target_w<0 && previous_w>0)) ? 4'd7:
        position_gain_i==0 ? 4'd8:point_braking_o ? 4'd15:next_reference==0 ? 4'd9:
        position_withdraw ? 4'd10:limited_o ? 4'd11:
        !speed_known_i ? 4'd3:hold ? 4'd5:4'd0;
    // R15G_TRACE_END
    reg signed [14:0] next_reference;
    always @* begin
        next_reference=0;
        if(known && products_valid)begin
            // Drop obsolete direction at this update, then build the new direction.
            if((target_w>0 && previous_w<0)||(target_w<0 && previous_w>0))next_reference=0;
            else if((previous_w>0 && target_w<=previous_w)||
                    (previous_w<0 && target_w>=previous_w))next_reference=target_w;
            else if(delta>step)next_reference=previous_w+step;
            else if(delta< -step)next_reference=previous_w-step;
            else next_reference=target_w;
        end
    end
    assign target_angle_o=known && products_valid ? {{18{target[13]}},target}:32'sd0;
    assign reference_angle_o=known ? {{17{next_reference[14]}},next_reference}:32'sd0;
    assign hold_o=known && hold && !(MOTION_CONTROL && motion_active_i);
    assign limited_o=known && (pi_unclipped!=position_clipped||next_reference!=target_w);
    // State2 describes a near-target moving arm, not successful braking.
    assign state_o=!ready_i ? 3'd0:!known ? 3'd5:recovery ? 3'd4:
        !speed_known_i ? 3'd5:
        hold ? 3'd3:(av>3277 && xp<={1'b0,hold_band_i}) ? 3'd2:3'd1;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin previous<=0;hold<=0;position_weight<=4;end
        else if(!known)begin previous<=0;hold<=0;position_weight<=4;end
        else if(update_i)begin
            previous<=next_reference[13:0];
            if(!speed_known_i || xp>={1'b0,hold_band_i})hold<=0;
            else if(xp<={1'b0,hold_enter} && quiet && !recovery)hold<=1;
            if(position_withdraw)begin if(position_weight!=0)position_weight<=position_weight-1'b1;end
            else if(position_weight<4)position_weight<=position_weight+1'b1;
        end
    end
    // synthesis translate_off
    always @(posedge clk)if(rst_n && ready_i && update_i && known)begin
        if(reference_angle_o>$signed({19'd0,angle_limit_i}) ||
           reference_angle_o< -$signed({19'd0,angle_limit_i}))$fatal(1,"R15 angle reference out of bounds");
    end
    // synthesis translate_on
endmodule

// R15I: |speed|*range > near_speed*distance identifies overspeed while
// approaching. Two shared serial 17x17 products; no square root/divider.
// This is a requested speed envelope, not measured actuator braking ability.
module arm_point_approach #(parameter integer TARGET_BITS=19)(
 input wire clk,rst_n,ready_i,speed_known_i,
 input wire signed [31:0] error_i,speed_i,
 input wire signed [TARGET_BITS-1:0] target_i,
 input wire [16:0] range_i,speed_limit_i,
 output wire near_o,toward_o,overspeed_o
);
 // Both thresholds are unsigned17. Saturation above 131071 preserves
 // every near/speed decision while removing two 32-bit negation chains.
 wire [17:0] distance=error_i[31:17]!={15{error_i[31]}} ? 18'h3ffff:
     (error_i[31] ? -error_i[17:0]:error_i[17:0]);
 wire [17:0] velocity=speed_i[31:17]!={15{speed_i[31]}} ? 18'h3ffff:
     (speed_i[31] ? -speed_i[17:0]:speed_i[17:0]);
 assign near_o=ready_i && distance<={1'b0,range_i};
 assign toward_o=ready_i && speed_known_i &&
     ((error_i>0 && speed_i<0)||(error_i<0 && speed_i>0));
 reg [1:0] state;
 reg [4:0] remaining;
 reg [33:0] product,velocity_product;
 reg [16:0] multiplicand,held_distance,held_limit;
 reg signed [TARGET_BITS-1:0] held_target,result_target;
 reg braking,valid;
 // Same 17 cycles per product, with an 18-bit rather than 34-bit adder.
 wire [17:0] product_sum={1'b0,product[33:17]}+
     (product[0] ? {1'b0,multiplicand}:18'd0);
 wire [33:0] next_product={product_sum,product[16:1]};
 // Only speeds <= configured limit require products; larger speeds are
 // unconditionally over the tapered envelope. No signed extreme can wrap.
 assign overspeed_o=near_o && toward_o && speed_known_i &&
     (velocity>{1'b0,speed_limit_i} || (valid && result_target==target_i && braking));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=0;remaining<=0;product<=0;velocity_product<=0;
   multiplicand<=0;held_distance<=0;held_limit<=0;held_target<=0;result_target<=0;braking<=0;valid<=0;
  end else if(!near_o || !toward_o || !speed_known_i || range_i==0)begin
   state<=0;valid<=0;braking<=0;
  end else if(valid && result_target!=target_i)begin
   state<=0;valid<=0;braking<=0;
  end else case(state)
   0:begin
    product<={17'd0,(velocity>65536 ? 17'd65536:velocity[16:0])};
    multiplicand<=range_i;
    held_distance<=distance[16:0];held_limit<=speed_limit_i;held_target<=target_i;
    remaining<=17;state<=1;
   end
   1:begin
    product<=next_product;remaining<=remaining-1'b1;
    if(remaining==1)begin
     velocity_product<=next_product;product<={17'd0,held_limit};
     multiplicand<=held_distance;remaining<=17;state<=2;
    end
   end
   2:begin
    product<=next_product;remaining<=remaining-1'b1;
    if(remaining==1)begin
     result_target<=held_target;valid<=held_target==target_i;
     if(velocity_product>next_product)braking<=1;
     else if(velocity_product<=next_product-(next_product>>3))braking<=0;
     state<=0;
    end
   end
   default:begin state<=0;valid<=0;braking<=0;end
  endcase
 end
endmodule
`default_nettype wire
