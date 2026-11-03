// R9: optional alternate balance speed; swing/capture still use raw theta_dot.
// Angles and speeds are signed Q16 radians / radians per second; U is 0.1% duty.
// Position reference is relative to the measured arm position on balance capture.
`timescale 1ns/1ps
`default_nettype none
module controller #(
    parameter integer DUAL_MODE=0,
    parameter integer SPEED_HANDOFF_CONTROL = 0,
    parameter integer MOTION_LEAN_CONTROL=0,
    parameter integer MOTION_CONTROL=0, MULTITURN_CONTROL=0, TARGET_BITS=(MULTITURN_CONTROL ? 32:19),
    parameter integer POINT_CONTROL=0,
    parameter integer COORDINATED_CONTROL=0, // R15: compile-time replacement of R14M planner
    parameter integer VELOCITY_SLEW_BYPASS=0, // R15D: mode1 steady-balance A/B only
    parameter integer LIVE_VELOCITY_BYPASS=0, // R15E: select STOP-applied serial ID90
    parameter integer R14M_ENABLE=0,
    parameter integer LIVE_BRAKE_CONFIG=0,
    parameter integer BRAKE_DISTANCE_GAIN_Q8=384,
    parameter integer BRAKE_STEP_U=20,
    parameter integer SPEED_LOOP_CONTROL = 0,
    parameter integer ARM_SLEW_U_PER_UPDATE = 0, // 0 bypass; top selects 10U per 5ms
    parameter integer K_THETA_TENTHS = 0,
    parameter integer START_MODE_CONTROL = 0,
    parameter integer INDEPENDENT_RETURN_CAP = 0,
    parameter integer NEAR_CONTROL = 0,
    parameter integer SPEED_FILTER_CONTROL = 0,
    parameter integer LIVE_CONFIG = 0,
    parameter integer POSITION_CONTROL = 0,
    parameter integer FIXED_POSITION = 0,
    parameter integer BALANCE_SPEED_INPUT = 0,
    parameter integer SETTLE_CONTROL = 0,
    parameter integer PID_CONTROL = 0,
    parameter integer HOLD_CONTROL = 0,
    parameter integer POSITION_I_CONTROL = 0,
    parameter integer RETURN_CONTROL = 0,
    parameter integer RETURN_TRIM_ENABLE = 1, // legacy compatibility; R8 top selects 0
    parameter integer       UPDATE_DIV  = 5,
    parameter integer       AUTO_SWINGUP = 0,
    parameter signed [31:0]  SWING_SETPOINT_Q16 = 32'sd2000,
    parameter integer       SWING_U = 250,
    parameter integer       SWING_KICK_UPDATES = 40,
    parameter integer       SWING_START_ONLY = 0,
    parameter integer       SWING_ARM_SPEED_GAIN = 80,
    parameter signed [31:0]  SWING_KICK_END_SPEED_Q16 = 32'sd262144,

    parameter signed [15:0] K_ALPHA     = 16'sd900,
    parameter signed [15:0] K_ALPHA_DOT = 16'sd50,
    parameter signed [15:0] K_THETA     = 16'sd5,
    parameter signed [15:0] K_THETA_DOT = 16'sd0,

    parameter integer       U_LIMIT     = 1000,
    parameter integer       POSITION_U_LIMIT = 100,

    parameter signed [31:0] ALPHA_ARM   = 32'sd19661,   // 0.30 rad ≈ 17°
    parameter signed [31:0] THETA_LIM   = 32'sd205887,  // 3.1416 rad = 180°

    parameter integer       SIGN_MOTOR  = 1,            // +1 / -1
    parameter integer       SIGN_ALPHA  = 1,            // 电位器装反时置 -1
    parameter integer       TEST_MODE   = 0,            // 1 = 常量占空比方向测试
    parameter signed [15:0] TEST_U      = 16'sd80
)(
    // R15G_TRACE_BEGIN: diagnostics only, sampled with the original control sum.
    output wire signed [31:0] trace_position_o,trace_alpha_o,trace_alpha_dot_o,
    output wire signed [13:0] trace_reference_o,trace_target_o,
    output wire [3:0] trace_reference_reason_o,
    output wire [12:0] trace_coordinate_flags_o,
    output reg signed [31:0] trace_sampled_position_o,
    output reg [3:0] trace_sampled_reason_o,
    output reg [12:0] trace_sampled_flags_o,
    // R15G_TRACE_END
    input wire signed [10:0] k_theta_x10_i,
    input wire manual_start_i,
    input wire [223:0] cruise_config_i,
    input wire [63:0] cruise_gains_i,
    input wire [127:0] arm_loop_config_i,
    output reg signed [31:0] dbg_loop_reference,
    input wire [95:0] coordinate_config_i,
    input wire [63:0] point_config_i,
    input wire [63:0] motion_pi_config_i,
    output reg signed [31:0] dbg_motion_position,dbg_motion_velocity,dbg_outer_i_angle,
    input wire velocity_slew_bypass_i,
    input wire [63:0] brake_config_i,
    output reg [7:0] dbg_brake_status,
    output reg [31:0] dbg_brake_distance,
    output reg signed [32:0] dbg_velocity_error,
    output reg [3:0] dbg_brake_flags,
    output reg [3:0] dbg_loop_state,
    input wire [6:0] return_position_cap_i,
    input wire [95:0] near_config_i,
    input wire [1:0] speed_filter_mode_i,
    input wire signed [31:0] balance_speed_i,
    input wire balance_speed_valid_i, balance_speed_select_i,
    input wire [36:0] speed_status_i,
    output reg [63:0] dbg_speed_status,
    output reg signed [31:0] dbg_original_arm_q16,dbg_arm_request_q16,
    output reg signed [31:0] dbg_raw_speed, dbg_est_speed,
    output reg dbg_speed_source, dbg_est_valid,
    input wire [255:0] settle_config_i,
    input wire [223:0] motion_config_i,
    input wire [191:0] position_i_config_i,
    input wire [95:0] return_config_i,
    output reg signed [31:0] dbg_vref,dbg_aref,
    output reg [31:0] dbg_hold_control,
    input wire [95:0] pid_config_i,
    input wire pid_permit_i,
    output reg signed [31:0] dbg_pi,
    output reg [31:0] dbg_settle_status,
    output reg signed [31:0] dbg_capture_travel, dbg_lock_origin, dbg_hold_position,
    input wire [1279:0] config_i,
    input wire run_i, rearm_i,
    input wire position_set_i,
    input wire signed [TARGET_BITS-1:0] position_target_i,
    output wire position_ready_o,
    output wire signed [31:0] position_actual_o,
    output wire signed [TARGET_BITS-1:0] position_target_o,
    input  wire               clk,
    input  wire               rst_n,

    input  wire               control_tick,
    input  wire [2:0]         system_state,
    input  wire               sensor_valid,

    input  wire signed [31:0] theta,
    input  wire signed [31:0] theta_dot,
    input  wire signed [31:0] alpha,
    input  wire signed [31:0] alpha_dot,

    output reg signed [15:0]  control_u,
    output reg                control_valid,
    output reg                motor_enable,
    output reg signed [31:0] dbg_theta, dbg_theta_dot, dbg_alpha, dbg_alpha_dot,
    output reg signed [31:0] dbg_pa, dbg_pd, dbg_pt, dbg_pv,
    output reg dbg_arm_ok, dbg_ready,
    output reg [2:0] dbg_phase,
    output reg [3:0] dbg_reason
);
    wire signed [31:0] cfg_K_ALPHA = DUAL_MODE && !loop_mode ? $signed(cruise_gains_i[31:0]):LIVE_CONFIG ? $signed(config_i[0 +: 32]) : K_ALPHA;
    wire signed [31:0] cfg_K_ALPHA_DOT = DUAL_MODE && !loop_mode ? $signed(cruise_gains_i[63:32]):LIVE_CONFIG ? $signed(config_i[32 +: 32]) : K_ALPHA_DOT;
    wire signed [31:0] cfg_K_THETA = LIVE_CONFIG ? $signed(config_i[64 +: 32]) : K_THETA;
    wire signed [31:0] cfg_K_THETA_DOT = LIVE_CONFIG ? $signed(config_i[96 +: 32]) : K_THETA_DOT;
    wire signed [31:0] cfg_SWING_U = LIVE_CONFIG ? $signed(config_i[160 +: 32]) : SWING_U;
    wire signed [31:0] cfg_SWING_KICK_UPDATES = LIVE_CONFIG ? $signed(config_i[192 +: 32]) : SWING_KICK_UPDATES;
    wire signed [31:0] cfg_SWING_KICK_END_SPEED_Q16 = LIVE_CONFIG ? $signed(config_i[224 +: 32]) : SWING_KICK_END_SPEED_Q16;
    wire signed [31:0] cfg_SWING_ARM_SPEED_GAIN = LIVE_CONFIG ? $signed(config_i[256 +: 32]) : SWING_ARM_SPEED_GAIN;
    wire signed [31:0] cfg_POSITION_U_LIMIT = LIVE_CONFIG ? $signed(config_i[1024 +: 32]) : POSITION_U_LIMIT;
    wire signed [31:0] cfg_U_LIMIT = LIVE_CONFIG ? $signed(config_i[1056 +: 32]) : U_LIMIT;
    wire signed [31:0] cfg_ALPHA_ARM = LIVE_CONFIG ? $signed(config_i[1088 +: 32]) : ALPHA_ARM;


    localparam [2:0] RUN = 3'd3;

    localparam integer DIV_W = (UPDATE_DIV <= 2) ? 1 : $clog2(UPDATE_DIV);

    wire signed [31:0] KA  = cfg_K_ALPHA     * 65536;
    wire signed [31:0] KAD = cfg_K_ALPHA_DOT * 65536;
    wire signed [31:0] KT  = cfg_K_THETA     * 65536;
    wire signed [31:0] KTD = cfg_K_THETA_DOT * 65536;

    reg [DIV_W-1:0] div_cnt;

    wire signed [31:0] alpha_s     = (SIGN_ALPHA < 0) ? -alpha     : alpha;
    wire signed [31:0] alpha_dot_s = (SIGN_ALPHA < 0) ? -alpha_dot : alpha_dot;

    localparam signed [31:0] THETA_PI_C     = 32'sd205887;
    localparam signed [31:0] THETA_TWO_PI_C = 32'sd411775;

    // Diagnostic-only conversion. Preserve fractions; saturate instead of wrapping.
    function signed [31:0] arm_debug_q16;
        input signed [63:0] q32;
        reg signed [63:0] q16;
        begin q16=q32 >>> 16;
            arm_debug_q16=q16>64'sd2147483647 ? 32'sh7fffffff :
                q16< -64'sd2147483648 ? 32'sh80000000 : q16[31:0];
        end
    endfunction
    function signed [31:0] wrap_theta_q16;
        input signed [31:0] value;
        reg [31:0] remainder_mag;
        reg signed [31:0] signed_remainder;
        integer bit_index;
        begin
            remainder_mag = value[31] ? (~value + 32'd1) : value;
            for (bit_index = 13; bit_index >= 0; bit_index = bit_index - 1) begin
                if (remainder_mag >= ($unsigned(THETA_TWO_PI_C) << bit_index))
                    remainder_mag = remainder_mag - ($unsigned(THETA_TWO_PI_C) << bit_index);
            end
            signed_remainder = value[31] ? -$signed(remainder_mag) : $signed(remainder_mag);
            if (signed_remainder > THETA_PI_C)
                wrap_theta_q16 = signed_remainder - THETA_TWO_PI_C;
            else if (signed_remainder < -THETA_PI_C)
                wrap_theta_q16 = signed_remainder + THETA_TWO_PI_C;
            else
                wrap_theta_q16 = signed_remainder;
        end
    endfunction

    wire signed [31:0] theta_reduced = wrap_theta_q16(theta);
    wire signed [18:0] theta_small = theta_reduced[18:0];
    wire signed [31:0] theta_w = {{13{theta_small[18]}},theta_small};

    reg position_active;
    reg signed [31:0] theta_previous;
    reg signed [31:0] position_error;
    wire signed [19:0] position_delta_raw =
        $signed({theta_w[18],theta_w[18:0]}) -
        $signed({theta_previous[18],theta_previous[18:0]});
    wire signed [19:0] position_delta_small =
        (position_delta_raw > 20'sd205887) ? position_delta_raw-20'sd411775 :
        (position_delta_raw < -20'sd205887) ? position_delta_raw+20'sd411775 : position_delta_raw;
    wire signed [31:0] position_delta = {{12{position_delta_small[19]}},position_delta_small};
    wire signed [32:0] position_sum =
        $signed({position_error[31],position_error}) +
        $signed({position_delta[31],position_delta});
    wire signed [31:0] position_next =
        (position_sum > 33'sd2147483647) ? 32'sh7fffffff :
        (position_sum < -33'sd2147483648) ? 32'sh80000000 : position_sum[31:0];
    // R5: reference is relative to the arm position captured on entering balance.
    // Existing position_error remains the measured, unwrapped displacement.
    // Only the reference changes; the same limited position term drives tracking.
    reg signed [TARGET_BITS-1:0] position_target, position_reference;
    reg [5:0] position_quiet;
    wire position_stable = ($signed(alpha_s) < 32'sd5719) &&
        ($signed(alpha_s) > -32'sd5719) &&
        ($signed(alpha_dot_s) < 32'sd65536) &&
        ($signed(alpha_dot_s) > -32'sd65536);
    assign position_ready_o = POSITION_CONTROL && position_active &&
        system_ready && balance_selected && arm_ok &&
        (POINT_CONTROL ? (loop_mode && position_reference_enabled && pid_permit_i):
            (position_stable && position_quiet==40)) &&
        (!settle_mode || settle_stage==3);
    wire settle_mode = SETTLE_CONTROL && settle_config_i[0];
    wire [1:0] settle_stage;
    wire [8:0] settle_weight;
    wire signed [15:0] effective_kv,base_effective_kv;
    wire [7:0] point_status;
    wire point_near,point_braking;
    assign effective_kv=POINT_CONTROL && point_near && balance_velocity_known &&
        !speed_provisional && !coordinate_recovery && point_config_i[42:35]>base_effective_kv ?
        $signed({8'd0,point_config_i[42:35]}):base_effective_kv;
    wire signed [31:0] lock_origin;
    wire settle_locked = settle_mode && settle_stage>=2;
    function signed [31:0] subtract_saturated;
        input signed [31:0] a,b;
        reg signed [32:0] diff;
        begin
            diff=$signed({a[31],a})-$signed({b[31],b});
            subtract_saturated=diff>33'sd2147483647 ? 32'sh7fffffff :
                diff < -33'sd2147483648 ? 32'sh80000000 : diff[31:0];
        end
    endfunction
    assign position_actual_o = !position_active ? 32'sd0 :
        settle_locked ? subtract_saturated(position_error,lock_origin) : position_error;
    assign position_target_o = POSITION_CONTROL ? position_target : 19'sd0;
    wire signed [TARGET_BITS-1:0] reference_step = !position_ready_o ? 19'sd0 :
        (position_target-position_reference > 19'sd32) ? 19'sd32 :
        (position_target-position_reference < -19'sd32) ? -19'sd32 :
        position_target-position_reference;
    wire signed [TARGET_BITS-1:0] reference_next = position_reference + reference_step;
    wire motion_enabled=MOTION_CONTROL && (!DUAL_MODE || loop_mode) && motion_pi_config_i[0];
    wire signed [31:0] motion_position,motion_velocity;
    wire signed [11:0] motion_acceleration;
    wire signed [14:0] motion_lean=MOTION_LEAN_CONTROL && motion_enabled && motion_active ?
        -($signed({{3{motion_acceleration[11]}},motion_acceleration}) <<< 2):15'sd0;
    wire signed [32:0] catch_error=inner_angle_error;
    wire [32:0] catch_abs=catch_error[32] ? -catch_error:catch_error;
    reg motion_rebase_pending;
    reg [4:0] motion_start_quiet;
    wire motion_start_safe=balance_velocity_confirmed && balance_speed_i<=32'sd6554 && balance_speed_i>=-32'sd6554 &&
        catch_abs<{19'd0,coordinate_config_i[51:38]} &&
        alpha_dot_s<=32'sd32768 && alpha_dot_s>=-32'sd32768;
    wire motion_rebase=MOTION_LEAN_CONTROL && motion_rebase_pending && motion_start_quiet>=20 &&
        motion_velocity==0 && motion_acceleration==0;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin motion_rebase_pending<=0;motion_start_quiet<=0;end
        else if(!MOTION_LEAN_CONTROL || !loop_ready || !position_active || !motion_enabled)begin
            motion_rebase_pending<=0;motion_start_quiet<=0;
        end else begin
            if(update_now)begin
                if(!motion_start_safe)motion_start_quiet<=0;
                else if(motion_start_quiet<20)motion_start_quiet<=motion_start_quiet+1'b1;
            end
            if(position_set_i && position_ready_o)
                motion_rebase_pending<=motion_velocity==0 && motion_acceleration==0;
            else if(motion_rebase || motion_velocity!=0 || motion_acceleration!=0)
                motion_rebase_pending<=0;
        end
    end
    wire motion_active,motion_paused,motion_calculating;
    wire [3:0] outer_i_status;wire signed [13:0] outer_i_angle;
    wire signed [31:0] goal_error=subtract_saturated(position_next,
        {{(32-TARGET_BITS){position_target[TARGET_BITS-1]}},position_target});
    generate if(MOTION_CONTROL)begin: planned_motion
        arm_motion_reference_r15l reference_i(.clk(clk),.rst_n(rst_n),.update_i(update_now),
            .ready_i(loop_ready && position_active && position_reference_enabled),.enable_i(motion_enabled),
            .pause_i(MOTION_LEAN_CONTROL ?
                (((catch_error>0 && alpha_dot_s>0)||(catch_error<0 && alpha_dot_s<0)) &&
                 catch_abs>({19'd0,coordinate_config_i[51:38]}<<1)):
                (pendulum_worsening && alpha_abs32>({18'd0,coordinate_config_i[51:38]}<<1))),
            .rebase_i(motion_rebase),.acceleration_o(motion_acceleration),
            .actual_i(position_next),.goal_i({{(32-TARGET_BITS){position_target[TARGET_BITS-1]}},position_target}),
            .max_velocity_i(motion_pi_config_i[18:1]),.acceleration_i(motion_pi_config_i[29:19]),
            .following_band_i(motion_pi_config_i[61:46]),.position_o(motion_position),.velocity_o(motion_velocity),
            .active_o(motion_active),.paused_o(motion_paused),.calculating_o(motion_calculating));
    end else begin
        assign motion_position={{(32-TARGET_BITS){position_target[TARGET_BITS-1]}},position_target};
        assign motion_acceleration=0;assign motion_velocity=0;assign motion_active=0;assign motion_paused=0;assign motion_calculating=0;
    end endgenerate
    wire signed [31:0] effective_position_reference=POINT_CONTROL && loop_mode ?
        (motion_enabled ? motion_position:$signed(position_target)):$signed(position_reference);
    wire signed [32:0] tracking_error =
        $signed({position_next[31],position_next}) -
        $signed({effective_position_reference[31],effective_position_reference});
    wire signed [31:0] tracking_limited =
        tracking_error > 33'sd2147483647 ? 32'sh7fffffff :
        tracking_error < -33'sd2147483648 ? 32'sh80000000 : tracking_error[31:0];
    wire signed [31:0] legacy_position_feedback = !position_active ? 32'sd0 :
        settle_locked ? subtract_saturated(position_next,lock_origin) :
        POSITION_CONTROL ? tracking_limited : position_next;

    wire cruise_selected,cruise_correction,cruise_recovering;
    wire signed [31:0] cruise_vref,cruise_error,cruise_position;
    wire [2:0] cruise_state;
    wire cruise_ready=system_ready && balance_selected && arm_ok && position_active && pid_permit_i && !rearm_i && TEST_MODE==0;
    wire cruise_speed_known=!balance_speed_select_i || speed_confirmed;
    generate if(DUAL_MODE)begin: dual_cruise
      cruise_recovery supervisor(.clk(clk),.rst_n(rst_n),.update_i(update_now),
        .ready_i(cruise_ready),.speed_known_i(cruise_speed_known),.config_i(cruise_config_i),
        .position_i(position_next),.alpha_i(alpha_s),.alpha_dot_i(alpha_dot_s),
        .selected_o(cruise_selected),.correction_o(cruise_correction),.recovering_o(cruise_recovering),
        .velocity_ref_o(cruise_vref),.position_error_o(cruise_error),.position_ref_o(cruise_position),.state_o(cruise_state));
    end else begin
      assign cruise_selected=0;assign cruise_correction=0;assign cruise_recovering=0;
      assign cruise_vref=0;assign cruise_error=0;assign cruise_position=0;assign cruise_state=0;
    end endgenerate
    wire signed [31:0] position_feedback=cruise_selected ? cruise_error:legacy_position_feedback;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position_target<=0; position_reference<=0; position_quiet<=0;
        end else if (!POSITION_CONTROL || !system_ready || !balance_selected ||
                     !arm_ok || !position_active || rearm_i) begin
            position_target<=0; position_reference<=0; position_quiet<=0;
        end else begin
            if (FIXED_POSITION) begin position_target<=0; position_reference<=0; end
            else if (position_set_i && position_ready_o &&
                ((MULTITURN_CONTROL && $signed(position_target_i)!=32'sh80000000) ||
                (!MULTITURN_CONTROL && $signed(position_target_i)<=19'sd102944 && $signed(position_target_i)>=-19'sd102944)))
                position_target<=position_target_i;
            if (update_now) begin
                if (!position_stable) position_quiet<=0;
                else if (position_quiet<40) position_quiet<=position_quiet+1'b1;
                if (!FIXED_POSITION) position_reference<=reference_next;
            end
        end
    end

    wire signed [63:0] POSITION_LIMIT_Q32 =
        cfg_POSITION_U_LIMIT * 64'sd4294967296;

    localparam signed [63:0] SPEED_START_Q16 = 64'sd393216; // 6 rad/s
    localparam signed [63:0] SPEED_TRIM_LIMIT_Q32 = 64'sd343597383680; // 80 U
    wire [31:0] speed_abs32 = theta_dot[31] ? (~theta_dot+32'd1) : theta_dot;
    wire [31:0] alpha_abs32 = alpha_s[31] ? (~alpha_s+32'd1) : alpha_s;
    wire signed [63:0] alpha_magnitude = $signed({32'd0,alpha_abs32});
    wire [19:0] trim_excess_q16 = (speed_abs32<=32'd393216) ? 20'd0 :
        (speed_abs32>=32'd917504) ? 20'd524288 : speed_abs32-32'd393216;
    wire [22:0] trim_ten_q16 = trim_excess_q16 * 4'd10;
    wire [38:0] trim_mag39 = {trim_ten_q16,16'd0};
    wire signed [63:0] trim_magnitude = $signed({25'd0,trim_mag39});
    wire returning = (position_feedback != 0) && (theta_dot != 0) &&
                      (position_feedback[31] != theta_dot[31]);
    wire [12:0] trim_factor13 = (alpha_abs32>32'd6863 && alpha_abs32<32'd13726) ?
        32'd13726-alpha_abs32 : 13'd0;
    wire [51:0] trim_numerator52 = trim_mag39 * trim_factor13;
    wire [51:0] trim_quotient52 = trim_numerator52 / 13'd6863;
    wire [38:0] trim_taper39 = (alpha_abs32<=32'd6863) ? trim_mag39 :
        (alpha_abs32>=32'd13726) ? 39'd0 : trim_quotient52[38:0];
    wire signed [63:0] trim_taper = $signed({25'd0,trim_taper39});
    wire signed [63:0] trim_target = !returning ? 64'sd0 :
        (theta_dot[31] ? trim_taper : -trim_taper);
    reg signed [39:0] speed_trim_small;
    wire signed [63:0] speed_trim_q32 = {{24{speed_trim_small[39]}},speed_trim_small};
    wire trim_allowed = RETURN_TRIM_ENABLE && system_ready && balance_selected && arm_ok && TEST_MODE == 0 && position_active;
    wire signed [39:0] trim_target40 = trim_target[39:0];
    wire signed [40:0] trim_difference41 =
        $signed({trim_target40[39],trim_target40}) -
        $signed({speed_trim_small[39],speed_trim_small});
    wire signed [40:0] trim_smoothed41 =
        $signed({speed_trim_small[39],speed_trim_small}) + trim_difference41 / 41'sd4;
    wire signed [63:0] trim_smoothed = {{23{trim_smoothed41[40]}},trim_smoothed41};
    wire signed [63:0] trim_applied = (trim_allowed && alpha_magnitude < 13726) ?
        trim_smoothed : 64'sd0;

    // R15 factors Ka*alpha_ref - Ka*alpha into ONE angle-error product.
    // Widen before subtraction, so even corrupted signed sensor extremes cannot wrap.
    wire signed [13:0] inner_reference=COORDINATED_CONTROL && loop_mode && position_active ?
        coordinate_reference[13:0]:14'sd0;
    wire signed [32:0] inner_angle_error=$signed({alpha_s[31],alpha_s})-
        $signed({{19{inner_reference[13]}},inner_reference});
    wire signed [48:0] p_alpha_q16=$signed(inner_angle_error)*$signed(cfg_K_ALPHA[15:0]);
    wire signed [63:0] p_alpha = $signed({{15{p_alpha_q16[48]}},p_alpha_q16}) <<< 16;
    wire signed [47:0] p_alpha_dot_q16 = $signed(alpha_dot_s) * $signed(cfg_K_ALPHA_DOT[15:0]);
    wire signed [63:0] p_alpha_dot = $signed({{16{p_alpha_dot_q16[47]}},p_alpha_dot_q16}) <<< 16;
    wire signed [47:0] p_theta_linear_q16 = $signed(position_feedback) * $signed(cfg_K_THETA[15:0]);
    wire signed [47:0] p_theta_raw_q16;
    generate if(K_THETA_TENTHS) begin: decimal_raw_debug
        assign p_theta_raw_q16=p_theta_raw >>> 16;
    end else if(NEAR_CONTROL)begin: near_control_enabled
        near_position_shape shape_i(.enabled_i(near_config_i[0] && return_mode),
            .position_i(position_feedback),.far_gain_i(cfg_K_THETA[15:0]),
            .near_gain_i(near_config_i[70:64]),.range_i(near_config_i[47:32]),
            .linear_i(p_theta_linear_q16),.shaped_o(p_theta_raw_q16));
    end else begin: near_control_disabled
        assign p_theta_raw_q16=p_theta_linear_q16;
    end endgenerate
    wire signed [63:0] p_theta_raw;
    generate if(DUAL_MODE && COORDINATED_CONTROL)begin: compact_parallel_position
        // Dual bank fixes Ktheta=8, near K=4, near radius=11438.
        // cruise_recovery bounds its error to +/-11438 BEFORE this interface.
        // Mode1 consumes coordinate_reference, never this legacy position P.
        cruise_position_feedback compact_i(.error_i(cruise_error[14:0]),
            .near_i(NEAR_CONTROL && near_config_i[0] && return_mode),.request_o(p_theta_raw));
    end else if(K_THETA_TENTHS) begin: decimal_theta_p
        theta_position_tenths p_i(.position_i(position_feedback),
            .gain_x10_i(k_theta_x10_i),
            .near_enabled_i(NEAR_CONTROL && near_config_i[0] && return_mode),
            .near_gain_i(near_config_i[70:64]),.range_i(near_config_i[47:32]),
            .p_q32_o(p_theta_raw));
    end else begin: integer_theta_p
        assign p_theta_raw=$signed({{16{p_theta_raw_q16[47]}},p_theta_raw_q16}) <<< 16;
    end endgenerate
    // R14B: cap the existing position term, keep its near-target gain unchanged.
    // R14F top: cap_U = min(POSITION_U_LIMIT, ID78), hard maximum 100U.
    // Legacy elaborations retain Kv*Vmax only for regression compatibility.
    // The speed term uses the selected Kv ONCE; no extra braking term.
    wire [26:0] return_cap_q16;
    wire return_cap_ready;
    wire return_mode=RETURN_CONTROL && return_config_i[0] && return_cap_ready && balance_selected;
    generate if(DUAL_MODE && RETURN_CONTROL && INDEPENDENT_RETURN_CAP)begin: dual_return_cap
        wire [26:0] legacy_cap;wire legacy_ready;
        return_cap_config cap_i(.clk(clk),.rst_n(rst_n),.config_i(return_config_i),.cap_q16_o(legacy_cap),.ready_o(legacy_ready));
        wire [6:0] safe_cap=return_position_cap_i>100 ? 7'd100:return_position_cap_i;
        assign return_cap_q16=loop_mode ? {4'd0,safe_cap,16'd0}:legacy_cap;
        assign return_cap_ready=loop_mode || legacy_ready;
    end else if(RETURN_CONTROL && INDEPENDENT_RETURN_CAP)begin: independent_return_enabled
        // Q16 conversion by concatenation, no multiplier or rounding.
        wire [6:0] safe_cap=(return_position_cap_i>7'd100)?7'd100:return_position_cap_i;
        assign return_cap_q16={4'd0,safe_cap,16'd0};
        // A zero direct input clamps position P to zero, not legacy 100U.
        assign return_cap_ready=1'b1;
    end else if(RETURN_CONTROL)begin: return_enabled
        return_cap_config cap_i(.clk(clk),.rst_n(rst_n),.config_i(return_config_i),
            .cap_q16_o(return_cap_q16),.ready_o(return_cap_ready));
    end else begin: return_disabled
        assign return_cap_q16=0;assign return_cap_ready=0;
    end endgenerate
    wire signed [63:0] return_cap_q32=$signed({21'd0,return_cap_q16,16'd0});
    wire signed [63:0] selected_position_limit=(return_mode && return_cap_q32<POSITION_LIMIT_Q32)
        ? return_cap_q32 : POSITION_LIMIT_Q32;
    wire signed [63:0] p_theta_limited =
        (p_theta_raw > selected_position_limit) ? selected_position_limit :
        (p_theta_raw < -selected_position_limit) ? -selected_position_limit : p_theta_raw;
    wire signed [63:0] p_theta_weighted = p_theta_limited * $signed({1'b0,settle_weight});
    wire signed [63:0] p_theta = settle_mode ? (p_theta_weighted >>> 8) : p_theta_limited;
    wire alternate_speed = BALANCE_SPEED_INPUT && balance_selected &&
        balance_speed_select_i;
    // R15K: SV=1/reason0 or6 is bounded provisional evidence, not a
    // confirmed period. Permit it only in the coordinated balance mode.
    wire speed_provisional=SPEED_HANDOFF_CONTROL && balance_speed_valid_i &&
        (speed_status_i[19:17]==3'd0 || speed_status_i[19:17]==3'd6);
    wire speed_confirmed=balance_speed_valid_i && !speed_provisional;
    wire speed_usable=speed_confirmed ||
        (speed_provisional && COORDINATED_CONTROL && loop_mode);
    wire signed [31:0] balance_speed = alternate_speed ? (speed_usable ? balance_speed_i : 32'sd0) : theta_dot;
    wire [1:0] speed_filter_mode=SPEED_FILTER_CONTROL ? speed_filter_mode_i : 2'd0;
    wire signed [31:0] feedback_speed;
    generate if(SPEED_FILTER_CONTROL)begin: speed_filter_enabled
        balance_speed_filter filter_i(.clk(clk),.rst_n(rst_n),.update_i(update_now),
            .ready_i(position_i_ready),.mode_i(speed_filter_mode),
            .speed_i(balance_speed),.position_i(position_next),.speed_o(feedback_speed));
    end else begin: speed_filter_disabled
        assign feedback_speed=balance_speed;
    end endgenerate
    wire signed [31:0] selected_velocity=cruise_selected && position_active ?
        (cruise_speed_known ? subtract_saturated(balance_speed,cruise_vref):32'sd0):COORDINATED_CONTROL && loop_mode && position_active ?
        (speed_usable ? (motion_enabled && speed_confirmed ?
            subtract_saturated(balance_speed_i,motion_velocity):balance_speed_i):32'sd0):feedback_speed;
    wire signed [47:0] p_theta_dot_q16 = $signed(selected_velocity) * $signed(effective_kv);
    wire signed [63:0] p_theta_dot = $signed({{16{p_theta_dot_q16[47]}},p_theta_dot_q16}) <<< 16;

    generate if(SETTLE_CONTROL) begin: settle_enabled
        settle_lock_coordinator coordinator(
            .clk(clk),.rst_n(rst_n),.update_i(update_now),
            .enabled_i(settle_mode),
            .ready_i(system_ready && balance_selected && arm_ok && position_active && TEST_MODE==0 && !rearm_i),
            .config_i(settle_config_i),.base_kv_i(cfg_K_THETA_DOT[15:0]),
            .position_i(position_next),.speed_i(balance_speed),.raw_speed_i(theta_dot),
            .alpha_i(alpha_s),.alpha_dot_i(alpha_dot_s),
            .stage_o(settle_stage),.position_weight_o(settle_weight),
            .effective_kv_o(base_effective_kv),.lock_origin_o(lock_origin));
    end else begin: settle_disabled
        assign settle_stage=0;assign settle_weight=256;
        assign base_effective_kv=COORDINATED_CONTROL && loop_mode && position_active ?
            $signed({8'd0,arm_loop_config_i[71:64]}):
            return_mode ? $signed({8'd0,return_config_i[71:64]}):cfg_K_THETA_DOT[15:0];assign lock_origin=0;
    end endgenerate

    wire hold_mode=HOLD_CONTROL && motion_config_i[32];
    wire hold_ready=system_ready && balance_selected && arm_ok && position_active &&
                    TEST_MODE==0 && !rearm_i && pid_permit_i;
    wire signed [31:0] arm_vref,arm_aref;
    wire [8:0] hold_blend;
    wire hold_limited;
    generate if(HOLD_CONTROL)begin: hold_enabled
        arm_reference_coordinator #(.PERIOD_MS(UPDATE_DIV)) hold_i(
            .clk(clk),.rst_n(rst_n),.update_i(update_now),.ready_i(hold_ready),
            .enabled_i(hold_mode),.config_i(motion_config_i[223:32]),
            .position_i(position_feedback),.speed_i(balance_speed),
            .velocity_ref_o(arm_vref),.angle_ref_o(arm_aref),
            .blend_o(hold_blend),.limited_o(hold_limited));
    end else begin: hold_disabled
        assign arm_vref=0;assign arm_aref=0;assign hold_blend=0;assign hold_limited=0;
    end endgenerate
    wire signed [31:0] coordinate_target,coordinate_reference;
    wire [2:0] coordinate_state;
    wire coordinate_hold,coordinate_recovery,coordinate_limited,coordinate_known;
    // R15G_TRACE_BEGIN
    wire [3:0] trace_coordinate_reason;
    assign trace_position_o=position_feedback;
    assign trace_alpha_o=alpha_s;
    assign trace_alpha_dot_o=alpha_dot_s;
    assign trace_reference_o=inner_reference;
    assign trace_target_o=coordinate_target[13:0];
    assign trace_reference_reason_o=!loop_mode ? 4'd12:!position_active ? 4'd13:
        !position_reference_enabled ? 4'd14:trace_coordinate_reason;
    // [12] bypass [11] velocity capped [10:8] state [7] position active,
    // [6] hold [5] recovery [4] confirmed zero [3] angle limited [2] known,
    // [1] mode1 [0] position reference enabled.
    assign trace_coordinate_flags_o={velocity_slew_bypassed,velocity_limited,
        coordinate_state,position_active,(coordinate_hold && !speed_provisional),coordinate_recovery,
        confirmed_stopped,coordinate_limited,coordinate_known,loop_mode,
        position_reference_enabled};
    // R15G_TRACE_END
    wire confirmed_stopped=balance_speed_select_i && !balance_speed_valid_i &&
        speed_status_i[19:17]==3'd5 && balance_speed_i==0;
    // Legacy HOLD-only term. R15's reference is already in inner_angle_error.
    wire signed [47:0] reference_p_q16;
    generate if(COORDINATED_CONTROL)begin: factored_reference_p
        assign reference_p_q16=0;
    end else begin: legacy_reference_p
        assign reference_p_q16=$signed(arm_aref)*$signed(cfg_K_ALPHA[15:0]);
    end endgenerate
    wire signed [63:0] reference_p=$signed({{16{reference_p_q16[47]}},reference_p_q16}) <<< 16;
    wire signed [63:0] original_arm=p_theta+p_theta_dot+trim_applied;
    wire signed [63:0] blend_delta=(reference_p-original_arm)*$signed({1'b0,hold_blend});
    wire brake_enabled=COORDINATED_CONTROL ? 1'b1:LIVE_BRAKE_CONFIG?brake_config_i[16]:1'b1;
    wire [31:0] planned_brake_distance;
    wire planned_brake_risk,planned_distance_valid;
    wire loop_mode=SPEED_LOOP_CONTROL && arm_loop_config_i[31:0]==1;
    wire loop_ready=loop_mode && system_ready && balance_selected && arm_ok && pid_permit_i && !rearm_i && TEST_MODE==0;
    wire signed [31:0] loop_reference;
    wire signed [63:0] loop_request;
    wire [2:0] loop_state;
    wire position_reference_enabled=COORDINATED_CONTROL && loop_mode && arm_loop_config_i[63:32]!=0;
    wire balance_velocity_known=loop_ready && position_active && balance_speed_select_i &&
        (speed_usable || confirmed_stopped);
    wire balance_velocity_confirmed=loop_ready && position_active && balance_speed_select_i &&
        (speed_confirmed || confirmed_stopped);
    wire velocity_config_ok=arm_loop_config_i[95:64]<=200 &&
        coordinate_config_i[82:70]>=1 && coordinate_config_i[82:70]<=500;
    wire signed [63:0] velocity_cap_q32=$signed({23'd0,coordinate_config_i[78:70],32'd0});
    wire signed [63:0] velocity_clipped=p_theta_dot>velocity_cap_q32 ? velocity_cap_q32:
        p_theta_dot< -velocity_cap_q32 ? -velocity_cap_q32:p_theta_dot;
    wire velocity_limited=COORDINATED_CONTROL && balance_velocity_known &&
        (p_theta_dot>velocity_cap_q32 || p_theta_dot< -velocity_cap_q32);
    // Exactly one confirmed control sample blends a previous provisional
    // contribution toward the full request. Only increases are blended;
    // withdrawal and wrong-sign rejection remain immediate. Q16 U fits
    // signed26 because the existing cap is <=500U. Inner pendulum PD and
    // subsequent confirmed updates bypass this narrow handoff completely.
    reg provisional_seen;
    reg signed [25:0] handoff_previous;
    wire signed [63:0] velocity_mapped=arm_loop_config_i[96] ? -velocity_clipped:velocity_clipped;
    wire signed [25:0] handoff_target=velocity_mapped[41:16];
    wire signed [25:0] handoff_from=handoff_previous!=0 &&
        handoff_previous[25]==handoff_target[25] ? handoff_previous:26'sd0;
    wire [25:0] handoff_target_abs=handoff_target[25] ? -handoff_target:handoff_target;
    wire [25:0] handoff_from_abs=handoff_from[25] ? -handoff_from:handoff_from;
    wire handoff_rejoining=SPEED_HANDOFF_CONTROL && COORDINATED_CONTROL &&
        balance_velocity_confirmed && velocity_config_ok && provisional_seen &&
        handoff_target_abs>handoff_from_abs;
    wire signed [26:0] handoff_sum=$signed({handoff_from[25],handoff_from})+
        $signed({handoff_target[25],handoff_target});
    wire signed [25:0] handoff_blend=handoff_sum>>>1;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin provisional_seen<=0;handoff_previous<=0;end
        else if(!SPEED_HANDOFF_CONTROL || !loop_ready || !position_active || !velocity_config_ok)begin
            provisional_seen<=0;handoff_previous<=0;
        end else if(update_now)begin
            handoff_previous<=arm_request[41:16];
            if(speed_provisional && balance_velocity_known)provisional_seen<=1;
            else if(speed_confirmed || confirmed_stopped)provisional_seen<=0;
        end
    end
    generate if(COORDINATED_CONTROL)begin: coordinated_outer_loop
        arm_balance_coordinator_r15 #(.POINT_CONTROL(POINT_CONTROL),.TARGET_BITS(TARGET_BITS),.MOTION_CONTROL(MOTION_CONTROL),.MOTION_LEAN_CONTROL(MOTION_LEAN_CONTROL)) coordinate_i(
            // R15G_TRACE_BEGIN
            .trace_reference_reason_o(trace_coordinate_reason),
            // R15G_TRACE_END
            .clk(clk),.rst_n(rst_n),.update_i(update_now),
            .catch_direction_known_i((!PID_CONTROL || (DUAL_MODE && loop_mode)) && cfg_K_ALPHA[15:0]!=0 &&
                !cfg_K_ALPHA[15] && !cfg_K_ALPHA_DOT[15]),
            .point_config_i(point_config_i),.point_target_i(position_target),
            .motion_lean_i(motion_lean),.motion_active_i(motion_active),.goal_error_i(goal_error),.integral_config_i(motion_pi_config_i[45:30]),
            .output_saturated_i(u_sgn>=cfg_U_LIMIT || u_sgn<=-cfg_U_LIMIT),.integral_angle_o(outer_i_angle),.integral_status_o(outer_i_status),
            .point_status_o(point_status),.point_near_o(point_near),.point_braking_o(point_braking),
            .ready_i(loop_ready && position_active && position_reference_enabled),
            .speed_known_i(balance_speed_select_i && (speed_confirmed || confirmed_stopped)),
            .stopped_i(confirmed_stopped),.position_error_i(position_feedback),
            .speed_i(balance_speed_i),.alpha_i(alpha_s),.alpha_dot_i(alpha_dot_s),
            .position_gain_i(arm_loop_config_i[46:32]),
            .reverse_i(arm_loop_config_i[96]),
            .angle_limit_i(coordinate_config_i[12:0]),.angle_step_i(coordinate_config_i[22:13]),
            .hold_band_i(coordinate_config_i[37:23]),.alpha_gate_i(coordinate_config_i[51:38]),
            .alpha_dot_gate_i(coordinate_config_i[69:52]),
            .target_angle_o(coordinate_target),.reference_angle_o(coordinate_reference),
            .state_o(coordinate_state),.hold_o(coordinate_hold),.recovery_o(coordinate_recovery),
            .limited_o(coordinate_limited),.known_o(coordinate_known));
        assign loop_reference=0;
        assign loop_request=balance_velocity_known && velocity_config_ok ?
            (handoff_rejoining ? {{22{handoff_blend[25]}},handoff_blend,16'd0}:
                velocity_mapped):64'sd0;
        assign loop_state=coordinate_state;
        assign planned_brake_distance=0;assign planned_brake_risk=0;assign planned_distance_valid=0;
    end else if(SPEED_LOOP_CONTROL)begin: speed_loop_enabled
        assign point_status=0;assign point_near=0;assign point_braking=0;
        // R15G_TRACE_BEGIN
        assign trace_coordinate_reason=4'd12;
        // R15G_TRACE_END
        wire done,reference_valid,overrun;
        wire signed [31:0] planned_reference;
        reg result_available;
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)result_available<=0;
            else if(!loop_ready)result_available<=0;
            else if(done)result_available<=1;
            else if(update_now)result_available<=0;
        end
        if(R14M_ENABLE)begin: conservative_planner
        arm_velocity_planner_r14m #(.LIVE_BRAKE_CONFIG(LIVE_BRAKE_CONFIG),.BRAKE_DISTANCE_GAIN_Q8(BRAKE_DISTANCE_GAIN_Q8)) planner_i(
            .brake_gain_i(brake_config_i[9:0]),.brake_enable_i(brake_enabled),.brake_accel_i(brake_config_i[35:17]),.approach_speed_i(brake_config_i[52:36]),
            .brake_distance_o(planned_brake_distance),.brake_risk_o(planned_brake_risk),.brake_distance_valid_o(planned_distance_valid),.clk(clk),.rst_n(rst_n),.update_i(update_now),
            .enabled_i(loop_ready),.speed_valid_i(balance_speed_valid_i && balance_speed_select_i),
            .position_target_i({{13{position_reference[18]}},position_reference}),
            .position_actual_i(position_active ? position_next : 32'sd0),
            .actual_arm_speed_i(balance_speed_i),.sample_id_i(32'd0),
            .valid_o(done),.reference_valid_o(reference_valid),.arm_velocity_reference_o(planned_reference),
            .braking_state_o(loop_state),.overrun_o(overrun));
        end else begin: baseline_planner
        assign planned_brake_distance=0;assign planned_brake_risk=0;assign planned_distance_valid=0;
        arm_velocity_planner planner_i(.clk(clk),.rst_n(rst_n),.update_i(update_now),
            .enabled_i(loop_ready),.speed_valid_i(balance_speed_valid_i && balance_speed_select_i),
            .position_target_i({{13{position_reference[18]}},position_reference}),
            .position_actual_i(position_active ? position_next : 32'sd0),
            .actual_arm_speed_i(balance_speed_i),.sample_id_i(32'd0),
            .valid_o(done),.reference_valid_o(reference_valid),.arm_velocity_reference_o(planned_reference),
            .braking_state_o(loop_state),.overrun_o(overrun));
        end
        // Old-direction transition reference is never applied: actively brake to zero.
        wire signed [32:0] current_error=$signed({{14{position_reference[18]}},position_reference})-$signed({position_next[31],position_next});
        wire reference_direction_ok=(planned_reference==0) || (planned_reference>0 && current_error>0) || (planned_reference<0 && current_error<0);
        // R14M: an unavailable/overdue plan means zero-reference damping,
        // not an open velocity loop. Invalid measured speed still inhibits output.
        assign loop_reference=reference_valid && (!R14M_ENABLE || !overrun) &&
            (R14M_ENABLE || reference_direction_ok) && balance_speed_valid_i ? planned_reference:32'sd0;
        arm_speed_loop speed_p_i(.enabled_i(loop_ready),
            .valid_i((R14M_ENABLE || (result_available && !overrun)) && balance_speed_valid_i && balance_speed_select_i),
            .reference_i(loop_reference),.speed_i(balance_speed_i),
            .gain_i(arm_loop_config_i[63:32]),.limit_i(arm_loop_config_i[95:64]),
            .sign_i(arm_loop_config_i[127:96]),.request_o(loop_request));
    end else begin
        assign loop_reference=0;assign loop_request=0;assign loop_state=0;
        assign point_status=0;assign point_near=0;assign point_braking=0;
        assign planned_brake_distance=0;assign planned_brake_risk=0;assign planned_distance_valid=0;
    end endgenerate
    generate if(!COORDINATED_CONTROL)begin: no_coordinate_diagnostics
        // R15G_TRACE_BEGIN
        if(!SPEED_LOOP_CONTROL)assign trace_coordinate_reason=4'd12;
        // R15G_TRACE_END
        assign outer_i_angle=0;assign outer_i_status=0;
        assign coordinate_target=0;assign coordinate_reference=0;assign coordinate_state=0;
        assign coordinate_hold=0;assign coordinate_recovery=0;assign coordinate_limited=0;assign coordinate_known=0;
    end endgenerate
    // First capture update retains the original capture request. Later updates use the selected outer loop.
    wire signed [63:0] arm_target=loop_mode ?
        (position_active ? loop_request:p_theta_dot):
        (hold_mode ? original_arm+(blend_delta >>> 8) : original_arm);
    // "brake" is a request in the existing mapping, not proof of measured deceleration.
    wire [31:0] brake_speed_abs=balance_speed_i[31]?-balance_speed_i:balance_speed_i;
    wire [31:0] brake_ref_abs=loop_reference[31]?-loop_reference:loop_reference;
    wire brake_toward=(position_next>0 && balance_speed_i<0) || (position_next<0 && balance_speed_i>0);
    wire brake_requested=loop_ready && position_active && balance_speed_valid_i && balance_speed_select_i &&
        (brake_speed_abs>brake_ref_abs || !brake_toward) && brake_speed_abs!=0 && loop_request!=0 &&
        (arm_loop_config_i[127:96]==0 ? (loop_request[63]==balance_speed_i[31]) : (loop_request[63]!=balance_speed_i[31]));
    wire pendulum_worsening=(alpha_s>0 && alpha_dot_s>0)||(alpha_s<0 && alpha_dot_s<0);
    // R15C: share the existing inner sum with the final output and catch-priority
    // sign test. This is not a second motor controller or velocity feedback term.
    wire signed [63:0] pendulum_request=-p_alpha-p_alpha_dot-integral_term;
    wire [31:0] alpha_dot_abs32=alpha_dot_s[31] ? -alpha_dot_s:alpha_dot_s;
    wire velocity_catch_priority=pendulum_worsening &&
        (alpha_abs32>={18'd0,coordinate_config_i[51:38]} ||
         alpha_dot_abs32>={14'd0,coordinate_config_i[69:52]});
    wire brake_fast=R14M_ENABLE && brake_enabled && brake_requested && !pendulum_worsening;
    // States:0 inactive,1 normal,2 braking request,3 reversal,4 near+moving,
    // 5 near+slow (not a hold latch),6 speed invalid,7 worsening pendulum.
    wire [2:0] brake_state=!loop_ready?0:!balance_speed_valid_i || !balance_speed_select_i?6:!brake_enabled?0:pendulum_worsening?7:
        loop_state==6?3:loop_state==5?(brake_speed_abs>3277?4:5):brake_requested?2:1;
    wire signed [63:0] arm_request,legacy_slew_request;
    wire velocity_releasing,velocity_catch_release;
    wire arm_slew_permit=system_ready && balance_selected && arm_ok &&
        pid_permit_i && !rearm_i && TEST_MODE==0;
    wire brake_output_active=brake_enabled && loop_ready && position_active && balance_speed_valid_i && balance_speed_select_i &&
        brake_speed_abs!=0 && arm_request!=0 &&
        (arm_loop_config_i[127:96]==0 ? (arm_request[63]==balance_speed_i[31]) : (arm_request[63]!=balance_speed_i[31]));
    generate if(ARM_SLEW_U_PER_UPDATE>0)begin: arm_slew_enabled
        if(R14M_ENABLE)begin: conservative_slew
        arm_output_slew_r14m #(.STEP_U(ARM_SLEW_U_PER_UPDATE),.LIVE_BRAKE_CONFIG(COORDINATED_CONTROL?0:LIVE_BRAKE_CONFIG),
            .BRAKE_STEP_U(BRAKE_STEP_U),.VELOCITY_TRANSITION(COORDINATED_CONTROL),.RELEASE_STEP_U(50)) slew_i(
            .brake_step_i(brake_config_i[15:10]),.release_rate_i(brake_config_i[62:53]),.release_enable_i(loop_mode && brake_enabled),
            .brake_i(brake_fast),.clk(clk),.rst_n(rst_n),.update_i(update_now),
            // Speed validity chooses the velocity TARGET, not the history reset.
            // Brief estimator pulses between control updates cannot erase state.
            .ready_i(arm_slew_permit && (COORDINATED_CONTROL && loop_mode ? velocity_config_ok:
                (!loop_mode || !position_active || (balance_speed_valid_i && balance_speed_select_i)))),
            .transition_i(loop_mode && position_active),.speed_valid_i(balance_speed_valid_i && balance_speed_select_i),
            .catch_priority_i(velocity_catch_priority),.pendulum_request_i(pendulum_request),
            .stage_i(balance_selected),.request_i(arm_target),.request_o(legacy_slew_request),
            .transition_releasing_o(velocity_releasing),.transition_catch_o(velocity_catch_release));
        end else begin: baseline_slew
        arm_output_slew #(.STEP_U(ARM_SLEW_U_PER_UPDATE)) slew_i(
            .clk(clk),.rst_n(rst_n),.update_i(update_now),
            .ready_i(system_ready && balance_selected && arm_ok &&
                     pid_permit_i && !rearm_i && TEST_MODE==0),
            .stage_i(balance_selected),.request_i(arm_target),.request_o(legacy_slew_request));
        assign velocity_releasing=0;assign velocity_catch_release=0;
        end
    end else begin: arm_slew_bypass
        assign legacy_slew_request=arm_target;
        assign velocity_releasing=0;assign velocity_catch_release=0;
    end endgenerate
    // R15B: only POSITION uses the bounded angle reference. Velocity damping
    // keeps the validated U-domain gain/cap and original 10/20U slew; it is never
    // clipped by the +/-2deg position reference or its recovery/hold gates.
    // R15C adds 50U/5ms withdrawal and urgent stale-contribution removal.
    // Pendulum P/D remains outside this velocity-only limiter.
    // Only bypass the velocity contribution's rate limiter. loop_request already
    // checks speed validity, controller permission and the independent U cap.
    // Mode0 and the first capture update continue through the original path.
    wire velocity_bypass_requested=LIVE_VELOCITY_BYPASS ? velocity_slew_bypass_i:VELOCITY_SLEW_BYPASS;
    wire velocity_slew_bypassed=COORDINATED_CONTROL && velocity_bypass_requested &&
        loop_mode && position_active;
    generate if(DUAL_MODE)begin: parallel_request
      wire signed [63:0] cruise_request;
      cruise_parallel_feedback combine_i(.selected_i(cruise_selected && position_active),
        .ready_i(cruise_ready),.recovering_i(cruise_recovering),
        .position_request_i(p_theta),.velocity_request_i(p_theta_dot),
        .pendulum_request_i(pendulum_request),.limit_u_i(cruise_config_i[223:192]),.request_o(cruise_request));
      assign arm_request=cruise_selected && position_active ? cruise_request:
        velocity_slew_bypassed ? loop_request:legacy_slew_request;
    end else begin
      assign arm_request=velocity_slew_bypassed ? loop_request:legacy_slew_request;
    end endgenerate
    wire signed [31:0] pid_error=hold_mode ? subtract_saturated(alpha_s,arm_aref) : alpha_s;
    wire signed [31:0] integral_q16;
    wire pid_ready=(!DUAL_MODE || !loop_mode) && pid_permit_i && system_ready && balance_selected && arm_ok && TEST_MODE==0 && !rearm_i && (!hold_mode || hold_blend==256);
    wire signed [63:0] integral_term=$signed({{32{integral_q16[31]}},integral_q16}) <<< 16;
    generate if(PID_CONTROL) begin: pid_enabled
        balance_pid_integrator #(.PERIOD_MS(UPDATE_DIV),.COMPACT(DUAL_MODE)) pid_i(
            .clk(clk),.rst_n(rst_n),.update_i(update_now),.ready_i(pid_ready),
            .config_i(pid_config_i),.alpha_i(pid_error),
            .total_q16_i(acc[63:16]),.limit_u_i(cfg_U_LIMIT),.integral_q16_o(integral_q16));
    end else begin: pid_disabled
        assign integral_q16=0;
    end endgenerate
    // R14 adds ONLY a bounded position integral; original P and velocity remain.
    wire signed [31:0] position_integral_q16;
    wire [5:0] position_i_status;
    wire position_i_ready=pid_permit_i && system_ready && balance_selected && arm_ok &&
        position_active && !loop_mode && TEST_MODE==0 && !rearm_i;
    wire signed [63:0] position_integral_term=
        $signed({{32{position_integral_q16[31]}},position_integral_q16}) <<< 16;
    generate if(POSITION_I_CONTROL)begin: position_i_enabled
        arm_position_integrator #(.PERIOD_MS(UPDATE_DIV)) position_i_inst(
            .clk(clk),.rst_n(rst_n),.update_i(update_now),.ready_i(position_i_ready),
            .config_i(position_i_config_i),.position_i(position_feedback),
            .speed_i(balance_speed),.alpha_i(alpha_s),.alpha_dot_i(alpha_dot_s),
            .total_q16_i(acc[63:16]),.limit_u_i(cfg_U_LIMIT),
            .integral_q16_o(position_integral_q16),.status_o(position_i_status));
    end else begin: position_i_disabled
        assign position_integral_q16=0;assign position_i_status=0;
    end endgenerate
    wire signed [63:0] acc =
          arm_request
        + pendulum_request
        + (loop_mode ? 64'sd0:position_integral_term);

    wire signed [63:0] acc_magnitude = acc[63] ? -acc : acc;
    wire signed [31:0] u_magnitude = acc_magnitude[63:32];
    wire signed [31:0] u_lin = acc[63] ? -u_magnitude : u_magnitude;

    wire signed [31:0] u_sgn = (SIGN_MOTOR < 0) ? -u_lin : u_lin;

    wire signed [31:0] u_lim =
        (u_sgn >  cfg_U_LIMIT) ?  cfg_U_LIMIT :
        (u_sgn < -cfg_U_LIMIT) ? -cfg_U_LIMIT : u_sgn;

    wire signed [31:0] alpha_abs = alpha[31] ? -alpha : alpha;
    wire signed [31:0] theta_abs = theta_w[31] ? -theta_w : theta_w;

    wire arm_ok      = (alpha_abs <= cfg_ALPHA_ARM) && (theta_abs <= THETA_LIM);
    wire system_ready = rst_n && (system_state == RUN) && sensor_valid && (!LIVE_CONFIG || run_i);

    wire update_now=control_tick && (div_cnt >= UPDATE_DIV-1);
    wire balance_selected;
    wire signed [15:0] swing_u;
    wire [2:0] swing_phase;
    wire [3:0] swing_reason;
    wire swing_top_boost;
    wire signed [31:0] swing_travel, swing_base_dbg, swing_speed_dbg;
    // Entry selection only: automatic supervisor and balance arithmetic stay intact.
    wire manual_mode=START_MODE_CONTROL && manual_start_i;
    wire auto_balance_selected,auto_swing_top_boost;
    wire signed [15:0] auto_swing_u;
    wire [2:0] auto_swing_phase,manual_phase;
    wire [3:0] auto_swing_reason,manual_reason;
    wire signed [31:0] auto_swing_travel,auto_swing_base_dbg,auto_swing_speed_dbg;
    generate if(START_MODE_CONTROL) begin: manual_entry_enabled
        manual_balance_entry entry_i(.clk(clk),.rst_n(rst_n),.update_i(update_now),
            .enabled_i(manual_mode),.run_i(run_i),.rearm_i(rearm_i),
            .ready_i(system_ready && pid_permit_i),.balance_window_i(arm_ok),
            .alpha_i(alpha),.alpha_dot_i(alpha_dot),.theta_dot_i(theta_dot),
            .capture_angle_i(config_i[576+:32]),.capture_ad_i(config_i[608+:32]),
            .capture_td_i(config_i[928+:32]),.phase_o(manual_phase),.reason_o(manual_reason));
    end else begin: no_manual_entry
        assign manual_phase=0;assign manual_reason=0;
    end endgenerate
    assign balance_selected=manual_mode ? (manual_phase==3'd5) : auto_balance_selected;
    assign swing_u=manual_mode ? 16'sd0 : auto_swing_u;
    assign swing_phase=manual_mode ? manual_phase : auto_swing_phase;
    assign swing_reason=manual_mode ? manual_reason : auto_swing_reason;
    assign swing_travel=manual_mode ? 32'sd0 : auto_swing_travel;
    assign swing_base_dbg=manual_mode ? 32'sd0 : auto_swing_base_dbg;
    assign swing_speed_dbg=manual_mode ? 32'sd0 : auto_swing_speed_dbg;
    assign swing_top_boost=manual_mode ? 1'b0 : auto_swing_top_boost;
    swingup_supervisor #(
        .LIVE_CONFIG(LIVE_CONFIG),
        .ENABLE(AUTO_SWINGUP), .SETPOINT_Q16(SWING_SETPOINT_Q16), .SWING_U(SWING_U),
        .KICK_UPDATES(SWING_KICK_UPDATES), .START_ONLY(SWING_START_ONLY),
        .ARM_SPEED_GAIN(SWING_ARM_SPEED_GAIN), .KICK_END_SPEED_Q16(SWING_KICK_END_SPEED_Q16)
    ) u_swingup (
        .config_i(config_i), .run_i(run_i && !manual_mode), .rearm_i(rearm_i),
        .clk(clk), .rst_n(rst_n), .update_i(update_now),
        .ready_i(system_ready && !manual_mode), .balance_window_i(arm_ok),
        .theta_i(theta_w), .theta_dot_i(theta_dot),
        .alpha_i(alpha), .alpha_dot_i(alpha_dot),
        .balance_selected_o(auto_balance_selected), .swing_u_o(auto_swing_u),
        .phase_o(auto_swing_phase), .reason_o(auto_swing_reason), .travel_o(auto_swing_travel),
        .base_u_o(auto_swing_base_dbg), .speed_u_o(auto_swing_speed_dbg),
        .top_boost_o(auto_swing_top_boost)
    );

    always @(posedge clk) begin
        if (!rst_n || (LIVE_CONFIG && rearm_i)) begin
            // R15G_TRACE_BEGIN
            trace_sampled_position_o<=0;trace_sampled_reason_o<=13;trace_sampled_flags_o<=0;
            // R15G_TRACE_END
            speed_trim_small <= 0;
            position_active <= 0;
            theta_previous <= 0;
            position_error <= 0;
            div_cnt       <= {DIV_W{1'b0}};
            control_u     <= 16'sd0;
            control_valid <= 1'b0;
            motor_enable  <= 1'b0;
            dbg_motion_position<=0;dbg_motion_velocity<=0;dbg_outer_i_angle<=0;
            dbg_theta <= 0; dbg_theta_dot <= 0;
            dbg_settle_status<=0;dbg_capture_travel<=0;dbg_lock_origin<=0;dbg_hold_position<=0;
            dbg_brake_distance<=0;dbg_velocity_error<=0;dbg_brake_flags<=0;dbg_brake_status<=0;dbg_loop_reference<=0;dbg_loop_state<=0;dbg_original_arm_q16<=0;dbg_arm_request_q16<=0;dbg_speed_status<=0;dbg_raw_speed<=0; dbg_est_speed<=0; dbg_speed_source<=0; dbg_est_valid<=0;
            dbg_alpha <= 0; dbg_alpha_dot <= 0;
            dbg_vref<=0;dbg_aref<=0;dbg_hold_control<=0;
            dbg_pi<=0;
            dbg_pa <= 0; dbg_pd <= 0; dbg_pt <= 0; dbg_pv <= 0;
            dbg_arm_ok <= 0; dbg_ready <= 0; dbg_phase<=0; dbg_reason<=0;
        end
        else begin
            control_valid <= 1'b0;
            motor_enable  <= system_ready;

            if (control_tick) begin
                if (div_cnt >= UPDATE_DIV - 1) begin
                    div_cnt       <= {DIV_W{1'b0}};
                    control_valid <= 1'b1;
                    // R15G_TRACE_BEGIN
                    trace_sampled_position_o<=trace_position_o;
                    trace_sampled_reason_o<=trace_reference_reason_o;
                    trace_sampled_flags_o<=trace_coordinate_flags_o;
                    // R15G_TRACE_END
                    speed_trim_small <= trim_applied;
                    theta_previous <= theta_w;
                    if (!system_ready || !balance_selected || !arm_ok || TEST_MODE != 0) begin
                        position_active <= 0;
                        position_error <= 0;
                    end else begin
                        position_active <= 1;
                        position_error <= position_active ? position_next : 32'sd0;
                    end
                    dbg_vref<=arm_vref;dbg_aref<=arm_aref;
                    dbg_hold_control<={15'd0,hold_blend,4'd0,hold_limited,hold_ready && hold_mode,hold_mode,
                        motion_config_i[0] && balance_selected};
                    if(POSITION_I_CONTROL)begin
                        // Marker bit31: VR = position I Q16 U; R14B AR = effective position cap Q16 U.
                        // bit7 I mode2, bit8 return coordination, bit9 negative motor sign.
                        dbg_vref <= (SIGN_MOTOR<0) ? -position_integral_q16 : position_integral_q16;
                        dbg_aref <= return_mode ? $signed(selected_position_limit >>> 16) : 32'sd0;
                        dbg_hold_control <= {1'b1,19'd0,speed_filter_mode,(SIGN_MOTOR<0),return_mode,(position_i_config_i[1:0]==2),position_i_status,
                            motion_config_i[0] && balance_selected};
                    end
                    if(COORDINATED_CONTROL)begin
                        // HC bit30 identifies R15 in both modes; bit31 remains the R14 I marker.
                        // [10:8] state, [7] position active, [6] hold, [5] recovery,
                        // [4] confirmed zero, [3] limited, [2] known, [1] mode1, [0] fine drive.
                        // bit28 R15C, bit13 controlled withdrawal, bit14 urgent
                        // stale velocity withdrawal; all align with this U sample.
                        // bit26 identifies R15D; bit27 reports the ACTUAL bypass.
                        // Do not report the disconnected limiter's flags as actions.
                        dbg_hold_control<=32'h74000000 |
                            (SPEED_HANDOFF_CONTROL ? 32'h00008000:32'd0) |
                            (POINT_CONTROL ? (32'h01000000 | {8'd0,point_status,16'd0}):32'd0) |
                            (LIVE_VELOCITY_BYPASS ? 32'h02000000:32'd0) |
                            (velocity_slew_bypassed ? 32'h08000000:32'd0) |
                            (position_reference_enabled ? 32'h00000800:32'd0) |
                            (velocity_limited ? 32'h00001000:32'd0) |
                            (!velocity_slew_bypassed && velocity_releasing ? 32'h00002000:32'd0) |
                            (!velocity_slew_bypassed && velocity_catch_release ? 32'h00004000:32'd0) |
                            {21'd0,coordinate_state,position_active,
                            (coordinate_hold && !speed_provisional),coordinate_recovery,confirmed_stopped,coordinate_limited,
                            balance_velocity_confirmed,loop_mode,motion_config_i[0] && balance_selected};
                        dbg_aref<=loop_mode ? coordinate_reference:0;
                        dbg_vref<=loop_mode ? coordinate_target:0;
                    end
                    if(DUAL_MODE && cruise_selected)begin
                      // HC bit31+29, bit30 clear: dual parallel cruise telemetry.
                      dbg_hold_control<=32'ha0000000 | {25'd0,cruise_state,cruise_speed_known,cruise_recovering,cruise_correction,cruise_selected};
                      dbg_vref<=cruise_vref;dbg_aref<=cruise_config_i[63:32];
                    end
                    dbg_settle_status<={4'd0,settle_mode,effective_kv,settle_weight,settle_stage};
                    dbg_capture_travel<=position_active ? position_next : 32'sd0;
                    dbg_lock_origin<=lock_origin;
                    dbg_hold_position<=cruise_selected ? cruise_position:position_active ? (settle_locked ? subtract_saturated(position_next,lock_origin) : position_next) : 32'sd0;
                    dbg_motion_position<=effective_position_reference;dbg_motion_velocity<=motion_enabled ? motion_velocity:32'sd0;
                    dbg_outer_i_angle<={{18{outer_i_angle[13]}},outer_i_angle};
                    dbg_theta <= balance_selected ? position_feedback : swing_travel;
                    dbg_theta_dot <= feedback_speed;
                    dbg_raw_speed <= theta_dot;
                    dbg_est_speed <= BALANCE_SPEED_INPUT ? balance_speed_i : theta_dot;
                    dbg_brake_status<={brake_enabled,brake_requested,(R14M_ENABLE!=0 && loop_mode),planned_brake_risk,brake_output_active,brake_state};
                    dbg_brake_distance<=planned_brake_distance;
                    dbg_velocity_error<=POINT_CONTROL && loop_mode ? $signed({position_feedback[31],position_feedback}):
                        $signed({loop_reference[31],loop_reference})-$signed({balance_speed_i[31],balance_speed_i});
                    dbg_brake_flags<={pendulum_worsening,planned_distance_valid,loop_ready,position_active};
                    dbg_loop_reference<=POINT_CONTROL && loop_mode ? {{(32-TARGET_BITS){position_target[TARGET_BITS-1]}},position_target}:
                        loop_mode ? loop_reference:0;
                    dbg_loop_state<={loop_mode,loop_state};
                    dbg_original_arm_q16<=balance_selected ? arm_debug_q16(arm_target) : 32'sd0;
                    dbg_arm_request_q16<=balance_selected ? arm_debug_q16(arm_request) : 32'sd0;
                    dbg_speed_source <= alternate_speed;
                    // selection reason: 0 configured raw,1 outside balance,2 valid hybrid,3 unconfirmed zero.
                    // SD bit22: actual one-sample restoration blend (HC15 capability).
                    dbg_speed_status <= {15'd0,speed_status_i[36:20],(MOTION_CONTROL!=0),motion_enabled,motion_active,motion_paused,outer_i_status,motion_calculating,handoff_rejoining,(!balance_speed_select_i ? 2'd0 : !balance_selected ? 2'd1 : balance_speed_valid_i ? 2'd2 : 2'd3),speed_status_i[19:0]};
                    dbg_est_valid <= BALANCE_SPEED_INPUT && balance_speed_valid_i;
                    dbg_alpha <= alpha_s;
                    dbg_alpha_dot <= alpha_dot_s;
                    dbg_pi <= ((SIGN_MOTOR < 0) ? integral_term : -integral_term) >>> 32;
                    dbg_pa <= ((SIGN_MOTOR < 0) ? p_alpha : -p_alpha) >>> 32;
                    dbg_pd <= ((SIGN_MOTOR < 0) ? p_alpha_dot : -p_alpha_dot) >>> 32;
                    dbg_pt <= loop_mode ? 32'sd0 : ((SIGN_MOTOR < 0) ? -p_theta : p_theta) >>> 32;
                    dbg_pv <= loop_mode && position_active ? (((SIGN_MOTOR<0)?-loop_request:loop_request) >>> 32) : ((SIGN_MOTOR < 0) ? -(p_theta_dot+trim_applied) : (p_theta_dot+trim_applied)) >>> 32;
                    if(COORDINATED_CONTROL && loop_mode && position_active)
                        dbg_pv<=((SIGN_MOTOR<0) ? -arm_request:arm_request) >>> 32;
                    if(hold_mode)begin
                        dbg_pt<=0;
                        dbg_pv<=((SIGN_MOTOR<0) ? -arm_request : arm_request) >>> 32;
                    end
                    if (!balance_selected) begin
                        dbg_pa<=swing_base_dbg; dbg_pd<=swing_top_boost ? 32'sd1 : 32'sd0;
                        dbg_pi<=0; dbg_pt<=0; dbg_pv<=swing_speed_dbg;
                    end
                    dbg_phase<=swing_phase; dbg_reason<=swing_reason;
                    dbg_arm_ok <= arm_ok;
                    dbg_ready <= system_ready;

                    if (!system_ready)
                        control_u <= 16'sd0;
                    else if (TEST_MODE != 0)
                        control_u <= (SIGN_MOTOR < 0) ? -TEST_U : TEST_U;
                    else if (!balance_selected)
                        control_u <= swing_u;
                    else if (!arm_ok)
                        control_u <= 16'sd0;
                    else
                        control_u <= u_lim[15:0];
                end
                else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire

// R11 coordinator. No independent motor request: outputs weight and ONE speed gain.
`default_nettype none
module settle_lock_coordinator(
    input wire clk,rst_n,update_i,enabled_i,ready_i,
    input wire [255:0] config_i,
    input wire signed [15:0] base_kv_i,
    input wire signed [31:0] position_i,speed_i,raw_speed_i,alpha_i,alpha_dot_i,
    output reg [1:0] stage_o,
    output reg [8:0] position_weight_o,
    output wire signed [15:0] effective_kv_o,
    output reg signed [31:0] lock_origin_o
);
    // stage: 0 original/idle, 1 decelerate, 2 restore position, 3 hold locked point.
    reg [8:0] gain_weight,quiet_count;
    reg signed [31:0] quiet_origin;
    function [31:0] magnitude;
        input signed [31:0] value;
        begin magnitude=value[31] ? (~value+32'd1) : value;end
    endfunction
    wire [31:0] speed_abs=magnitude(speed_i),raw_abs=magnitude(raw_speed_i);
    wire [31:0] alpha_abs=magnitude(alpha_i),ad_abs=magnitude(alpha_dot_i);
    wire [31:0] lock_speed=config_i[64+:32],lock_angle=config_i[96+:32];
    wire [31:0] lock_ad=config_i[128+:32],hold_ticks=config_i[160+:32];
    wire [31:0] gate_angle=config_i[192+:32],gate_ad=config_i[224+:32];
    wire signed [15:0] stop_kv=config_i[32+:16];
    wire tilt_worsening=(alpha_i!=0) && (alpha_dot_i!=0) && (alpha_i[31]==alpha_dot_i[31]);
    wire gain_allowed=(stage_o==1) && (alpha_abs<=gate_angle) && (ad_abs<=gate_ad) &&
                      !tilt_worsening && (speed_abs>lock_speed);
    wire quiet_now=(speed_abs<=lock_speed) && (raw_abs<=lock_speed) &&
                   (alpha_abs<=lock_angle) && (ad_abs<=lock_ad);
    wire signed [32:0] quiet_delta=$signed({position_i[31],position_i})-
                                    $signed({quiet_origin[31],quiet_origin});
    wire within_drift=(quiet_delta<=33'sd1144) && (quiet_delta>=-33'sd1144); // 1 degree
    wire signed [16:0] gain_difference=$signed({stop_kv[15],stop_kv})-
                                      $signed({base_kv_i[15],base_kv_i});
    wire signed [26:0] gain_product=gain_difference*$signed({1'b0,gain_weight});
    assign effective_kv_o=(!enabled_i || !ready_i) ? base_kv_i :
                          $signed(base_kv_i)+(gain_product >>> 8);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            stage_o<=0;position_weight_o<=256;gain_weight<=0;
            quiet_count<=0;quiet_origin<=0;lock_origin_o<=0;
        end else if(!enabled_i || !ready_i) begin
            stage_o<=0;position_weight_o<=256;gain_weight<=0;
            quiet_count<=0;quiet_origin<=0;lock_origin_o<=0;
        end else if(update_i) begin
            if(gain_allowed) begin if(gain_weight<256)gain_weight<=gain_weight+9'd4;end
            else if(gain_weight!=0)gain_weight<=gain_weight-9'd4;
            case(stage_o)
                0:begin stage_o<=1;position_weight_o<=256;quiet_count<=0;end
                1:begin
                    if(position_weight_o!=0)position_weight_o<=position_weight_o-9'd4;
                    if(position_weight_o!=0 || !quiet_now)quiet_count<=0;
                    else if(quiet_count==0 || !within_drift) begin
                        quiet_origin<=position_i;quiet_count<=1;
                    end else if(quiet_count>=hold_ticks-1) begin
                        lock_origin_o<=position_i;stage_o<=2;quiet_count<=0;
                    end else quiet_count<=quiet_count+1'b1;
                end
                2:begin
                    if(position_weight_o<256)position_weight_o<=position_weight_o+9'd4;
                    if(position_weight_o>=252)stage_o<=3;
                end
                3:begin position_weight_o<=256;quiet_count<=0;end
                default:begin stage_o<=0;position_weight_o<=256;gain_weight<=0;quiet_count<=0;end
            endcase
        end
    end
endmodule
`default_nettype wire

// R12: integral OUTPUT accumulator in Q16 U, where U=0.1% duty.
// Only integrates inside the configured small-angle window. Outside it, old
// integral decays (1/64 per update), rather than carrying a large bias into recovery.
`default_nettype none
module balance_pid_integrator #(parameter integer PERIOD_MS=5,COMPACT=0)(
    input wire clk,rst_n,update_i,ready_i,
    input wire [95:0] config_i,
    input wire signed [31:0] alpha_i,limit_u_i,
    input wire signed [47:0] total_q16_i,
    output wire signed [31:0] integral_q16_o
);
    reg signed [31:0] integral;
    wire enabled=(config_i[31:0]!=0) && (config_i[63:32]!=0);
    wire signed [31:0] gate=$signed(config_i[95:64]);
    wire in_window=(alpha_i<=gate) && (alpha_i>=-gate);
    // Valid gate <=11438 fits signed16; out-of-window samples never integrate.
    wire signed [31:0] delta;
    generate if(COMPACT && PERIOD_MS==5)begin: compact_delta
        // Only in-window samples use delta. Valid gate <=11438, Ki<=500:
        // |alpha| fits14, |alpha|*Ki fits23. floor(product/200)
        // =floor((product>>3)/25); restore sign for truncation toward zero.
        wire [13:0] magnitude=alpha_i[31] ? -alpha_i[13:0]:alpha_i[13:0];
        wire [22:0] product=magnitude*config_i[8:0];
        function [19:0] divide25;
          input [19:0] value;
          reg [4:0] remainder;
          reg [5:0] trial;
          reg [19:0] quotient;
          integer bit_index;
          begin
            remainder=0;quotient=0;trial=0;
            for(bit_index=19;bit_index>=0;bit_index=bit_index-1)begin
              trial={remainder,value[bit_index]};
              if(trial>=6'd25)begin
                remainder=trial-6'd25;quotient[bit_index]=1;
              end else remainder=trial[4:0];
            end
            divide25=quotient;
          end
        endfunction
        wire [19:0] quotient=divide25(product[22:3]);
        wire signed [31:0] positive={12'd0,quotient};
        assign delta=alpha_i[31] ? -positive:positive;
    end else begin: original_delta
        wire signed [25:0] product=$signed(alpha_i[15:0])*$signed({1'b0,config_i[8:0]});
        wire signed [41:0] timed_product=$signed(product)*PERIOD_MS;
        assign delta=timed_product / 42'sd1000;
    end endgenerate
    wire signed [31:0] bound=$signed({1'b0,config_i[38:32],16'd0});
    wire signed [32:0] proposed=$signed({integral[31],integral})+$signed({delta[31],delta});
    wire signed [31:0] candidate=(proposed>bound)?bound:((proposed < -bound)?-bound:proposed[31:0]);
    wire signed [32:0] change=$signed({candidate[31],candidate})-$signed({integral[31],integral});
    wire signed [47:0] output_bound=$signed(limit_u_i)*48'sd65536;
    wire signed [48:0] predicted=$signed({total_q16_i[47],total_q16_i})-change;
    // u = base - integral: reject only increments that worsen output saturation.
    wire windup=((predicted < -output_bound) && (change>0)) ||
                ((predicted > output_bound) && (change<0));
    wire [31:0] mag=integral[31] ? -integral : integral;
    wire signed [31:0] decay=(mag>>6)!=0 ? (mag>>6) : 32'd1;
    assign integral_q16_o=(ready_i && enabled) ? integral : 32'sd0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)integral<=0;
        else if(!ready_i || !enabled)integral<=0;
        else if(update_i) begin
            if(!in_window) begin
                if(integral>0)integral<=integral-decay;
                else if(integral<0)integral<=integral+decay;
            end else if(!windup)integral<=candidate;
        end
    end
endmodule
`default_nettype wire

// R13 reference planner; KX Q16 /s; KV Q16 s. No actuator output here.
`default_nettype none
module arm_reference_coordinator #(parameter integer PERIOD_MS=5)(
 input wire clk,rst_n,update_i,ready_i,enabled_i,
 input wire [191:0] config_i,
 input wire signed [31:0] position_i,speed_i,
 output wire signed [31:0] velocity_ref_o,angle_ref_o,
 output wire [8:0] blend_o,
 output wire limited_o
);
 // R13A: the 200Hz interface is unchanged. Snapshot a sample, then share ONE
 // shift/add datapath for Kx*x and Kv*(v-vref). Complete in 37 clk edges.
 // The controller consumes the completed reference on the NEXT control update,
 // exactly as in R13; never move the controller/PWM/watchdog update schedule.
 localparam IDLE=0,MUL_X=1,PREP_V=2,MUL_V=3,COMMIT=4;
 reg [2:0] state;
 reg [5:0] remaining;
 reg [49:0] multiplicand,accumulator;
 reg [17:0] multiplier;
 reg negative;
 reg signed [49:0] product_result;
 reg signed [31:0] speed_sample;
 reg signed [18:0] work_velocity;
 reg signed [15:0] kv_sample;
 reg [17:0] vmax_sample;
 reg [12:0] amax_sample;
 reg [15:0] rate_sample;
 reg velocity_limited;
 reg signed [13:0] angle_ref;
 reg signed [18:0] velocity_ref;
 reg [8:0] blend;
 reg limited;
 wire [49:0] add_result=accumulator+(multiplier[0] ? multiplicand : 50'd0);
 wire [31:0] position_mag=position_i[31] ? (~position_i+32'd1) : position_i;
 wire signed [49:0] vraw=-(product_result >>> 16);
 wire signed [31:0] vmax=$signed({14'd0,vmax_sample});
 wire signed [31:0] amax=$signed({19'd0,amax_sample});
 wire signed [31:0] vtarget=(vraw>vmax)?vmax:((vraw < -vmax)?-vmax:vraw[31:0]);
 wire signed [32:0] velocity_error=$signed({speed_sample[31],speed_sample})-$signed({vtarget[31],vtarget});
 wire [32:0] velocity_mag=velocity_error[32] ? (~velocity_error+33'd1) : velocity_error;
 wire [15:0] kv_mag=kv_sample[15] ? (~kv_sample+16'd1) : kv_sample;
 wire signed [49:0] araw=product_result >>> 16;
 wire signed [31:0] atarget=(araw>amax)?amax:((araw < -amax)?-amax:araw[31:0]);
 wire [31:0] step_raw;
 generate if(PERIOD_MS==5)begin: step_5ms
     // floor(rate*5/1000) == floor(rate/200), nonnegative uint16 input.
     wire [15:0] quotient=rate_sample / 16'd200;
     assign step_raw={16'd0,quotient};
 end else if(PERIOD_MS==1)begin: step_1ms
     wire [15:0] quotient=rate_sample / 16'd1000;
     assign step_raw={16'd0,quotient};
 end else begin: step_other
     localparam integer PW=16+$clog2(PERIOD_MS+1);
     wire [PW-1:0] scaled_rate=rate_sample*PERIOD_MS;
     assign step_raw=scaled_rate/1000;
 end endgenerate
 wire signed [31:0] step=(step_raw<1)?32'sd1:$signed(step_raw);
 wire signed [32:0] diff=$signed({atarget[31],atarget})-$signed({{19{angle_ref[13]}},angle_ref});
 assign velocity_ref_o=(enabled_i && ready_i)?velocity_ref:32'sd0;
 assign angle_ref_o=(enabled_i && ready_i)?angle_ref:32'sd0;
 assign blend_o=(enabled_i && ready_i)?blend:9'd0;
 assign limited_o=enabled_i && ready_i && limited;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;remaining<=0;multiplicand<=0;accumulator<=0;multiplier<=0;negative<=0;product_result<=0;
   speed_sample<=0;work_velocity<=0;kv_sample<=0;vmax_sample<=0;amax_sample<=0;rate_sample<=0;
   velocity_limited<=0;angle_ref<=0;velocity_ref<=0;blend<=0;limited<=0;
  end else if(!enabled_i || !ready_i)begin
   state<=IDLE;angle_ref<=0;velocity_ref<=0;blend<=0;limited<=0;
  end else begin
   case(state)
    IDLE:if(update_i)begin
     speed_sample<=speed_i;kv_sample<=config_i[79:64];vmax_sample<=config_i[113:96];
     amax_sample<=config_i[140:128];rate_sample<=config_i[175:160];
     multiplicand<={18'd0,position_mag};multiplier<=config_i[49:32];accumulator<=0;
     negative<=position_i[31];remaining<=18;state<=MUL_X;
    end
    MUL_X,MUL_V:begin
     accumulator<=add_result;multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
     remaining<=remaining-1'b1;
     if(remaining==1)begin
      product_result<=negative ? -$signed(add_result) : $signed(add_result);
      state<=(state==MUL_X) ? PREP_V : COMMIT;
     end
    end
    PREP_V:begin
     work_velocity<=vtarget;velocity_limited<=(vraw>vmax || vraw < -vmax);
     multiplicand<={17'd0,velocity_mag};multiplier<={2'd0,kv_mag};accumulator<=0;
     negative<=velocity_error[32]^kv_sample[15];remaining<=16;state<=MUL_V;
    end
    COMMIT:begin
     velocity_ref<=work_velocity;
     if(diff>step)angle_ref<=angle_ref+step;
     else if(diff < -step)angle_ref<=angle_ref-step;
     else angle_ref<=atarget;
     if(blend<256)blend<=blend+9'd4;
     limited<=velocity_limited || araw>amax || araw < -amax || diff>step || diff < -step;
     state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
`default_nettype wire

`default_nettype none
// R14 position I, Q16 U output (one U = 0.1% duty).
// x = actual - capture target; u = baseline + I_x. Signed Ki is independently tuned.
// Validated ranges: Ki[-64,64], limit[0,50]U, alpha gate<=5719,
// ad gate<=131072, speed gate<=196608. Clamp x to +/-1 rad before integration.
// Shift/add multiplication + serial constant division: no wide combinational
// multiplier/divider. At PERIOD_MS=5: 7 multiply + 26 divide + 2 service clocks.
// Same sampled-input timing as the original I: current output uses old state,
// newly committed I is consumed on the next 5ms control update.
module arm_position_integrator #(parameter integer PERIOD_MS=5)(
 input wire clk,rst_n,update_i,ready_i,
 input wire [191:0] config_i,
 input wire signed [31:0] position_i,speed_i,alpha_i,alpha_dot_i,
 input wire signed [47:0] total_q16_i,
 input wire signed [31:0] limit_u_i,
 output wire signed [31:0] integral_q16_o,
 output wire [5:0] status_o
);
 localparam integer DEN=1000/PERIOD_MS;
 localparam integer RW=$clog2(DEN)+1;
 localparam IDLE=0,MUL=1,DIVIDE=2,COMMIT=3;
 reg [1:0] state;
 reg [5:0] count,quiet_ticks;
 reg signed [23:0] integral;
 reg [25:0] multiplicand,product,quotient;
 reg [6:0] multiplier;
 reg [RW-1:0] remainder;
 reg negative;
 reg signed [47:0] total_sample;
 reg [9:0] output_limit_sample;
 reg [5:0] integral_limit_sample;
 reg integrated,last_limited,last_windup;
 wire signed [7:0] ki=config_i[39:32];
 wire improved=config_i[1:0]==2;
 wire enabled=(config_i[1:0]==1 || improved) && ki!=0 && config_i[69:64]!=0;
 wire [31:0] aa=alpha_i[31] ? -alpha_i : alpha_i;
 wire [31:0] ad=alpha_dot_i[31] ? -alpha_dot_i : alpha_dot_i;
 wire [31:0] vv=speed_i[31] ? -speed_i : speed_i;
 wire tilt_ok=aa<={19'd0,config_i[108:96]} && ad<={14'd0,config_i[145:128]};
 wire speed_ok=vv<={14'd0,config_i[177:160]};
 // R14A mode2: 50ms qualification; mild violations freeze without rearming.
 // Severe tilt/ad (>2x entry gates) or >1rad position resets and leaks I.
 // No additional motor-output term; all existing P/D/velocity gains unchanged.
 wire near_target=position_i<=32'sd65536 && position_i>=-32'sd65536;
 wire severe=aa>{18'd0,config_i[108:96],1'b0} ||
             ad>{13'd0,config_i[145:128],1'b0};
 wire [5:0] dwell=improved?6'd10:6'd40;
 wire can_integrate=tilt_ok && speed_ok && (!improved || near_target);
 wire leak_now=improved?(severe || !near_target):!tilt_ok;
 wire signed [31:0] x=(position_i>65536)?32'sd65536:
                              ((position_i < -65536)?-32'sd65536:position_i);
 wire [31:0] xmag=x[31] ? -x : x;
 wire [7:0] kimag=ki[7] ? -ki : ki;
 wire [25:0] mul_next=product+(multiplier[0]?multiplicand:26'd0);
 wire [RW-1:0] rem_next=(remainder<<1) | quotient[25];
 wire div_subtract=(rem_next>=DEN);
 wire signed [26:0] delta=negative ? -$signed({1'b0,quotient}) : $signed({1'b0,quotient});
 wire signed [27:0] proposed=$signed({{4{integral[23]}},integral})+$signed({delta[26],delta});
 wire signed [27:0] bound=$signed({6'd0,integral_limit_sample,16'd0});
 wire signed [27:0] candidate=(proposed>bound)?bound:((proposed < -bound)?-bound:proposed);
 wire signed [28:0] change=$signed({candidate[27],candidate})-$signed({{5{integral[23]}},integral});
 wire signed [47:0] predicted=total_sample+$signed({{19{change[28]}},change});
 wire signed [47:0] ulim=$signed({22'd0,output_limit_sample,16'd0});
 wire windup=(predicted>ulim && change>0) || (predicted < -ulim && change<0);
 wire [23:0] imag=integral[23] ? -integral : integral;
 wire [23:0] decay=(imag<64)?24'd1:(imag>>6);
 assign integral_q16_o=(enabled && ready_i)?{{8{integral[23]}},integral}:32'sd0;
 // bits0 enabled,1 last update integrated,2 limited,3 antiwindup,
 // bit4 instantaneous integration gate,5 waiting (mode1 200ms/mode2 50ms).
 assign status_o={enabled && ready_i && quiet_ticks<dwell,
     enabled && ready_i && (!can_integrate),last_windup,last_limited,
     integrated && enabled && ready_i,enabled};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;count<=0;quiet_ticks<=0;integral<=0;integrated<=0;last_limited<=0;last_windup<=0;
   multiplicand<=0;product<=0;quotient<=0;multiplier<=0;remainder<=0;negative<=0;
   total_sample<=0;output_limit_sample<=0;integral_limit_sample<=0;
  end else if(!ready_i || !enabled)begin
   state<=IDLE;quiet_ticks<=0;integral<=0;integrated<=0;last_limited<=0;last_windup<=0;
  end else case(state)
   IDLE:if(update_i)begin
    integrated<=0;last_limited<=0;last_windup<=0;
    if(leak_now)begin
     quiet_ticks<=0;
     if(integral>0)integral<=integral-$signed(decay);
     else if(integral<0)integral<=integral+$signed(decay);
    end else begin
     if(quiet_ticks<dwell)begin
      if(!improved || can_integrate)quiet_ticks<=quiet_ticks+1'b1;
      else quiet_ticks<=0;
     end
     // High speed freezes I; mode2 mild tilt violations freeze without rearming.
     if(quiet_ticks==dwell && can_integrate)begin
      multiplicand<={9'd0,xmag[16:0]};multiplier<=kimag[6:0];product<=0;
      negative<=x[31]^ki[7];count<=7;state<=MUL;
      total_sample<=total_q16_i;output_limit_sample<=limit_u_i[9:0];
      integral_limit_sample<=config_i[69:64];
     end
    end
   end
   MUL:begin
    product<=mul_next;multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
    count<=count-1'b1;
    if(count==1)begin quotient<=mul_next;remainder<=0;count<=26;state<=DIVIDE;end
   end
   DIVIDE:begin
    quotient<={quotient[24:0],div_subtract};
    remainder<=div_subtract ? rem_next-DEN : rem_next;
    count<=count-1'b1;
    if(count==1)state<=COMMIT;
   end
   COMMIT:begin
    // Inputs are captured on update_i; STOP/ready loss cancels a pending commit.
    last_windup<=windup;last_limited<=proposed>bound || proposed < -bound;
    if(!windup)begin integral<=candidate[23:0];integrated<=1;end
    state<=IDLE;
   end
   default:state<=IDLE;
  endcase
 end
endmodule

`default_nettype wire

`default_nettype none
// R14B static configuration product. 8 shift/add clocks, 27-bit datapath.
// Recomputed on config change (STOP-only APPLY), never delays a control update.
// Invalid or uncommitted configuration masks ready immediately.
module return_cap_config(
 input wire clk,rst_n,
 input wire [95:0] config_i,
 output wire [26:0] cap_q16_o,
 output wire ready_o
);
 wire [18:0] vmax=config_i[50:32];
 wire [7:0] kv=config_i[71:64];
 wire valid=config_i[63:32]>=32'd16384 && config_i[63:32]<=32'd262144 &&
            config_i[95:64]>=32'd1 && config_i[95:64]<=32'd200;
 wire [26:0] key={kv,vmax};
 reg [26:0] sampled,operand,sum,result;
 reg [7:0] multiplier;
 reg [3:0] count;
 reg busy,done;
 wire [26:0] next_sum=sum+(multiplier[0]?operand:27'd0);
 assign ready_o=valid && done && sampled==key;
 assign cap_q16_o=result;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sampled<=0;operand<=0;sum<=0;result<=0;multiplier<=0;count<=0;busy<=0;done<=0;end
  else if(!valid)begin busy<=0;done<=0;end
  else if(sampled!=key || (!busy && !done))begin
   sampled<=key;operand<={8'd0,vmax};multiplier<=kv;sum<=0;count<=8;busy<=1;done<=0;
  end else if(busy)begin
   sum<=next_sum;operand<=operand<<1;multiplier<=multiplier>>1;count<=count-1'b1;
   if(count==1)begin result<=next_sum;busy<=0;done<=1;end
  end
 end
endmodule
`default_nettype wire

`default_nettype none
// R14D: only the velocity contribution uses this path. Raw protection and
// integral gates retain their original inputs. speed_o is the current update's
// candidate, not a register delayed by an extra 5ms control period.
module balance_speed_filter(
 input wire clk,rst_n,update_i,ready_i,
 input wire [1:0] mode_i,
 input wire signed [31:0] speed_i,position_i,
 output wire signed [31:0] speed_o
);
 reg seeded;
 reg [1:0] previous_mode;
 reg signed [31:0] previous_speed,previous_position;
 reg [3:0] zero_ticks;
 wire enabled=(mode_i==1 || mode_i==2);
 wire moved=position_i!=previous_position;
 wire negative_motion=position_i<previous_position;
 wire reversal=seeded && moved && previous_speed!=0 &&
     negative_motion!=previous_speed[31];
 wire matching_speed=speed_i!=0 && speed_i[31]==negative_motion;
 wire signed [32:0] difference=$signed({speed_i[31],speed_i})-
     $signed({previous_speed[31],previous_speed});
 wire [32:0] magnitude=difference[32] ? -difference : difference;
 wire [32:0] step_mag=(mode_i==2)?(magnitude>>2):(magnitude>>1);
 wire signed [32:0] step=difference[32] ? -$signed(step_mag) : $signed(step_mag);
 wire signed [32:0] summed=$signed({previous_speed[31],previous_speed})+step;
 // Snap the final sub-LSB residual instead of leaving a permanent offset.
 wire signed [31:0] smoothed=(step_mag==0)?speed_i:summed[31:0];
 wire stopped=seeded && !moved && speed_i==0 && zero_ticks>=7;
 wire signed [31:0] candidate=(!seeded || previous_mode!=mode_i)?speed_i:
     reversal?(matching_speed?speed_i:32'sd0):stopped?32'sd0:smoothed;
 assign speed_o=(!enabled || !ready_i)?speed_i:candidate;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin seeded<=0;previous_mode<=0;previous_speed<=0;previous_position<=0;zero_ticks<=0;end
  else if(!ready_i || !enabled)begin seeded<=0;previous_mode<=mode_i;previous_speed<=0;previous_position<=position_i;zero_ticks<=0;end
  else if(update_i)begin
   seeded<=1;previous_mode<=mode_i;previous_speed<=candidate;previous_position<=position_i;
   if(moved || speed_i!=0)zero_ticks<=0;
   else if(zero_ticks<8)zero_ticks<=zero_ticks+1'b1;
  end
 end
endmodule
`default_nettype wire

// R14E: continuous two-slope position request, no extra velocity feedback.
// P(x)=Knear*x inside r; outside = sign(x)*(Knear*r+Kfar*(abs(x)-r)).
// Existing full position product is reused; only a bounded 16x8 correction.
`default_nettype none
module near_position_shape(input wire enabled_i,
 input wire signed [31:0] position_i,input wire signed [15:0] far_gain_i,
 input wire [6:0] near_gain_i,input wire [15:0] range_i,
 input wire signed [47:0] linear_i,output wire signed [47:0] shaped_o);
 wire valid=enabled_i && far_gain_i>0 && far_gain_i<=100 && near_gain_i<=far_gain_i && range_i!=0;
 wire [31:0] mag=position_i[31] ? (~position_i+32'd1) : position_i;
 wire [15:0] limited=(mag>{16'd0,range_i})?range_i:mag[15:0];
 wire [7:0] difference=far_gain_i[7:0]-{1'b0,near_gain_i};
 wire [23:0] correction=limited*difference;
 wire signed [47:0] correction_signed=$signed({24'd0,correction});
 assign shaped_o=!valid ? linear_i : position_i[31] ? linear_i+correction_signed : linear_i-correction_signed;
endmodule
`default_nettype wire

// Profile17: exact tenths gain, preserving existing Q32 U output and near Kn.
// Divide AFTER shifting to Q32: no integer-gain truncation and no double /10.
// |position|<=2^31, |gain_x10|<=1000: 43-bit product, 59-bit shifted value.
// Division on unsigned magnitude then restore sign gives odd symmetry.
`default_nettype none
module theta_position_tenths(
    input wire signed [31:0] position_i,
    input wire signed [10:0] gain_x10_i,
    input wire near_enabled_i,
    input wire [6:0] near_gain_i,
    input wire [15:0] range_i,
    output wire signed [63:0] p_q32_o
);
    wire [10:0] near_x10=near_gain_i*11'd10;
    wire shape_valid=near_enabled_i && gain_x10_i>0 && gain_x10_i<=1000 &&
        near_x10<=$unsigned(gain_x10_i) && range_i!=0;
    wire [31:0] mag=position_i[31] ? (~position_i+32'd1) : position_i;
    wire [15:0] limited=(mag>{16'd0,range_i}) ? range_i : mag[15:0];
    wire [9:0] gain_difference=$unsigned(gain_x10_i)-near_x10;
    wire [25:0] correction=limited*gain_difference;
    wire signed [42:0] linear=$signed(position_i)*$signed(gain_x10_i);
    wire signed [42:0] correction_signed=$signed({17'd0,correction});
    wire signed [42:0] shaped=!shape_valid ? linear :
        position_i[31] ? linear+correction_signed : linear-correction_signed;
    // Exact constant division using a 5-bit remainder, not a generic wide
    // divider. Divide the Q16 numerator first and retain its remainder, then
    // append floor(remainder*65536/10). No fractional gain is discarded.
    function [58:0] divide_ten_q32;
        input [42:0] value;
        reg [42:0] quotient;
        reg [4:0] rem;
        reg [15:0] fraction;
        integer bit_index;
        begin
            quotient=0;rem=0;
            for(bit_index=42;bit_index>=0;bit_index=bit_index-1)begin
                rem={rem[3:0],value[bit_index]};
                if(rem>=5'd10)begin rem=rem-5'd10;quotient[bit_index]=1'b1;end
            end
            case(rem)
                0:fraction=16'd0; 1:fraction=16'd6553;
                2:fraction=16'd13107; 3:fraction=16'd19660;
                4:fraction=16'd26214; 5:fraction=16'd32768;
                6:fraction=16'd39321; 7:fraction=16'd45875;
                8:fraction=16'd52428; 9:fraction=16'd58982;
                default:fraction=0;
            endcase
            divide_ten_q32={quotient,fraction};
        end
    endfunction
    wire [42:0] magnitude=shaped[42] ? (~shaped+43'd1) : shaped;
    wire [58:0] divided=divide_ten_q32(magnitude);
    wire signed [58:0] result=shaped[42] ? -$signed(divided) : $signed(divided);
    assign p_q32_o={{5{result[58]}},result};
endmodule
`default_nettype wire

`default_nettype none
// R14G entry-only state machine. Existing capture thresholds, two 200Hz samples.
// No output controller, gain, sensor reference or motor implementation here.
module manual_balance_entry(
 input wire clk,rst_n,update_i,enabled_i,run_i,rearm_i,ready_i,balance_window_i,
 input wire signed [31:0] alpha_i,alpha_dot_i,theta_dot_i,
 input wire signed [31:0] capture_angle_i,capture_ad_i,capture_td_i,
 output reg [2:0] phase_o,
 output reg [3:0] reason_o
);
 reg capture_seen;
 function [32:0] magnitude;
  input signed [31:0] v;
  reg signed [32:0] extended;
  begin extended={v[31],v};magnitude=v[31]?-extended:extended;end
 endfunction
 wire capture_ok=ready_i && balance_window_i &&
     capture_angle_i>=0 && capture_ad_i>=0 && capture_td_i>=0 &&
     magnitude(alpha_i)<={1'b0,capture_angle_i} &&
     magnitude(alpha_dot_i)<={1'b0,capture_ad_i} &&
     magnitude(theta_dot_i)<={1'b0,capture_td_i};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin phase_o<=0;reason_o<=0;capture_seen<=0;end
  else if(!enabled_i || !run_i || rearm_i) begin phase_o<=0;reason_o<=0;capture_seen<=0;end
  else if(update_i) begin
   case(phase_o)
    3'd0,3'd7: begin
     reason_o<=0;capture_seen<=capture_ok;
     if(!ready_i) phase_o<=0;
     else if(capture_ok && capture_seen) begin phase_o<=5;capture_seen<=0;end
     else phase_o<=7;
    end
    3'd5: begin
     if(!ready_i) begin phase_o<=6;reason_o<=1;end
     else if(!balance_window_i) begin phase_o<=6;reason_o<=5;end
    end
    3'd6: begin phase_o<=6;capture_seen<=0;end
    default: begin phase_o<=6;reason_o<=6;capture_seen<=0;end
   endcase
  end
 end
endmodule
`default_nettype wire

`default_nettype none
// Q32 U, 1U=0.1% duty. Combinational next value is consumed by control_u
// on the SAME update edge; state commits there, avoiding a one-update delay.
// R14J: bounded Q32 arm-only slew. All comparisons/arithmetic are explicitly signed.
// The 48-bit state represents +/-32768 U; the independent hard envelope is +/-1000 U.
// This does not replace POSITION_U_LIMIT, RETURN_POSITION_U_CAP or final cfg_U_LIMIT.
module arm_output_slew #(parameter integer STEP_U=10)(
 input wire clk,rst_n,update_i,ready_i,
 input wire stage_i, // actual balance stage, independent of temporary permission loss
 input wire signed [63:0] request_i,
 output wire signed [63:0] request_o
);
 localparam signed [64:0] CAP_Q32=65'sd4294967296000; // 1000 U
 localparam signed [48:0] STEP_Q32=$signed(STEP_U)*49'sd4294967296;
 reg signed [47:0] previous;
 reg entry_pending;
 reg signed [64:0] bounded_input;
 reg signed [48:0] target_w,previous_w,next_w,delta_w;
 reg signed [63:0] safe_output;
 wire signed [64:0] input_w=$signed({request_i[63],request_i});
 wire state_bad=($signed(previous)>48'sd4294967296000) ||
                ($signed(previous)< -48'sd4294967296000);
 always @* begin
  // Clamp BEFORE narrowing. Do not negate an unbounded input (INT64_MIN).
  if(input_w>CAP_Q32) bounded_input=CAP_Q32;
  else if(input_w< -CAP_Q32) bounded_input=-CAP_Q32;
  else bounded_input=input_w;
  target_w=$signed(bounded_input[48:0]);
  previous_w=$signed({previous[47],previous});
  delta_w=target_w-previous_w;
  next_w=49'sd0;
  if(rst_n && ready_i && stage_i) begin
   if(state_bad) next_w=49'sd0; // corrupt history must never reach the motor
   else if(entry_pending) next_w=target_w; // capture entry: seed current bounded request
   else if((target_w>0 && previous_w<0) || (target_w<0 && previous_w>0))
    next_w=49'sd0; // release old direction at this update; new direction starts next update
   else if((previous_w>0 && target_w<=previous_w) ||
           (previous_w<0 && target_w>=previous_w)) next_w=target_w;
   else if(delta_w>STEP_Q32) next_w=previous_w+STEP_Q32;
   else if(delta_w< -STEP_Q32) next_w=previous_w-STEP_Q32;
   else next_w=target_w;
  end
  // Independent post-arithmetic range guard, BEFORE slicing/sign extension.
  if(next_w>49'sd4294967296000 || next_w< -49'sd4294967296000)
   safe_output=64'sd0;
  else safe_output=$signed({{15{next_w[48]}},next_w});
 end
 assign request_o=safe_output;
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
 initial if(STEP_U<1 || STEP_U>1000)$fatal(1,"Invalid arm slew step");
 always @(posedge clk) if(rst_n && update_i && ready_i) begin
  if(request_o>64'sd4294967296000 || request_o< -64'sd4294967296000)
   $fatal(1,"Arm output outside independent legal envelope");
 end
 // synthesis translate_on
endmodule
`default_nettype wire

`default_nettype none
// R15L: 200Hz cumulative motion reference; Q24 rad/update internally.
// Fast-clock serial stop prediction includes acceleration and jerk transition.
// Acceleration changes are jerk-limited; the pendulum PD is never filtered.
module arm_motion_reference_r15l(
 input wire clk,rst_n,update_i,ready_i,enable_i,pause_i,rebase_i,
 output wire signed [11:0] acceleration_o,
 input wire signed [31:0] actual_i,goal_i,
 input wire [17:0] max_velocity_i,input wire [10:0] acceleration_i,
 input wire [15:0] following_band_i,
 output wire signed [31:0] position_o,velocity_o,
 output wire active_o,paused_o,calculating_o
);
 reg seeded;reg signed [39:0] position_q24;
 reg signed [18:0] velocity_q24;reg signed [11:0] acceleration_q24;
 wire signed [31:0] position_now=position_q24[39:8];
 wire signed [32:0] error=$signed({goal_i[31],goal_i})-$signed({position_now[31],position_now});
 wire [23:0] distance=error[32:23]!={10{error[23]}} ? 24'hffffff:
     (error[32] ? -error[23:0]:error[23:0]);
 wire [17:0] speed=velocity_q24[18] ? -velocity_q24:velocity_q24;
 wire signed [32:0] following=$signed({actual_i[31],actual_i})-$signed({position_now[31],position_now});
 wire [17:0] following_abs=following[32:17]!={16{following[17]}} ? 18'h3ffff:
     (following[32] ? -following[17:0]:following[17:0]);
 wire config_ok=max_velocity_i>=4194 && max_velocity_i<=167772 &&
     acceleration_i>=16 && acceleration_i<=1024 && following_band_i>=2288 && following_band_i<=34315;
 wire enabled=ready_i && enable_i && config_ok;
 assign position_o=position_now;
 assign acceleration_o=enabled && seeded ? acceleration_q24:12'sd0;
 wire rebase=enabled && seeded && rebase_i && velocity_q24==0 && acceleration_q24==0 &&
     following_abs>{2'd0,following_band_i};
 wire signed [23:0] velocity_times25=$signed(velocity_q24)*6'sd25;
 assign velocity_o={{13{velocity_times25[23]}},velocity_times25[23:5]};
 assign active_o=enabled && seeded && (position_now!=goal_i || velocity_q24!=0);
 // Follow error is actual-reference; goal error supplies travel direction.
 // An arm ahead of the reference must not stop the reference behind it.
 // Speed feedback remains responsible for excess actual speed.
 wire lagging=following!=0 && (following[32]!=error[32]);
 reg following_wait;
 wire [15:0] following_gate=following_wait ? (following_band_i>>1):following_band_i;
 wire lag_excess=lagging && following_abs>{2'd0,following_gate};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)following_wait<=0;
  else if(!enabled || !seeded || rebase)following_wait<=0;
  else if(update_i)following_wait<=lag_excess;
 end
 // Immediate entry, hysteretic release. No position/reference teleport.
 wire pause=pause_i || lag_excess;
 assign paused_o=active_o && pause;
 // Predict a jerk-limited stop in FPGA clocks between the 200Hz updates.
 // Each fast clock simulates one future motion update using only add/subtract.
 // Legal vmax/amin complete in <12000 FPGA clocks (<0.45ms at 27MHz).
 reg calc_busy,calculation_valid,brake;
 reg [17:0] brake_speed;
 reg signed [11:0] brake_acceleration;
 reg [32:0] brake_distance;
 reg [23:0] saved_distance;
 reg [10:0] saved_acceleration;
 reg signed [31:0] calc_goal;
 wire signed [11:0] projected_acceleration=velocity_q24[18] ? -acceleration_q24:acceleration_q24;
 wire signed [12:0] brake_amax=-$signed({2'd0,saved_acceleration});
 wire signed [12:0] brake_jerk=(saved_acceleration>>4)==0 ? 13'sd1:$signed({2'd0,saved_acceleration})>>>4;
 wire signed [12:0] brake_accel_delta=brake_amax-$signed({brake_acceleration[11],brake_acceleration});
 wire signed [12:0] brake_accel_next=$signed({brake_acceleration[11],brake_acceleration})+
     (brake_accel_delta < -brake_jerk ? -brake_jerk:brake_accel_delta);
 wire signed [19:0] brake_velocity_next=$signed({2'd0,brake_speed})+
     $signed({{7{brake_accel_next[12]}},brake_accel_next});
 assign calculating_o=calc_busy;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   calc_busy<=0;calculation_valid<=0;brake<=0;brake_speed<=0;brake_acceleration<=0;
   brake_distance<=0;saved_distance<=0;saved_acceleration<=0;calc_goal<=0;
  end else if(!enabled || !seeded || rebase)begin calc_busy<=0;calculation_valid<=0;brake<=0;end
  else if(update_i && !calc_busy)begin
   calc_goal<=goal_i;saved_distance<=distance;saved_acceleration<=acceleration_i;
   brake_speed<=speed;brake_acceleration<=projected_acceleration;
   brake_distance<=({15'd0,speed}<<1)+({22'd0,acceleration_i}<<1);
   calc_busy<=1;
  end else if(calc_busy)begin
   if(calc_goal!=goal_i)begin calc_busy<=0;calculation_valid<=0;end
   else if(brake_speed==0 || brake_velocity_next<=0)begin
    brake<=brake_distance>={1'b0,saved_distance,8'd0};
    calculation_valid<=1;calc_busy<=0;
   end else begin
    brake_speed<=brake_velocity_next[17:0];brake_acceleration<=brake_accel_next[11:0];
    brake_distance<=brake_distance+{15'd0,brake_velocity_next[17:0]};
   end
  end
 end
 wire bound_ready=calculation_valid && calc_goal==goal_i;
 wire going_away=velocity_q24!=0 && velocity_q24[18]!=error[32];
 wire must_brake=pause || !bound_ready || going_away || brake || speed>max_velocity_i;
 wire signed [11:0] amax=$signed({1'b0,acceleration_i});
 wire signed [11:0] desired_acceleration=must_brake ?
     (velocity_q24>0 ? -amax:velocity_q24<0 ? amax:12'sd0):
     speed==max_velocity_i ? 12'sd0:error[32] ? -amax:amax;
 wire signed [12:0] acceleration_delta=$signed({desired_acceleration[11],desired_acceleration})-
     $signed({acceleration_q24[11],acceleration_q24});
 wire signed [12:0] jerk=$signed({6'd0,(acceleration_i>>4)})==0 ? 13'sd1:
     $signed({2'd0,acceleration_i})>>>4;
 wire signed [12:0] new_acceleration=$signed({acceleration_q24[11],acceleration_q24})+
     (acceleration_delta>jerk ? jerk:acceleration_delta < -jerk ? -jerk:acceleration_delta);
 wire signed [19:0] proposed_velocity=$signed({velocity_q24[18],velocity_q24})+
     $signed({{7{new_acceleration[12]}},new_acceleration});
 wire signed [19:0] vmax=$signed({2'd0,max_velocity_i});
 wire stopped_crossing=must_brake && !going_away &&
     ((velocity_q24>0 && proposed_velocity<0)||(velocity_q24<0 && proposed_velocity>0));
 wire signed [18:0] next_velocity=stopped_crossing ? 19'sd0:
     proposed_velocity>vmax ? $signed({1'b0,max_velocity_i}):
     proposed_velocity < -vmax ? -$signed({1'b0,max_velocity_i}):proposed_velocity[18:0];
 wire signed [40:0] proposed_position=$signed({position_q24[39],position_q24})+
     $signed({{22{next_velocity[18]}},next_velocity});
 wire signed [40:0] rail=41'sh07fffffff00;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin seeded<=0;position_q24<=0;velocity_q24<=0;acceleration_q24<=0;end
  else if(!enabled)begin seeded<=0;position_q24<={actual_i,8'd0};velocity_q24<=0;acceleration_q24<=0;end
  else if(rebase)begin position_q24<={actual_i,8'd0};velocity_q24<=0;acceleration_q24<=0;end
  else if(!seeded)begin seeded<=1;position_q24<={actual_i,8'd0};velocity_q24<=0;acceleration_q24<=0;end
  else if(update_i)begin
   // Terminal blend below encoder resolution: <=0.25deg and <=0.05rad/s.
   if(distance<=286 && speed<=4194)begin
    position_q24<={goal_i,8'd0};velocity_q24<=0;acceleration_q24<=0;
   end else begin
    position_q24<=proposed_position[40:39]==2'b01 ||
        (proposed_position[40:39]==2'b00 && proposed_position[39:8]==32'h7fffffff) ? rail[39:0]:
        proposed_position[40:39]==2'b10 || proposed_position[39:8]==32'h80000000 ?
        -$signed(rail[39:0]):proposed_position[39:0];
    velocity_q24<=next_velocity;
    acceleration_q24<=stopped_crossing ? 12'sd0:new_acceleration[11:0];
   end
  end
 end
endmodule
`default_nettype wire
