// R9: optional alternate balance speed; swing/capture still use raw theta_dot.
// Angles and speeds are signed Q16 radians / radians per second; U is 0.1% duty.
// Position reference is relative to the measured arm position on balance capture.
`timescale 1ns/1ps
`default_nettype none
module cruise_original_controller #(
    parameter integer CRUISE_CONTROL = 0,
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
    input wire [223:0] cruise_config_i,
    input wire [95:0] near_config_i,
    input wire [1:0] speed_filter_mode_i,
    input wire signed [31:0] balance_speed_i,
    input wire balance_speed_valid_i, balance_speed_select_i,
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
    input wire signed [18:0] position_target_i,
    output wire position_ready_o,
    output wire signed [31:0] position_actual_o,
    output wire signed [18:0] position_target_o,
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
    wire signed [31:0] cfg_K_ALPHA = LIVE_CONFIG ? $signed(config_i[0 +: 32]) : K_ALPHA;
    wire signed [31:0] cfg_K_ALPHA_DOT = LIVE_CONFIG ? $signed(config_i[32 +: 32]) : K_ALPHA_DOT;
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
    reg signed [18:0] position_target, position_reference;
    reg [5:0] position_quiet;
    wire position_stable = ($signed(alpha_s) < 32'sd5719) &&
        ($signed(alpha_s) > -32'sd5719) &&
        ($signed(alpha_dot_s) < 32'sd65536) &&
        ($signed(alpha_dot_s) > -32'sd65536);
    assign position_ready_o = POSITION_CONTROL && position_active &&
        system_ready && balance_selected && arm_ok && position_stable && position_quiet==40 &&
        (!settle_mode || settle_stage==3);
    wire settle_mode = SETTLE_CONTROL && settle_config_i[0];
    wire [1:0] settle_stage;
    wire [8:0] settle_weight;
    wire signed [15:0] effective_kv;
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
    wire signed [18:0] reference_step = !position_ready_o ? 19'sd0 :
        (position_target-position_reference > 19'sd32) ? 19'sd32 :
        (position_target-position_reference < -19'sd32) ? -19'sd32 :
        position_target-position_reference;
    wire signed [18:0] reference_next = position_reference + reference_step;
    wire signed [32:0] tracking_error =
        $signed({position_next[31],position_next}) -
        $signed({{14{position_reference[18]}},position_reference});
    wire signed [31:0] tracking_limited =
        tracking_error > 33'sd2147483647 ? 32'sh7fffffff :
        tracking_error < -33'sd2147483648 ? 32'sh80000000 : tracking_error[31:0];
    wire signed [31:0] legacy_position_feedback = !position_active ? 32'sd0 :
        settle_locked ? subtract_saturated(position_next,lock_origin) :
        POSITION_CONTROL ? tracking_limited : position_next;

    wire cruise_selected,cruise_correction,cruise_recovering;
    wire signed [31:0] cruise_vref,cruise_error,cruise_position;
    wire [2:0] cruise_state;
    wire cruise_ready=system_ready && balance_selected && arm_ok && position_active &&
        pid_permit_i && !rearm_i && TEST_MODE==0;
    // Selected estimator must be valid. Raw speed remains available when it is
    // explicitly selected; an invalid hybrid estimate never qualifies as zero.
    wire cruise_speed_known=!balance_speed_select_i || balance_speed_valid_i;
    generate if(CRUISE_CONTROL)begin: cruise_enabled
        cruise_recovery supervisor(.clk(clk),.rst_n(rst_n),.update_i(update_now),
            .ready_i(cruise_ready),.speed_known_i(cruise_speed_known),.config_i(cruise_config_i),
            .position_i(position_next),.alpha_i(alpha_s),.alpha_dot_i(alpha_dot_s),
            .selected_o(cruise_selected),.correction_o(cruise_correction),.recovering_o(cruise_recovering),
            .velocity_ref_o(cruise_vref),.position_error_o(cruise_error),
            .position_ref_o(cruise_position),.state_o(cruise_state));
    end else begin: cruise_disabled
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
                $signed(position_target_i)<=19'sd102944 &&
                $signed(position_target_i)>=-19'sd102944)
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

    wire signed [47:0] p_alpha_q16 = $signed(alpha_s) * $signed(cfg_K_ALPHA[15:0]);
    wire signed [63:0] p_alpha = $signed({{16{p_alpha_q16[47]}},p_alpha_q16}) <<< 16;
    wire signed [47:0] p_alpha_dot_q16 = $signed(alpha_dot_s) * $signed(cfg_K_ALPHA_DOT[15:0]);
    wire signed [63:0] p_alpha_dot = $signed({{16{p_alpha_dot_q16[47]}},p_alpha_dot_q16}) <<< 16;
    wire signed [47:0] p_theta_linear_q16 = $signed(position_feedback) * $signed(cfg_K_THETA[15:0]);
    wire signed [47:0] p_theta_raw_q16;
    generate if(NEAR_CONTROL)begin: near_control_enabled
        cruise_original_near_position_shape shape_i(.enabled_i(near_config_i[0] && return_mode),
            .position_i(position_feedback),.far_gain_i(cfg_K_THETA[15:0]),
            .near_gain_i(near_config_i[70:64]),.range_i(near_config_i[47:32]),
            .linear_i(p_theta_linear_q16),.shaped_o(p_theta_raw_q16));
    end else begin: near_control_disabled
        assign p_theta_raw_q16=p_theta_linear_q16;
    end endgenerate
    wire signed [63:0] p_theta_raw = $signed({{16{p_theta_raw_q16[47]}},p_theta_raw_q16}) <<< 16;
    // R14B: cap the existing position term, keep its near-target gain unchanged.
    // cap_U = min(original POSITION_U_LIMIT, return_Kv * Vmax).
    // The speed term uses the selected Kv ONCE; no extra braking term.
    wire [26:0] return_cap_q16;
    wire return_cap_ready;
    wire return_mode=RETURN_CONTROL && return_config_i[0] && return_cap_ready && balance_selected;
    generate if(RETURN_CONTROL)begin: return_enabled
        cruise_original_return_cap_config cap_i(.clk(clk),.rst_n(rst_n),.config_i(return_config_i),
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
        balance_speed_select_i && balance_speed_valid_i;
    wire signed [31:0] balance_speed = alternate_speed ? balance_speed_i : theta_dot;
    wire [1:0] speed_filter_mode=SPEED_FILTER_CONTROL ? speed_filter_mode_i : 2'd0;
    wire signed [31:0] feedback_speed;
    generate if(SPEED_FILTER_CONTROL)begin: speed_filter_enabled
        cruise_original_balance_speed_filter filter_i(.clk(clk),.rst_n(rst_n),.update_i(update_now),
            .ready_i(position_i_ready),.mode_i(speed_filter_mode),
            .speed_i(balance_speed),.position_i(position_next),.speed_o(feedback_speed));
    end else begin: speed_filter_disabled
        assign feedback_speed=balance_speed;
    end endgenerate
    wire signed [31:0] velocity_feedback=cruise_selected && position_active ?
        (cruise_speed_known ? subtract_saturated(balance_speed,cruise_vref):32'sd0):feedback_speed;
    wire signed [47:0] p_theta_dot_q16 = $signed(velocity_feedback) * $signed(effective_kv);
    wire signed [63:0] p_theta_dot = $signed({{16{p_theta_dot_q16[47]}},p_theta_dot_q16}) <<< 16;

    generate if(SETTLE_CONTROL) begin: settle_enabled
        cruise_original_settle_lock_coordinator coordinator(
            .clk(clk),.rst_n(rst_n),.update_i(update_now),
            .enabled_i(settle_mode),
            .ready_i(system_ready && balance_selected && arm_ok && position_active && TEST_MODE==0 && !rearm_i),
            .config_i(settle_config_i),.base_kv_i(cfg_K_THETA_DOT[15:0]),
            .position_i(position_next),.speed_i(balance_speed),.raw_speed_i(theta_dot),
            .alpha_i(alpha_s),.alpha_dot_i(alpha_dot_s),
            .stage_o(settle_stage),.position_weight_o(settle_weight),
            .effective_kv_o(effective_kv),.lock_origin_o(lock_origin));
    end else begin: settle_disabled
        assign settle_stage=0;assign settle_weight=256;
        assign effective_kv=return_mode ? $signed({8'd0,return_config_i[71:64]}) : cfg_K_THETA_DOT[15:0];assign lock_origin=0;
    end endgenerate

    wire hold_mode=HOLD_CONTROL && motion_config_i[32];
    wire hold_ready=system_ready && balance_selected && arm_ok && position_active &&
                    TEST_MODE==0 && !rearm_i && pid_permit_i;
    wire signed [31:0] arm_vref,arm_aref;
    wire [8:0] hold_blend;
    wire hold_limited;
    generate if(HOLD_CONTROL)begin: hold_enabled
        cruise_original_arm_reference_coordinator #(.PERIOD_MS(UPDATE_DIV)) hold_i(
            .clk(clk),.rst_n(rst_n),.update_i(update_now),.ready_i(hold_ready),
            .enabled_i(hold_mode),.config_i(motion_config_i[223:32]),
            .position_i(position_feedback),.speed_i(balance_speed),
            .velocity_ref_o(arm_vref),.angle_ref_o(arm_aref),
            .blend_o(hold_blend),.limited_o(hold_limited));
    end else begin: hold_disabled
        assign arm_vref=0;assign arm_aref=0;assign hold_blend=0;assign hold_limited=0;
    end endgenerate
    wire signed [47:0] reference_p_q16=$signed(arm_aref)*$signed(cfg_K_ALPHA[15:0]);
    wire signed [63:0] reference_p=$signed({{16{reference_p_q16[47]}},reference_p_q16}) <<< 16;
    wire signed [63:0] original_arm=p_theta+p_theta_dot+trim_applied;
    wire signed [63:0] blend_delta=(reference_p-original_arm)*$signed({1'b0,hold_blend});
    wire signed [63:0] cruise_limit=$signed({1'b0,cruise_config_i[223:192]})*64'sd4294967296;
    wire signed [63:0] cruise_bounded=original_arm>cruise_limit ? cruise_limit:
        original_arm< -cruise_limit ? -cruise_limit:original_arm;
    wire signed [63:0] catch_request=-p_alpha-p_alpha_dot-integral_term;
    wire cruise_opposes_catch=cruise_recovering && cruise_bounded!=0 &&
        (catch_request==0 || cruise_bounded[63]!=catch_request[63]);
    wire signed [63:0] arm_request=cruise_selected && position_active ?
        (!cruise_ready || cruise_opposes_catch ? 64'sd0:cruise_bounded):
        hold_mode ? original_arm+(blend_delta >>> 8) : original_arm;
    wire signed [31:0] pid_error=hold_mode ? subtract_saturated(alpha_s,arm_aref) : alpha_s;
    wire signed [31:0] integral_q16;
    wire pid_ready=pid_permit_i && system_ready && balance_selected && arm_ok && TEST_MODE==0 && !rearm_i && (!hold_mode || hold_blend==256);
    wire signed [63:0] integral_term=$signed({{32{integral_q16[31]}},integral_q16}) <<< 16;
    generate if(PID_CONTROL) begin: pid_enabled
        cruise_original_balance_pid_integrator #(.PERIOD_MS(UPDATE_DIV)) pid_i(
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
        position_active && TEST_MODE==0 && !rearm_i && !cruise_selected;
    wire signed [63:0] position_integral_term=
        $signed({{32{position_integral_q16[31]}},position_integral_q16}) <<< 16;
    generate if(POSITION_I_CONTROL)begin: position_i_enabled
        cruise_original_arm_position_integrator #(.PERIOD_MS(UPDATE_DIV)) position_i_inst(
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
        - p_alpha
        - p_alpha_dot
        - integral_term
        + position_integral_term;

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
    swingup_supervisor #(
        .LIVE_CONFIG(LIVE_CONFIG),
        .ENABLE(AUTO_SWINGUP), .SETPOINT_Q16(SWING_SETPOINT_Q16), .SWING_U(SWING_U),
        .KICK_UPDATES(SWING_KICK_UPDATES), .START_ONLY(SWING_START_ONLY),
        .ARM_SPEED_GAIN(SWING_ARM_SPEED_GAIN), .KICK_END_SPEED_Q16(SWING_KICK_END_SPEED_Q16)
    ) u_swingup (
        .config_i(config_i), .run_i(run_i), .rearm_i(rearm_i),
        .clk(clk), .rst_n(rst_n), .update_i(update_now),
        .ready_i(system_ready), .balance_window_i(arm_ok),
        .theta_i(theta_w), .theta_dot_i(theta_dot),
        .alpha_i(alpha), .alpha_dot_i(alpha_dot),
        .balance_selected_o(balance_selected), .swing_u_o(swing_u),
        .phase_o(swing_phase), .reason_o(swing_reason), .travel_o(swing_travel),
        .base_u_o(swing_base_dbg), .speed_u_o(swing_speed_dbg),
        .top_boost_o(swing_top_boost)
    );

    always @(posedge clk) begin
        if (!rst_n || (LIVE_CONFIG && rearm_i)) begin
            speed_trim_small <= 0;
            position_active <= 0;
            theta_previous <= 0;
            position_error <= 0;
            div_cnt       <= {DIV_W{1'b0}};
            control_u     <= 16'sd0;
            control_valid <= 1'b0;
            motor_enable  <= 1'b0;
            dbg_theta <= 0; dbg_theta_dot <= 0;
            dbg_settle_status<=0;dbg_capture_travel<=0;dbg_lock_origin<=0;dbg_hold_position<=0;
            dbg_raw_speed<=0; dbg_est_speed<=0; dbg_speed_source<=0; dbg_est_valid<=0;
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
                        HOLD_CONTROL && motion_config_i[0] && balance_selected};
                    if(POSITION_I_CONTROL)begin
                        // Marker bit31: VR = position I Q16 U; R14B AR = effective position cap Q16 U.
                        // bit7 I mode2, bit8 return coordination, bit9 negative motor sign.
                        dbg_vref <= (SIGN_MOTOR<0) ? -position_integral_q16 : position_integral_q16;
                        dbg_aref <= return_mode ? $signed(selected_position_limit >>> 16) : 32'sd0;
                        dbg_hold_control <= {1'b1,19'd0,speed_filter_mode,(SIGN_MOTOR<0),return_mode,(position_i_config_i[1:0]==2),position_i_status,
                            motion_config_i[0] && balance_selected};
                    end
                    if(CRUISE_CONTROL && cruise_selected)begin
                        // Marker bits31/30. VR effective speed; AR requested speed;
                        // XP moving position reference; state in HC[6:4].
                        dbg_vref<=cruise_vref;dbg_aref<=cruise_config_i[63:32];
                        dbg_hold_control<=32'hc0000000 | {25'd0,cruise_state,cruise_speed_known,
                            cruise_recovering,cruise_correction,cruise_selected};
                    end
                    dbg_settle_status<={4'd0,settle_mode,effective_kv,settle_weight,settle_stage};
                    dbg_capture_travel<=position_active ? position_next : 32'sd0;
                    dbg_lock_origin<=lock_origin;
                    dbg_hold_position<=cruise_selected ? cruise_position:position_active ? (settle_locked ? subtract_saturated(position_next,lock_origin) : position_next) : 32'sd0;
                    dbg_theta <= balance_selected ? position_feedback : swing_travel;
                    dbg_theta_dot <= cruise_selected ? balance_speed:feedback_speed;
                    dbg_raw_speed <= theta_dot;
                    dbg_est_speed <= BALANCE_SPEED_INPUT ? balance_speed_i : theta_dot;
                    dbg_speed_source <= alternate_speed;
                    dbg_est_valid <= BALANCE_SPEED_INPUT && balance_speed_valid_i;
                    dbg_alpha <= alpha_s;
                    dbg_alpha_dot <= alpha_dot_s;
                    dbg_pi <= ((SIGN_MOTOR < 0) ? integral_term : -integral_term) >>> 32;
                    dbg_pa <= ((SIGN_MOTOR < 0) ? p_alpha : -p_alpha) >>> 32;
                    dbg_pd <= ((SIGN_MOTOR < 0) ? p_alpha_dot : -p_alpha_dot) >>> 32;
                    dbg_pt <= ((SIGN_MOTOR < 0) ? -p_theta : p_theta) >>> 32;
                    dbg_pv <= ((SIGN_MOTOR < 0) ? -(p_theta_dot+trim_applied) : (p_theta_dot+trim_applied)) >>> 32;
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
module cruise_original_settle_lock_coordinator(
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
module cruise_original_balance_pid_integrator #(parameter integer PERIOD_MS=5)(
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
    wire signed [25:0] product=$signed(alpha_i[15:0])*$signed({1'b0,config_i[8:0]});
    wire signed [41:0] timed_product=$signed(product)*PERIOD_MS;
    wire signed [31:0] delta=timed_product / 42'sd1000;
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
module cruise_original_arm_reference_coordinator #(parameter integer PERIOD_MS=5)(
 input wire clk,rst_n,update_i,ready_i,enabled_i,
 input wire [191:0] config_i,
 input wire signed [31:0] position_i,speed_i,
 output wire signed [31:0] velocity_ref_o,angle_ref_o,
 output wire [8:0] blend_o,
 output wire limited_o
);
 // R13A: the 200Hz interface is unchanged. Snapshot a sample, then share ONE
 // shift/add datapath for Kx*x and Kv*(v-vref). Complete in 37 clk edges.
 // The cruise_original_controller consumes the completed reference on the NEXT control update,
 // exactly as in R13; never move the cruise_original_controller/PWM/watchdog update schedule.
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
module cruise_original_arm_position_integrator #(parameter integer PERIOD_MS=5)(
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
module cruise_original_return_cap_config(
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
module cruise_original_balance_speed_filter(
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
module cruise_original_near_position_shape(input wire enabled_i,
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
