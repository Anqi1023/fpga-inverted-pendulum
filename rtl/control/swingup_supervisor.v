`timescale 1ns/1ps
`default_nettype none
// D33 slower confirmation for shallow, slow horizontal crossings; D32 retained.
// D13 balance unchanged. One swing-up attempt per reset.
// Inputs: Q16 rad / rad/s. update_i must be exactly 200 Hz.
// alpha_i is the D13 balance error; physical angle restores the setpoint.
module swingup_supervisor #(
    parameter integer LIVE_CONFIG = 0,
    parameter signed [31:0] PUMP_CAP_RELEASE_STEP = 32'sd20,
    parameter signed [31:0] PUMP_CAP_FAST_RELEASE_STEP = 32'sd60,
    parameter signed [31:0] PUMP_CAP_FAST_SPEED_Q16 = 32'sd327680,
    parameter signed [31:0] PUMP_BRAKE_ON_Q16 = 32'sd393216,
    parameter signed [31:0] PUMP_BRAKE_OFF_Q16 = 32'sd262144,
    parameter signed [31:0] PUMP_BRAKE_MIN_U = 32'sd150,
    parameter integer ENABLE = 1,
    parameter signed [31:0] SETPOINT_Q16 = 32'sd2000,
    parameter integer SWING_U = 250,          // 25% request, no hidden boost
    parameter integer KICK_UPDATES = 40,     // top overrides to 80 = 400 ms
    parameter integer ACTIVE_BRAKE_ENABLE = 0, // user-tested D29 baseline: braking disabled
    // D30: one bounded assist on each approach to the physical upright.
    parameter integer TOP_BOOST_ENABLE = 1,
    parameter signed [31:0] TOP_BOOST_ANGLE_Q16 = 32'sd34315, // 30 degrees, as in latest hardware logs
    parameter signed [31:0] TOP_BOOST_REARM_Q16 = 32'sd45753, // 40 degrees
    parameter integer TOP_BOOST_UPDATES = 12, // at most 60 ms at 200 Hz
    parameter integer TOP_BOOST_U = 550, // request; capped by SWING_U, then existing limits
    parameter integer FAST_CAP_RECOVERY_ENABLE = 1, // D32; 0 restores fixed 20-U recovery
    parameter signed [31:0] TOP_BOOST_AD_MIN_Q16 = 32'sd32768, // .5 rad/s, approaching
    parameter signed [31:0] TOP_BOOST_AD_MAX_Q16 = 32'sd524288, // 8 rad/s, skip fast passes
    parameter integer ARM_SPEED_GAIN = 80,  // U per rad/s; swing-up only
    parameter signed [31:0] KICK_END_SPEED_Q16 = 32'sd262144, // 4 rad/s
    parameter integer START_ONLY = 0,       // top sets 1 for the first diagnostic
    parameter signed [31:0] DIRECTION_AD_MIN_Q16 = 32'sd32768, // .5 rad/s
    parameter integer DIRECTION_CONFIRM_UPDATES = 3, // 15 ms
    parameter integer HORIZONTAL_CONFIRM_ENABLE = 1,
    parameter signed [31:0] HORIZONTAL_COS_MAX_Q16 = 32'sd11469, // |cos|<=.175, near horizontal
    parameter signed [31:0] HORIZONTAL_AD_MAX_Q16 = 32'sd262144, // |AD|<4rad/s only
    parameter integer HORIZONTAL_CONFIRM_UPDATES = 8, // about 40 ms at 200 Hz
    parameter integer MIN_DIRECTION_UPDATES = 6, // 30 ms
    parameter integer QUIET_UPDATES = 200,   // 1 second
    parameter integer WAIT_UPDATES = 600,    // 3 seconds after quiet qualification
    parameter integer MAX_UPDATES = 2400,    // 12 seconds including kick
    parameter signed [31:0] SPEED_LIMIT_Q16 = 32'sd524288, // 8 rad/s arm
    parameter signed [31:0] TRAVEL_LIMIT_Q16 = 32'sd411775, // +/- one arm turn
    parameter signed [31:0] CAPTURE_ANGLE_Q16 = 32'sd11796, // .18 rad, balance error
    parameter signed [31:0] CAPTURE_AD_Q16 = 32'sd163840, // 2.5 rad/s
    parameter signed [31:0] CAPTURE_TD_Q16 = 32'sd400000, // ~6.10 rad/s
    // E/(m*g*l) = cos(alpha) + J_pivot/(2*m*g*l)*alpha_dot^2.
    // Nominal reference m=.140, l=.0675, J_pivot=.0008656, g=9.8.
    // The actual hardware inertia is NOT identified; validate using swing logs.
    parameter integer KINETIC_Q16 = 280,
    parameter integer ENERGY_BAND_Q16 = 655, // retain user-tested 1% band
    parameter integer SWING_SIGN = 1
)(
    input wire [1279:0] config_i,
    input wire run_i, rearm_i,
    input wire clk, rst_n, update_i, ready_i, balance_window_i,
    input wire signed [31:0] theta_i, theta_dot_i, alpha_i, alpha_dot_i,
    output wire balance_selected_o,
    output wire signed [15:0] swing_u_o,
    output reg [2:0] phase_o,
    output reg [3:0] reason_o,
    output wire signed [31:0] travel_o,
    output wire signed [31:0] base_u_o, speed_u_o,
    output wire top_boost_o
);
    // D34: runtime bus, 32 bits per documented parameter ID.
    wire signed [31:0] cfg_SWING_U = LIVE_CONFIG ? $signed(config_i[160 +: 32]) : SWING_U;
    wire signed [31:0] cfg_KICK_UPDATES = LIVE_CONFIG ? $signed(config_i[192 +: 32]) : KICK_UPDATES;
    wire signed [31:0] cfg_KICK_END_SPEED_Q16 = LIVE_CONFIG ? $signed(config_i[224 +: 32]) : KICK_END_SPEED_Q16;
    wire signed [31:0] cfg_ARM_SPEED_GAIN = LIVE_CONFIG ? $signed(config_i[256 +: 32]) : ARM_SPEED_GAIN;
    wire signed [31:0] cfg_HORIZONTAL_CONFIRM_ENABLE = LIVE_CONFIG ? $signed(config_i[288 +: 32]) : HORIZONTAL_CONFIRM_ENABLE;
    wire signed [31:0] cfg_HORIZONTAL_CONFIRM_UPDATES = LIVE_CONFIG ? $signed(config_i[320 +: 32]) : HORIZONTAL_CONFIRM_UPDATES;
    wire signed [31:0] cfg_HORIZONTAL_AD_MAX_Q16 = LIVE_CONFIG ? $signed(config_i[352 +: 32]) : HORIZONTAL_AD_MAX_Q16;
    wire signed [31:0] cfg_DIRECTION_CONFIRM_UPDATES = LIVE_CONFIG ? $signed(config_i[384 +: 32]) : DIRECTION_CONFIRM_UPDATES;
    wire signed [31:0] cfg_MIN_DIRECTION_UPDATES = LIVE_CONFIG ? $signed(config_i[416 +: 32]) : MIN_DIRECTION_UPDATES;
    wire signed [31:0] cfg_DIRECTION_AD_MIN_Q16 = LIVE_CONFIG ? $signed(config_i[448 +: 32]) : DIRECTION_AD_MIN_Q16;
    wire signed [31:0] cfg_TOP_BOOST_ENABLE = LIVE_CONFIG ? $signed(config_i[480 +: 32]) : TOP_BOOST_ENABLE;
    wire signed [31:0] cfg_TOP_BOOST_ANGLE_Q16 = LIVE_CONFIG ? $signed(config_i[512 +: 32]) : TOP_BOOST_ANGLE_Q16;
    wire signed [31:0] cfg_TOP_BOOST_UPDATES = LIVE_CONFIG ? $signed(config_i[544 +: 32]) : TOP_BOOST_UPDATES;
    wire signed [31:0] cfg_CAPTURE_ANGLE_Q16 = LIVE_CONFIG ? $signed(config_i[576 +: 32]) : CAPTURE_ANGLE_Q16;
    wire signed [31:0] cfg_CAPTURE_AD_Q16 = LIVE_CONFIG ? $signed(config_i[608 +: 32]) : CAPTURE_AD_Q16;
    wire signed [31:0] cfg_HORIZONTAL_COS_MAX_Q16 = LIVE_CONFIG ? $signed(config_i[640 +: 32]) : HORIZONTAL_COS_MAX_Q16;
    wire signed [31:0] cfg_TOP_BOOST_REARM_Q16 = LIVE_CONFIG ? $signed(config_i[672 +: 32]) : TOP_BOOST_REARM_Q16;
    wire signed [31:0] cfg_TOP_BOOST_U = LIVE_CONFIG ? $signed(config_i[704 +: 32]) : TOP_BOOST_U;
    wire signed [31:0] cfg_TOP_BOOST_AD_MIN_Q16 = LIVE_CONFIG ? $signed(config_i[736 +: 32]) : TOP_BOOST_AD_MIN_Q16;
    wire signed [31:0] cfg_TOP_BOOST_AD_MAX_Q16 = LIVE_CONFIG ? $signed(config_i[768 +: 32]) : TOP_BOOST_AD_MAX_Q16;
    wire signed [31:0] cfg_FAST_CAP_RECOVERY_ENABLE = LIVE_CONFIG ? $signed(config_i[800 +: 32]) : FAST_CAP_RECOVERY_ENABLE;
    wire signed [31:0] cfg_PUMP_CAP_RELEASE_STEP = LIVE_CONFIG ? $signed(config_i[832 +: 32]) : 20;
    wire signed [31:0] cfg_PUMP_CAP_FAST_RELEASE_STEP = LIVE_CONFIG ? $signed(config_i[864 +: 32]) : 60;
    wire signed [31:0] cfg_PUMP_CAP_FAST_SPEED_Q16 = LIVE_CONFIG ? $signed(config_i[896 +: 32]) : 32'sd327680;
    wire signed [31:0] cfg_CAPTURE_TD_Q16 = LIVE_CONFIG ? $signed(config_i[928 +: 32]) : CAPTURE_TD_Q16;
    wire signed [31:0] cfg_KINETIC_Q16 = LIVE_CONFIG ? $signed(config_i[960 +: 32]) : KINETIC_Q16;
    wire signed [31:0] cfg_ENERGY_BAND_Q16 = LIVE_CONFIG ? $signed(config_i[992 +: 32]) : ENERGY_BAND_Q16;
    wire signed [31:0] cfg_ACTIVE_BRAKE_ENABLE = LIVE_CONFIG ? $signed(config_i[1120 +: 32]) : ACTIVE_BRAKE_ENABLE;
    wire signed [31:0] cfg_PUMP_BRAKE_ON_Q16 = LIVE_CONFIG ? $signed(config_i[1152 +: 32]) : 64'sd393216;
    wire signed [31:0] cfg_PUMP_BRAKE_OFF_Q16 = LIVE_CONFIG ? $signed(config_i[1184 +: 32]) : 64'sd262144;
    wire signed [31:0] cfg_PUMP_BRAKE_MIN_U = LIVE_CONFIG ? $signed(config_i[1216 +: 32]) : 150;
    wire signed [31:0] cfg_MAX_UPDATES = LIVE_CONFIG ? $signed(config_i[1248 +: 32]) : MAX_UPDATES;
    wire signed [31:0] cfg_SETPOINT_Q16 = LIVE_CONFIG ? $signed(config_i[128 +: 32]) : SETPOINT_Q16;

    localparam WAIT_SYSTEM=3'd0, WAIT_DOWN=3'd1, COUNTDOWN=3'd2,
               KICK=3'd3, PUMP=3'd4, BALANCE=3'd5, STOPPED=3'd6;
    localparam signed [31:0] PI_Q16=32'sd205887, TWO_PI_Q16=32'sd411775;
    // Sensor theta is wrapped; accumulate small per-update deltas for the swing guard.
    function signed [31:0] wrap_q16;
        input signed [31:0] v;
        begin
            if (v > PI_Q16) wrap_q16=v-TWO_PI_Q16;
            else if (v < -PI_Q16) wrap_q16=v+TWO_PI_Q16;
            else wrap_q16=v;
        end
    endfunction
    function signed [63:0] magnitude;
        input signed [31:0] v;
        reg signed [63:0] w;
        begin w={{32{v[31]}},v}; magnitude=v[31] ? -w : w; end
    endfunction
    // Even cosine lookup at 1/16 rad intervals; quantization <=.0625 rad.
    // Full circle wrap remains in the existing sensor path.
    function signed [31:0] cosine;
        input signed [31:0] v;
        reg [31:0] mag;
        reg signed [31:0] cos_q16;
        begin
            mag=v[31] ? -v : v;
            case (mag[17:12])
                6'd0: cos_q16 = 32'sd65536;
                6'd1: cos_q16 = 32'sd65408;
                6'd2: cos_q16 = 32'sd65025;
                6'd3: cos_q16 = 32'sd64387;
                6'd4: cos_q16 = 32'sd63499;
                6'd5: cos_q16 = 32'sd62362;
                6'd6: cos_q16 = 32'sd60982;
                6'd7: cos_q16 = 32'sd59363;
                6'd8: cos_q16 = 32'sd57513;
                6'd9: cos_q16 = 32'sd55439;
                6'd10: cos_q16 = 32'sd53147;
                6'd11: cos_q16 = 32'sd50649;
                6'd12: cos_q16 = 32'sd47952;
                6'd13: cos_q16 = 32'sd45068;
                6'd14: cos_q16 = 32'sd42008;
                6'd15: cos_q16 = 32'sd38785;
                6'd16: cos_q16 = 32'sd35409;
                6'd17: cos_q16 = 32'sd31896;
                6'd18: cos_q16 = 32'sd28258;
                6'd19: cos_q16 = 32'sd24509;
                6'd20: cos_q16 = 32'sd20665;
                6'd21: cos_q16 = 32'sd16740;
                6'd22: cos_q16 = 32'sd12750;
                6'd23: cos_q16 = 32'sd8710;
                6'd24: cos_q16 = 32'sd4636;
                6'd25: cos_q16 = 32'sd544;
                6'd26: cos_q16 = -32'sd3551;
                6'd27: cos_q16 = -32'sd7631;
                6'd28: cos_q16 = -32'sd11682;
                6'd29: cos_q16 = -32'sd15687;
                6'd30: cos_q16 = -32'sd19630;
                6'd31: cos_q16 = -32'sd23497;
                6'd32: cos_q16 = -32'sd27273;
                6'd33: cos_q16 = -32'sd30941;
                6'd34: cos_q16 = -32'sd34489;
                6'd35: cos_q16 = -32'sd37903;
                6'd36: cos_q16 = -32'sd41168;
                6'd37: cos_q16 = -32'sd44273;
                6'd38: cos_q16 = -32'sd47204;
                6'd39: cos_q16 = -32'sd49951;
                6'd40: cos_q16 = -32'sd52504;
                6'd41: cos_q16 = -32'sd54851;
                6'd42: cos_q16 = -32'sd56984;
                6'd43: cos_q16 = -32'sd58895;
                6'd44: cos_q16 = -32'sd60575;
                6'd45: cos_q16 = -32'sd62019;
                6'd46: cos_q16 = -32'sd63221;
                6'd47: cos_q16 = -32'sd64176;
                6'd48: cos_q16 = -32'sd64880;
                6'd49: cos_q16 = -32'sd65331;
                6'd50: cos_q16 = -32'sd65527;
                default: cos_q16=-32'sd65536;
            endcase
            cosine=cos_q16;
        end
    endfunction
    wire signed [31:0] physical_alpha=wrap_q16(alpha_i+cfg_SETPOINT_Q16);
    wire signed [31:0] cos_alpha=cosine(physical_alpha);
    // Bound before squaring: invalid/very high speeds are handled by the guard.
    // +/-2621440 fits signed 23 bits. Avoid describing a 64x64 square.
    wire signed [22:0] ad_bounded=(alpha_dot_i>32'sd2621440) ? 23'sd2621440 :
        (alpha_dot_i < -32'sd2621440) ? -23'sd2621440 : alpha_dot_i[22:0];
    wire signed [45:0] ad_square=ad_bounded*ad_bounded;
    wire signed [63:0] kinetic_product=$signed(ad_square)*$signed({1'b0,cfg_KINETIC_Q16[9:0]});
    wire signed [63:0] kinetic=kinetic_product >>> 32;
    wire signed [63:0] energy=$signed(cos_alpha)+kinetic;
    wire signed [63:0] energy_error=64'sd65536-energy;
    wire energy_low=energy_error>cfg_ENERGY_BAND_Q16;
    wire energy_high=energy_error < -cfg_ENERGY_BAND_Q16;
    // With the measured D13 signs, dd(alpha) has +U*cos(alpha) coupling.
    // Pump when E is low and remove energy when E is high.
    wire pump_negative=alpha_dot_i[31]^cos_alpha[31]^energy_high;
    reg pump_negative_hold, pending_negative;
    reg [7:0] direction_count, direction_age;
    wire direction_trusted=(magnitude(alpha_dot_i)>=cfg_DIRECTION_AD_MIN_Q16) &&
        (magnitude(cos_alpha)>=64'sd4096); // avoid ambiguous cosine sign near horizontal
    // Preserve fast horizontal passes and the original untrusted dead zone.
    // Only a SLOW near-horizontal candidate needs additional consistent samples.
    // Leaving this region restores the normal threshold using the same pending
    // candidate count; candidate changes, zero energy and untrusted inputs still reset it.
    wire horizontal_direction_confirm=(cfg_HORIZONTAL_CONFIRM_ENABLE!=0) &&
        (magnitude(cos_alpha)<=cfg_HORIZONTAL_COS_MAX_Q16) &&
        (magnitude(alpha_dot_i)<cfg_HORIZONTAL_AD_MAX_Q16);
    wire [7:0] direction_confirm_required=horizontal_direction_confirm ?
        cfg_HORIZONTAL_CONFIRM_UPDATES : cfg_DIRECTION_CONFIRM_UPDATES;
    wire signed [15:0] pump_u=(!energy_low && !energy_high) ? 16'sd0 :
        (pump_negative_hold ? -cfg_SWING_U : cfg_SWING_U);
    // The observed baseline extracts energy before reaching upright. During
    // a slow/moderate approach, temporarily request acceleration toward upright.
    // This bypasses only energy selection and its direction latch, NOT arm-speed
    // damping, the soft envelope, the optional D29 brake or any original guard.
    reg top_boost_armed, top_boost_side_negative;
    reg [15:0] top_boost_left;
    wire top_boost_approaching=(physical_alpha>0 && alpha_dot_i<0) ||
                               (physical_alpha<0 && alpha_dot_i>0);
    wire top_boost_speed_ok=(magnitude(alpha_dot_i)>=cfg_TOP_BOOST_AD_MIN_Q16) &&
                            (magnitude(alpha_dot_i)<=cfg_TOP_BOOST_AD_MAX_Q16);
    wire top_boost_zone=magnitude(physical_alpha)<=cfg_TOP_BOOST_ANGLE_Q16;
    wire top_boost_start=(cfg_TOP_BOOST_ENABLE!=0) && top_boost_armed &&
        top_boost_left==0 && phase_o==PUMP && swing_guard &&
        top_boost_zone && top_boost_approaching && top_boost_speed_ok;
    // A crossing, retreat, excessive speed or leaving the zone ends the pulse.
    // Stay disarmed until the pendulum leaves the wider 40-degree rearm zone.
    wire top_boost_keep=top_boost_zone && top_boost_approaching &&
        top_boost_speed_ok && (physical_alpha[31]==top_boost_side_negative);
    assign top_boost_o=(cfg_TOP_BOOST_ENABLE!=0) && phase_o==PUMP && swing_guard &&
        (top_boost_start || (top_boost_left!=0 && top_boost_keep));
    wire signed [31:0] TOP_BOOST_BOUNDED_U=(cfg_TOP_BOOST_U<cfg_SWING_U) ? cfg_TOP_BOOST_U : cfg_SWING_U;
    wire signed [15:0] top_boost_u=physical_alpha[31] ?
        TOP_BOOST_BOUNDED_U : -TOP_BOOST_BOUNDED_U;
    wire signed [15:0] pump_base_u=top_boost_o ? top_boost_u : pump_u;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (LIVE_CONFIG && (!run_i || rearm_i))) begin
            top_boost_armed<=1; top_boost_side_negative<=0; top_boost_left<=0;
        end else if (update_i) begin
            if (cfg_TOP_BOOST_ENABLE==0 || phase_o!=PUMP || !swing_guard) begin
                top_boost_armed<=1; top_boost_side_negative<=0; top_boost_left<=0;
            end else begin
                if (magnitude(physical_alpha)>=cfg_TOP_BOOST_REARM_Q16)
                    top_boost_armed<=1;
                if (top_boost_start) begin
                    top_boost_armed<=0;
                    top_boost_side_negative<=physical_alpha[31];
                    top_boost_left<=cfg_TOP_BOOST_UPDATES-1;
                end else if (top_boost_left!=0 && top_boost_keep)
                    top_boost_left<=top_boost_left-1'b1;
                else top_boost_left<=0;
            end
        end
    end
    // D17: measured arm-speed damping. Do not wait for the 8 rad/s stop guard.
    // Filter only this new swing-up feedback; raw TD is retained for protection.
    reg signed [31:0] pump_speed_q16;
    wire signed [63:0] speed_difference=
        $signed({{32{theta_dot_i[31]}},theta_dot_i})-
        $signed({{32{pump_speed_q16[31]}},pump_speed_q16});
    wire signed [63:0] speed_correction_q16=
        -$signed(pump_speed_q16)*$signed({1'b0,cfg_ARM_SPEED_GAIN[7:0]});
    // Signed division truncates toward zero, preserving mirrored arithmetic.
    wire signed [63:0] speed_correction_u=speed_correction_q16 / 64'sd65536;
    wire signed [63:0] pump_total=$signed(pump_base_u)+speed_correction_u;
    wire signed [15:0] pump_bounded=(pump_total>cfg_SWING_U) ? cfg_SWING_U :
        (pump_total < -cfg_SWING_U) ? -cfg_SWING_U : pump_total;
    // Do not turn tiny damping residuals into +/-10% floor pulses at zero.
    // This 3% request deadband is swing-up only; the D13 balance path is untouched.
    // D25: taper only commands in the direction of measured arm motion.
    // Raw TD avoids adding the existing speed filter's delay to this envelope.
    // Keep the original raw >8rad/s hard guard independently below.
    localparam signed [63:0] PUMP_SOFT_BEGIN_Q16=64'sd262144; // 4rad/s
    localparam signed [63:0] PUMP_SOFT_END_Q16=64'sd458752;   // 7rad/s
    // D28: retain separate acceleration caps for the two motor directions.
    // Tighten at once. Recover 60 U only if BOTH raw and filtered speed <=5rad/s;
    // otherwise preserve the original 20 U/update. Instantaneous cap still wins.
    wire pump_fast_cap_recovery=(cfg_FAST_CAP_RECOVERY_ENABLE!=0) &&
        (magnitude(theta_dot_i)<=cfg_PUMP_CAP_FAST_SPEED_Q16) &&
        (magnitude(pump_speed_q16)<=cfg_PUMP_CAP_FAST_SPEED_Q16);
    wire [15:0] pump_cap_release_step=pump_fast_cap_recovery ?
        cfg_PUMP_CAP_FAST_RELEASE_STEP : cfg_PUMP_CAP_RELEASE_STEP;
    reg [15:0] pump_cap_positive, pump_cap_negative;
    // D31 exact unsigned reformulation for the positive taper interval:
    // floor(N/(3*65536)) == floor(floor(N/65536)/3).
    // N <= 1000*196608 < 2^28. Only a 12-bit /3 remains, rather than /196608
    // on a signed 64-bit expression. Endpoints and truncation are unchanged.
    wire [31:0] pump_raw_speed_mag=theta_dot_i[31] ? -theta_dot_i : theta_dot_i;
    wire pump_taper_interval=(pump_raw_speed_mag>32'd262144) &&
                             (pump_raw_speed_mag<32'd458752);
    wire [17:0] pump_taper_distance=pump_taper_interval ?
        (32'd458752-pump_raw_speed_mag) : 18'd0;
    wire [9:0] PUMP_TAPER_U=cfg_SWING_U;
    wire [27:0] pump_taper_product=pump_taper_distance*PUMP_TAPER_U;
    wire [11:0] pump_taper_whole=pump_taper_product[27:16];
    wire [11:0] pump_taper_quotient=pump_taper_whole/12'd3;
    wire signed [63:0] pump_instant_cap=(pump_raw_speed_mag<=32'd262144) ? cfg_SWING_U :
        (pump_raw_speed_mag>=32'd458752) ? 64'sd0 :
        $signed({52'd0,pump_taper_quotient});
    wire [15:0] pump_positive_target=(theta_dot_i>0) ? pump_instant_cap : cfg_SWING_U;
    wire [15:0] pump_negative_target=(theta_dot_i<0) ? pump_instant_cap : cfg_SWING_U;
    function [15:0] recover_cap;
        input [15:0] previous_cap, target_cap;
        input [15:0] release_step;
        reg [16:0] recovered_cap;
        begin
            recovered_cap={1'b0,previous_cap}+{1'b0,release_step};
            if (target_cap<previous_cap) recover_cap=target_cap;
            else if (recovered_cap>={1'b0,target_cap}) recover_cap=target_cap;
            else recover_cap=recovered_cap[15:0];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (LIVE_CONFIG && (!run_i || rearm_i))) begin pump_cap_positive<=cfg_SWING_U; pump_cap_negative<=cfg_SWING_U; end
        else if (update_i) begin
            if (phase_o!=PUMP || !ready_i) begin
                pump_cap_positive<=cfg_SWING_U; pump_cap_negative<=cfg_SWING_U;
            end else begin
                pump_cap_positive<=recover_cap(pump_cap_positive,pump_positive_target,pump_cap_release_step);
                pump_cap_negative<=recover_cap(pump_cap_negative,pump_negative_target,pump_cap_release_step);
            end
        end
    end
    // A zero raw sample does not establish a direction reversal: use the
    // existing filtered sign there. Nonzero raw reversal retains braking authority.
    wire signed [31:0] pump_motion_direction=(theta_dot_i!=0) ? theta_dot_i : pump_speed_q16;
    wire pump_same_direction=(pump_bounded>0 && pump_motion_direction>0) ||
                             (pump_bounded<0 && pump_motion_direction<0);
    wire [15:0] pump_saved_cap=pump_bounded[15] ? pump_cap_negative : pump_cap_positive;
    wire signed [63:0] pump_soft_cap=(pump_instant_cap<pump_saved_cap) ?
        pump_instant_cap : $signed({48'd0,pump_saved_cap});
    wire pump_soft_active=pump_same_direction && (pump_soft_cap<cfg_SWING_U);
    wire signed [15:0] pump_soft_clipped= !pump_soft_active ? pump_bounded :
        (magnitude(pump_bounded)>pump_soft_cap) ?
        (pump_bounded[15] ? -pump_soft_cap : pump_soft_cap) : pump_bounded;
    // Under the envelope, do not let an undersized command be raised by the
    // execution chain's unchanged 8/10-percent motor duty floors.
    wire pump_soft_below_floor=pump_soft_active &&
        (magnitude(pump_soft_clipped)<64'sd100);
    wire signed [15:0] pump_limited=(pump_soft_below_floor ||
        (pump_soft_clipped<30 && pump_soft_clipped > -30)) ?
        16'sd0 : pump_soft_clipped;
    // D29: remove energy through a reverse request before the hard stop.
    // Applies only to PUMP; the original raw-speed guard remains final authority.
    reg pump_brake_active, pump_brake_low_seen;
    wire pump_brake_high=(magnitude(theta_dot_i)>=cfg_PUMP_BRAKE_ON_Q16);
    wire pump_brake_low=(magnitude(theta_dot_i)<=cfg_PUMP_BRAKE_OFF_Q16);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (LIVE_CONFIG && (!run_i || rearm_i))) begin pump_brake_active<=0; pump_brake_low_seen<=0; end
        else if (update_i) begin
            if (cfg_ACTIVE_BRAKE_ENABLE==0 || phase_o!=PUMP || !ready_i) begin
                pump_brake_active<=0; pump_brake_low_seen<=0;
            end else if (!pump_brake_active) begin
                pump_brake_active<=pump_brake_high; pump_brake_low_seen<=0;
            end else if (pump_brake_low) begin
                if (pump_brake_low_seen) begin pump_brake_active<=0; pump_brake_low_seen<=0; end
                else pump_brake_low_seen<=1;
            end else pump_brake_low_seen<=0;
        end
    end
    wire pump_brake_now=(cfg_ACTIVE_BRAKE_ENABLE!=0) && phase_o==PUMP && ready_i &&
        (pump_brake_active || pump_brake_high);
    wire signed [15:0] pump_brake_min=(cfg_SWING_U<cfg_PUMP_BRAKE_MIN_U) ? cfg_SWING_U : cfg_PUMP_BRAKE_MIN_U;
    wire pump_already_braking=(pump_limited>0 && theta_dot_i<0) ||
                              (pump_limited<0 && theta_dot_i>0);
    wire signed [15:0] pump_braking_request=(theta_dot_i==0) ? 16'sd0 :
        (pump_already_braking && magnitude(pump_limited)>=pump_brake_min) ? pump_limited :
        (theta_dot_i[31] ? pump_brake_min : -pump_brake_min);
    wire signed [15:0] pump_request=pump_brake_now ? pump_braking_request : pump_limited;
    wire signed [15:0] requested=(phase_o==KICK) ? cfg_SWING_U :
        (phase_o==PUMP) ? pump_request : 16'sd0;
    // D24: use the existing arm-speed filter for phase switching only.
    // Both the current and previous KICK update must meet the threshold.
    wire kick_speed_reached=(magnitude(pump_speed_q16)>=cfg_KICK_END_SPEED_Q16);
    reg kick_speed_seen;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (LIVE_CONFIG && (!run_i || rearm_i))) kick_speed_seen<=1'b0;
        else if (update_i) begin
            if (phase_o==KICK && ready_i && START_ONLY==0)
                kick_speed_seen<=kick_speed_reached;
            else kick_speed_seen<=1'b0;
        end
    end
    wire signed [31:0] base_dbg=(phase_o==KICK) ? cfg_SWING_U :
        (phase_o==PUMP) ? $signed(pump_base_u) : 32'sd0;
    wire signed [31:0] speed_dbg=(phase_o==PUMP) ? speed_correction_u : 32'sd0;
    assign base_u_o=(SWING_SIGN<0) ? -base_dbg : base_dbg;
    assign speed_u_o=(SWING_SIGN<0) ? -speed_dbg : speed_dbg;

    reg [15:0] count, elapsed;
    reg capture_seen;
    reg signed [31:0] previous_theta, travel;
    wire signed [31:0] delta=wrap_q16(theta_i-previous_theta);
    wire signed [32:0] next_travel=$signed({travel[31],travel})+
        $signed({delta[31],delta});
    wire down_quiet=(magnitude(physical_alpha)>=64'sd183501) && // 160 degrees
        (magnitude(alpha_dot_i)<=64'sd22938) && // .35 rad/s
        (magnitude(theta_dot_i)<=64'sd85197); // 1.3 rad/s; encoder quantization
    wire capture_ok=balance_window_i &&
        (magnitude(alpha_i)<=cfg_CAPTURE_ANGLE_Q16) &&
        (magnitude(alpha_dot_i)<=cfg_CAPTURE_AD_Q16) &&
        (magnitude(theta_dot_i)<=cfg_CAPTURE_TD_Q16);
    wire speed_bad=(magnitude(theta_dot_i)>SPEED_LIMIT_Q16) ||
                   (magnitude(alpha_dot_i)>64'sd2621440); // 40 rad/s pendulum sanity
    wire travel_bad=(next_travel > TRAVEL_LIMIT_Q16) ||
                    (next_travel < -TRAVEL_LIMIT_Q16);
    wire swing_phase=(phase_o==KICK)||(phase_o==PUMP);
    wire swing_guard=ready_i && !speed_bad && !travel_bad && (elapsed<cfg_MAX_UPDATES);
    assign balance_selected_o=(ENABLE==0)||(phase_o==BALANCE);
    assign swing_u_o=(ENABLE!=0 && swing_phase && swing_guard) ?
        ((SWING_SIGN<0) ? -requested : requested) : 16'sd0;
    assign travel_o=next_travel[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (LIVE_CONFIG && (!run_i || rearm_i))) pump_speed_q16<=0;
        else if (update_i) begin
            if (!swing_phase || !ready_i) pump_speed_q16<=0;
            else pump_speed_q16<=pump_speed_q16+(speed_difference/64'sd2);
        end
    end

    // A motor-direction latch, NOT a filter on the balance controller derivative.
    // Below .5 rad/s keep the previous pump direction, initialized by the +kick.
    // A reversal needs 3 trusted samples, or 8 during a slow horizontal crossing.
    // The original minimum 30 ms dwell remains independent.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (LIVE_CONFIG && (!run_i || rearm_i))) begin
            pump_negative_hold<=0; pending_negative<=0;
            direction_count<=0; direction_age<=0;
        end else if (update_i) begin
            if (phase_o!=PUMP || !swing_guard) begin
                pump_negative_hold<=0; pending_negative<=0;
                direction_count<=0; direction_age<=0;
            end else begin
                if (direction_age<cfg_MIN_DIRECTION_UPDATES) direction_age<=direction_age+1'b1;
                if ((!energy_low && !energy_high) || !direction_trusted ||
                    pump_negative==pump_negative_hold) begin
                    direction_count<=0;
                end else if (direction_count==0 || pending_negative!=pump_negative) begin
                    pending_negative<=pump_negative;
                    direction_count<=1;
                end else if (direction_count>=direction_confirm_required-1'b1 &&
                             direction_age>=cfg_MIN_DIRECTION_UPDATES-1) begin
                    pump_negative_hold<=pump_negative;
                    direction_count<=0; direction_age<=0;
                end else if (direction_count<direction_confirm_required)
                    direction_count<=direction_count+1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (LIVE_CONFIG && (!run_i || rearm_i))) begin
            phase_o<=WAIT_SYSTEM; reason_o<=0; count<=0; elapsed<=0;
            capture_seen<=0; previous_theta<=0; travel<=0;
        end else if (update_i) begin
            previous_theta<=theta_i;
            if (ENABLE==0) begin
                phase_o<=BALANCE; reason_o<=0; count<=0; elapsed<=0;
                capture_seen<=0; travel<=0;
            end else begin
                case (phase_o)
                    WAIT_SYSTEM: begin
                        count<=0; elapsed<=0; travel<=0; capture_seen<=0;
                        if (ready_i) phase_o<=WAIT_DOWN;
                    end
                    WAIT_DOWN: begin
                        travel<=0; elapsed<=0; capture_seen<=0;
                        if (!ready_i) begin phase_o<=WAIT_SYSTEM; count<=0; end
                        else if (!down_quiet) count<=0;
                        else if (count>=QUIET_UPDATES-1) begin phase_o<=COUNTDOWN; count<=0; end
                        else count<=count+1'b1;
                    end
                    COUNTDOWN: begin
                        travel<=0;
                        if (!ready_i) begin phase_o<=WAIT_SYSTEM; count<=0; end
                        else if (!down_quiet) begin phase_o<=WAIT_DOWN; count<=0; end
                        else if (count>=WAIT_UPDATES-1) begin phase_o<=KICK; count<=0; elapsed<=0; end
                        else count<=count+1'b1;
                    end
                    KICK, PUMP: begin
                        travel<=next_travel[31:0]; elapsed<=elapsed+1'b1;
                        capture_seen<=capture_ok;
                        if (!ready_i) begin phase_o<=STOPPED; reason_o<=1; end
                        else if (speed_bad) begin phase_o<=STOPPED; reason_o<=2; end
                        else if (travel_bad) begin phase_o<=STOPPED; reason_o<=3; end
                        else if (elapsed>=cfg_MAX_UPDATES-1) begin phase_o<=STOPPED; reason_o<=4; end
                        else if (START_ONLY==0 && capture_ok && capture_seen) begin phase_o<=BALANCE; count<=0; end
                        else if (phase_o==KICK) begin
                            if (count>=cfg_KICK_UPDATES-1 || (START_ONLY==0 && kick_speed_reached && kick_speed_seen)) begin
                                count<=0;
                                if (START_ONLY!=0) begin phase_o<=STOPPED; reason_o<=7; end
                                else phase_o<=PUMP;
                            end
                            else count<=count+1'b1;
                        end
                    end
                    BALANCE: begin
                        if (!ready_i) begin phase_o<=STOPPED; reason_o<=1; end
                        else if (!balance_window_i) begin phase_o<=STOPPED; reason_o<=5; end
                    end
                    STOPPED: begin
                        // No automatic restart or post-fall swing-up. Reset to retry.
                        phase_o<=STOPPED;
                    end
                    default: begin phase_o<=STOPPED; reason_o<=6; end
                endcase
            end
        end
    end
    // synthesis translate_off
    initial begin
        if (HORIZONTAL_CONFIRM_ENABLE!=0 &&
            (HORIZONTAL_COS_MAX_Q16<4096 || HORIZONTAL_COS_MAX_Q16>65536 ||
             HORIZONTAL_AD_MAX_Q16<DIRECTION_AD_MIN_Q16 ||
             DIRECTION_CONFIRM_UPDATES<1 || DIRECTION_CONFIRM_UPDATES>255 ||
             HORIZONTAL_CONFIRM_UPDATES<DIRECTION_CONFIRM_UPDATES || HORIZONTAL_CONFIRM_UPDATES>255))
            $fatal(1,"Invalid D33 horizontal confirmation parameters");
        if (SWING_U<0 || SWING_U>1000)
            $fatal(1,"SWING_U must fit the existing 0..1000 motor protocol");
        if (TOP_BOOST_ENABLE!=0 && (TOP_BOOST_ANGLE_Q16<=0 ||
            TOP_BOOST_ANGLE_Q16>=102943 || TOP_BOOST_REARM_Q16<=TOP_BOOST_ANGLE_Q16 ||
            TOP_BOOST_REARM_Q16>PI_Q16 || TOP_BOOST_UPDATES<1 || TOP_BOOST_UPDATES>65535 ||
            TOP_BOOST_U<100 || TOP_BOOST_U>1000 || TOP_BOOST_AD_MIN_Q16<=0 ||
            TOP_BOOST_AD_MAX_Q16<TOP_BOOST_AD_MIN_Q16))
            $fatal(1,"Invalid D30 top-assist parameters");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
