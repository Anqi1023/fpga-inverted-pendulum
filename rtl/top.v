
`timescale 1ns/1ps
`default_nettype none

// R15F: read-only event trace; serial ID90 selects the velocity path after STOP + APPLY.
// All PWM, position-reference and protection settings stay intact.
module top #(parameter integer R14M_ENABLE=1, BRAKE_DISTANCE_GAIN_Q8=384, BRAKE_STEP_U=20) (
    input  wire clk,
    input wire uart_rx_i,
    input  wire encoder_a,
    input  wire encoder_b,
    inout  wire ads_scl,
    inout  wire ads_sda,
    output wire uart_tx_o,
    output wire rpwm,
    output wire lpwm,
    output wire en
);

    // =====================================================
    // 0. 系统参数
    //
    // PWM_HZ * 100 必须整除 CLK_HZ（10 kHz -> 2700，27*100 = 2700 刚好）
    // STOP_CYCLES 只是换向死区，1 ms 足够；原来是 1 s，会让电机
    //   每次命令归零后整整锁死 1 秒，平衡环根本没法工作。
    // =====================================================
    localparam integer CLK_HZ      = 27000000;
    localparam integer PWM_HZ      = 10000;
    // 2026-10-18: 1000 us -> 100 us。
    // 控制周期提到 5 ms 以后，1 ms 的换向消隐占了周期的 20%，
    // 每次 u 过零都要丢掉 1 ms 的出力，摆杆在这段时间是自由下落的。
    // BTN7960 是集成半桥，内部死区在 1 us 量级，50~100 us 足够安全。
    localparam integer STOP_CYCLES = 2700;    // 100 us
    localparam integer AUTO_CLEAR  = 1;       // 1 = 允许每 500 ms 自动重试清故障

    // =====================================================
    // 0b. D14 preserves the UPLOADED D13 offset (+2000 Q16 rad).
    // Error = measured upright-relative angle - setpoint; +S adds +KA*S to U.
    // Keep the rod upright during startup calibration, then lower and release it.
    // AUTO_SWINGUP waits for hanging stillness before one bounded attempt.
    // =====================================================
    localparam signed [31:0] ALPHA_SETPOINT_Q16 = -32'sd500;

    // 软启动时间：偏置从 0 线性加到 ALPHA_SETPOINT_Q16 用多少毫秒。
    // 一上电就给满值等于对静止的臂来一次阶跃冲击，摆杆容易被甩开。
    // 300 ms 对应减速箱电机从静止到稳定出力的时间尺度；0 = 关闭。
    localparam integer SOFTRAMP_MS = 300;

    // =====================================================
    // 1. Power-on reset
    // =====================================================

    reg [7:0] por_count = 8'd0;
    reg rst_n = 1'b0;

    always @(posedge clk) begin
        if (por_count != 8'hFF) begin
            por_count <= por_count + 1'b1;
            rst_n <= 1'b0;
        end
        else begin
            rst_n <= 1'b1;
        end
    end

    // D34 defaults to DISARMED. UART STOP gates the existing execution chain.
    wire [127:0] arm_loop_config;
    wire [63:0] brake_config;
    wire [31:0] brake_distance_debug;
    wire signed [32:0] velocity_error_debug;
    wire [3:0] brake_flags_debug;
    wire [7:0] brake_status_debug;
    wire signed [31:0] loop_reference_debug;
    wire [3:0] loop_state_debug;
    wire [1279:0] live_config;
    wire [63:0] speed_config;
    wire [95:0] boost_config;
    wire [255:0] settle_config;
    wire [95:0] pid_config;
    wire [223:0] motion_config;
    wire [191:0] position_i_config;
    wire [95:0] near_config,coordinate_config;
    wire signed [10:0] k_theta_x10;
    wire [6:0] return_position_cap;
    wire manual_start;
    wire [95:0] return_config;
    wire [1:0] speed_filter_mode;
    wire signed [31:0] arm_vref_debug,arm_aref_debug;
    wire [31:0] hold_control_debug;
    wire signed [31:0] pid_integral_u;
    wire [31:0] settle_status;
    wire signed [31:0] capture_travel,lock_origin,hold_position;
    wire signed [31:0] encoder_count_debug;
    wire [3:0] encoder_ab_trace;wire encoder_illegal_trace;
    wire estimator_reverse_trace;wire [16:0] estimator_period_trace;
    wire [4:0] estimator_flags_trace;
    wire trace_arm,trace_reference_mode,trace_freeze;wire [6:0] trace_index;
    wire [415:0] trace_data;
    wire signed [31:0] trace_position,trace_alpha,trace_ad,trace_sampled_position;
    wire signed [13:0] trace_reference,trace_target;
    wire [3:0] trace_reference_reason,trace_sampled_reason;
    wire [12:0] trace_coordinate_flags,trace_sampled_flags;
    wire [31:0] trace_status,trace_reference_cycle,trace_version,trace_arm_ms,trace_reference_ms;
    wire signed [31:0] estimated_arm_speed;
    wire estimated_speed_valid;
    wire [36:0] estimated_speed_status;
    wire [63:0] sampled_speed_status;
    wire signed [31:0] sampled_original_arm,sampled_arm_request;
    wire signed [31:0] sampled_raw_speed, sampled_est_speed;
    wire sampled_speed_source, sampled_est_valid;
    wire user_run, user_rearm;
    wire [31:0] config_version;
    wire reply_pending, reply_start;
    wire [7:0] reply_data;
    wire sensor_frame_busy;
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    wire cmd_valid;
    wire [3:0] cmd_op;
    wire [7:0] cmd_id;
    wire signed [31:0] cmd_value;
    reg [9:0] config_settle_ms;
    wire config_applied;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) config_settle_ms<=0;
        else if (config_applied) config_settle_ms<=100;
        else if (control_tick && config_settle_ms!=0) config_settle_ms<=config_settle_ms-1'b1;

    // =====================================================
    // 2. 1 kHz control tick
    // =====================================================

    reg [14:0] tick_counter;
    reg control_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_counter <= 0;
            control_tick <= 0;
        end
        else begin
            control_tick <= 0;

            if (tick_counter >= 15'd26999) begin
                tick_counter <= 0;
                control_tick <= 1'b1;
            end
            else begin
                tick_counter <= tick_counter + 1'b1;
            end
        end
    end

    // =====================================================
    // 3. System state machine
    // =====================================================

    wire [2:0] system_state;

    control_fsm u_control_fsm (
        .clk(clk),
        .rst_n(rst_n),
        .tick_i(control_tick),
        .state_o(system_state)
    );

    // =====================================================
    // 4. Sensors
    // =====================================================

    wire signed [31:0] theta;
    wire signed [31:0] theta_dot;
    wire signed [31:0] alpha;
    wire signed [31:0] alpha_base;
    wire signed [31:0] live_offset=$signed(live_config[128 +: 32]);
    wire signed [32:0] alpha_shift=$signed({alpha_base[31],alpha_base}) + ALPHA_SETPOINT_Q16 - $signed({live_offset[31],live_offset});
    assign alpha=(alpha_shift>33'sd205887) ? alpha_shift-33'sd411775 :
                 (alpha_shift < -33'sd205887) ? alpha_shift+33'sd411775 : alpha_shift[31:0];
    wire signed [31:0] alpha_dot;
    wire sensor_valid;

    wire encoder_ready_debug;
    wire math_valid_debug;
    wire fault_latched_debug;
    wire adc_fault_debug;
    wire encoder_fault_latched_debug;
    wire adc_fault_latched_debug;
    wire [7:0] adc_err_count;

    // 2026-10-21: 零点标定诊断
    wire signed [31:0] dbg_alpha_zero;
    wire signed [31:0] dbg_alpha_wrap;
    wire               dbg_zero_done;
    wire signed [15:0] dbg_adc_raw;
    wire signed [15:0] dbg_adc_sample;
    wire [7:0] dbg_adc_reseeds;

    sensor_top #(
        .ALPHA_SETPOINT_Q16(ALPHA_SETPOINT_Q16),
        .SOFTRAMP_MS(SOFTRAMP_MS)
    ) u_sensor_top (
        .clk(clk),
        .rst_n(rst_n),

        .encoder_a(encoder_a),
        .encoder_b(encoder_b),

        .ads_scl(ads_scl),
        .ads_sda(ads_sda),

        .theta(theta),
        .dbg_encoder_count(encoder_count_debug),
        .dbg_encoder_ab(encoder_ab_trace),.dbg_encoder_illegal(encoder_illegal_trace),
        .theta_dot(theta_dot),
        .alpha(alpha_base),
        .alpha_dot(alpha_dot),

        .sensor_valid(sensor_valid),

        .encoder_ready_debug(encoder_ready_debug),
        .math_valid_debug(math_valid_debug),
        .fault_latched_debug(fault_latched_debug),
        .adc_fault_debug(adc_fault_debug),
        .encoder_fault_latched_debug(encoder_fault_latched_debug),
        .adc_fault_latched_debug(adc_fault_latched_debug),

        .adc_err_count_out(adc_err_count),

        .dbg_alpha_zero(dbg_alpha_zero),
        .dbg_alpha_wrap(dbg_alpha_wrap),
        .dbg_zero_done(dbg_zero_done),
        .dbg_adc_raw(dbg_adc_raw),
        .dbg_adc_sample(dbg_adc_sample), .dbg_adc_reseeds(dbg_adc_reseeds)
    );

    // =====================================================
    // 5. 倒立摆状态反馈控制器
    //
    // 在线增益以 serial_parameter_bank 默认值与网页回读为准。
    // 整定顺序见 controller.v 文件头。
    // =====================================================

    wire signed [15:0] control_u;
    wire control_valid;
    wire motor_enable;

    wire signed [31:0] d02_theta, d02_td, d02_alpha, d02_ad;
    wire signed [31:0] d02_pa, d02_pd, d02_pt, d02_pv;
    wire d02_arm_ok, d02_ready;
    wire [2:0] d14_phase;
    wire [3:0] d14_reason;

    wire position_set, position_ready;
    wire [63:0] point_config;
    wire signed [31:0] position_value, position_target;
    wire [63:0] motion_pi_config;
    wire [223:0] cruise_config;
    wire [63:0] cruise_gains;
    wire signed [31:0] sampled_motion_position,sampled_motion_velocity,sampled_outer_i;
    wire signed [31:0] position_actual;
    hybrid_arm_speed #(.CLK_HZ(CLK_HZ),.HANDOFF_CONTROL(1)) u_speed_estimator (
        .clk(clk), .rst_n(rst_n && sensor_valid && encoder_ready_debug && !config_applied && !user_rearm),
        .sample_i(control_tick), .count_i(encoder_count_debug), .raw_speed_i(theta_dot),
        .trace_reverse_o(estimator_reverse_trace),.trace_period_o(estimator_period_trace),
        .trace_flags_o(estimator_flags_trace),.timeout_ms_i(speed_config[38:32]), .speed_o(estimated_arm_speed), .valid_o(estimated_speed_valid),.status_o(estimated_speed_status));
    controller #(.DUAL_MODE(1),.MOTION_LEAN_CONTROL(1),.MOTION_CONTROL(1),.MULTITURN_CONTROL(1),.SPEED_HANDOFF_CONTROL(1),.POINT_CONTROL(1),.COORDINATED_CONTROL(1),.LIVE_VELOCITY_BYPASS(1),.LIVE_BRAKE_CONFIG(1),.R14M_ENABLE(R14M_ENABLE),.BRAKE_DISTANCE_GAIN_Q8(BRAKE_DISTANCE_GAIN_Q8),.BRAKE_STEP_U(BRAKE_STEP_U),.SPEED_LOOP_CONTROL(1),.ARM_SLEW_U_PER_UPDATE(10), .START_MODE_CONTROL(1), .INDEPENDENT_RETURN_CAP(1), .K_THETA_TENTHS(1), .NEAR_CONTROL(1), .SPEED_FILTER_CONTROL(1),
        .POSITION_CONTROL(1), .FIXED_POSITION(0), .RETURN_TRIM_ENABLE(0), .BALANCE_SPEED_INPUT(1), .SETTLE_CONTROL(0), .PID_CONTROL(1), .HOLD_CONTROL(0), .POSITION_I_CONTROL(0), .RETURN_CONTROL(1),
        .LIVE_CONFIG(1),
        // 1 kHz tick / 5 = 200 Hz 控制周期（原 20 -> 50 Hz）
        .UPDATE_DIV  (5),
        .AUTO_SWINGUP(1),
        .SWING_SETPOINT_Q16(ALPHA_SETPOINT_Q16),
        .SWING_U(550),           // Current baseline: 55% startup and energy request
        .SWING_KICK_UPDATES(80),  // D17: maximum 400ms; speed threshold may end it early
        .SWING_ARM_SPEED_GAIN(40), // D17: arm damping during swing-up only
        .SWING_KICK_END_SPEED_Q16(32'sd262144), // D17: end kick at >=4rad/s
        .SWING_START_ONLY(0),     // D17: continue energy swing-up, then balance capture
        // V12: K_ALPHA 900 -> 1800。
        // V11 实测（teraterm.log 2026-10-23）：平衡能维持约 2 s，但一旦偏差
        // 越过 ~6° 电机就追不回来（U 一路加到 +311 仍坠摆），原因就是
        // KA 太小 —— alpha=0.15 rad 时只给出 13% 占空比，重力加速度早已
        // 赢了。线性化仿真（sim/stability_probe.py，24 组机型参数扫描）：
        //   KA=900 ：a0=2.9° 稳 18/24，a0=8.6° 稳 6/24，a0=14.3° 稳 0/24
        //   KA=1800：a0=2.9° 稳 20/24，a0=8.6° 稳 20/24，a0=14.3° 稳 20/24
        // 且对 10 ms 环路延迟仍保持 15/24 以上。KAD=50 保持不变：
        // 仿真里加大 KAD 反而缩小稳定域（这个结构下阻尼项经臂加速耦合
        // 会引入反相）。
        .K_ALPHA     (16'sd2300),
        .K_ALPHA_DOT (16'sd105),
        .K_THETA     (16'sd8),
        .K_THETA_DOT (16'sd35),
        .U_LIMIT     (1000),
        .POSITION_U_LIMIT(100),
        // V12: 19661(0.30 rad) -> 29491(0.45 rad)。
        // V11 的两次坠摆都在 |alpha|=0.30 rad 处被窗口切断出力，之后
        // 自由落体无可挽回。窗口外移到 0.45 rad，KA=1800 时边缘出力
        // 约 81%，接得住的偏差范围明显变宽；真正的"摆杆已倒"保护
        // 仍由窗口下沿兜底。
        //
        // V13: 29491(0.45 rad) -> 39322(0.60 rad)。
        // V12 实测（teraterm(3).log 2026-10-23）：平衡稳定 2.7 s 后受扰，
        // U 一路加到 -790（79%），坠摆角速度确实被压下来了（AD 1.0 ->
        // 0.54，接摆动作有效），但 |alpha| 越过 0.45 rad 的瞬间窗口切断，
        // 接摆功亏一篑。窗沿再外移到 0.60 rad，给接摆留出完成时间；
        // 窗边占空比此时已饱和到 100%，保护语义不变（摆杆真倒了电机
        // 照样停）。
        .ALPHA_ARM   (32'sd39322),
        .THETA_LIM   (32'sd205887),
        .SIGN_MOTOR  (1),
        .SIGN_ALPHA  (1),
        .TEST_MODE   (0),
        .TEST_U      (16'sd80)
    ) u_controller (
        .trace_position_o(trace_position),.trace_alpha_o(trace_alpha),.trace_alpha_dot_o(trace_ad),
        .trace_reference_o(trace_reference),.trace_target_o(trace_target),
        .trace_reference_reason_o(trace_reference_reason),.trace_coordinate_flags_o(trace_coordinate_flags),
        .trace_sampled_position_o(trace_sampled_position),.trace_sampled_reason_o(trace_sampled_reason),
        .trace_sampled_flags_o(trace_sampled_flags),
        .velocity_slew_bypass_i(velocity_slew_bypass),.point_config_i(point_config),.cruise_gains_i(cruise_gains),.cruise_config_i(cruise_config),.motion_pi_config_i(motion_pi_config),
        .dbg_motion_position(sampled_motion_position),.dbg_motion_velocity(sampled_motion_velocity),.dbg_outer_i_angle(sampled_outer_i),
        .coordinate_config_i(coordinate_config),.k_theta_x10_i(k_theta_x10), .return_position_cap_i(return_position_cap), .arm_loop_config_i(arm_loop_config),.dbg_loop_reference(loop_reference_debug),.dbg_loop_state(loop_state_debug),.dbg_brake_status(brake_status_debug),.dbg_brake_distance(brake_distance_debug),.dbg_velocity_error(velocity_error_debug),.dbg_brake_flags(brake_flags_debug),.brake_config_i(brake_config),
        .manual_start_i(manual_start),
        .pid_permit_i(safety_permit && !protection_locked && !motor_fault_latched && !estop_latched && !timeout_latched),
        .motion_config_i(motion_config),.position_i_config_i(position_i_config),.near_config_i(near_config),.return_config_i(return_config),.speed_filter_mode_i(speed_filter_mode),.dbg_vref(arm_vref_debug),.dbg_aref(arm_aref_debug),.dbg_hold_control(hold_control_debug),
        .pid_config_i(pid_config), .dbg_pi(pid_integral_u),
        .settle_config_i(settle_config), .dbg_settle_status(settle_status),
        .dbg_capture_travel(capture_travel), .dbg_lock_origin(lock_origin), .dbg_hold_position(hold_position),
        .dbg_original_arm_q16(sampled_original_arm),.dbg_arm_request_q16(sampled_arm_request),
        .speed_status_i(estimated_speed_status),.dbg_speed_status(sampled_speed_status),
        .balance_speed_i(estimated_arm_speed), .balance_speed_valid_i(estimated_speed_valid),
        .balance_speed_select_i(speed_config[0]), .dbg_raw_speed(sampled_raw_speed),
        .dbg_est_speed(sampled_est_speed), .dbg_speed_source(sampled_speed_source), .dbg_est_valid(sampled_est_valid),
        .config_i(live_config), .run_i(user_run), .rearm_i(user_rearm),
        .position_set_i(position_set), .position_target_i(position_value),
        .position_ready_o(position_ready), .position_actual_o(position_actual),
        .position_target_o(position_target),
        .clk(clk),
        .rst_n(rst_n),
        .control_tick(control_tick),
        .system_state(system_state),
        // 用 20 ms 去抖后的 sensor_ok，而不是原始 sensor_valid：
        // I2C 偶发一次 NACK 会让 sensor_valid 掉 1~2 ms，原来会立刻把
        // u 清零并让电机链走一遍 STOP->OFF->重启，等于每次通信抖动都
        // 打一次"断力"脉冲。真正的持续故障仍然会在 20 ms 内切断。
        .sensor_valid(sensor_ok),
        .theta(theta),
        .theta_dot(theta_dot),
        .alpha(alpha),
        .alpha_dot(alpha_dot),
        .control_u(control_u),
        .control_valid(control_valid),
        .motor_enable(motor_enable),
        .dbg_theta(d02_theta), .dbg_theta_dot(d02_td),
        .dbg_alpha(d02_alpha), .dbg_alpha_dot(d02_ad),
        .dbg_pa(d02_pa), .dbg_pd(d02_pd), .dbg_pt(d02_pt), .dbg_pv(d02_pv),
        .dbg_arm_ok(d02_arm_ok), .dbg_ready(d02_ready),
        .dbg_phase(d14_phase), .dbg_reason(d14_reason)
    );

    // =====================================================
    // 6. 上电安全启动
    //
    // 等到传感器有效，再等 100 ms，发一次清故障脉冲，
    // 然后才允许把控制器输出送进电机链。
    // =====================================================

    localparam [1:0]
        INIT_WAIT_SENSOR = 2'd0,
        INIT_DELAY       = 2'd1,
        INIT_CLEAR       = 2'd2,
        INIT_READY       = 2'd3;

    reg [1:0]  init_state;
    reg [21:0] init_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_state   <= INIT_WAIT_SENSOR;
            init_counter <= 0;
        end
        else begin
            case (init_state)

                INIT_WAIT_SENSOR: begin
                    init_counter <= 0;

                    if (sensor_valid)
                        init_state <= INIT_DELAY;
                end

                INIT_DELAY: begin
                    if (!sensor_valid) begin
                        init_counter <= 0;
                        init_state <= INIT_WAIT_SENSOR;
                    end
                    else if (init_counter >= 22'd2699999) begin
                        init_counter <= 0;
                        init_state <= INIT_CLEAR;
                    end
                    else begin
                        init_counter <= init_counter + 1'b1;
                    end
                end

                INIT_CLEAR: begin
                    init_state <= INIT_READY;
                end

                INIT_READY: begin
                    init_state <= INIT_READY;
                end

                default: begin
                    init_state   <= INIT_WAIT_SENSOR;
                    init_counter <= 0;
                end

            endcase
        end
    end

    // =====================================================
    // 7. 传感器有效性去抖（20 ms）
    //
    // 原来只要 sensor_valid 掉一个时钟，保护就永久锁死，
    // 且 clear_request 只在初始化发过一次，之后再也解不开，
    // 表现就是"电机彻底不动，只能重新下载"。
    // 这里先做 20 ms 去抖，再配合第 9 节的自动重清。
    // =====================================================

    reg [19:0] sv_low_cnt;
    reg        sensor_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sv_low_cnt <= 20'd0;
            sensor_ok  <= 1'b0;
        end
        else if (sensor_valid) begin
            sv_low_cnt <= 20'd0;
            sensor_ok  <= 1'b1;
        end
        else if (sv_low_cnt >= 20'd539999) begin
            sensor_ok  <= 1'b0;
        end
        else begin
            sv_low_cnt <= sv_low_cnt + 1'b1;
        end
    end

    // =====================================================
    // 8. 电机链信号
    // =====================================================

    wire motor_dir;
    wire [2:0] motor_state;
    wire motor_permit;
    wire protection_locked;
    wire motor_fault_latched;
    wire estop_latched;
    wire clear_accepted;
    wire timeout_latched;
    wire command_error_latched;

    wire [1:0] motor_command;
    wire [9:0] motor_duty;
    wire motor_arm;
    wire motor_clear_fault;

    wire motor_fault = !sensor_ok;

    wire motor_r_en;
    wire motor_l_en;

    // =====================================================
    // 9. 周期自动清故障（每 500 ms 一次脉冲）
    //
    // 只有在"电机本来就该停"的时候后级才会接受清除
    // （command=STOP、arm=0、无故障），所以这个脉冲在正常
    // 运行时是无害的；一旦跳闸，等条件恢复后会自动回到可运行
    // 状态，不需要重新下载。
    // AUTO_CLEAR=0 可关掉，退回"必须重新上电"的老行为。
    // =====================================================

    reg [23:0] clear_gap;
    reg        clear_pulse_auto;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clear_gap        <= 24'd0;
            clear_pulse_auto <= 1'b0;
        end
        else begin
            clear_pulse_auto <= 1'b0;

            if (init_state != INIT_READY) begin
                clear_gap <= 24'd0;
            end
            else if (AUTO_CLEAR != 0) begin
                if (clear_gap >= 24'd13499999) begin
                    clear_gap        <= 24'd0;
                    clear_pulse_auto <= 1'b1;
                end
                else begin
                    clear_gap <= clear_gap + 1'b1;
                end
            end
        end
    end

    wire motor_clear_request =
        (init_state == INIT_CLEAR) || clear_pulse_auto;

    // =====================================================
    // 10. 命令安全闸门
    //
    // 保护锁死或故障锁存时必须把 motor_enable 拉低：
    // 否则后级认为"还在运行"，永远不会接受清除命令，
    // 形成死锁（这也是"电机不动"的原因之一）。
    // =====================================================

    wire safety_permit =
        (init_state == INIT_READY) &&
        (system_state == 3'd3) &&
        sensor_ok &&
        rst_n && user_run &&
        !protection_locked &&
        !motor_fault_latched;

    wire signed [15:0] control_u_safe;
    wire control_valid_safe;
    wire motor_enable_safe;

    assign control_u_safe     = safety_permit ? control_u     : 16'sd0;
    assign control_valid_safe = safety_permit ? control_valid : 1'b0;
    // 直接用 safety_permit：它已经含 sensor_ok(20 ms 去抖)、状态机 RUN、
    // 初始化完成、未锁死。用 controller 的 motor_enable 会引入原始
    // sensor_valid 的瞬态，导致后级反复清命令、电机一顿一顿。
    assign motor_enable_safe  = safety_permit;

    // =====================================================
    // 11. 电机控制链
    // =====================================================

    motor_control_u_chain #(
        .CLK_HZ(CLK_HZ),
        .PWM_HZ(PWM_HZ),
        .STOP_CYCLES(STOP_CYCLES),
        .TIMEOUT_CYCLES(2700000),
        // D09: floor 10% at start/reversal, 8% during recent same-direction motion.
        .START_MIN_U(100),
        .RUN_MIN_U(80), .LIVE_START_BOOST(1), .LIVE_FINE_DRIVE(1)
    ) u_motor_control_chain (
        .clk(clk),
        .rst_n(rst_n),

        .fine_drive_enable(motion_config[0] && d14_phase==3'd5),
        .boost_enable(boost_config[0] && d14_phase == 3'd5),
        .boost_min_u(boost_config[39:32]), .boost_ticks(boost_config[67:64]),
        .control_u(control_u_safe),
        .motion_theta(theta),
        .control_valid(control_valid_safe),
        .motor_enable(motor_enable_safe),

        .permit_in(1'b1),
        .fault(motor_fault),
        .estop(1'b0),
        .clear_request(motor_clear_request),

        .rpwm(rpwm),
        .lpwm(lpwm),
        .r_en(motor_r_en),
        .l_en(motor_l_en),

        .motor_dir(motor_dir),
        .state_o(motor_state),
        .permit_o(motor_permit),
        .protection_locked(protection_locked),
        .fault_latched(motor_fault_latched),
        .estop_latched(estop_latched),
        .clear_accepted(clear_accepted),

        .timeout_latched(timeout_latched),
        .command_error_latched(command_error_latched),

        .command(motor_command),
        .duty_permille(motor_duty),
        .arm(motor_arm),
        .clear_fault(motor_clear_fault)
    );

    // IBT-2 的 R_EN / L_EN 在板上并到同一个引脚 T12
    assign en = motor_r_en & motor_l_en;

    // =====================================================
    // 12. UART monitor
    // =====================================================

    wire [7:0] uart_data;
    wire uart_start;
    wire uart_busy;

    // D02: controller inputs/U are captured together; execution is observed
    // 200 us later, after the unchanged 100 us direction-change stop interval.
    // This monitor has no path back into the control or protection logic.
    reg [31:0] d02_ms;
    reg [15:0] d02_seq;
    reg [31:0] d02_sample_ms;
    reg [12:0] d02_delay;
    reg d02_sample;
    reg telemetry_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d02_ms <= 0; d02_seq <= 0; d02_sample_ms <= 0;
            d02_delay <= 0; d02_sample <= 0; telemetry_div<=0;
        end else begin
            d02_sample <= 0;
            if (control_tick) d02_ms <= d02_ms + 1'b1;
            if (control_valid) begin
                d02_seq <= d02_seq + 1'b1;
                d02_sample_ms <= d02_ms;
                d02_delay <= 13'd5400;
            end else if (d02_delay != 0) begin
                d02_delay <= d02_delay - 1'b1;
                if (d02_delay == 1) begin d02_sample <= telemetry_div; telemetry_div<=!telemetry_div; end
            end
        end
    end
    wire [31:0] d02_exec = {
        7'd0, motor_duty, motor_state, motor_command, motor_dir, en,
        motor_permit, protection_locked, timeout_latched,
        command_error_latched, motor_fault_latched, estop_latched,
        safety_permit, sensor_valid
    };
    wire [31:0] d02_flags = {
        6'd0, d14_reason, d14_phase, d02_ready, d02_arm_ok, adc_fault_latched_debug,
        adc_err_count, system_state, encoder_ready_debug, math_valid_debug,
        fault_latched_debug, adc_fault_debug, encoder_fault_latched_debug
    };
    sensor_packet_lite u_sensor_packet (
        .velocity_ref_i(arm_vref_debug),.angle_ref_i(arm_aref_debug),.hold_control_i(hold_control_debug),
        .pid_integral_i(pid_integral_u),
        .settle_status_i(settle_status), .capture_travel_i(capture_travel),
        .lock_origin_i(lock_origin), .hold_position_i(hold_position),
        .boost_settings_i({19'd0,boost_config[67:64],boost_config[39:32],boost_config[0]}),
        .raw_speed_i(sampled_raw_speed), .est_speed_i(sampled_est_speed),
        .brake_gain_i(brake_config[9:0]),.brake_accel_i(brake_config[35:17]),.approach_speed_i(brake_config[52:36]),.release_rate_i(brake_config[62:53]),.brake_distance_i(brake_distance_debug),.velocity_error_i(velocity_error_debug),.brake_flags_i(brake_flags_debug),.brake_status_i(brake_status_debug),.loop_reference_i(loop_reference_debug),.loop_state_i(loop_state_debug),.original_arm_i(sampled_original_arm),.arm_request_i(sampled_arm_request),
        .speed_status_i(sampled_speed_status),
        .motion_position_i(sampled_motion_position),.motion_velocity_i(sampled_motion_velocity),.outer_i_i(sampled_outer_i),
        .speed_source_i(sampled_speed_source), .est_valid_i(sampled_est_valid),
        .clk(clk), .rst_n(rst_n), .send_i(d02_sample && !reply_pending),
        .frame_busy_o(sensor_frame_busy), .version_i(config_version), .run_i(user_run),
        .ms_i(d02_sample_ms), .seq_i(d02_seq),
        // R4: TH reports calibrated wrapped arm angle; controller travel stays internal.
        .theta(theta), .theta_dot(d02_td),
        .alpha(d02_alpha), .alpha_dot(d02_ad),
        .control_u(control_u), .safe_u(control_u_safe),
        .pa_i(d02_pa), .pd_i(d02_pd), .pt_i(d02_pt), .pv_i(d02_pv),
        .exec_i(d02_exec), .flags_i(d02_flags),
        .raw_i(dbg_adc_raw), .adc_i(dbg_adc_sample), .reseeds_i(dbg_adc_reseeds), .zero_i(dbg_alpha_zero), .cal_i(dbg_zero_done),
        .go_i((d14_phase==3'd5) && d02_ready && safety_permit && dbg_zero_done && sensor_valid),
        .uart_busy_i(uart_busy), .uart_data_o(uart_data),
        .uart_start_o(uart_start)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(460800)) u_uart_rx (
        .clk(clk), .rst_n(rst_n), .rx_i(uart_rx_i),
        .data_o(rx_data), .valid_o(rx_valid), .error_o(rx_error));
    uart_command_parser u_commands (
        .clk(clk), .rst_n(rst_n), .data_i(rx_data), .valid_i(rx_valid), .error_i(rx_error),
        .command_o(cmd_op), .id_o(cmd_id), .value_o(cmd_value), .command_valid_o(cmd_valid));
    wire velocity_slew_bypass;
    serial_parameter_bank #(.EXT_POSITION(37), .DEFAULT_KAD(60)) u_parameters (
        .clk(clk), .rst_n(rst_n), .tick_i(control_tick),
        .command_i(cmd_op), .command_valid_i(cmd_valid), .id_i(cmd_id), .value_i(cmd_value),
        .stopped_i(!motor_arm && !en && !rpwm && !lpwm && motor_command==2'b00),
        .ready_i(sensor_ok && dbg_zero_done && init_state==INIT_READY &&
                 system_state==3'd3 && !protection_locked && !motor_fault_latched && config_settle_ms==0),
        .position_ready_i(position_ready), .position_actual_i(position_actual),
        .position_target_i(position_target), .position_set_o(position_set),
        .position_value_o(position_value),
        .phase_i(d14_phase), .reason_i(d14_reason),
        .tx_busy_i(uart_busy || sensor_frame_busy), .tx_data_o(reply_data), .tx_start_o(reply_start),
        .response_pending_o(reply_pending), .config_o(live_config), .speed_config_o(speed_config), .boost_config_o(boost_config), .settle_config_o(settle_config), .pid_config_o(pid_config), .motion_config_o(motion_config), .position_i_config_o(position_i_config),.near_config_o(near_config),.return_config_o(return_config),.speed_filter_mode_o(speed_filter_mode), .k_theta_x10_o(k_theta_x10), .return_position_cap_o(return_position_cap), .coordinate_config_o(coordinate_config),.brake_config_o(brake_config),.arm_loop_config_o(arm_loop_config),.manual_start_o(manual_start), .version_o(config_version),
        .velocity_slew_bypass_o(velocity_slew_bypass),.point_config_o(point_config),.cruise_gains_o(cruise_gains),.cruise_config_o(cruise_config),.motion_pi_config_o(motion_pi_config),
        .trace_status_i(trace_status),.trace_reference_cycle_i(trace_reference_cycle),
        .trace_capture_version_i(trace_version),.trace_arm_ms_i(trace_arm_ms),.trace_reference_ms_i(trace_reference_ms),
        .trace_data_i(trace_data),.trace_read_index_o(trace_index),.trace_arm_o(trace_arm),
        .trace_reference_mode_o(trace_reference_mode),.trace_freeze_o(trace_freeze),
        .run_o(user_run), .rearm_o(user_rearm), .applied_o(config_applied));

    // Route-lite: omit passive event recorder; normal UART snapshots/CSV remain.
    assign trace_data=416'd0;
    assign trace_status=32'd0;
    assign trace_reference_cycle=32'd0;
    assign trace_version=32'd0;
    assign trace_arm_ms=32'd0;
    assign trace_reference_ms=32'd0;

    uart_tx #(
        .CLK_FREQ_HZ(27000000),
        .BAUD_RATE(460800)
    ) u_uart_tx (
        .clk(clk),
        .rst_n(rst_n),
        .data_i(reply_start ? reply_data : uart_data),
        .start_i(reply_start || uart_start),
        .tx_o(uart_tx_o),
        .busy_o(uart_busy)
    );

endmodule

`default_nettype wire
