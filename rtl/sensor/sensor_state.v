
`timescale 1ns/1ps
`default_nettype none

module sensor_state #(
    // 摆杆平衡点偏置（Q16 rad）。0 = 竖直；正值 = 让摆杆稳在偏一侧的位置，
    // 平衡时 alpha 项不为零，电机长期出力越过减速箱静摩擦死区。
    // 4000 ≈ 3.5°，对应约 5.5% 的稳态占空比。
    parameter signed [31:0] ALPHA_SETPOINT_Q16 = 32'sd0,
    // 软启动时间（ms）：偏置从 0 斜坡到目标值的时间，0 = 关闭
    parameter integer      SOFTRAMP_MS         = 300,
    // D18: same raw-code period as sensor_math; no scale/zero retuning.
    parameter integer      ALPHA_SPAN_RAW      = 27512
)(
    input wire clk,
    input wire rst_n,

    input wire encoder_a,
    input wire encoder_b,

    input wire signed [15:0] adc_sample,
    input wire adc_sample_valid,
    input wire adc_ready,
    input wire adc_fault,

    output wire signed [31:0] theta,
    output wire signed [31:0] theta_dot,
    output wire signed [31:0] alpha,
    output wire signed [31:0] alpha_dot,

    output wire sensor_valid,

    output wire signed [31:0] dbg_encoder_count,
    output wire [3:0] dbg_encoder_ab,
    output wire dbg_encoder_illegal,
    output reg state_update,
    output reg signed [31:0] raw_encoder,
    output reg signed [15:0] raw_adc,
    output reg signed [15:0] raw_adc_average,

    // Debug signals
    output wire encoder_ready_debug,
    output wire math_valid_debug,
    output wire fault_latched_debug,
    output wire adc_fault_debug,
    output wire encoder_fault_latched_debug,
    output wire adc_fault_latched_debug,
    // 上一报表周期内的 I2C 出错次数（8 位饱和计数）
    output wire [7:0] adc_err_count_out,

    // 2026-10-21: 零点标定结果（诊断用，验证零点是否落在正确位置）
    output wire signed [31:0] dbg_alpha_zero,
    output wire signed [31:0] dbg_alpha_wrap,
    output wire               dbg_zero_done,
    output wire signed [15:0] dbg_adc_raw,
    output wire signed [15:0] dbg_adc_sample,
    output wire [7:0] dbg_adc_reseeds
);

    // 2026-10-21: 诊断用 ADC 值取实时 EMA（低 8 位小数丢弃），不用 raw_adc_average。
    // raw_adc_average 只在 state_update（5 ms）那拍刷新，上电头几帧还是 0，
    // 而诊断行只发 64 帧（1.3 s）——用快照的话学生可能截到一串 R=0000。
    // assign 语句放在 adc_ema_q8 声明之后（见下方），避免前向引用。


    wire signed [31:0] encoder_count;
    assign dbg_encoder_count = encoder_count;
    wire encoder_ready;
    wire encoder_error;
    assign dbg_encoder_illegal=encoder_error;
    wire math_valid;

    // ------------------------------------------------------------------
    // ADC 平滑：Q8 指数滑动平均（EMA）
    //
    // 2026-10-18 改动说明：
    // 原来是"攒够 16 个样本求一次平均"，但 ADC 采样周期约 2.0 ms
    // （33750 计数等待 + 约 0.8 ms 的 I2C 读取时间），16 个样本要
    // 31~32 ms 才凑齐，而 state_update 是固定 20 ms 一次。两者不同步，
    // 于是控制周期有时拿到"同一份均值"（alpha 差分=0），有时跨了
    // 两个均值（差分翻倍）——alpha_dot 会间歇性归零或翻倍。
    // 速度项是唯一能压住摆杆的阻尼，它一旦抖动，增益怎么调都稳不住。
    // 改成 EMA 后每个 ADC 样本都更新一次，5 ms 的控制周期拿到的
    // alpha 永远是"刚刚过去几毫秒"的值，差分才有意义。
    // ------------------------------------------------------------------
    localparam integer EMA_SHIFT = 2;   // 时间常数 ≈ 4 个样本 ≈ 8 ms

    // ------------------------------------------------------------------
    // ADC 野值剔除（2026-10-20）
    //
    // V6 上板日志里出现 130 帧 alpha_dot 尖峰（最大 312 rad/s，实际物理上
    // 不可能），而 EC 计数是 00——说明 I2C 读到的是"半帧/错位的数据"，
    // 既不 NACK 也不超时，被当成好数据算了进来。
    //
    // 判据：新样本与当前 EMA 的偏差超过 ADC_SPIKE_LIMIT 就丢弃。
    // 正常摆动时 alpha 的最大角速度约 15 rad/s，5 ms 内变化不到 0.075 rad
    // ≈ 440 个 ADC 码（按 ALPHA_GAIN_Q24=3992 反算），取 2000 码留足余量。
    // 上电头几个样本（EMA 还没建立）不做判据。
    //
    // 2026-10-21 修正：上面的 440 码算错了。alpha_calc 是 Q(n+24)，
    // 再取 [39:8] 才是 Q16，所以每个 ADC 码对应的角度是
    //   3992 / 2^8 / 65536 = 0.000238 rad = 0.0136°
    // 15 rad/s × 5 ms = 0.075 rad = 315 码，取 2000 码仍有 6 倍余量，阈值本身没问题。
    // 真正的问题在上电阶段（见下方 adc_seed）。
    //
    // 连续丢太多说明传感器真的坏了，用 adc_spike_cnt 计数上报（EC 字段），
    // 但不清零 EMA——摆杆不会因为这条链路抖一下就失去控制。
    // ------------------------------------------------------------------
    localparam signed [31:0] ADC_SPIKE_LIMIT = 32'sd2000;
    localparam integer       EMA_WARMUP      = 8;    // 前 8 个样本不做野值判据

    reg [22:0] adc_ema_q8; // D19: unsigned Q8 canonical state, 0 <= value < PERIOD
    reg [7:0]         adc_sample_cnt;
    reg [7:0]         adc_spike_cnt;
    // D05: one isolated spike is still rejected. Four consecutive outliers
    // with mutually consistent raw readings re-establish the moving average.
    reg [4:0] adc_reject_run;
    reg [15:0] adc_reject_last; // normalized code, not raw signed ADC
    reg [7:0] adc_reseed_count;
    reg adc_rebase_pending;
    // D19: explicit-width normalization, no repeated functions or loop cascade.
    // A signed16 ADC magnitude is <=32768, so one unsigned subtraction suffices.
    localparam [16:0] ADC_PERIOD_CODES=ALPHA_SPAN_RAW;
    localparam signed [24:0] ADC_PERIOD_Q8=ALPHA_SPAN_RAW*256;
    localparam signed [24:0] ADC_HALF_Q8=ADC_PERIOD_Q8/2;
    localparam signed [17:0] ADC_PERIOD_C18=ALPHA_SPAN_RAW;
    localparam signed [17:0] ADC_HALF_C18=ALPHA_SPAN_RAW/2;
    wire [15:0] adc_mag=adc_sample[15] ? (~adc_sample+16'd1) : adc_sample;
    wire [16:0] adc_remainder=({1'b0,adc_mag}>=ADC_PERIOD_CODES) ?
        ({1'b0,adc_mag}-ADC_PERIOD_CODES) : {1'b0,adc_mag};
    wire [16:0] adc_normalized=(adc_sample[15] && adc_remainder!=0) ?
        ADC_PERIOD_CODES-adc_remainder : adc_remainder;
    wire [22:0] adc_x_q8={adc_normalized[14:0],8'd0};
    wire signed [24:0] error_linear=
        $signed({2'b0,adc_x_q8})-$signed({2'b0,adc_ema_q8});
    wire signed [24:0] error_short=(error_linear>ADC_HALF_Q8) ?
        error_linear-ADC_PERIOD_Q8 : (error_linear < -ADC_HALF_Q8) ?
        error_linear+ADC_PERIOD_Q8 : error_linear;
    wire signed [31:0] adc_err_q8={{7{error_short[24]}},error_short};
    wire signed [31:0] adc_err_abs=adc_err_q8[31] ? -adc_err_q8 : adc_err_q8;
    wire signed [24:0] ema_step=error_short >>> EMA_SHIFT;
    wire signed [24:0] ema_sum=$signed({2'b0,adc_ema_q8})+ema_step;
    wire [24:0] ema_candidate=ema_sum[24] ? $unsigned(ema_sum+ADC_PERIOD_Q8) :
        ($unsigned(ema_sum)>=$unsigned(ADC_PERIOD_Q8)) ?
        $unsigned(ema_sum-ADC_PERIOD_Q8) : $unsigned(ema_sum);
    wire ema_candidate_ok=(ema_candidate<$unsigned(ADC_PERIOD_Q8));

    wire signed [17:0] reject_linear=
        $signed({1'b0,adc_normalized})-$signed({2'b0,adc_reject_last});
    wire signed [17:0] reject_short=(reject_linear>ADC_HALF_C18) ?
        reject_linear-ADC_PERIOD_C18 : (reject_linear < -ADC_HALF_C18) ?
        reject_linear+ADC_PERIOD_C18 : reject_linear;
    wire [17:0] reject_abs=reject_short[17] ? -reject_short : reject_short;
    wire adc_cluster_consistent=(reject_abs<=ADC_SPIKE_LIMIT);
    // At the potentiometer seam, several intermediate contact samples may form
    // a false short cluster. Wait longer here before rebasing the derivative.
    wire near_adc_seam=(adc_ema_q8<(ADC_SPIKE_LIMIT*256)) ||
        (adc_ema_q8>((ALPHA_SPAN_RAW-ADC_SPIKE_LIMIT)*256));
    wire [4:0] recovery_before=near_adc_seam ? 5'd11 : 5'd3;
    wire adc_recover=adc_ready && adc_sample_valid && adc_spike &&
        (adc_reject_run>=recovery_before) && adc_cluster_consistent;
    assign dbg_adc_sample=adc_sample;
    assign dbg_adc_reseeds=adc_reseed_count;
    assign dbg_adc_raw=$signed({1'b0,adc_ema_q8[22:8]});

    wire adc_spike =
        (adc_sample_cnt >= EMA_WARMUP) &&
        (adc_err_abs > (ADC_SPIKE_LIMIT <<< 8));

    wire adc_take_sample =
        adc_ready && adc_sample_valid && !adc_spike && ema_candidate_ok;

    // 2026-10-21: EMA 首样本直接置位。
    //
    // 原来 adc_ema_q8 复位成 0，而摆杆真实码值在 2 万上下 —— 偏差 2 万码，
    // 远超 ADC_SPIKE_LIMIT=2000，但 warmup 阶段不做判据，所以头 8 个样本
    // 照收，EMA 每样本只补 1/4 的差额，要十几帧才追上。
    // 一旦 warmup 结束 EMA 还没追上，后面每个样本都会被判成野值并丢弃，
    // adc_sample_cnt 永不推进，EMA 永远停在半路 —— 死锁，EC 直接冲到 FF。
    // 学生的日志就是这种现象（EC=FF 全程，sensor_math 拿到的 adc_average
    // 停在错误值上）。
    //
    // 改成：第一个有效样本直接写进 EMA，之后才走滑动平均。这样 EMA 从
    // 一开始就在真值附近，野值判据面对的偏差是真实的噪声幅度而不是
    // "从 0 起算"的假偏差。
    wire adc_seed = (adc_sample_cnt == 8'd0);

    wire signed [15:0] adc_average=$signed({1'b0,adc_ema_q8[22:8]});

    reg adc_fault_latched;
    reg fault_latched;
    reg encoder_fault_latched;

    // ------------------------------------------------------------------
    // ADC 数据新鲜度（2026-10-19）
    //
    // 实测这条 I2C 链路会断续出错（日志 AF/AFL=1 但 AL 仍在缓慢变化，
    // 即读操作"偶尔成功、经常失败"）。原来的判据是"错一次就锁存并拉低
    // sensor_valid"，结果每次通信抖动都把整个控制环切断，电机一顿一顿
    // 甚至完全不动。改成新鲜度判据：50 ms 内读到过一帧好数据就算有效。
    // adc_fault_latched 降级为诊断位，仍在串口 AFL 字段输出。
    // ------------------------------------------------------------------
    localparam [23:0] ADC_STALE_CYCLES = 24'd1350000;   // 50 ms
    reg [23:0] adc_good_timer;
    wire adc_stale = (adc_good_timer >= ADC_STALE_CYCLES);

    // I2C 出错计数，每个报表周期（20 ms）快照一次并清零
    reg [7:0] adc_err_cnt;
    reg [7:0] adc_err_snapshot;

    // =====================================================
    // Quadrature encoder
    // =====================================================
    quadrature_decoder #(
        .COUNT_WIDTH(32)
    ) enc (
        .clk(clk),
        .rst_n(rst_n),
        .enc_a(encoder_a),
        .enc_b(encoder_b),
        .position_count(encoder_count),
        .ready(encoder_ready),
        .illegal_transition(encoder_error),
        .trace_ab_o(dbg_encoder_ab)
    );

    // =====================================================
    // Sensor mathematics
    // =====================================================
    sensor_math #(
        .UPDATE_HZ(200),
        .ALPHA_SPAN_RAW(ALPHA_SPAN_RAW),
        // 200 Hz 下每 5 ms 一个计数差，增益须是 50 Hz 版的 4 倍，
        // 否则 theta_dot 只有真值的 1/4，臂的阻尼等于没有。
        .SPEED_GAIN_Q16(32'sd79188),
        // 平衡点偏置从上层透传（sensor_math 内部已改为上电自动标定零点）
        .ALPHA_SETPOINT_Q16(ALPHA_SETPOINT_Q16),
        .SOFTRAMP_MS(SOFTRAMP_MS)
    ) math (
        .clk(clk),
        .rst_n(rst_n),
        .encoder_count(encoder_count),
        .adc_average(adc_average),
        .update(state_update),
        .cal_gate(cal_gate),
        .rebase_velocity(adc_rebase_pending),
        .theta(theta),
        .theta_dot(theta_dot),
        .alpha(alpha),
        .alpha_dot(alpha_dot),
        .valid(math_valid),
        .dbg_alpha_zero(dbg_alpha_zero),
        .dbg_alpha_wrap(dbg_alpha_wrap),
        .dbg_zero_done(dbg_zero_done)
    );

    // =====================================================
    // 200 Hz 更新脉冲
    // 27 MHz / 135000 = 200 Hz（原来 540001 -> 50 Hz）
    // 与控制器 UPDATE_DIV=5 严格同步，也正好是 ADC 采样周期
    // （约 2.0 ms）的整数倍关系，不再是"50 Hz 对 31.5 ms"的错拍。
    // =====================================================
    reg [19:0] update_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_ema_q8            <= 32'sd0;
            state_update          <= 1'b0;
            update_counter        <= 20'd0;

            fault_latched         <= 1'b0;
            adc_fault_latched     <= 1'b0;
            encoder_fault_latched <= 1'b0;

            adc_good_timer  <= ADC_STALE_CYCLES;  // 上电先按"无数据"处理
            adc_err_cnt     <= 8'd0;
            adc_err_snapshot <= 8'd0;
            adc_sample_cnt  <= 8'd0;
            adc_spike_cnt   <= 8'd0;
            adc_reject_run <= 0; adc_reject_last <= 0;
            adc_reseed_count <= 0; adc_rebase_pending <= 0;
        end
        else begin
            state_update <= 1'b0;
            // math consumes the pending marker on this update edge.
            if (state_update) adc_rebase_pending <= 0;

            if (update_counter == 20'd134999) begin
                update_counter   <= 20'd0;
                state_update     <= 1'b1;
                adc_err_snapshot <= adc_err_cnt;
                adc_err_cnt      <= 8'd0;
            end
            else begin
                update_counter <= update_counter + 1'b1;
            end

            // D05: freshness means an ACCEPTED measurement, not merely an
            // electrically successful read that the filter keeps rejecting.
            if (adc_take_sample || adc_recover) begin
                adc_good_timer <= 24'd0;
            end
            else if (adc_good_timer < ADC_STALE_CYCLES) begin
                adc_good_timer <= adc_good_timer + 24'd1;
            end

            if (adc_fault && (adc_err_cnt != 8'hFF))
                adc_err_cnt <= adc_err_cnt + 8'd1;

            // 野值计数：和 I2C 出错一起累加，串口 EC 字段看得到。
            // 若 EC 长期是 00 而现在开始出现 1~3，说明链路有断续但可控；
            // 一直是大数（>20）说明接线/上拉有问题，要先处理硬件。
            if (adc_ready && adc_sample_valid && adc_spike && (adc_spike_cnt != 8'hFF))
                adc_spike_cnt <= adc_spike_cnt + 8'd1;

            // Encoder fault history: 只做诊断，不参与 sensor_valid。
            // 4 倍频解码器在机械抖动/电气噪声下偶发"两相同时变化"
            // 是正常的，把它当致命故障锁存会导致传感器永久无效。
            if (encoder_error)
                encoder_fault_latched <= 1'b1;

            // ADC fault history: I2C 一帧出错是可恢复的，
            // 只要之后又读到一帧好数据就自动解除。
            if (adc_sample_valid && adc_ready)
                adc_fault_latched <= 1'b0;
            else if (adc_fault)
                adc_fault_latched <= 1'b1;

            // Overall sensor fault history（仅诊断用）
            if (encoder_error || adc_fault)
                fault_latched <= 1'b1;
            else if (!encoder_fault_latched && !adc_fault_latched)
                fault_latched <= 1'b0;

            // EMA：y += (x - y) >>> EMA_SHIFT，逐样本更新
            // 2026-10-20: 改用 adc_take_sample（已剔除野值）作为更新条件。
            // 2026-10-21: 首样本直接置位（adc_seed），避免从 0 起算时
            //             EMA 追不上真值、把后续样本全判成野值的死锁。
            if (adc_ready && adc_sample_valid && adc_seed) begin
                adc_ema_q8      <= adc_x_q8;
                adc_sample_cnt  <= 8'd1;
            end
            else if (adc_take_sample) begin
                adc_ema_q8 <= ema_candidate[22:0];
                if (adc_sample_cnt != 8'hFF)
                    adc_sample_cnt <= adc_sample_cnt + 8'd1;
            end

            if (adc_ready && adc_sample_valid) begin
                if (adc_spike) begin
                    adc_reject_last <= adc_normalized[15:0];
                    if (adc_recover) begin
                        adc_ema_q8 <= adc_x_q8;
                        adc_reject_run <= 0;
                        adc_rebase_pending <= 1;
                        if (adc_reseed_count != 8'hFF)
                            adc_reseed_count <= adc_reseed_count + 1'b1;
                    end else if (adc_reject_run != 0 && adc_cluster_consistent && adc_reject_run<31)
                        adc_reject_run <= adc_reject_run + 1'b1;
                    else
                        adc_reject_run <= 1;
                end else
                    adc_reject_run <= 0;
            end

        end
    end

    // =====================================================
    // Raw sensor debug snapshot
    // =====================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_encoder     <= 32'sd0;
            raw_adc         <= 16'sd0;
            raw_adc_average <= 16'sd0;
        end
        else if (state_update) begin
            raw_encoder     <= encoder_count;
            raw_adc         <= adc_sample;
            raw_adc_average <= adc_average;
        end
    end

    // =====================================================
    // 零点标定闸门（V10）
    //
    // V9 上板诊断行打出 Z=00000000 R=0000 D=1：标定窗口上电立刻开跑，
    // 而 ADS1115 那会儿还没开始出数，64 个样本平均成 0，零点=0。
    // alpha 全错后，控制器只在 raw≈1000~1500 的窄带里有输出——
    // 学生看到的就是"只有水平位置附近电机有反馈，其他角度不动"。
    //
    // 闸门条件（注意不能包含 math_valid：它依赖标定完成，而标定
    // 又依赖闸门，会形成环形死锁）：
    //   ① 上电已过 CAL_DELAY_UPDATES 个更新周期（200 拍 @200Hz = 1 s）
    //   ② 编码器就绪 && ADC 数据不陈旧（链路确认活着）
    // 满足后才开始计 ZERO_SAMPLES=64 个样本（约 0.32 s）完成标定。
    // 上电后请保持摆杆竖直静止至少 2 秒。
    // =====================================================
    localparam integer CAL_DELAY_UPDATES = 200;
    reg [19:0] cal_delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cal_delay_cnt <= 20'd0;
        else if (state_update && cal_delay_cnt < CAL_DELAY_UPDATES)
            cal_delay_cnt <= cal_delay_cnt + 20'd1;
    end

    wire cal_gate =
        (cal_delay_cnt >= CAL_DELAY_UPDATES) &&
        encoder_ready && !adc_stale;

    // =====================================================
    // Sensor validity and debug outputs
    // =====================================================
    // sensor_valid 决定控制器能不能出力，必须是"可恢复"的：
    // 编码器看当前状态；ADC 看数据新鲜度（50 ms 内有好数据即可），
    // 不再让单次 I2C 出错锁存切断输出。
    assign sensor_valid =
        encoder_ready &&
        math_valid &&
        !adc_stale;

    assign adc_err_count_out = adc_err_snapshot + adc_spike_cnt;

    assign encoder_ready_debug         = encoder_ready;
    assign math_valid_debug            = math_valid;
    assign fault_latched_debug         = fault_latched;
    assign adc_fault_debug             = adc_fault;
    assign encoder_fault_latched_debug = encoder_fault_latched;
    assign adc_fault_latched_debug     = adc_fault_latched;

endmodule

`default_nettype wire
