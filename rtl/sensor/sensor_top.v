`timescale 1ns/1ps
`default_nettype none
module sensor_top #(
    // 摆杆平衡点偏置（Q16 rad），透传到 sensor_math。0 = 竖直向上。
    parameter signed [31:0] ALPHA_SETPOINT_Q16 = 32'sd0,
    // 平衡点偏置软启动时间（ms），0 = 关闭
    parameter integer      SOFTRAMP_MS         = 300
)(
    input wire clk,
    input wire rst_n,
input wire encoder_a,
    input wire encoder_b,
inout wire ads_scl,
    inout wire ads_sda,
output wire signed [31:0] theta,
    output wire signed [31:0] theta_dot,
    output wire signed [31:0] alpha,
    output wire signed [31:0] alpha_dot,
    output wire sensor_valid,
output wire signed [31:0] dbg_encoder_count,
    output wire [3:0] dbg_encoder_ab,
    output wire dbg_encoder_illegal,
// Debug outputs
    output wire encoder_ready_debug,
    output wire math_valid_debug,
    output wire fault_latched_debug,
    output wire adc_fault_debug,
    output wire encoder_fault_latched_debug,
    output wire adc_fault_latched_debug,
    output wire [7:0] adc_err_count_out,
    // 2026-10-21: 零点标定结果（诊断）
    output wire signed [31:0] dbg_alpha_zero,
    output wire signed [31:0] dbg_alpha_wrap,
    output wire               dbg_zero_done,
    output wire signed [15:0] dbg_adc_raw,
    output wire signed [15:0] dbg_adc_sample,
    output wire [7:0] dbg_adc_reseeds
);
wire signed [15:0] adc_sample;
    wire adc_sample_valid;
    wire adc_ready;
    wire adc_fault;
// =====================================================
    // ADS1115 ADC reader
    // =====================================================
    ads1115_reader adc (
        .clk(clk),
        .rst_n(rst_n),
.scl(ads_scl),
        .sda(ads_sda),
.sample(adc_sample),
        .sample_valid(adc_sample_valid),
        .ready(adc_ready),
        .fault(adc_fault)
    );
// =====================================================
    // Sensor processing
    // =====================================================
    sensor_state #(
        .ALPHA_SETPOINT_Q16(ALPHA_SETPOINT_Q16),
        .SOFTRAMP_MS(SOFTRAMP_MS)
    ) path (
        .clk(clk),
        .rst_n(rst_n),
.encoder_a(encoder_a),
        .encoder_b(encoder_b),
.adc_sample(adc_sample),
        .adc_sample_valid(adc_sample_valid),
        .adc_ready(adc_ready),
        .adc_fault(adc_fault),
.theta(theta),
        .theta_dot(theta_dot),
        .alpha(alpha),
        .alpha_dot(alpha_dot),
        .sensor_valid(sensor_valid),
 .dbg_encoder_count(dbg_encoder_count),
        .dbg_encoder_ab(dbg_encoder_ab),.dbg_encoder_illegal(dbg_encoder_illegal),
// Diagnostic connections
        .encoder_ready_debug(encoder_ready_debug),
        .math_valid_debug(math_valid_debug),
        .fault_latched_debug(fault_latched_debug),
        .adc_fault_debug(adc_fault_debug),
        .encoder_fault_latched_debug(
            encoder_fault_latched_debug
        ),
        .adc_fault_latched_debug(
            adc_fault_latched_debug
        ),
        .adc_err_count_out(adc_err_count_out),

        .dbg_alpha_zero(dbg_alpha_zero),
        .dbg_alpha_wrap(dbg_alpha_wrap),
        .dbg_zero_done(dbg_zero_done),
        .dbg_adc_raw(dbg_adc_raw),
        .dbg_adc_sample(dbg_adc_sample), .dbg_adc_reseeds(dbg_adc_reseeds)
    );
endmodule