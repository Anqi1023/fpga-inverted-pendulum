module sensor_top(
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

    output wire sensor_valid
);


wire signed [15:0] adc_sample;
wire adc_sample_valid;
wire adc_ready;
wire adc_fault;


ads1115_reader adc(
    .clk(clk),
    .rst_n(rst_n),

    .scl(ads_scl),
    .sda(ads_sda),

    .sample(adc_sample),

    .sample_valid(adc_sample_valid),
    .ready(adc_ready),
    .fault(adc_fault)
);


sensor_state path(

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

    .sensor_valid(sensor_valid)

);


endmodule