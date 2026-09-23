module top (
    input  wire clk,
    input  wire enc_a,
    input  wire enc_b,

    inout  wire i2c_sda,
    output wire i2c_scl,

    output wire led,
    output wire uart_tx_o
);

    // ============================================================
    // Power-on reset
    // ============================================================

    reg [7:0] reset_cnt = 8'd0;
    wire rst_n;

    always @(posedge clk) begin
        if (!rst_n)
            reset_cnt <= reset_cnt + 1'b1;
    end

    assign rst_n = &reset_cnt;


    // ============================================================
    // Rotary arm encoder
    // ============================================================

    wire signed [31:0] encoder_count;

    encoder u_encoder (
        .clk   (clk),
        .rst_n (rst_n),
        .enc_a (enc_a),
        .enc_b (enc_b),
        .count (encoder_count)
    );

    // LED indicates encoder count activity
    assign led = encoder_count[6];


    // ============================================================
    // theta_raw
    // encoder count -> rotary arm angle
    // unit: mrad
    // ============================================================

    wire signed [31:0] theta_raw_mrad;

    theta_calc u_theta_calc (
        .count_i      (encoder_count),
        .theta_mrad_o (theta_raw_mrad)
    );


    // ============================================================
    // ADS1115
    // Reads WDD35D4 pendulum angle sensor
    // ============================================================

    wire signed [15:0] adc_raw;
    wire               adc_valid;
    wire               ads_ack_error;

    ads1115_i2c u_ads1115_i2c (
        .clk        (clk),
        .rst_n      (rst_n),

        .i2c_sda    (i2c_sda),
        .i2c_scl    (i2c_scl),

        .adc_raw    (adc_raw),
        .data_valid (adc_valid),
        .ack_error  (ads_ack_error)
    );


    // ============================================================
    // alpha_raw
    // ADC -> pendulum angle
    // unit: mrad
    // ============================================================

    wire signed [31:0] alpha_raw_mrad;

    angle_sensor u_angle_sensor (
        .clk        (clk),
        .rst_n      (rst_n),

        .adc_raw    (adc_raw),
        .adc_valid  (adc_valid),

        .alpha_mrad (alpha_raw_mrad)
    );


    // ============================================================
    // Calibration
    //
    // Capture initial theta and alpha zero positions
    // ============================================================

    wire signed [31:0] theta_mrad;
    wire signed [31:0] alpha_mrad;
    wire               calibrated;

    calibration u_calibration (
        .clk          (clk),
        .rst_n        (rst_n),

        .theta_raw_i  (theta_raw_mrad),
        .alpha_raw_i  (alpha_raw_mrad),

        .theta_cal_o  (theta_mrad),
        .alpha_cal_o  (alpha_mrad),

        .calibrated_o (calibrated)
    );


    // ============================================================
    // theta_dot
    // unit: mrad/s
    // ============================================================

    wire signed [31:0] theta_dot_mrad_s;

    theta_velocity u_theta_velocity (
        .clk                (clk),
        .rst_n              (rst_n),

        .theta_mrad_i       (theta_mrad),

        .theta_dot_mrad_s_o (theta_dot_mrad_s)
    );


    // ============================================================
    // alpha_dot
    // unit: mrad/s
    // ============================================================

    wire signed [31:0] alpha_dot_mrad_s;

    alpha_velocity u_alpha_velocity (
        .clk                (clk),
        .rst_n              (rst_n),

        .alpha_valid_i      (adc_valid),
        .alpha_mrad_i       (alpha_mrad),

        .alpha_dot_mrad_s_o (alpha_dot_mrad_s)
    );


    // ============================================================
    // UART state monitor
    //
    // T  = theta
    // TD = theta_dot
    // A  = alpha
    // AD = alpha_dot
    // ============================================================

    wire [7:0] uart_data;
    wire       uart_start;
    wire       uart_busy;

    state_uart u_state_uart (
        .clk         (clk),
        .rst_n       (rst_n),

        .theta_i     (theta_mrad),
        .theta_dot_i (theta_dot_mrad_s),
        .alpha_i     (alpha_mrad),
        .alpha_dot_i (alpha_dot_mrad_s),

        .uart_data   (uart_data),
        .uart_start  (uart_start),
        .uart_busy   (uart_busy)
    );


    // ============================================================
    // UART transmitter
    // ============================================================

    uart_tx #(
        .CLK_FREQ_HZ (27_000_000),
        .BAUD_RATE   (115200)
    ) u_uart_tx (
        .clk     (clk),
        .rst_n   (rst_n),

        .data_i  (uart_data),
        .start_i (uart_start),

        .tx_o    (uart_tx_o),
        .busy_o  (uart_busy)
    );

endmodule