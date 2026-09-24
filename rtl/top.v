module top(

    input wire clk,

    // encoder
    input wire encoder_a,
    input wire encoder_b,

    // ADS1115
    inout wire ads_scl,
    inout wire ads_sda,

    // UART
    output wire uart_tx_o
);

    // =========================================================
    // Power-on reset
    // 上电后自动产生内部复位，不再需要外部 T10 rst_n
    // =========================================================
    reg [7:0] por_count = 8'd0;
    reg       rst_n     = 1'b0;

    always @(posedge clk) begin
        if (por_count != 8'hFF) begin
            por_count <= por_count + 1'b1;
            rst_n     <= 1'b0;
        end
        else begin
            rst_n <= 1'b1;
        end
    end


    // =========================================================
    // Internal signals
    // =========================================================
    wire       control_tick;
    wire [2:0] system_state;

    wire [7:0] uart_data;
    wire       uart_start;
    wire       uart_busy;
    wire signed [31:0] theta;
    wire signed [31:0] theta_dot;

    wire signed [31:0] alpha;
    wire signed [31:0] alpha_dot;

    wire sensor_valid;


    // =========================================================
    // Control Tick
    // =========================================================
    control_tick u_control_tick (
        .clk    (clk),
        .rst_n  (rst_n),
        .tick_o (control_tick)
    );


    // =========================================================
    // Control FSM
    // =========================================================
    control_fsm u_control_fsm (
        .clk     (clk),
        .rst_n   (rst_n),
        .tick_i  (control_tick),
        .state_o (system_state)
    );


    // =========================================================
    // UART Packet
    // =========================================================
         sensor_packet u_sensor_packet (
        .clk          (clk),
        .rst_n        (rst_n),

        .send_i       (control_tick),
        .state_i      (system_state),

        .uart_busy_i  (uart_busy),

        .uart_data_o  (uart_data),
        .uart_start_o (uart_start),
        .theta(theta),
        .theta_dot(theta_dot),

        .alpha(alpha),
        .alpha_dot(alpha_dot)
    );


    // =========================================================
    // UART TX
    // =========================================================
    uart_tx #(
    .CLK_FREQ_HZ (27_000_000),
    .BAUD_RATE   (115200)
 ) u_uart_tx (
    .clk      (clk),
    .rst_n    (rst_n),
    .data_i   (uart_data),
    .start_i  (uart_start),
    .tx_o     (uart_tx_o),
    .busy_o   (uart_busy)
);
       
     //sensor_top
     sensor_top u_sensor(

    .clk(clk),
    .rst_n(rst_n),

    .encoder_a(encoder_a),
    .encoder_b(encoder_b),

    .ads_scl(ads_scl),
    .ads_sda(ads_sda),

    .theta(theta),
    .theta_dot(theta_dot),

    .alpha(alpha),
    .alpha_dot(alpha_dot),

    .sensor_valid(sensor_valid)
);

endmodule