module top(

    input wire clk,

    // =========================================================
    // A - Encoder
    // =========================================================
    input wire encoder_a,
    input wire encoder_b,

    // =========================================================
    // A - ADS1115
    // =========================================================
    inout wire ads_scl,
    inout wire ads_sda,

    // =========================================================
    // UART
    // =========================================================
    output wire uart_tx_o,

    // =========================================================
    // B - Motor physical outputs
    // =========================================================
    output wire rpwm,
    output wire lpwm,
    output wire en
);


// =============================================================
// Power-on reset
// 27 MHz clock
// =============================================================
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


// =============================================================
// Basic system signals
// =============================================================
wire       control_tick;
wire [2:0] system_state;


// =============================================================
// UART signals
// =============================================================
wire [7:0] uart_data;
wire       uart_start;
wire       uart_busy;


// =============================================================
// Sensor signals
// =============================================================
wire signed [31:0] theta;
wire signed [31:0] theta_dot;

wire signed [31:0] alpha;
wire signed [31:0] alpha_dot;

wire sensor_valid;


// =============================================================
// C -> B normal control interface
// =============================================================
wire signed [15:0] control_u;
wire               control_valid;
wire               motor_enable;


// =============================================================
// B internal motor outputs
// =============================================================
wire motor_rpwm;
wire motor_lpwm;

wire motor_r_en;
wire motor_l_en;


// =============================================================
// Motor status
// =============================================================
wire motor_dir;

wire [2:0] motor_state;

wire motor_permit;

wire protection_locked;

wire fault_latched;
wire estop_latched;

wire clear_accepted;

wire timeout_latched;
wire command_error_latched;

wire [1:0] motor_command;
wire [6:0] motor_duty;

wire motor_arm;
wire motor_clear_fault;


// =============================================================
// Motor safe-start state machine
// =============================================================
localparam MOTOR_INIT_WAIT       = 2'd0;
localparam MOTOR_INIT_CLEAR_REQ  = 2'd1;
localparam MOTOR_INIT_CLEAR_HOLD = 2'd2;
localparam MOTOR_INIT_READY      = 2'd3;

reg [1:0]  motor_init_state;
reg [21:0] motor_init_count;

wire motor_clear_request;

wire signed [15:0] control_u_safe;
wire               control_valid_safe;
wire               motor_enable_safe;


// =============================================================
// Control Tick
// =============================================================
control_tick u_control_tick (
    .clk    (clk),
    .rst_n  (rst_n),
    .tick_o (control_tick)
);


// =============================================================
// Control FSM
// =============================================================
control_fsm u_control_fsm (
    .clk     (clk),
    .rst_n   (rst_n),
    .tick_i  (control_tick),
    .state_o (system_state)
);


// =============================================================
// C controller
//
// 正式控制链入口。
// 当前 controller.v 为安全 V1：
// RUN + sensor_valid 时按 1 kHz 发送 control_u = 0。
// 先验证正式链路，后续再在 controller.v 中加入闭环算法。
// =============================================================
controller u_controller (

    .clk           (clk),
    .rst_n         (rst_n),

    .control_tick  (control_tick),
    .system_state  (system_state),
    .sensor_valid  (sensor_valid),

    .theta         (theta),
    .theta_dot     (theta_dot),
    .alpha         (alpha),
    .alpha_dot     (alpha_dot),

    .control_u     (control_u),
    .control_valid (control_valid),
    .motor_enable  (motor_enable)

);


// =============================================================
// Motor safe startup
//
// 上电：
// WAIT 100 ms
//      ↓
// CLEAR_REQ
//      ↓
// CLEAR_HOLD
//      ↓
// READY
// =============================================================
always @(posedge clk) begin

    if (!rst_n) begin

        motor_init_state <= MOTOR_INIT_WAIT;
        motor_init_count <= 22'd0;

    end
    else begin

        case (motor_init_state)

            MOTOR_INIT_WAIT: begin

                // 27 MHz × 0.1 s = 2,700,000 clocks
                if (motor_init_count >= 22'd2699999) begin

                    motor_init_count <= 22'd0;
                    motor_init_state <= MOTOR_INIT_CLEAR_REQ;

                end
                else begin

                    motor_init_count <= motor_init_count + 1'b1;

                end

            end


            MOTOR_INIT_CLEAR_REQ: begin

                // 给 adapter 一个 clear_request 上升沿
                motor_init_state <= MOTOR_INIT_CLEAR_HOLD;

            end


            MOTOR_INIT_CLEAR_HOLD: begin

                // 多保持一个时钟，
                // 让 clear_fault 被保护模块正确采样
                motor_init_state <= MOTOR_INIT_READY;

            end


            MOTOR_INIT_READY: begin

                motor_init_state <= MOTOR_INIT_READY;

            end


            default: begin

                motor_init_state <= MOTOR_INIT_WAIT;
                motor_init_count <= 22'd0;

            end

        endcase

    end

end


// =============================================================
// Protection clear request
// =============================================================
assign motor_clear_request =
       (motor_init_state == MOTOR_INIT_CLEAR_REQ);


// =============================================================
// Formal controller -> B safe gate
//
// motor_init_state 未进入 READY 时：
//   control_u     = 0
//   control_valid = 0
//   motor_enable  = 0
//
// motor_init_state 进入 READY 后：
//   正式接入 controller.v 的三个输出。
// =============================================================
assign control_u_safe =
       (motor_init_state == MOTOR_INIT_READY)
       ? control_u
       : 16'sd0;

assign control_valid_safe =
       (motor_init_state == MOTOR_INIT_READY)
       ? control_valid
       : 1'b0;

assign motor_enable_safe =
       (motor_init_state == MOTOR_INIT_READY)
       ? motor_enable
       : 1'b0;


// =============================================================
// B - Motor control chain
// =============================================================
motor_control_u_chain #(
    .CLK_HZ(27000000),
    .PWM_HZ(1000),

    // 27 MHz × 0.002 s = 54,000
    // 正反换向先停止约 2 ms
    .STOP_CYCLES(54000)
)
u_motor_control_chain (

    .clk(clk),
    .rst_n(rst_n),

    // C -> B
    .control_u(control_u_safe),
    .control_valid(control_valid_safe),
    .motor_enable(motor_enable_safe),

    // Protection inputs
    .permit_in(1'b1),
    .fault(1'b0),
    .estop(1'b0),
    .clear_request(motor_clear_request),

    // Internal motor outputs
    .rpwm(motor_rpwm),
    .lpwm(motor_lpwm),

    .r_en(motor_r_en),
    .l_en(motor_l_en),

    // Motor status
    .motor_dir(motor_dir),
    .state_o(motor_state),

    .permit_o(motor_permit),
    .protection_locked(protection_locked),

    .fault_latched(fault_latched),
    .estop_latched(estop_latched),

    .clear_accepted(clear_accepted),

    // Debug
    .timeout_latched(timeout_latched),
    .command_error_latched(command_error_latched),

    .command(motor_command),
    .duty_pct(motor_duty),

    .arm(motor_arm),
    .clear_fault(motor_clear_fault)

);


// =============================================================
// Physical motor pin mapping
//
// T11 -> RPWM
// P11 -> LPWM
// T12 -> EN
//
// T12 在面包板处分两根：
//      -> R_EN
//      -> L_EN
// =============================================================
assign rpwm = motor_rpwm;
assign lpwm = motor_lpwm;
assign en   = motor_r_en & motor_l_en;


// =============================================================
// UART Packet
// =============================================================
sensor_packet u_sensor_packet (

    .clk          (clk),
    .rst_n        (rst_n),

    .send_i       (control_tick),
    .state_i      (system_state),

    .uart_busy_i  (uart_busy),

    .uart_data_o  (uart_data),
    .uart_start_o (uart_start),

    .theta         (theta),
    .theta_dot     (theta_dot),

    .alpha         (alpha),
    .alpha_dot     (alpha_dot)

);


// =============================================================
// UART TX
// =============================================================
uart_tx #(
    .CLK_FREQ_HZ (27_000_000),
    .BAUD_RATE   (115200)
)
u_uart_tx (

    .clk      (clk),
    .rst_n    (rst_n),

    .data_i   (uart_data),
    .start_i  (uart_start),

    .tx_o     (uart_tx_o),
    .busy_o   (uart_busy)

);


// =============================================================
// A - Sensor top
// =============================================================
sensor_top u_sensor (

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