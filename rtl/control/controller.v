`default_nettype none

module controller (

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
    output reg                motor_enable

);


always @(posedge clk) begin

    if (!rst_n) begin

        control_u     <= 16'sd0;
        control_valid <= 1'b0;
        motor_enable  <= 1'b0;

    end

    else begin

        // 默认关闭
        control_valid <= 1'b0;


        if (control_tick) begin

            // 强制测试：
            // +100 = 正向10% PWM

if (theta > 32'sd5719) begin

    // theta为正，需要负方向修正
    control_u <= -16'sd100;

end

else if (theta < -32'sd5719) begin

    // theta为负，需要正方向修正
    control_u <= 16'sd100;

end

else begin

    control_u <= 16'sd0;

end


control_valid <= 1'b1;
motor_enable  <= 1'b1;
        end

    end

end


endmodule


`default_nettype wire