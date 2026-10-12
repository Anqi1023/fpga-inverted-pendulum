module control_u_source #(
    // 单位：0.1% PWM
    // 100 = 10.0%
    // -100 = -10.0%
    parameter integer TEST_U = 0
)(
    input  wire clk,
    input  wire rst_n,

    // C的控制更新节拍
    input  wire control_tick,

    // C的系统状态
    input  wire [2:0] system_state,

    // A传感器数据有效
    input  wire sensor_valid,

    // 给B
    output reg signed [15:0] control_u,
    output reg               control_valid,
    output reg               motor_enable
);

    // 与当前 control_fsm.v 保持一致
    localparam RUN = 3'd3;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            control_u     <= 16'sd0;
            control_valid <= 1'b0;
            motor_enable  <= 1'b0;

        end
        else begin

            // valid默认只保持一个clk
            control_valid <= 1'b0;

            // 只有RUN并且传感器有效时，才允许电机
            if ((system_state == RUN) && sensor_valid) begin

                motor_enable <= 1'b1;

                if (control_tick) begin

                    // 安全限幅：最大30%
                    if (TEST_U > 300)
                        control_u <= 16'sd300;

                    else if (TEST_U < -300)
                        control_u <= -16'sd300;

                    else
                        control_u <= TEST_U;

                    control_valid <= 1'b1;

                end

            end
            else begin

                motor_enable <= 1'b0;
                control_u    <= 16'sd0;

            end

        end

    end

endmodule