module calibration (
    input  wire               clk,
    input  wire               rst_n,

    // Raw state values from sensor modules
    input  wire signed [31:0] theta_raw_i,
    input  wire signed [31:0] alpha_raw_i,

    // Calibrated state values
    output wire signed [31:0] theta_cal_o,
    output wire signed [31:0] alpha_cal_o,

    // Goes high after calibration is complete
    output reg                calibrated_o
);

    // ------------------------------------------------------
    // Zero offsets
    // ------------------------------------------------------
    reg signed [31:0] theta_zero;
    reg signed [31:0] alpha_zero;

    // Wait for sensor values to settle after reset.
    // 27 MHz clock:
    // 13,500,000 clocks = 0.5 s
    localparam [23:0] CAL_WAIT_COUNT = 24'd13500000;

    reg [23:0] wait_cnt;

    // ------------------------------------------------------
    // Calibration
    // ------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            theta_zero   <= 32'sd0;
            alpha_zero   <= 32'sd0;
            wait_cnt     <= 24'd0;
            calibrated_o <= 1'b0;
        end
        else begin

            if (!calibrated_o) begin

                if (wait_cnt == CAL_WAIT_COUNT - 1'b1) begin

                    // Record current position as zero position
                    theta_zero   <= theta_raw_i;
                    alpha_zero   <= alpha_raw_i;

                    calibrated_o <= 1'b1;

                end
                else begin
                    wait_cnt <= wait_cnt + 1'b1;
                end

            end

        end
    end

    // ------------------------------------------------------
    // Zero-offset compensation
    // ------------------------------------------------------
    assign theta_cal_o =
        $signed(theta_raw_i) - $signed(theta_zero);

    assign alpha_cal_o =
        $signed(alpha_raw_i) - $signed(alpha_zero);

endmodule