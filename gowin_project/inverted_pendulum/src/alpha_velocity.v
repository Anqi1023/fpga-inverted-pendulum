module alpha_velocity (
    input  wire               clk,
    input  wire               rst_n,

    // Kept for compatibility with current top.v
    input  wire               alpha_valid_i,

    // Pendulum angle, unit: mrad
    input  wire signed [31:0] alpha_mrad_i,

    // Pendulum angular velocity, unit: mrad/s
    output reg signed [31:0]  alpha_dot_mrad_s_o
);

    // 27 MHz × 20 ms = 540000 clocks
    localparam [19:0] SAMPLE_COUNT = 20'd539999;

    // One complete revolution ≈ 2*pi rad
    // Unit: mrad
    localparam signed [31:0] TWO_PI_MRAD = 32'sd6283;

    // pi rad
    // Unit: mrad
    localparam signed [31:0] PI_MRAD = 32'sd3142;

    reg [19:0] sample_cnt;

    reg signed [31:0] alpha_prev;
    reg signed [31:0] delta_alpha;

    reg first_sample;


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            sample_cnt         <= 20'd0;
            alpha_prev         <= 32'sd0;
            delta_alpha        <= 32'sd0;
            alpha_dot_mrad_s_o <= 32'sd0;
            first_sample       <= 1'b1;

        end
        else begin

            if (sample_cnt == SAMPLE_COUNT) begin

                sample_cnt <= 20'd0;

                // First sample only establishes the reference
                if (first_sample) begin

                    alpha_prev         <= alpha_mrad_i;
                    alpha_dot_mrad_s_o <= 32'sd0;
                    first_sample       <= 1'b0;

                end
                else begin

                    // Raw angle difference
                    delta_alpha =
                        $signed(alpha_mrad_i) -
                        $signed(alpha_prev);


                    // ============================================
                    // Angle wrap correction
                    //
                    // Example:
                    //
                    // -2871 mrad -> +2864 mrad
                    //
                    // raw delta = +5735 mrad
                    //
                    // corrected:
                    // 5735 - 6283 = -548 mrad
                    //
                    // Therefore it is treated as a normal crossing
                    // of the periodic angle boundary.
                    // ============================================

                    if (delta_alpha > PI_MRAD)
                        delta_alpha =
                            delta_alpha - TWO_PI_MRAD;

                    else if (delta_alpha < -PI_MRAD)
                        delta_alpha =
                            delta_alpha + TWO_PI_MRAD;


                    // Sampling period = 20 ms
                    //
                    // alpha_dot =
                    // delta_alpha / 0.020
                    // =
                    // delta_alpha * 50
                    //
                    // Unit: mrad/s
                    alpha_dot_mrad_s_o <=
                        $signed(delta_alpha) * 32'sd50;


                    // Save current angle
                    alpha_prev <= alpha_mrad_i;

                end

            end
            else begin

                sample_cnt <= sample_cnt + 1'b1;

            end

        end

    end

endmodule