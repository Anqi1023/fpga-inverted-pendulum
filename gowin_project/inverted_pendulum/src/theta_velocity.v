module theta_velocity (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] theta_mrad_i,
    output reg signed [31:0]  theta_dot_mrad_s_o
);

    // 27 MHz clock
    // 20 ms sample period
    // 27,000,000 * 0.020 = 540,000 clocks

    localparam integer SAMPLE_COUNT = 540_000;

    reg [19:0] sample_cnt;
    reg signed [31:0] theta_prev;
    reg first_sample;

    wire signed [31:0] delta_theta;
    wire signed [63:0] delta_ext;
    wire signed [63:0] velocity_temp;

    // Current angle - previous angle
    assign delta_theta =
        theta_mrad_i - theta_prev;

    // Sign extend 32 bit -> 64 bit
    assign delta_ext =
        {{32{delta_theta[31]}}, delta_theta};

    // theta_dot = delta_theta * 50
    // unit = mrad/s
    assign velocity_temp =
        delta_ext * 64'sd50;


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            sample_cnt         <= 20'd0;
            theta_prev         <= 32'sd0;
            theta_dot_mrad_s_o <= 32'sd0;
            first_sample       <= 1'b1;

        end else begin

            if (sample_cnt == SAMPLE_COUNT - 1) begin

                sample_cnt <= 20'd0;

                if (first_sample) begin

                    // First sample:
                    // only save current angle
                    theta_prev         <= theta_mrad_i;
                    theta_dot_mrad_s_o <= 32'sd0;
                    first_sample       <= 1'b0;

                end else begin

                    // Update velocity every 20 ms
                    theta_dot_mrad_s_o <= velocity_temp[31:0];

                    // Save current angle
                    theta_prev <= theta_mrad_i;

                end

            end else begin

                sample_cnt <= sample_cnt + 1'b1;

            end

        end

    end

endmodule