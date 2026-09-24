module angle_sensor (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] adc_raw,
    input  wire               adc_valid,

    // Pendulum angle alpha
    // Unit: milliradian (mrad)
    //
    // Direction:
    // right  -> positive
    // upright -> zero
    // left   -> negative
    output reg signed [31:0] alpha_mrad
);

    // ============================================================
    // Final three-point calibration
    //
    // Measured:
    //
    // Right horizontal:
    //     ADC ≈ 0x0705 = 1797
    //     alpha = +1571 mrad
    //
    // Upright:
    //     ADC ≈ 0x55F5 = 22005
    //     alpha = 0 mrad
    //
    // Left horizontal:
    //     ADC ≈ 0x3BD5 = 15317
    //     alpha = -1571 mrad
    //
    // Left side:
    //     22005 - 15317 = 6688 counts
    //
    // Effective ADC period:
    //     26896 counts
    //
    // Scale:
    //     6688 counts = 1571 mrad
    // ============================================================

    localparam signed [31:0] ADC_ZERO   = 32'sd22005;
    localparam signed [31:0] ADC_PERIOD = 32'sd26896;

    localparam signed [31:0] SCALE_NUM  = 32'sd1571;
    localparam signed [31:0] SCALE_DEN  = 32'sd6688;


    reg signed [31:0] adc_value;
    reg signed [31:0] adc_unwrapped;
    reg signed [31:0] adc_delta;

    reg signed [63:0] mult_temp;


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            adc_value     <= 32'sd0;
            adc_unwrapped <= 32'sd0;
            adc_delta     <= 32'sd0;
            mult_temp     <= 64'sd0;

            alpha_mrad    <= 32'sd0;

        end
        else if (adc_valid) begin

            // ----------------------------------------------------
            // ADS1115 raw value -> positive integer
            // ----------------------------------------------------

            adc_value =
                $signed({1'b0, adc_raw[14:0]});


            // ----------------------------------------------------
            // ADC unwrap
            //
            // Right side crosses the potentiometer wrap point.
            //
            // Values in the low region (07xx etc.) are actually
            // the continuation above the high ADC region.
            // ----------------------------------------------------

            if ($unsigned(adc_raw) < 16'h2000) begin

                adc_unwrapped =
                    adc_value + ADC_PERIOD;

            end
            else begin

                adc_unwrapped =
                    adc_value;

            end


            // ----------------------------------------------------
            // Relative to upright position
            // ----------------------------------------------------

            adc_delta =
                adc_unwrapped - ADC_ZERO;


            // ----------------------------------------------------
            // Convert ADC counts -> milliradians
            //
            // 6688 counts = 1571 mrad
            // ----------------------------------------------------

            mult_temp =
                $signed(adc_delta) * 64'sd1571;

            alpha_mrad <=
                mult_temp / 64'sd6688;

        end

    end

endmodule