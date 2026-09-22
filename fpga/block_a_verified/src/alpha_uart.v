module alpha_uart (
    input  wire               clk,
    input  wire               rst_n,

    input  wire signed [15:0] adc_raw_i,
    input  wire signed [31:0] alpha_mrad_i,
    input  wire signed [31:0] alpha_dot_i,

    output reg  [7:0]         uart_data,
    output reg                uart_start,
    input  wire               uart_busy
);

    // Send one line every 500 ms
    localparam integer SEND_PERIOD = 13_500_000;

    reg [23:0] timer;
    reg [5:0]  state;

    reg [15:0] adc_latched;

    reg [31:0] alpha_abs;
    reg        alpha_negative;

    reg [31:0] adot_abs;
    reg        adot_negative;

    reg wait_busy_high;
    reg wait_busy_low;


    // ============================================================
    // UART states
    //
    // Output:
    //
    // ADC=55F5 ALPHA=+00000 ADOT=+000000
    // ============================================================

    localparam S_IDLE  = 6'd0;

    localparam S_A1    = 6'd1;
    localparam S_D1    = 6'd2;
    localparam S_C1    = 6'd3;
    localparam S_EQ1   = 6'd4;

    localparam S_H3    = 6'd5;
    localparam S_H2    = 6'd6;
    localparam S_H1    = 6'd7;
    localparam S_H0    = 6'd8;

    localparam S_SP1   = 6'd9;

    localparam S_A2    = 6'd10;
    localparam S_L     = 6'd11;
    localparam S_P     = 6'd12;
    localparam S_H     = 6'd13;
    localparam S_A3    = 6'd14;
    localparam S_EQ2   = 6'd15;

    localparam S_ASIGN = 6'd16;
    localparam S_AN4   = 6'd17;
    localparam S_AN3   = 6'd18;
    localparam S_AN2   = 6'd19;
    localparam S_AN1   = 6'd20;
    localparam S_AN0   = 6'd21;

    localparam S_SP2   = 6'd22;

    localparam S_AD_A  = 6'd23;
    localparam S_AD_D  = 6'd24;
    localparam S_AD_O  = 6'd25;
    localparam S_AD_T  = 6'd26;
    localparam S_AD_EQ = 6'd27;

    localparam S_DSIGN = 6'd28;

    localparam S_DN5   = 6'd29;
    localparam S_DN4   = 6'd30;
    localparam S_DN3   = 6'd31;
    localparam S_DN2   = 6'd32;
    localparam S_DN1   = 6'd33;
    localparam S_DN0   = 6'd34;

    localparam S_CR    = 6'd35;
    localparam S_LF    = 6'd36;


    // ============================================================
    // HEX -> ASCII
    // ============================================================

    function [7:0] hex_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 10)
                hex_ascii = "0" + nibble;
            else
                hex_ascii = "A" + (nibble - 10);
        end
    endfunction


    // ============================================================
    // Decimal -> ASCII
    // ============================================================

    function [7:0] dec_ascii;
        input [3:0] digit;
        begin
            dec_ascii = "0" + digit;
        end
    endfunction


    // ============================================================
    // Character generator
    // ============================================================

    function [7:0] get_char;
        input [5:0] s;

        reg [31:0] av;
        reg [31:0] dv;

        begin

            av = alpha_abs;
            dv = adot_abs;

            case (s)

                // ADC=
                S_A1:    get_char = "A";
                S_D1:    get_char = "D";
                S_C1:    get_char = "C";
                S_EQ1:   get_char = "=";

                S_H3:    get_char = hex_ascii(adc_latched[15:12]);
                S_H2:    get_char = hex_ascii(adc_latched[11:8]);
                S_H1:    get_char = hex_ascii(adc_latched[7:4]);
                S_H0:    get_char = hex_ascii(adc_latched[3:0]);

                S_SP1:   get_char = " ";

                // ALPHA=
                S_A2:    get_char = "A";
                S_L:     get_char = "L";
                S_P:     get_char = "P";
                S_H:     get_char = "H";
                S_A3:    get_char = "A";
                S_EQ2:   get_char = "=";

                S_ASIGN:
                    get_char = alpha_negative ? "-" : "+";

                S_AN4:
                    get_char = dec_ascii((av / 10000) % 10);

                S_AN3:
                    get_char = dec_ascii((av / 1000) % 10);

                S_AN2:
                    get_char = dec_ascii((av / 100) % 10);

                S_AN1:
                    get_char = dec_ascii((av / 10) % 10);

                S_AN0:
                    get_char = dec_ascii(av % 10);

                S_SP2:
                    get_char = " ";

                // ADOT=
                S_AD_A:  get_char = "A";
                S_AD_D:  get_char = "D";
                S_AD_O:  get_char = "O";
                S_AD_T:  get_char = "T";
                S_AD_EQ: get_char = "=";

                S_DSIGN:
                    get_char = adot_negative ? "-" : "+";

                // 6 digits for angular velocity
                S_DN5:
                    get_char = dec_ascii((dv / 100000) % 10);

                S_DN4:
                    get_char = dec_ascii((dv / 10000) % 10);

                S_DN3:
                    get_char = dec_ascii((dv / 1000) % 10);

                S_DN2:
                    get_char = dec_ascii((dv / 100) % 10);

                S_DN1:
                    get_char = dec_ascii((dv / 10) % 10);

                S_DN0:
                    get_char = dec_ascii(dv % 10);

                S_CR:
                    get_char = 8'h0D;

                S_LF:
                    get_char = 8'h0A;

                default:
                    get_char = 8'h20;

            endcase
        end
    endfunction


    // ============================================================
    // Main FSM
    // ============================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            timer          <= 24'd0;
            state          <= S_IDLE;

            uart_data      <= 8'd0;
            uart_start     <= 1'b0;

            adc_latched    <= 16'd0;

            alpha_abs      <= 32'd0;
            alpha_negative <= 1'b0;

            adot_abs       <= 32'd0;
            adot_negative  <= 1'b0;

            wait_busy_high <= 1'b0;
            wait_busy_low  <= 1'b0;

        end
        else begin

            uart_start <= 1'b0;


            // ====================================================
            // Capture values every 500 ms
            // ====================================================

            if (state == S_IDLE) begin

                if (timer >= SEND_PERIOD - 1) begin

                    timer <= 24'd0;

                    adc_latched <= adc_raw_i;


                    // --------------------------------------------
                    // alpha
                    // --------------------------------------------

                    if (alpha_mrad_i < 0) begin

                        alpha_negative <= 1'b1;
                        alpha_abs      <= -alpha_mrad_i;

                    end
                    else begin

                        alpha_negative <= 1'b0;
                        alpha_abs      <= alpha_mrad_i;

                    end


                    // --------------------------------------------
                    // alpha_dot
                    // --------------------------------------------

                    if (alpha_dot_i < 0) begin

                        adot_negative <= 1'b1;
                        adot_abs      <= -alpha_dot_i;

                    end
                    else begin

                        adot_negative <= 1'b0;
                        adot_abs      <= alpha_dot_i;

                    end


                    state <= S_A1;

                end
                else begin

                    timer <= timer + 1'b1;

                end

            end


            // ====================================================
            // UART send
            // ====================================================

            else begin

                if (!wait_busy_high && !wait_busy_low) begin

                    if (!uart_busy) begin

                        uart_data      <= get_char(state);
                        uart_start     <= 1'b1;
                        wait_busy_high <= 1'b1;

                    end

                end
                else if (wait_busy_high) begin

                    if (uart_busy) begin

                        wait_busy_high <= 1'b0;
                        wait_busy_low  <= 1'b1;

                    end

                end
                else if (wait_busy_low) begin

                    if (!uart_busy) begin

                        wait_busy_low <= 1'b0;

                        if (state == S_LF)
                            state <= S_IDLE;
                        else
                            state <= state + 1'b1;

                    end

                end

            end

        end

    end

endmodule