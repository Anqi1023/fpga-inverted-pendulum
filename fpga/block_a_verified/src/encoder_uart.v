module encoder_uart (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] theta_dot_i,

    output reg  [7:0]         uart_data,
    output reg                uart_start,
    input  wire               uart_busy
);

    // =========================================================
    // 500 ms refresh
    // 27 MHz * 0.5 s = 13,500,000
    // =========================================================

    localparam TIMER_MAX = 24'd13_499_999;

    reg [23:0] timer;

    reg [31:0] saved_theta_dot;

    reg [4:0] char_index;

    localparam WAIT_TIME = 3'd0;
    localparam LOAD_CHAR = 3'd1;
    localparam START_TX  = 3'd2;
    localparam WAIT_BUSY = 3'd3;
    localparam WAIT_DONE = 3'd4;

    reg [2:0] state;


    // =========================================================
    // HEX -> ASCII
    // =========================================================

    function [7:0] hex_ascii;

        input [3:0] hex;

        begin

            if (hex < 4'd10)
                hex_ascii = 8'd48 + hex;       // 0-9
            else
                hex_ascii = 8'd65 + hex - 10;  // A-F

        end

    endfunction


    // =========================================================
    // Main
    //
    // Output:
    //
    // TDOT=12345678\r\n
    //
    // total = 15 characters
    // =========================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            timer           <= 24'd0;
            saved_theta_dot <= 32'd0;

            char_index      <= 5'd0;

            uart_data       <= 8'h00;
            uart_start      <= 1'b0;

            state           <= WAIT_TIME;

        end else begin

            uart_start <= 1'b0;

            case (state)

                // =============================================
                // Wait 500 ms
                // =============================================

                WAIT_TIME: begin

                    if (timer >= TIMER_MAX) begin

                        timer <= 24'd0;

                        saved_theta_dot <= theta_dot_i;

                        char_index <= 5'd0;

                        state <= LOAD_CHAR;

                    end else begin

                        timer <= timer + 1'b1;

                    end

                end


                // =============================================
                // Select character
                // =============================================

                LOAD_CHAR: begin

                    case (char_index)

                        5'd0:
                            uart_data <= "T";

                        5'd1:
                            uart_data <= "D";

                        5'd2:
                            uart_data <= "O";

                        5'd3:
                            uart_data <= "T";

                        5'd4:
                            uart_data <= "=";


                        // 32-bit theta_dot
                        // 8 HEX digits

                        5'd5:
                            uart_data <=
                                hex_ascii(saved_theta_dot[31:28]);

                        5'd6:
                            uart_data <=
                                hex_ascii(saved_theta_dot[27:24]);

                        5'd7:
                            uart_data <=
                                hex_ascii(saved_theta_dot[23:20]);

                        5'd8:
                            uart_data <=
                                hex_ascii(saved_theta_dot[19:16]);

                        5'd9:
                            uart_data <=
                                hex_ascii(saved_theta_dot[15:12]);

                        5'd10:
                            uart_data <=
                                hex_ascii(saved_theta_dot[11:8]);

                        5'd11:
                            uart_data <=
                                hex_ascii(saved_theta_dot[7:4]);

                        5'd12:
                            uart_data <=
                                hex_ascii(saved_theta_dot[3:0]);


                        // CR
                        5'd13:
                            uart_data <= 8'h0D;

                        // LF
                        5'd14:
                            uart_data <= 8'h0A;

                        default:
                            uart_data <= 8'h00;

                    endcase

                    state <= START_TX;

                end


                // =============================================
                // Start UART
                // =============================================

                START_TX: begin

                    if (!uart_busy) begin

                        uart_start <= 1'b1;

                        state <= WAIT_BUSY;

                    end

                end


                // =============================================
                // Wait until UART accepts byte
                // =============================================

                WAIT_BUSY: begin

                    if (uart_busy)
                        state <= WAIT_DONE;

                end


                // =============================================
                // Wait until byte transmission completes
                // =============================================

                WAIT_DONE: begin

                    if (!uart_busy) begin

                        if (char_index == 5'd14) begin

                            state <= WAIT_TIME;

                        end else begin

                            char_index <= char_index + 1'b1;

                            state <= LOAD_CHAR;

                        end

                    end

                end


                default: begin

                    state <= WAIT_TIME;

                end

            endcase

        end

    end

endmodule