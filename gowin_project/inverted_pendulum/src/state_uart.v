module state_uart (
    input  wire               clk,
    input  wire               rst_n,

    input  wire signed [31:0] theta_i,
    input  wire signed [31:0] theta_dot_i,
    input  wire signed [31:0] alpha_i,
    input  wire signed [31:0] alpha_dot_i,

    output reg  [7:0]         uart_data,
    output reg                uart_start,
    input  wire               uart_busy
);

    localparam [23:0] SEND_INTERVAL = 24'd13500000;

    reg [23:0] interval_cnt;

    reg [31:0] theta_latch;
    reg [31:0] theta_dot_latch;
    reg [31:0] alpha_latch;
    reg [31:0] alpha_dot_latch;

    reg [5:0] char_index;

    reg sending;
    reg wait_busy_high;
    reg wait_busy_low;


    // Convert 4-bit value to ASCII HEX
    function [7:0] hex_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 4'd10)
                hex_ascii = 8'h30 + nibble;
            else
                hex_ascii = 8'h41 + (nibble - 4'd10);
        end
    endfunction


    // Output format:
    //
    // T=XXXXXXXX TD=XXXXXXXX A=XXXXXXXX AD=XXXXXXXX
    //
    // All values are 32-bit hexadecimal.
    function [7:0] get_char;
        input [5:0] idx;
        begin

            case (idx)

                // T=
                6'd0:  get_char = "T";
                6'd1:  get_char = "=";

                6'd2:  get_char = hex_ascii(theta_latch[31:28]);
                6'd3:  get_char = hex_ascii(theta_latch[27:24]);
                6'd4:  get_char = hex_ascii(theta_latch[23:20]);
                6'd5:  get_char = hex_ascii(theta_latch[19:16]);
                6'd6:  get_char = hex_ascii(theta_latch[15:12]);
                6'd7:  get_char = hex_ascii(theta_latch[11:8]);
                6'd8:  get_char = hex_ascii(theta_latch[7:4]);
                6'd9:  get_char = hex_ascii(theta_latch[3:0]);

                6'd10: get_char = " ";

                // TD=
                6'd11: get_char = "T";
                6'd12: get_char = "D";
                6'd13: get_char = "=";

                6'd14: get_char = hex_ascii(theta_dot_latch[31:28]);
                6'd15: get_char = hex_ascii(theta_dot_latch[27:24]);
                6'd16: get_char = hex_ascii(theta_dot_latch[23:20]);
                6'd17: get_char = hex_ascii(theta_dot_latch[19:16]);
                6'd18: get_char = hex_ascii(theta_dot_latch[15:12]);
                6'd19: get_char = hex_ascii(theta_dot_latch[11:8]);
                6'd20: get_char = hex_ascii(theta_dot_latch[7:4]);
                6'd21: get_char = hex_ascii(theta_dot_latch[3:0]);

                6'd22: get_char = " ";

                // A=
                6'd23: get_char = "A";
                6'd24: get_char = "=";

                6'd25: get_char = hex_ascii(alpha_latch[31:28]);
                6'd26: get_char = hex_ascii(alpha_latch[27:24]);
                6'd27: get_char = hex_ascii(alpha_latch[23:20]);
                6'd28: get_char = hex_ascii(alpha_latch[19:16]);
                6'd29: get_char = hex_ascii(alpha_latch[15:12]);
                6'd30: get_char = hex_ascii(alpha_latch[11:8]);
                6'd31: get_char = hex_ascii(alpha_latch[7:4]);
                6'd32: get_char = hex_ascii(alpha_latch[3:0]);

                6'd33: get_char = " ";

                // AD=
                6'd34: get_char = "A";
                6'd35: get_char = "D";
                6'd36: get_char = "=";

                6'd37: get_char = hex_ascii(alpha_dot_latch[31:28]);
                6'd38: get_char = hex_ascii(alpha_dot_latch[27:24]);
                6'd39: get_char = hex_ascii(alpha_dot_latch[23:20]);
                6'd40: get_char = hex_ascii(alpha_dot_latch[19:16]);
                6'd41: get_char = hex_ascii(alpha_dot_latch[15:12]);
                6'd42: get_char = hex_ascii(alpha_dot_latch[11:8]);
                6'd43: get_char = hex_ascii(alpha_dot_latch[7:4]);
                6'd44: get_char = hex_ascii(alpha_dot_latch[3:0]);

                6'd45: get_char = 8'h0D;
                6'd46: get_char = 8'h0A;

                default: get_char = 8'h20;

            endcase

        end
    endfunction


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            interval_cnt    <= 24'd0;

            theta_latch     <= 32'd0;
            theta_dot_latch <= 32'd0;
            alpha_latch     <= 32'd0;
            alpha_dot_latch <= 32'd0;

            char_index      <= 6'd0;

            sending         <= 1'b0;
            wait_busy_high  <= 1'b0;
            wait_busy_low   <= 1'b0;

            uart_data       <= 8'd0;
            uart_start      <= 1'b0;

        end
        else begin

            uart_start <= 1'b0;


            // Send one line every 0.5 s
            if (!sending) begin

                if (interval_cnt == SEND_INTERVAL - 1'b1) begin

                    interval_cnt <= 24'd0;

                    theta_latch     <= theta_i;
                    theta_dot_latch <= theta_dot_i;
                    alpha_latch     <= alpha_i;
                    alpha_dot_latch <= alpha_dot_i;

                    char_index      <= 6'd0;
                    sending         <= 1'b1;

                    wait_busy_high  <= 1'b0;
                    wait_busy_low   <= 1'b0;

                end
                else begin

                    interval_cnt <= interval_cnt + 1'b1;

                end

            end


            // UART handshake
            if (sending) begin

                if (!wait_busy_high && !wait_busy_low) begin

                    if (!uart_busy) begin

                        uart_data      <= get_char(char_index);
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

                        if (char_index == 6'd46) begin

                            char_index <= 6'd0;
                            sending    <= 1'b0;

                        end
                        else begin

                            char_index <= char_index + 1'b1;

                        end

                    end

                end

            end

        end

    end

endmodule