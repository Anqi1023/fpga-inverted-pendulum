module adc_uart (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] adc_raw_i,

    output reg  [7:0]         uart_data,
    output reg                uart_start,
    input  wire               uart_busy
);

    // ============================================================
    // Send ADC value every 500 ms
    // 27 MHz * 0.5 s = 13,500,000 clocks
    // ============================================================

    localparam TIMER_MAX = 24'd13_499_999;

    reg [23:0] timer;
    reg [15:0] saved_adc;
    reg [3:0]  char_index;

    localparam WAIT_TIME = 3'd0;
    localparam LOAD_CHAR = 3'd1;
    localparam START_TX  = 3'd2;
    localparam WAIT_BUSY = 3'd3;
    localparam WAIT_DONE = 3'd4;

    reg [2:0] state;


    // ============================================================
    // Convert 4-bit HEX to ASCII
    // ============================================================

    function [7:0] hex_ascii;
        input [3:0] hex;
        begin
            if (hex < 4'd10)
                hex_ascii = 8'd48 + hex;       // '0' ~ '9'
            else
                hex_ascii = 8'd65 + hex - 10;  // 'A' ~ 'F'
        end
    endfunction


    // ============================================================
    // UART output state machine
    //
    // Output:
    //
    // ADC=XXXX
    //
    // ============================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            timer      <= 24'd0;
            saved_adc  <= 16'd0;
            char_index <= 4'd0;

            uart_data  <= 8'h00;
            uart_start <= 1'b0;

            state      <= WAIT_TIME;

        end else begin

            uart_start <= 1'b0;

            case (state)

                // ------------------------------------------------
                // Wait 500 ms
                // ------------------------------------------------
                WAIT_TIME: begin

                    if (timer >= TIMER_MAX) begin

                        timer      <= 24'd0;

                        // Save one ADC sample
                        saved_adc  <= adc_raw_i;

                        char_index <= 4'd0;

                        state      <= LOAD_CHAR;

                    end else begin

                        timer <= timer + 1'b1;

                    end
                end


                // ------------------------------------------------
                // Prepare one character
                // ------------------------------------------------
                LOAD_CHAR: begin

                    case (char_index)

                        4'd0: uart_data <= "A";
                        4'd1: uart_data <= "D";
                        4'd2: uart_data <= "C";
                        4'd3: uart_data <= "=";

                        4'd4: uart_data <= hex_ascii(saved_adc[15:12]);
                        4'd5: uart_data <= hex_ascii(saved_adc[11:8]);
                        4'd6: uart_data <= hex_ascii(saved_adc[7:4]);
                        4'd7: uart_data <= hex_ascii(saved_adc[3:0]);

                        4'd8: uart_data <= 8'h0D; // CR
                        4'd9: uart_data <= 8'h0A; // LF

                        default:
                            uart_data <= 8'h00;

                    endcase

                    state <= START_TX;

                end


                // ------------------------------------------------
                // Start UART transmission
                // ------------------------------------------------
                START_TX: begin

                    if (!uart_busy) begin

                        uart_start <= 1'b1;
                        state      <= WAIT_BUSY;

                    end
                end


                // ------------------------------------------------
                // Wait until uart_tx becomes busy
                // ------------------------------------------------
                WAIT_BUSY: begin

                    if (uart_busy)
                        state <= WAIT_DONE;

                end


                // ------------------------------------------------
                // Wait until character transmission finishes
                // ------------------------------------------------
                WAIT_DONE: begin

                    if (!uart_busy) begin

                        if (char_index == 4'd9) begin

                            state <= WAIT_TIME;

                        end else begin

                            char_index <= char_index + 1'b1;
                            state      <= LOAD_CHAR;

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