module uart_packet (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       send_i,
    input  wire [2:0] state_i,

    input  wire       uart_busy_i,

    output reg  [7:0] uart_data_o,
    output reg        uart_start_o
);

    localparam IDLE        = 3'd0;
    localparam CALIBRATION = 3'd1;
    localparam READY       = 3'd2;
    localparam RUN         = 3'd3;
    localparam FAULT       = 3'd4;

    localparam PKT_IDLE      = 3'd0;
    localparam PKT_SEND      = 3'd1;
    localparam PKT_WAIT_BUSY = 3'd2;
    localparam PKT_WAIT_DONE = 3'd3;

    reg [2:0] packet_state;

    reg [2:0] state_latch;

    reg [4:0] char_index;
    reg [4:0] message_length;

    // 1 kHz tick 下计数 500 次 = 500 ms
    reg [9:0] report_counter;


    // =========================================================
    // 字符串长度
    // =========================================================
    function [4:0] get_length;
        input [2:0] state;
        begin
            case (state)
                IDLE:        get_length = 5'd12;
                CALIBRATION: get_length = 5'd19;
                READY:       get_length = 5'd13;
                RUN:         get_length = 5'd11;
                FAULT:       get_length = 5'd13;
                default:     get_length = 5'd11;
            endcase
        end
    endfunction


    // =========================================================
    // ASCII 字符生成
    // =========================================================
    function [7:0] get_char;

        input [2:0] state;
        input [4:0] index;

        begin

            case (index)

                5'd0: get_char = "S";
                5'd1: get_char = "T";
                5'd2: get_char = "A";
                5'd3: get_char = "T";
                5'd4: get_char = "E";
                5'd5: get_char = "=";

                default: begin

                    case (state)

                        IDLE: begin
                            case (index)
                                5'd6:  get_char = "I";
                                5'd7:  get_char = "D";
                                5'd8:  get_char = "L";
                                5'd9:  get_char = "E";
                                5'd10: get_char = 8'h0D;
                                5'd11: get_char = 8'h0A;
                                default: get_char = "?";
                            endcase
                        end

                        CALIBRATION: begin
                            case (index)
                                5'd6:  get_char = "C";
                                5'd7:  get_char = "A";
                                5'd8:  get_char = "L";
                                5'd9:  get_char = "I";
                                5'd10: get_char = "B";
                                5'd11: get_char = "R";
                                5'd12: get_char = "A";
                                5'd13: get_char = "T";
                                5'd14: get_char = "I";
                                5'd15: get_char = "O";
                                5'd16: get_char = "N";
                                5'd17: get_char = 8'h0D;
                                5'd18: get_char = 8'h0A;
                                default: get_char = "?";
                            endcase
                        end

                        READY: begin
                            case (index)
                                5'd6:  get_char = "R";
                                5'd7:  get_char = "E";
                                5'd8:  get_char = "A";
                                5'd9:  get_char = "D";
                                5'd10: get_char = "Y";
                                5'd11: get_char = 8'h0D;
                                5'd12: get_char = 8'h0A;
                                default: get_char = "?";
                            endcase
                        end

                        RUN: begin
                            case (index)
                                5'd6:  get_char = "R";
                                5'd7:  get_char = "U";
                                5'd8:  get_char = "N";
                                5'd9:  get_char = 8'h0D;
                                5'd10: get_char = 8'h0A;
                                default: get_char = "?";
                            endcase
                        end

                        FAULT: begin
                            case (index)
                                5'd6:  get_char = "F";
                                5'd7:  get_char = "A";
                                5'd8:  get_char = "U";
                                5'd9:  get_char = "L";
                                5'd10: get_char = "T";
                                5'd11: get_char = 8'h0D;
                                5'd12: get_char = 8'h0A;
                                default: get_char = "?";
                            endcase
                        end

                        default: begin
                            get_char = "?";
                        end

                    endcase
                end
            endcase
        end
    endfunction


    // =========================================================
    // 主状态机
    // =========================================================
    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            packet_state   <= PKT_IDLE;

            uart_data_o    <= 8'd0;
            uart_start_o   <= 1'b0;

            state_latch    <= IDLE;

            char_index     <= 5'd0;
            message_length <= 5'd0;

            report_counter <= 10'd0;

        end
        else begin

            uart_start_o <= 1'b0;

            case (packet_state)

                // =============================================
                // 每 500 ms 启动一次状态发送
                // =============================================
                PKT_IDLE: begin

                    if (send_i) begin

                        if (report_counter >= 10'd499) begin

                            report_counter <= 10'd0;

                            state_latch    <= state_i;
                            char_index     <= 5'd0;
                            message_length <= get_length(state_i);

                            packet_state   <= PKT_SEND;

                        end
                        else begin

                            report_counter <= report_counter + 1'b1;

                        end
                    end

                end


                // =============================================
                // 发送字符
                // =============================================
                PKT_SEND: begin

                    if (!uart_busy_i) begin

                        uart_data_o <= get_char(
                            state_latch,
                            char_index
                        );

                        uart_start_o <= 1'b1;

                        packet_state <= PKT_WAIT_BUSY;

                    end

                end


                // 等 UART 开始发送
                PKT_WAIT_BUSY: begin

                    if (uart_busy_i)
                        packet_state <= PKT_WAIT_DONE;

                end


                // 等当前字符发送完成
                PKT_WAIT_DONE: begin

                    if (!uart_busy_i) begin

                        if (char_index >= message_length - 1'b1) begin

                            packet_state <= PKT_IDLE;

                        end
                        else begin

                            char_index   <= char_index + 1'b1;
                            packet_state <= PKT_SEND;

                        end

                    end

                end


                default: begin

                    packet_state <= PKT_IDLE;

                end

            endcase

        end
    end

endmodule