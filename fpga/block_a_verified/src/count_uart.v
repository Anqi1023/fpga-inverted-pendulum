module count_uart (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] count_i,

    output reg  [7:0]         uart_data,
    output reg                uart_start,
    input  wire               uart_busy
);

    // 27 MHz，500ms发送一次
    localparam TIMER_MAX = 24'd13_499_999;

    reg [23:0] timer;

    reg signed [31:0] saved_count;
    reg [31:0] temp;

    reg negative;

    // 最长：
    // COUNT=-0000000000\r\n
    // 共19字符
    reg [7:0] message [0:18];

    reg [4:0] msg_len;
    reg [4:0] msg_index;

    reg [3:0] d0;
    reg [3:0] d1;
    reg [3:0] d2;
    reg [3:0] d3;
    reg [3:0] d4;
    reg [3:0] d5;
    reg [3:0] d6;
    reg [3:0] d7;
    reg [3:0] d8;
    reg [3:0] d9;


    // =========================
    // State machine
    // =========================

    localparam WAIT_TIME = 3'd0;
    localparam PREPARE   = 3'd1;
    localparam START_TX  = 3'd2;
    localparam WAIT_BUSY = 3'd3;
    localparam WAIT_DONE = 3'd4;

    reg [2:0] state;


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            timer       <= 24'd0;
            saved_count <= 32'sd0;
            temp        <= 32'd0;

            negative    <= 1'b0;

            uart_data   <= 8'h00;
            uart_start  <= 1'b0;

            msg_len     <= 5'd0;
            msg_index   <= 5'd0;

            state       <= WAIT_TIME;

        end else begin

            // 默认不启动UART
            uart_start <= 1'b0;


            case (state)

                // =====================================
                // 等待500ms
                // =====================================

                WAIT_TIME: begin

                    if (timer >= TIMER_MAX) begin

                        timer       <= 24'd0;
                        saved_count <= count_i;

                        state <= PREPARE;

                    end else begin

                        timer <= timer + 1'b1;

                    end

                end


                // =====================================
                // 生成ASCII字符串
                // =====================================

                PREPARE: begin

                    if (saved_count < 0) begin

                        negative = 1'b1;
                        temp = -saved_count;

                    end else begin

                        negative = 1'b0;
                        temp = saved_count;

                    end


                    // 十位十进制数字
                    d9 = (temp / 1_000_000_000) % 10;
                    d8 = (temp /   100_000_000) % 10;
                    d7 = (temp /    10_000_000) % 10;
                    d6 = (temp /     1_000_000) % 10;
                    d5 = (temp /       100_000) % 10;
                    d4 = (temp /        10_000) % 10;
                    d3 = (temp /         1_000) % 10;
                    d2 = (temp /           100) % 10;
                    d1 = (temp /            10) % 10;
                    d0 =  temp                  % 10;


                    // COUNT=
                    message[0] = "C";
                    message[1] = "O";
                    message[2] = "U";
                    message[3] = "N";
                    message[4] = "T";
                    message[5] = "=";


                    if (negative) begin

                        // COUNT=-0000000000\r\n

                        message[6] = "-";

                        message[7]  = 8'd48 + d9;
                        message[8]  = 8'd48 + d8;
                        message[9]  = 8'd48 + d7;
                        message[10] = 8'd48 + d6;
                        message[11] = 8'd48 + d5;
                        message[12] = 8'd48 + d4;
                        message[13] = 8'd48 + d3;
                        message[14] = 8'd48 + d2;
                        message[15] = 8'd48 + d1;
                        message[16] = 8'd48 + d0;

                        // CR + LF
                        message[17] = 8'h0D;
                        message[18] = 8'h0A;

                        msg_len = 5'd19;

                    end else begin

                        // COUNT=0000000000\r\n

                        message[6]  = 8'd48 + d9;
                        message[7]  = 8'd48 + d8;
                        message[8]  = 8'd48 + d7;
                        message[9]  = 8'd48 + d6;
                        message[10] = 8'd48 + d5;
                        message[11] = 8'd48 + d4;
                        message[12] = 8'd48 + d3;
                        message[13] = 8'd48 + d2;
                        message[14] = 8'd48 + d1;
                        message[15] = 8'd48 + d0;

                        // CR + LF
                        message[16] = 8'h0D;
                        message[17] = 8'h0A;

                        msg_len = 5'd18;

                    end


                    msg_index <= 5'd0;

                    state <= START_TX;

                end


                // =====================================
                // 启动一个字符
                // =====================================

                START_TX: begin

                    if (!uart_busy) begin

                        uart_data  <= message[msg_index];
                        uart_start <= 1'b1;

                        state <= WAIT_BUSY;

                    end

                end


                // =====================================
                // 确认UART进入BUSY
                // =====================================

                WAIT_BUSY: begin

                    if (uart_busy) begin

                        state <= WAIT_DONE;

                    end

                end


                // =====================================
                // 等待字符发送完成
                // =====================================

                WAIT_DONE: begin

                    if (!uart_busy) begin

                        if (msg_index == msg_len - 1'b1) begin

                            state <= WAIT_TIME;

                        end else begin

                            msg_index <= msg_index + 1'b1;

                            state <= START_TX;

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