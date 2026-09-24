module uart_tx #(
    parameter CLK_FREQ_HZ = 27_000_000,
    parameter BAUD_RATE   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data_i,
    input  wire       start_i,

    output reg        tx_o,
    output reg        busy_o
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;

    reg [15:0] baud_cnt;
    reg [3:0]  bit_index;
    reg [9:0]  tx_shift;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            tx_o      <= 1'b1;
            busy_o    <= 1'b0;
            baud_cnt  <= 16'd0;
            bit_index <= 4'd0;
            tx_shift  <= 10'h3FF;

        end
        else begin

            // Start a new UART frame
            if (!busy_o) begin

                tx_o <= 1'b1;

                if (start_i) begin

                    // stop + 8 data bits + start
                    tx_shift <= {
                        1'b1,
                        data_i,
                        1'b0
                    };

                    busy_o    <= 1'b1;
                    baud_cnt  <= 16'd0;
                    bit_index <= 4'd0;

                    // Start bit
                    tx_o <= 1'b0;

                end

            end
            else begin

                if (baud_cnt == CLKS_PER_BIT - 1) begin

                    baud_cnt <= 16'd0;

                    if (bit_index == 4'd9) begin

                        busy_o    <= 1'b0;
                        bit_index <= 4'd0;
                        tx_o      <= 1'b1;

                    end
                    else begin

                        bit_index <= bit_index + 1'b1;

                        tx_shift <= {
                            1'b1,
                            tx_shift[9:1]
                        };

                        tx_o <= tx_shift[1];

                    end

                end
                else begin

                    baud_cnt <= baud_cnt + 1'b1;

                end

            end

        end

    end

endmodule