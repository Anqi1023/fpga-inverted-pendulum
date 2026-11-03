module uart_tx #(
    parameter integer CLK_FREQ_HZ = 27_000_000,
    parameter integer BAUD_RATE   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] data_i,
    input  wire       start_i,

    output reg        tx_o,
    output reg        busy_o
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;

    reg [31:0] clk_count;
    reg [3:0]  bit_index;
    reg [9:0]  tx_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_o      <= 1'b1;
            busy_o    <= 1'b0;
            clk_count <= 32'd0;
            bit_index <= 4'd0;
            tx_shift  <= 10'h3FF;
        end
        else begin

            if (!busy_o) begin
                tx_o <= 1'b1;

                if (start_i) begin
                    // UART:
                    // 1 start bit
                    // 8 data bits
                    // 1 stop bit
                    tx_shift <= {1'b1, data_i, 1'b0};
                    busy_o    <= 1'b1;
                    clk_count <= 32'd0;
                    bit_index <= 4'd0;
                end
            end
            else begin

                tx_o <= tx_shift[bit_index];

                if (clk_count >= CLKS_PER_BIT - 1) begin
                    clk_count <= 32'd0;

                    if (bit_index >= 4'd9) begin
                        bit_index <= 4'd0;
                        busy_o    <= 1'b0;
                        tx_o      <= 1'b1;
                    end
                    else begin
                        bit_index <= bit_index + 1'b1;
                    end
                end
                else begin
                    clk_count <= clk_count + 1'b1;
                end

            end
        end
    end

endmodule