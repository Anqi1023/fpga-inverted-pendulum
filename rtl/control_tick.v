module control_tick #(
    parameter CLK_FREQ_HZ = 27_000_000,
    parameter CONTROL_FREQ_HZ = 1_000
)(
    input  wire clk,
    input  wire rst_n,
    output reg  tick_o
);

    localparam integer CNT_MAX =
        CLK_FREQ_HZ / CONTROL_FREQ_HZ - 1;

    reg [31:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 32'd0;
            tick_o  <= 1'b0;
        end
        else begin
            if (counter >= CNT_MAX) begin
                counter <= 32'd0;
                tick_o  <= 1'b1;
            end
            else begin
                counter <= counter + 1'b1;
                tick_o  <= 1'b0;
            end
        end
    end

endmodule