`default_nettype none
// Tang Primer 20K Dock: 27 MHz clock, active-low LED4 on L14.
module led_blink #(
    parameter integer HALF_PERIOD_CYCLES = 27000000
) (
    input wire clk,
    output wire led_n
);
    // Configuration-time initialization; no unverified reset pin required.
    localparam integer COUNT_WIDTH = (HALF_PERIOD_CYCLES > 1) ?
                                     $clog2(HALF_PERIOD_CYCLES) : 1;
    reg [COUNT_WIDTH-1:0] count = 0;
    reg led_on = 1'b0;
    always @(posedge clk) begin
        if (count == HALF_PERIOD_CYCLES - 1) begin
            count <= 0;
            led_on <= ~led_on;
        end else begin
            count <= count + 1'b1;
        end
    end
    assign led_n = ~led_on;
endmodule
`default_nettype wire
