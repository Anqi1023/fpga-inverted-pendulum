`default_nettype none
// Ordinary human command button only; NOT the emergency-stop path.
// rst_n must be released synchronously to clk by the board reset_guard wrapper.
module button_input #(
    parameter integer STABLE_CYCLES = 270000, // 10 ms at confirmed 27 MHz
    parameter integer ACTIVE_LEVEL = 0
)(
    input wire clk,
    input wire rst_n,
    input wire button_raw,
    output reg pressed = 0,
    output reg press_pulse = 0
);
    localparam integer CW = STABLE_CYCLES > 1 ? $clog2(STABLE_CYCLES) : 1;
    reg sync_meta /* synthesis syn_preserve=1 */ = 0;
    reg sync_value /* synthesis syn_preserve=1 */ = 0;
    reg [1:0] valid = 0;
    reg candidate = 0;
    reg [CW-1:0] count = 0;
    reg release_seen = 0;
    wire normalized = (button_raw == ACTIVE_LEVEL);
    // Only sync_value feeds the debounce logic. The first stage must not fan out.
    always @(posedge clk) begin
        if (!rst_n) begin
            sync_meta <= 0; sync_value <= 0; valid <= 0;
        end else begin
            sync_meta <= normalized;
            sync_value <= sync_meta;
            valid <= {valid[0], 1'b1};
        end
    end
    always @(posedge clk) begin
        press_pulse <= 0;
        if (!rst_n) begin
            pressed <= 0; candidate <= 0; count <= 0; release_seen <= 0;
        end else if (valid[1]) begin
            if (STABLE_CYCLES == 1) begin
                pressed <= sync_value;
                if (!sync_value) release_seen <= 1;
                if (sync_value && !pressed && release_seen) press_pulse <= 1;
            end else if (sync_value != candidate) begin
                candidate <= sync_value;
                count <= 1;
            end else if (count == STABLE_CYCLES-1) begin
                pressed <= candidate;
                if (!candidate) release_seen <= 1;
                if (candidate && !pressed && release_seen) press_pulse <= 1;
                // Saturate until the synchronized input changes.
            end else begin
                count <= count + 1'b1;
            end
        end
    end
    // synthesis translate_off
    initial begin
        if (STABLE_CYCLES < 1) $fatal(1, "STABLE_CYCLES must be positive");
        if (ACTIVE_LEVEL != 0 && ACTIVE_LEVEL != 1) $fatal(1, "Invalid ACTIVE_LEVEL");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
