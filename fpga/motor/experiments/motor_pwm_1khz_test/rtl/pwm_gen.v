`default_nettype none
// Synchronous inputs. Duty changes take effect only at a PWM boundary.
module pwm_gen #(
    parameter integer CLK_HZ = 27000000,
    parameter integer PWM_HZ = 20000
)(
    input wire clk, input wire rst_n, input wire enable,
    input wire [6:0] duty_pct,
    output wire pwm
);
    localparam integer PERIOD = CLK_HZ / PWM_HZ;
    localparam integer CW = (PERIOD > 1) ? $clog2(PERIOD) : 1;
    reg [CW-1:0] count = 0;
    reg [CW:0] high_ticks = 0;
    // Invalid duty produces zero, never wraps to a high duty.
    wire [31:0] requested = (duty_pct <= 100) ? (PERIOD * duty_pct / 100) : 0;
    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            count <= 0;
            high_ticks <= 0;
        end else if (count == PERIOD-1) begin
            count <= 0;
            high_ticks <= requested[CW:0];
        end else count <= count + 1'b1;
    end
    assign pwm = rst_n && enable && (count < high_ticks);
    // synthesis translate_off
    initial begin
        if (CLK_HZ <= 0 || PWM_HZ <= 0 || PERIOD < 2 || CLK_HZ % PWM_HZ != 0)
            $fatal(1, "Require positive clocks, exact division and PERIOD >= 2");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
