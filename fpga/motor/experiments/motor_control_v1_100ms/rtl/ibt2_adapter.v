`default_nettype none
// Registered, mutually exclusive IBT-2 outputs. External non-inverting
// 3.3V -> 5V translation is REQUIRED. No direct 5V connection to FPGA.
module ibt2_adapter #(parameter integer INTERFACE_CONFIRMED = 1)(
    input wire clk, input wire rst_n,
    input wire pwm, input wire dir, input wire en,
    output wire rpwm, output wire lpwm,
    output wire r_en, output wire l_en
);
    reg held_dir = 0;
    reg [1:0] pwm_pair = 0;
    reg enabled = 0;
    wire allowed = (INTERFACE_CONFIRMED == 1) && en && rst_n;
    // Direction mismatch blanks BOTH PWM and EN immediately. The changed
    // direction is accepted with an additional full blank clock cycle.
    wire active = allowed && (dir == held_dir);
    always @(posedge clk) begin
        if (!rst_n || !allowed || dir != held_dir) begin
            pwm_pair <= 0;
            enabled <= 0;
            held_dir <= dir;
        end else begin
            pwm_pair <= dir ? {pwm,1'b0} : {1'b0,pwm};
            enabled <= 1;
        end
    end
    assign rpwm = active && enabled && pwm_pair[0];
    assign lpwm = active && enabled && pwm_pair[1];
    assign r_en = active && enabled;
    assign l_en = active && enabled;
endmodule
`default_nettype wire
