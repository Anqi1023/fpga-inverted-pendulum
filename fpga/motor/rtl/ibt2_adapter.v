`default_nettype none
// Provisional active-high IBT-2 model, NOT a board-level electrical approval.
// Keep INTERFACE_CONFIRMED=0 until truth table, voltage and pulls are verified.
module ibt2_adapter #(parameter integer INTERFACE_CONFIRMED = 0)(
    input wire pwm, input wire dir, input wire en,
    output wire rpwm, output wire lpwm,
    output wire r_en, output wire l_en
);
    wire allowed = (INTERFACE_CONFIRMED == 1) && en;
    assign rpwm = allowed && !dir && pwm;
    assign lpwm = allowed && dir && pwm;
    assign r_en = allowed;
    assign l_en = allowed;
endmodule
`default_nettype wire
