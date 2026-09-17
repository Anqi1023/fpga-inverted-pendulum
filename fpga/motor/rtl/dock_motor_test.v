`default_nettype none
// Dock standalone signal test ONLY. No hardware ESTOP or external permit yet.
// Keep driver and motor disconnected. S4 is deliberately unused.
module dock_motor_test #(
    parameter integer RELEASE_CYCLES=270000,
    parameter integer DEBOUNCE_CYCLES=270000,
    parameter integer STOP_CYCLES=27000000
)(
    input wire clk, reset_key_n, start_key_n, stop_key_n, direction_key_n,
    output wire rpwm, lpwm, r_en, l_en,
    output wire led_run_n, led_reverse_n, led_ready_n
);
    wire rst_n, reverse, running;
    wire [2:0] state;
    reset_guard #(.RELEASE_CYCLES(RELEASE_CYCLES)) reset_i
        (.clk(clk),.external_reset_n(reset_key_n),.reset_n(rst_n));
    motor_buttons_top #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.STOP_CYCLES(STOP_CYCLES)) control_i
        (.clk(clk),.rst_n(rst_n),.permit(1'b1),
         .start_raw(start_key_n),.stop_raw(stop_key_n),.direction_raw(direction_key_n),
         .rpwm(rpwm),.lpwm(lpwm),.r_en(r_en),.l_en(l_en),
         .selected_reverse(reverse),.running(running),.state_o(state));
    assign led_run_n=!(rst_n && running);
    assign led_reverse_n=!(rst_n && reverse);
    assign led_ready_n=!(rst_n && state==0);
endmodule
`default_nettype wire
