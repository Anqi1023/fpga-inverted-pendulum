`default_nettype none
// Offline synthesis harness. Ports are synchronous logical stimuli, NOT GPIO.
// TODO: verified button synchronizers/debounce, reset and permit circuitry.
module motor_test_top #(parameter [6:0] DUTY_PERCENT = 7'd10) (
    input wire clk, input wire rst_n,
    input wire [1:0] command,
    input wire arm, input wire permit,
    output wire motor_pwm, output wire motor_dir, output wire motor_en,
    output wire [2:0] state_o,
    output wire rpwm, output wire lpwm, output wire r_en, output wire l_en
);
    motor_driver driver_i (
        .clk(clk), .rst_n(rst_n), .command(command), .arm(arm), .permit(permit),
        .duty_pct(DUTY_PERCENT), .motor_pwm(motor_pwm), .motor_dir(motor_dir),
        .motor_en(motor_en), .state_o(state_o));
    // Physical outputs are locked low. Low is NOT yet verified as safe wiring.
    ibt2_adapter #(.INTERFACE_CONFIRMED(0)) adapter_i (
        .pwm(motor_pwm), .dir(motor_dir), .en(motor_en),
        .rpwm(rpwm), .lpwm(lpwm), .r_en(r_en), .l_en(l_en));
endmodule
`default_nettype wire
