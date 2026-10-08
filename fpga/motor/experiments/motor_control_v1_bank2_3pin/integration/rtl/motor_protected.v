`default_nettype none
// Reusable synchronous integration, not a pin-complete board top.
// Clear protection only in STOP with arm released, then give a NEW arm edge
// after motor_driver's stop interval. Clearing a trip never starts the motor.
module motor_protected #(
    parameter integer CLK_HZ = 27000000,
    parameter integer PWM_HZ = 1000,
    parameter integer STOP_CYCLES = 27000000
)(
    input wire clk,
    input wire rst_n,
    input wire [1:0] command,
    input wire arm,
    input wire permit_in,
    input wire [6:0] duty_pct,
    input wire fault,
    input wire estop,
    input wire clear_fault,
    output wire rpwm,
    output wire lpwm,
    output wire r_en,
    output wire l_en,
    output wire motor_dir,
    output wire [2:0] state_o,
    output wire permit_o,
    output wire protection_locked,
    output wire fault_latched,
    output wire estop_latched,
    output wire clear_accepted
);
    wire pwm, en;
    wire invalid_duty = duty_pct > 7'd30;
    // 11 remains a STOP command to the driver, but only explicit 00 clears.
    wire safe_clear = (command == 2'b00) && !arm;

    protection protection_i (
        .clk(clk), .rst_n(rst_n), .permit_in(permit_in),
        .fault(fault || invalid_duty), .estop(estop),
        .stop_request(safe_clear), .clear_fault(clear_fault),
        .permit(permit_o), .locked(protection_locked),
        .fault_latched(fault_latched), .estop_latched(estop_latched),
        .clear_accepted(clear_accepted)
    );
    motor_driver #(
        .CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ), .STOP_CYCLES(STOP_CYCLES)
    ) driver_i (
        .clk(clk), .rst_n(rst_n), .command(command), .arm(arm),
        .permit(permit_o), .duty_pct(duty_pct),
        .motor_pwm(pwm), .motor_dir(motor_dir), .motor_en(en), .state_o(state_o)
    );
    ibt2_adapter adapter_i (
        .clk(clk), .rst_n(rst_n), .pwm(pwm), .dir(motor_dir), .en(en),
        .rpwm(rpwm), .lpwm(lpwm), .r_en(r_en), .l_en(l_en)
    );
endmodule
`default_nettype wire
