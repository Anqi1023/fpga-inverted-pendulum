`default_nettype none
// Synthesizable connection example / reusable execution chain.
// C -> control_u_adapter -> unchanged motor_protected -> IBT-2 outputs.
// No mock command generator, no button logic, no fixed 200-ms run window.
module motor_control_u_chain #(
    parameter integer CLK_HZ=27000000,
    parameter integer PWM_HZ=1000,
    parameter integer STOP_CYCLES=27000000,
    parameter integer TIMEOUT_CYCLES=(CLK_HZ/1000)*100,
    parameter integer START_MIN_U=100,
    parameter integer RUN_MIN_U=80,
    parameter integer LIVE_FINE_DRIVE=0,
    parameter integer LIVE_START_BOOST=0
)(
    input wire clk, rst_n,
    input wire signed [15:0] control_u,
    input wire signed [31:0] motion_theta,
    input wire control_valid, motor_enable,
    input wire fine_drive_enable,
    input wire boost_enable,
    input wire [7:0] boost_min_u,
    input wire [3:0] boost_ticks,
    input wire permit_in, fault, estop, clear_request,
    output wire rpwm, lpwm, r_en, l_en,
    output wire motor_dir,
    output wire [2:0] state_o,
    output wire permit_o, protection_locked,
    output wire fault_latched, estop_latched, clear_accepted,
    output wire timeout_latched, command_error_latched,
    output wire [1:0] command,
    output wire [9:0] duty_permille,
    output wire arm, clear_fault
);
    wire adapter_fault;
    control_u_adapter #(.CLK_HZ(CLK_HZ),.TIMEOUT_CYCLES(TIMEOUT_CYCLES),
                        .START_MIN_U(START_MIN_U),.RUN_MIN_U(RUN_MIN_U),.LIVE_FINE_DRIVE(LIVE_FINE_DRIVE),.LIVE_START_BOOST(LIVE_START_BOOST)) adapter_i (
        .clk(clk),.rst_n(rst_n),.control_u(control_u),.motion_theta(motion_theta),.control_valid(control_valid),
        .fine_drive_enable(fine_drive_enable),.boost_enable(boost_enable),.boost_min_u(boost_min_u),.boost_ticks(boost_ticks),
        .motor_enable(motor_enable),.fault(fault || !permit_in),.estop(estop),
        .protection_locked(protection_locked),.driver_state(state_o),.clear_request(clear_request),
        .command(command),.duty_permille(duty_permille),.arm(arm),.clear_fault(clear_fault),
        .adapter_fault(adapter_fault),.timeout_latched(timeout_latched),
        .command_error_latched(command_error_latched)
    );
    motor_protected #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ),.STOP_CYCLES(STOP_CYCLES)) motor_i (
        .clk(clk),.rst_n(rst_n),.command(command),.arm(arm),.permit_in(permit_in),
        .duty_permille(duty_permille),.fault(fault || adapter_fault),.estop(estop),.clear_fault(clear_fault),
        .rpwm(rpwm),.lpwm(lpwm),.r_en(r_en),.l_en(l_en),.motor_dir(motor_dir),
        .state_o(state_o),.permit_o(permit_o),.protection_locked(protection_locked),
        .fault_latched(fault_latched),.estop_latched(estop_latched),.clear_accepted(clear_accepted)
    );
endmodule
`default_nettype wire
