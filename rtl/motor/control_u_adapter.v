`default_nettype none
// Synthesizable C -> existing motor_protected protocol adapter (not a testbench).
// All inputs use the same clk domain. Raw buttons require synchronization.
// Feed driver_state/protection_locked back from the SAME motor_protected.
module control_u_adapter #(
    parameter integer CLK_HZ = 27000000,
    parameter integer WATCHDOG_MS = 100,
    parameter integer TIMEOUT_CYCLES = (CLK_HZ / 1000) * WATCHDOG_MS
)(
    input wire clk,
    input wire rst_n,
    input wire signed [15:0] control_u,
    input wire control_valid,
    input wire motor_enable,
    input wire fault,
    input wire estop,
    input wire protection_locked,
    input wire [2:0] driver_state,
    input wire clear_request,
    output wire [1:0] command,
    output wire [6:0] duty_pct,
    output wire arm,
    output wire clear_fault,
    output wire adapter_fault,
    output reg timeout_latched = 0,
    output reg command_error_latched = 0
);
    // State encoding in the existing motor_driver.v; do not infer readiness
    // from motor_enable. OFF accepts arm only while command is STOP.
    localparam [2:0] OFF=0, READY=1, FORWARD=2, REVERSE=3, WAIT_STOP=4;
    localparam integer TW = (TIMEOUT_CYCLES > 1) ? $clog2(TIMEOUT_CYCLES) : 1;
    reg [TW-1:0] age = 0;
    reg valid_prev = 0, clear_prev = 1;
    reg clear_pulse = 0, have_command = 0;
    reg [1:0] held_command = 0;
    reg [6:0] held_duty = 0;

    wire range_error = control_u > 16'sd300 || control_u < -16'sd300;
    wire signed [16:0] extended_u = {control_u[15], control_u};
    wire [16:0] abs_u = control_u[15] ? -extended_u : extended_u;
    wire [16:0] truncated_pct = abs_u / 17'd10;
    wire clear_event = clear_request && !clear_prev;
    wire clear_safe = !motor_enable && !control_valid && !fault && !estop;
    wire blocked = !rst_n || fault || estop || protection_locked || adapter_fault;
    wire live_request = !blocked && motor_enable && have_command && held_duty != 0;
    wire can_command = driver_state == READY || driver_state == FORWARD || driver_state == REVERSE;

    // A clear event first removes local trips, then the one-clock pulse clears
    // the existing protection on the next edge. Neither stage issues arm.
    assign adapter_fault = timeout_latched || command_error_latched;
    assign clear_fault = rst_n && clear_pulse && clear_safe;
    // A connected OFF -> READY transition makes this arm exactly one clock.
    assign arm = live_request && driver_state == OFF;
    assign command = live_request && can_command ? held_command : 2'b00;
    assign duty_pct = live_request && can_command ? held_duty : 7'd0;

    always @(posedge clk) begin
        valid_prev <= control_valid;
        clear_prev <= clear_request;
        clear_pulse <= 0;
        if (!rst_n) begin
            age <= 0; valid_prev <= 0; clear_prev <= 1;
            clear_pulse <= 0; have_command <= 0; held_command <= 0; held_duty <= 0;
            timeout_latched <= 0; command_error_latched <= 0;
        end else if (clear_event && clear_safe) begin
            timeout_latched <= 0; command_error_latched <= 0;
            age <= 0; have_command <= 0; held_command <= 0; held_duty <= 0;
            clear_pulse <= 1;
        end else if (!motor_enable || blocked) begin
            age <= 0; have_command <= 0; held_command <= 0; held_duty <= 0;
        end else if (control_valid && (range_error || valid_prev)) begin
            // Reject before magnitude truncation, including signed -32768.
            // Detect stuck valid on a SECOND sampled high, not combinationally
            // from the post-edge valid_prev (which would reject a legal pulse).
            command_error_latched <= 1;
            age <= 0; have_command <= 0; held_command <= 0; held_duty <= 0;
        end else if (control_valid) begin
            age <= 0; have_command <= 1; held_duty <= truncated_pct[6:0];
            held_command <= truncated_pct == 0 ? 2'b00 :
                            (control_u[15] ? 2'b10 : 2'b01);
        end else if (age == TIMEOUT_CYCLES-1) begin
            // Fresh valid on this deadline takes the preceding branch.
            timeout_latched <= 1;
            age <= 0; have_command <= 0; held_command <= 0; held_duty <= 0;
        end else age <= age + 1'b1;
    end
    // synthesis translate_off
    initial if (CLK_HZ < 1000 || CLK_HZ % 1000 != 0 || WATCHDOG_MS < 1 || TIMEOUT_CYCLES < 1)
        $fatal(1, "Invalid adapter clock/watchdog parameters");
    // synthesis translate_on
endmodule
`default_nettype wire
