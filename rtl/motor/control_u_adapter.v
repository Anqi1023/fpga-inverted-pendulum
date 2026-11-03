`default_nettype none
// Synthesizable C -> existing motor_protected protocol adapter (not a testbench).
// All inputs use the same clk domain. Raw buttons require synchronization.
// Feed driver_state/protection_locked back from the SAME motor_protected.
module control_u_adapter #(
    parameter integer CLK_HZ = 27000000,
    parameter integer WATCHDOG_MS = 100,
    parameter integer TIMEOUT_CYCLES = (CLK_HZ / 1000) * WATCHDOG_MS,
    // D09 floors in 0.1% units; start/reversal 10%, recent same-direction motion 8%.
    parameter integer START_MIN_U = 100,
    parameter integer RUN_MIN_U = 80,
    parameter integer LIVE_FINE_DRIVE = 0,
    parameter integer LIVE_START_BOOST = 0
)(
    input wire clk,
    input wire rst_n,
    input wire signed [15:0] control_u,
    input wire signed [31:0] motion_theta,
    input wire control_valid,
    input wire fine_drive_enable,
    input wire boost_enable,
    input wire [7:0] boost_min_u,
    input wire [3:0] boost_ticks,
    input wire motor_enable,
    input wire fault,
    input wire estop,
    input wire protection_locked,
    input wire [2:0] driver_state,
    input wire clear_request,
    output wire [1:0] command,
    output wire [9:0] duty_permille,
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
    reg [9:0] held_duty = 0;

    // 2026-10-18: 与 controller 的 U_LIMIT 同步，300 -> 1000（100% 占空比）
    wire range_error = control_u > 16'sd1000 || control_u < -16'sd1000;
    wire signed [16:0] extended_u = {control_u[15], control_u};
    wire [16:0] abs_u = control_u[15] ? -extended_u : extended_u;
    // Normal mode stops below 10U. Fine mode passes every nonzero integer U;
    // its bounded start boost is separate. Zero is always STOP.
    // Estimate recent direction from encoder-derived theta at valid updates (200Hz).
    reg signed [31:0] previous_theta;
    reg motion_baseline;
    reg [3:0] motion_age;
    reg motion_negative;
    wire signed [31:0] delta_raw = motion_theta - previous_theta;
    wire signed [31:0] delta = (delta_raw > 32'sd205887) ? delta_raw-32'sd411775 :
                              (delta_raw < -32'sd205887) ? delta_raw+32'sd411775 : delta_raw;
    wire fresh_motion = motion_baseline && (delta != 0);
    wire recent_motion = fresh_motion || (motion_baseline && motion_age < 10);
    wire measured_negative = fresh_motion ? delta[31] : motion_negative;
    wire same_direction = recent_motion && (measured_negative == control_u[15]);
    // R10: optional bounded startup floor; one attempt per nonzero request episode.
    // Inputs are stop-only configuration. Top enables this only during balance.
    wire boost_mode = LIVE_START_BOOST && boost_enable;
    reg [3:0] still_ticks;
    reg boost_tried, boost_negative;
    reg [3:0] boost_left;
    wire stationary = motion_baseline && !fresh_motion && still_ticks >= 10;
    wire boost_start = boost_mode && stationary && !boost_tried && abs_u >= 10;
    wire boost_continue = boost_mode && boost_left != 0 && !fresh_motion &&
                          boost_negative == control_u[15] && abs_u >= 10;
    wire [16:0] startup_floor = (boost_min_u >= RUN_MIN_U && boost_min_u <= 250) ?
                                {9'd0,boost_min_u} : RUN_MIN_U;
    wire [16:0] floor_u = boost_mode ?
                        ((boost_start || boost_continue) ? startup_floor : RUN_MIN_U) :
                        (same_direction ? RUN_MIN_U : START_MIN_U);
    always @(posedge clk) begin
        if (!rst_n || !motor_enable || blocked || !boost_mode) begin
            still_ticks <= 0; boost_tried <= 0; boost_negative <= 0; boost_left <= 0;
        end else if (control_valid) begin
            if (fresh_motion) still_ticks <= 0;
            else if (motion_baseline && still_ticks < 10) still_ticks <= still_ticks + 1'b1;
            if (abs_u < 10) begin boost_tried <= 0; boost_left <= 0; end
            else if (boost_start) begin
                boost_tried <= 1; boost_negative <= control_u[15];
                boost_left <= (boost_ticks >= 1 && boost_ticks <= 10) ? boost_ticks - 1'b1 : 0;
            end else if (boost_continue) boost_left <= boost_left - 1'b1;
            else boost_left <= 0;
        end
    end
    wire fine_mode=LIVE_FINE_DRIVE && fine_drive_enable;
    // In fine mode nonzero controller U is passed at its real magnitude, except
    // an explicitly enabled, bounded stationary-start pulse. Zero stays STOP.
    wire [16:0] fine_u=(abs_u==0) ? 17'd0 :
        ((boost_start || boost_continue) && abs_u<startup_floor) ? startup_floor : abs_u;
    wire [16:0] move_u = fine_mode ? fine_u : (abs_u < 10) ? 17'd0 :
                         (abs_u < floor_u) ? floor_u : abs_u;
    always @(posedge clk) begin
        if (!rst_n || !motor_enable || blocked) begin
            previous_theta <= 0; motion_baseline <= 0;
            motion_age <= 10; motion_negative <= 0;
        end else if (control_valid) begin
            previous_theta <= motion_theta;
            motion_baseline <= 1;
            if (fresh_motion) begin motion_age <= 0; motion_negative <= delta[31]; end
            else if (motion_age < 10) motion_age <= motion_age + 1'b1;
        end
    end
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
    assign duty_permille = live_request && can_command ? held_duty : 7'd0;

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
            age <= 0; have_command <= 1; held_duty <= move_u[9:0];
            held_command <= move_u == 0 ? 2'b00 :
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
