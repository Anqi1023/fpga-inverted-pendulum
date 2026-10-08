`default_nettype none
// C -> B V1.0, all inputs synchronous to clk. control_valid is ONE clock
// wide, nominally every 20 ms. Defaults: 100-ms watchdog, 1-s stop/reversal
// wait retained from the bench baseline. No 200-ms limit in this core.
// V1.0 supplement: truncate abs(control_u)/10 to integer percent.
// A/B/C must review the reversal interval before continuous closed-loop use.
module motor_control_v1 #(
    parameter integer CLK_HZ=27000000, PWM_HZ=1000,
    parameter integer COMMAND_TIMEOUT_CYCLES=2700000,
    parameter integer STOP_CYCLES=27000000
)(
    input wire clk, rst_n,
    input wire signed [15:0] control_u,
    input wire control_valid, motor_enable,
    input wire fault, estop, clear_fault,
    output wire rpwm, lpwm, r_en, l_en,
    output wire motor_dir, ready, waiting, protection_locked,
    output wire fault_latched, estop_latched, clear_accepted,
    output reg timeout_latched=0,
    output reg command_error_latched=0,
    output wire [8:0] duty_tenths_o
);
    localparam integer TW=(COMMAND_TIMEOUT_CYCLES>1)?$clog2(COMMAND_TIMEOUT_CYCLES):1;
    localparam integer SW=(STOP_CYCLES>1)?$clog2(STOP_CYCLES+1):1;
    reg valid_prev=0;
    reg [TW-1:0] age=0;
    reg timeout_request=0, protocol_request=0;
    reg [6:0] magnitude_pct=0;
    reg requested_dir=0, have_command=0;
    reg running=0, held_dir=0;
    reg [SW-1:0] stop_count=STOP_CYCLES;
    wire permit;

    // Range comparison precedes absolute-value truncation, including -32768.
    wire range_error=control_u>16'sd300 || control_u< -16'sd300;
    wire malformed_valid=motor_enable && control_valid && valid_prev;
    // Detect a second sampled high, not the post-edge value of valid_prev:
    // a combinational previous-value check would blank a legal pulse.
    wire bad_command=(motor_enable && control_valid && range_error) || protocol_request;
    wire sample_ok=motor_enable && control_valid && !valid_prev && !range_error;
    wire signed [16:0] extended_u={control_u[15],control_u};
    wire [16:0] abs_u=control_u[15] ? -extended_u : extended_u;
    wire [16:0] truncated_pct=abs_u/17'd10;

    // Ordinary motor_enable=0 is NOT a latched fault. Only actual trips,
    // protocol/range errors and watchdog expiry lock the protection module.
    protection protection_i (
        .clk(clk),.rst_n(rst_n),.permit_in(1'b1),
        .fault(fault || bad_command || timeout_request),.estop(estop),
        .stop_request(!motor_enable && !control_valid),.clear_fault(clear_fault),
        .permit(permit),.locked(protection_locked),
        .fault_latched(fault_latched),.estop_latched(estop_latched),
        .clear_accepted(clear_accepted)
    );

    always @(posedge clk) begin
        valid_prev<=control_valid;
        timeout_request<=0;
        protocol_request<=0;
        if(!rst_n) begin
            valid_prev<=0; age<=0; magnitude_pct<=0; requested_dir<=0;
            have_command<=0; timeout_latched<=0; command_error_latched<=0;
        end else begin
            if(clear_accepted) begin timeout_latched<=0; command_error_latched<=0; end
            if(malformed_valid) protocol_request<=1;
            if(bad_command || malformed_valid) command_error_latched<=1;
            if(!motor_enable || !permit) begin
                age<=0; magnitude_pct<=0; have_command<=0;
            end else if(sample_ok) begin
                age<=0; magnitude_pct<=truncated_pct[6:0]; requested_dir<=control_u[15];
                have_command<=1;
            end else if(age==COMMAND_TIMEOUT_CYCLES-1) begin
                // Accepted sample at the deadline wins; a later one cannot
                // undo expiry. The one-cycle request is latched by protection.
                timeout_request<=1; timeout_latched<=1;
                age<=0; magnitude_pct<=0; have_command<=0;
            end else age<=age+1'b1;
        end
    end

    wire drive_request=permit && motor_enable && have_command && magnitude_pct!=0;
    // Effective request: protection/disable beats stored input immediately.
    // Quantized zero (including +/-1..9) has STOP command and disables EN.
    wire [1:0] command=drive_request ? (requested_dir ? 2'b10 : 2'b01) : 2'b00;
    wire [6:0] duty_pct=drive_request ? magnitude_pct : 7'd0;
    // Keep the existing status/PWM port in tenths, now always a multiple of 10.
    wire [8:0] effective_tenths={2'b00,duty_pct}*9'd10;
    // A sampled zero/sign change removes drive before waiting for a PWM edge.
    wire drive_allowed=drive_request && running && requested_dir==held_dir;
    always @(posedge clk) begin
        if(!rst_n) begin running<=0; held_dir<=0; stop_count<=STOP_CYCLES; end
        else if(running) begin
            if(!drive_request || requested_dir!=held_dir) begin
                running<=0; stop_count<=STOP_CYCLES;
            end
        end else if(stop_count!=0) stop_count<=stop_count-1'b1;
        else if(drive_request) begin running<=1; held_dir<=requested_dir; end
    end

    wire pwm;
    pwm_tenths #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) pwm_i (
        .clk(clk),.rst_n(rst_n),.enable(drive_allowed),
        .duty_tenths(effective_tenths),.pwm(pwm)
    );
    ibt2_adapter adapter_i (
        .clk(clk),.rst_n(rst_n),.pwm(pwm),.dir(held_dir),.en(drive_allowed),
        .rpwm(rpwm),.lpwm(lpwm),.r_en(r_en),.l_en(l_en)
    );
    assign motor_dir=held_dir;
    assign waiting=stop_count!=0;
    assign ready=permit && !running && !waiting;
    assign duty_tenths_o=effective_tenths;
    // synthesis translate_off
    initial if(COMMAND_TIMEOUT_CYCLES<1 || STOP_CYCLES<1)
        $fatal(1,"Watchdog and stop interval must be positive");
    // synthesis translate_on
endmodule
`default_nettype wire
