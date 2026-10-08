`default_nettype none
// BOARD TEST ONLY: simulate C with +100 / 20-ms command pulses.
// Each new S1 start is bounded to 200 ms of EN, with a 1-s cooldown.
// S1 clears when locked, starts when ready; S2 estop, S3 injected fault.
// LED3 ready, LED4 enabled, LED5 locked. No A or real C is instantiated.
module dock_motor_control_v1 #(
    parameter integer CLK_HZ=27000000, PWM_HZ=1000,
    parameter integer RELEASE_CYCLES=270000, DEBOUNCE_CYCLES=270000,
    parameter integer CONTROL_CYCLES=540000, TIMEOUT_CYCLES=2700000,
    parameter integer STOP_CYCLES=27000000, RUN_CYCLES=5400000
)(
    input wire clk, reset_key_n, start_key_n, estop_key_n, fault_key_n,
    output wire rpwm, lpwm, ibt_en,
    output wire led_run_n, led_locked_n, led_ready_n
);
    // One FPGA output drives TWO level-shifter inputs (B3 and B4).
    // Do not short independent FPGA or level-shifter outputs together.
    wire r_en, l_en;
    assign ibt_en = r_en && l_en;
    wire rst_n, start_event;
    reset_guard #(.RELEASE_CYCLES(RELEASE_CYCLES)) reset_i
        (.clk(clk),.external_reset_n(reset_key_n),.reset_n(rst_n));
    button_input #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) start_i
        (.clk(clk),.rst_n(rst_n),.button_raw(start_key_n),
         .pressed(),.press_pulse(start_event));
    reg estop_meta /* synthesis syn_preserve=1 */ =1;
    reg estop_sync /* synthesis syn_preserve=1 */ =1;
    reg fault_meta /* synthesis syn_preserve=1 */ =1;
    reg fault_sync /* synthesis syn_preserve=1 */ =1;
    always @(posedge clk) begin
        if(!rst_n) begin estop_meta<=1; estop_sync<=1; fault_meta<=1; fault_sync<=1; end
        else begin
            estop_meta<=!estop_key_n; estop_sync<=estop_meta;
            fault_meta<=!fault_key_n; fault_sync<=fault_meta;
        end
    end
    localparam integer RW=(RUN_CYCLES>1)?$clog2(RUN_CYCLES):1;
    localparam integer UW=(CONTROL_CYCLES>1)?$clog2(CONTROL_CYCLES):1;
    reg [RW-1:0] run_count=0;
    reg [UW-1:0] update_count=0;
    reg signed [15:0] control_u=0;
    reg control_valid=0, motor_enable=0;
    wire locked, core_ready;
    motor_control_v1 #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ),
        .COMMAND_TIMEOUT_CYCLES(TIMEOUT_CYCLES),.STOP_CYCLES(STOP_CYCLES)) motor_i (
        .clk(clk),.rst_n(rst_n),.control_u(control_u),.control_valid(control_valid),
        .motor_enable(motor_enable),.fault(fault_sync),.estop(estop_sync),
        .clear_fault(start_event && locked),.rpwm(rpwm),.lpwm(lpwm),.r_en(r_en),.l_en(l_en),
        .motor_dir(),.ready(core_ready),.waiting(),.protection_locked(locked),
        .fault_latched(),.estop_latched(),.clear_accepted(),.timeout_latched(),
        .command_error_latched(),.duty_tenths_o()
    );
    always @(posedge clk) begin
        control_valid<=0;
        if(!rst_n || locked || estop_sync || fault_sync) begin
            motor_enable<=0; control_u<=0; run_count<=0; update_count<=0;
        end else if(motor_enable) begin
            if(ibt_en && run_count==RUN_CYCLES-1) begin
                motor_enable<=0; control_u<=0; run_count<=0; update_count<=0;
            end else begin
                if(ibt_en) run_count<=run_count+1'b1;
                if(update_count==CONTROL_CYCLES-1) begin
                    update_count<=0; control_u<=16'sd100; control_valid<=1;
                end else update_count<=update_count+1'b1;
            end
        end else if(start_event && core_ready) begin
            motor_enable<=1; control_u<=16'sd100; control_valid<=1;
            run_count<=0; update_count<=0;
        end
    end
    assign led_ready_n=!(rst_n && core_ready && !motor_enable);
    assign led_run_n=!(rst_n && ibt_en);
    assign led_locked_n=!(rst_n && locked);
    // synthesis translate_off
    initial if(RUN_CYCLES<1 || CONTROL_CYCLES<2 || TIMEOUT_CYCLES<=CONTROL_CYCLES)
        $fatal(1,"Invalid board test timing");
    // synthesis translate_on
endmodule
`default_nettype wire
