`default_nettype none
// CONTROL-SIGNAL TEST: keep motor power CH1 OFF. Forward 10%, 1 kHz,
// maximum 5 seconds per fresh start. This is not a motor jog firmware.
// S0 reset; S1 clear WHEN LOCKED, start WHEN READY (separate presses);
// S2 logical emergency request; S3 simulated fault (NOT direction).
// LED2 ready, LED4 enabled, LED5 locked. External hardware cutoff still needed.
module dock_motor_protection #(
    parameter integer CLK_HZ=27000000, PWM_HZ=1000,
    parameter integer RELEASE_CYCLES=270000, DEBOUNCE_CYCLES=270000,
    parameter integer STOP_CYCLES=27000000, RUN_CYCLES=135000000
)(
    input wire clk, reset_key_n, start_key_n, estop_key_n, fault_key_n,
    output wire rpwm, lpwm, r_en, l_en,
    output wire led_run_n, led_locked_n, led_ready_n
);
    wire rst_n, start_event;
    reset_guard #(.RELEASE_CYCLES(RELEASE_CYCLES)) reset_i
        (.clk(clk),.external_reset_n(reset_key_n),.reset_n(rst_n));
    button_input #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) start_i
        (.clk(clk),.rst_n(rst_n),.button_raw(start_key_n),
         .pressed(),.press_pulse(start_event));

    // Human trip buttons: two-flop synchronization, NO 10-ms debounce delay.
    // Board RC and metastability response are outside the RTL timing model.
    reg estop_meta /* synthesis syn_preserve=1 */ =1;
    reg estop_sync /* synthesis syn_preserve=1 */ =1;
    reg fault_meta /* synthesis syn_preserve=1 */ =1;
    reg fault_sync /* synthesis syn_preserve=1 */ =1;
    always @(posedge clk) begin
        if(!rst_n) begin
            estop_meta<=1; estop_sync<=1; fault_meta<=1; fault_sync<=1;
        end else begin
            estop_meta<=!estop_key_n; estop_sync<=estop_meta;
            fault_meta<=!fault_key_n; fault_sync<=fault_meta;
        end
    end

    localparam integer RW=RUN_CYCLES>1 ? $clog2(RUN_CYCLES) : 1;
    reg [RW-1:0] run_count=0;
    reg [1:0] command=0;
    reg arm=0, pending_start=0;
    wire [2:0] state;
    wire permit, locked;
    wire clear_request=start_event && locked;

    motor_protected #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ),.STOP_CYCLES(STOP_CYCLES)) motor_i (
        .clk(clk),.rst_n(rst_n),.command(command),.arm(arm),.permit_in(1'b1),
        .duty_pct(7'd10),.fault(fault_sync),.estop(estop_sync),.clear_fault(clear_request),
        .rpwm(rpwm),.lpwm(lpwm),.r_en(r_en),.l_en(l_en),.motor_dir(),.state_o(state),
        .permit_o(permit),.protection_locked(locked),
        .fault_latched(),.estop_latched(),.clear_accepted()
    );

    always @(posedge clk) begin
        arm<=0;
        if(!rst_n || !permit || state==4) begin
            command<=0; pending_start<=0; run_count<=0;
        end else if(r_en) begin
            if(run_count==RUN_CYCLES-1) begin
                command<=0; run_count<=0;
            end else run_count<=run_count+1'b1;
        end else begin
            run_count<=0;
            if(pending_start && state==1) begin
                command<=2'b01; pending_start<=0;
            end else if(start_event && state==0 && !pending_start) begin
                arm<=1; pending_start<=1;
            end
        end
    end
    assign led_ready_n=!(rst_n && permit && state==0 && !pending_start);
    assign led_run_n=!(rst_n && r_en && l_en);
    assign led_locked_n=!(rst_n && locked);
    // synthesis translate_off
    initial if(RUN_CYCLES<1) $fatal(1,"RUN_CYCLES must be positive");
    // synthesis translate_on
endmodule
`default_nettype wire
