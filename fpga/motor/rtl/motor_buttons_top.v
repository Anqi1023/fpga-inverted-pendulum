`default_nettype none
// Offline integration top; button pins, polarity and reset wrapper remain TODO.
// permit is already synchronous. It is NOT a raw emergency-stop input.
module motor_buttons_top #(
    parameter integer CLK_HZ=27000000,
    parameter integer PWM_HZ=20000,
    parameter integer STOP_CYCLES=27000000,
    parameter integer DEBOUNCE_CYCLES=270000,
    parameter integer ACTIVE_LEVEL=0,
    parameter [6:0] DUTY_PERCENT=10
)(
    input wire clk, rst_n, permit,
    input wire start_raw, stop_raw, direction_raw,
    output wire rpwm, lpwm, r_en, l_en,
    output reg selected_reverse=0,
    output wire running,
    output wire [2:0] state_o
);
    wire start_event, direction_event, stop_held;
    button_input #(.STABLE_CYCLES(DEBOUNCE_CYCLES),.ACTIVE_LEVEL(ACTIVE_LEVEL)) start_i
        (.clk(clk),.rst_n(rst_n),.button_raw(start_raw),.pressed(),.press_pulse(start_event));
    button_input #(.STABLE_CYCLES(DEBOUNCE_CYCLES),.ACTIVE_LEVEL(ACTIVE_LEVEL)) stop_i
        (.clk(clk),.rst_n(rst_n),.button_raw(stop_raw),.pressed(stop_held),.press_pulse());
    button_input #(.STABLE_CYCLES(DEBOUNCE_CYCLES),.ACTIVE_LEVEL(ACTIVE_LEVEL)) direction_i
        (.clk(clk),.rst_n(rst_n),.button_raw(direction_raw),.pressed(),.press_pulse(direction_event));

    reg requested=0, arm=0, pending_start=0;
    wire driver_permit=permit && !stop_held && !direction_event;
    wire [1:0] command=requested ? (selected_reverse ? 2'b10 : 2'b01) : 2'b00;
    wire pwm, dir, en;
    always @(posedge clk) begin
        arm <= 0;
        if(!rst_n) begin requested<=0; selected_reverse<=0; end
        else if(!permit || stop_held) requested<=0;
        else if(direction_event) begin
            selected_reverse<=!selected_reverse;
            requested<=0; // Direction selection always stops; no automatic restart.
        end else if(state_o==4) requested<=0; // Ignore starts during lockout.
        else if(start_event && state_o==0) arm<=1;
        else if(state_o==1 && pending_start) requested<=1;
    end
    always @(posedge clk) begin
        if(!rst_n || !driver_permit || state_o==4) pending_start<=0;
        else if(start_event && state_o==0) pending_start<=1;
        else if(state_o==1) pending_start<=0;
    end
    // Keep STOP while arming; issue the selected direction only after READY.
    wire [1:0] driver_command=(state_o==1 && pending_start) ?
        (selected_reverse ? 2'b10 : 2'b01) : command;
    motor_driver #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ),.STOP_CYCLES(STOP_CYCLES)) driver_i
        (.clk(clk),.rst_n(rst_n),.command(driver_command),.arm(arm),.permit(driver_permit),
         .duty_pct(DUTY_PERCENT),.motor_pwm(pwm),.motor_dir(dir),.motor_en(en),.state_o(state_o));
    assign running=en;
    ibt2_adapter adapter_i(.clk(clk),.rst_n(rst_n),.pwm(pwm),.dir(dir),.en(en),
        .rpwm(rpwm),.lpwm(lpwm),.r_en(r_en),.l_en(l_en));
endmodule
`default_nettype wire
