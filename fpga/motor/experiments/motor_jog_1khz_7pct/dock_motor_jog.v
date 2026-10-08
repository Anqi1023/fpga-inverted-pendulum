`default_nettype none
// Bench jog only. Requires external power cutoff and suitable power wiring.
// S2 is a synchronized functional stop, NOT an independent hardware ESTOP.
module dock_motor_jog #(
    parameter integer CLK_HZ=27000000, PWM_HZ=1000,
    parameter integer RELEASE_CYCLES=270000, DEBOUNCE_CYCLES=270000,
    parameter integer JOG_CYCLES=5400000, LOCK_CYCLES=27000000,
    parameter [6:0] DUTY_PERCENT=7
)(
    input wire clk, reset_key_n, start_key_n, stop_key_n, direction_key_n,
    output wire rpwm, lpwm, r_en, l_en,
    output wire led_run_n, led_reverse_n, led_ready_n
);
    wire rst_n, start_event, direction_event;
    reset_guard #(.RELEASE_CYCLES(RELEASE_CYCLES)) reset_i
        (.clk(clk),.external_reset_n(reset_key_n),.reset_n(rst_n));
    button_input #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) start_i
        (.clk(clk),.rst_n(rst_n),.button_raw(start_key_n),.pressed(),.press_pulse(start_event));
    button_input #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) direction_i
        (.clk(clk),.rst_n(rst_n),.button_raw(direction_key_n),.pressed(),.press_pulse(direction_event));
    // Stop bypasses the human-button debounce; assertion latency is two clocks.
    reg stop_meta /* synthesis syn_preserve=1 */ =1;
    reg stop_sync /* synthesis syn_preserve=1 */ =1;
    always @(posedge clk) begin
        if(!rst_n) begin stop_meta<=1; stop_sync<=1; end
        else begin stop_meta<=!stop_key_n; stop_sync<=stop_meta; end
    end
    localparam integer JW=JOG_CYCLES>1 ? $clog2(JOG_CYCLES) : 1;
    localparam integer LW=LOCK_CYCLES>1 ? $clog2(LOCK_CYCLES+1) : 1;
    reg [JW-1:0] elapsed=0;
    reg [LW-1:0] cooldown=0;
    reg active=0, reverse=0;
    always @(posedge clk) begin
        if(!rst_n) begin active<=0; reverse<=0; elapsed<=0; cooldown<=0; end
        else begin
            if(cooldown!=0) cooldown<=cooldown-1'b1;
            if(stop_sync) begin
                active<=0; elapsed<=0;
                if(active) cooldown<=LOCK_CYCLES;
            end else if(direction_event) begin
                active<=0; elapsed<=0;
                // A direction press during a jog stops it without reversing.
                if(active) cooldown<=LOCK_CYCLES;
                else if(cooldown==0) reverse<=!reverse;
            end else if(active) begin
                if(elapsed==JOG_CYCLES-1) begin
                    active<=0; elapsed<=0; cooldown<=LOCK_CYCLES;
                end else elapsed<=elapsed+1'b1;
            end else if(cooldown==0 && start_event) begin
                active<=1; elapsed<=0;
            end
        end
    end
    wire enabled=rst_n && active && !stop_sync && !direction_event;
    wire pwm;
    pwm_gen #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) pwm_i
        (.clk(clk),.rst_n(rst_n),.enable(enabled),.duty_pct(DUTY_PERCENT),.pwm(pwm));
    assign rpwm=enabled && !reverse && pwm;
    assign lpwm=enabled && reverse && pwm;
    assign r_en=enabled;
    assign l_en=enabled;
    assign led_run_n=!enabled;
    assign led_reverse_n=!(rst_n && reverse);
    assign led_ready_n=!(rst_n && !active && cooldown==0 && !stop_sync);
    // synthesis translate_off
    initial begin
        if(JOG_CYCLES<1 || LOCK_CYCLES<1 || DUTY_PERCENT<1 || DUTY_PERCENT>30)
            $fatal(1,"Invalid bench jog parameters (duty capped at 30 percent)");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
