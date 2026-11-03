`default_nettype none
// Logical interface ONLY: en=1 means requested enable, not a verified pin level.
// All controls must already be synchronous to clk. No raw buttons here.
module motor_driver #(
    parameter integer CLK_HZ = 27000000,
    parameter integer PWM_HZ = 20000,
    parameter integer STOP_CYCLES = 27000000
)(
    input wire clk, input wire rst_n,
    input wire [1:0] command, // 00 STOP, 01 FORWARD, 10 REVERSE, 11 STOP
    input wire arm, input wire permit,
    input wire [9:0] duty_permille,
    output wire motor_pwm, output wire motor_dir, output wire motor_en,
    output wire [2:0] state_o
);
    localparam [2:0] OFF=0, READY=1, FORWARD=2, REVERSE=3, WAIT_STOP=4;
    localparam integer SW = (STOP_CYCLES > 1) ? $clog2(STOP_CYCLES) : 1;
    reg [2:0] state = OFF;
    reg [SW-1:0] wait_count = 0;
    reg arm_d = 1; // A held-high arm at power-up is not a fresh arm event.
    reg direction = 0;
    // 2026-10-18: 与 control_u_adapter / motor_protected 同步放开到 100%
    wire duty_ok = (duty_permille <= 1000);
    wire running = (state == FORWARD || state == REVERSE);
    wire match_command = (state == FORWARD && command == 1) ||
                         (state == REVERSE && command == 2);
    // Removal of permit/STOP/invalid duty suppresses output immediately.
    assign motor_en = rst_n && permit && duty_ok && running && match_command;
    assign motor_dir = direction;
    assign state_o = state;
    pwm_gen #(.CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ)) pwm_i (
        .clk(clk), .rst_n(rst_n), .enable(motor_en),
        .duty_permille(duty_permille), .pwm(motor_pwm));
    always @(posedge clk) begin
        arm_d <= arm;
        if (!rst_n) begin
            // Hold DIR through reset: disable first, change it only at next start.
            state <= WAIT_STOP; wait_count <= 0; arm_d <= 1;
        end else if (!permit || !duty_ok) begin
            state <= WAIT_STOP; wait_count <= 0;
        end else begin
            case (state)
                // OFF -> READY 只要求 arm 出现上升沿，不再要求 command==STOP。
                // 双向平衡控制里 command 会在 01/10 之间来回，若坚持"必须先看到
                // STOP 才允许重新上电"，一旦换向进入 OFF 而 u 又没有恰好落到 0，
                // 就会永久卡在 OFF（电机彻底不转）。真正的停歇由 WAIT_STOP 的
                // STOP_CYCLES 和 ibt2_adapter 的换向消隐保证。
                OFF: if (arm && !arm_d) state <= READY;
                READY: begin
                    if (command == 1) begin direction <= 0; state <= FORWARD; end
                    else if (command == 2) begin direction <= 1; state <= REVERSE; end
                end
                FORWARD, REVERSE: if (!match_command) begin
                    state <= WAIT_STOP; wait_count <= 0;
                end
                WAIT_STOP: begin
                    if (wait_count == STOP_CYCLES-1) begin state <= OFF; wait_count <= 0; end
                    else wait_count <= wait_count + 1'b1;
                end
                default: begin state <= WAIT_STOP; wait_count <= 0; end
            endcase
        end
    end
    // synthesis translate_off
    initial if (STOP_CYCLES < 1) $fatal(1, "STOP_CYCLES must be positive");
    // synthesis translate_on
endmodule
`default_nettype wire
