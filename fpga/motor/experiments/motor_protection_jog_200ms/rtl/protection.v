`default_nettype none
// Latched logical protection. All inputs are synchronous to clk; fault/estop
// must span a rising edge to be remembered. rst_n must span an edge and be
// released synchronously. Raw switches need board-level input conditioning.
// This is NOT an independent hardware power cutoff or a clock-failure monitor.
module protection (
    input wire clk,
    input wire rst_n,
    input wire permit_in,
    input wire fault,
    input wire estop,
    input wire stop_request,
    input wire clear_fault,
    output wire permit,
    output reg locked = 1'b1,
    output reg fault_latched = 1'b0,
    output reg estop_latched = 1'b0,
    output reg clear_accepted = 1'b0
);
    // A clear already held at configuration/reset is not a fresh command.
    reg clear_prev = 1'b1;
    wire trip = !permit_in || fault || estop;
    wire clear_event = clear_fault && !clear_prev;

    // Remove permission without waiting for the next PWM boundary or clock.
    // The sequential latch below prevents re-enabling when a trip goes away.
    assign permit = rst_n && permit_in && !fault && !estop && !locked;

    always @(posedge clk) begin
        clear_prev <= clear_fault;
        clear_accepted <= 1'b0;
        if (!rst_n) begin
            locked <= 1'b1;
            fault_latched <= 1'b0;
            estop_latched <= 1'b0;
            clear_prev <= 1'b1;
        end else if (trip) begin
            // Trip wins over clear. Reasons accumulate until a valid clear.
            locked <= 1'b1;
            fault_latched <= fault_latched || fault;
            estop_latched <= estop_latched || estop;
        end else if (clear_event && stop_request) begin
            locked <= 1'b0;
            fault_latched <= 1'b0;
            estop_latched <= 1'b0;
            clear_accepted <= 1'b1;
        end
    end
endmodule
`default_nettype wire
