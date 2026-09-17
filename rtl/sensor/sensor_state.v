`timescale 1ns/1ps
// A-group standalone acquisition wrapper, not the system top.
// Both sensors are provisionally quadrature encoders; actual types unconfirmed.
// position means rotary ARM count; angle means PENDULUM count (not mm/radians).
// TODO: replace with calibrated scaling parameters.
// A count error latches sensor_valid low until reset; no timeout detection yet.
module sensor_state #(
    parameter integer WIDTH = 32,
    parameter integer SAMPLE_CYCLES = 100000
) (
    input wire clk, input wire rst_n,
    input wire position_enc_a, input wire position_enc_b,
    input wire angle_enc_a, input wire angle_enc_b,
    output wire signed [WIDTH-1:0] angle,
    output wire signed [WIDTH-1:0] angle_vel,
    output wire signed [WIDTH-1:0] position,
    output wire signed [WIDTH-1:0] position_vel,
    output wire sensor_valid
);
    wire position_ready, angle_ready, position_error, angle_error;
    wire position_velocity_valid, angle_velocity_valid;
    wire acquisition_ready = position_ready & angle_ready;
    wire estimator_rst_n = rst_n & acquisition_ready;
    reg fault;
    quadrature_decoder #(.COUNT_WIDTH(WIDTH)) position_decoder (
        .clk(clk), .rst_n(rst_n), .enc_a(position_enc_a), .enc_b(position_enc_b),
        .position_count(position), .ready(position_ready),
        .illegal_transition(position_error)
    );
    quadrature_decoder #(.COUNT_WIDTH(WIDTH)) angle_decoder (
        .clk(clk), .rst_n(rst_n), .enc_a(angle_enc_a), .enc_b(angle_enc_b),
        .position_count(angle), .ready(angle_ready), .illegal_transition(angle_error)
    );
    velocity_estimator #(.WIDTH(WIDTH), .SAMPLE_CYCLES(SAMPLE_CYCLES)) position_speed (
        .clk(clk), .rst_n(estimator_rst_n), .position(position),
        .velocity(position_vel), .valid(position_velocity_valid), .sample_tick()
    );
    velocity_estimator #(.WIDTH(WIDTH), .SAMPLE_CYCLES(SAMPLE_CYCLES)) angle_speed (
        .clk(clk), .rst_n(estimator_rst_n), .position(angle),
        .velocity(angle_vel), .valid(angle_velocity_valid), .sample_tick()
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fault <= 0;
        else if (position_error | angle_error) fault <= 1;
    end
    assign sensor_valid = rst_n & acquisition_ready & position_velocity_valid &
                          angle_velocity_valid & ~fault & ~position_error & ~angle_error;
endmodule
