`timescale 1ns/1ps
`default_nettype none
// Shared Q32 U interface; no motor/PWM ownership in this module.
module cruise_parallel_feedback(
 input wire selected_i,ready_i,recovering_i,
 input wire signed [63:0] position_request_i,velocity_request_i,pendulum_request_i,
 input wire [31:0] limit_u_i,
 output wire signed [63:0] request_o
);
 // Controller requests are sign-extended Q16 U products shifted left16.
 // Legal cap10..200U fits8 unsigned bits. Narrow only at this checked interface.
 wire signed [48:0] total=$signed({position_request_i[63],position_request_i[63:16]})+
     $signed({velocity_request_i[63],velocity_request_i[63:16]});
 wire signed [48:0] cap=$signed({25'd0,limit_u_i[7:0],16'd0});
 wire signed [48:0] bounded=total>cap ? cap:total< -cap ? -cap:total;
 wire opposes=recovering_i && bounded!=0 && (pendulum_request_i==0 || bounded[48]!=pendulum_request_i[63]);
 assign request_o=selected_i && ready_i && !opposes ? {bounded[47:0],16'd0}:64'sd0;

endmodule
`default_nettype wire

`default_nettype none
// Bounds come from cruise_recovery and readonly dual-mode position constants.
module cruise_position_feedback(input wire signed [14:0] error_i,input wire near_i,
 output wire signed [63:0] request_o);
 wire signed [17:0] x={{3{error_i[14]}},error_i};
 wire signed [17:0] numerator=near_i ? (x <<< 2):(x <<< 3);
 assign request_o={{30{numerator[17]}},numerator,16'd0};
endmodule
`default_nettype wire
