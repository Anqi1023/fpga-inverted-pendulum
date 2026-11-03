`default_nettype none
// R14L resource fix: legal gain <=200 needs 8 unsigned bits;
// signed33 x signed9 -> signed42, with original full-width config validation.
// Independent velocity P. Error Q16 rad/s, gain U/(rad/s), output Q32 U.
module arm_speed_loop(input wire enabled_i,valid_i,
 input wire signed [31:0] reference_i,speed_i,
 input wire [31:0] gain_i,limit_i,sign_i,
 output wire signed [32:0] error_o,output wire signed [63:0] request_o);
 assign error_o=$signed({reference_i[31],reference_i})-$signed({speed_i[31],speed_i});
 wire signed [41:0] product=$signed(error_o)*$signed({1'b0,gain_i[7:0]});
 wire signed [41:0] cap=$signed({17'd0,limit_i[8:0],16'd0});
 wire config_ok=gain_i<=200 && limit_i>=1 && limit_i<=500 && sign_i<=1;
 // Symmetric saturation commutes with sign reversal. Clamp first so the
 // sign mapper needs 26 rather than 42 bits: 500*65536 < 2^25.
 wire signed [25:0] clipped=product>cap?$signed(cap[25:0]):
                         (product< -cap?-$signed(cap[25:0]):$signed(product[25:0]));
 wire signed [25:0] bounded=sign_i[0] ? clipped:-clipped;
 assign request_o=enabled_i && valid_i && config_ok ? ($signed({{38{bounded[25]}},bounded}) <<< 16):64'sd0;
endmodule
`default_nettype wire
