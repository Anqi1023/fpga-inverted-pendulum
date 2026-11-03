module tb; localparam signed [31:0] TWO_PI_Q16=411775, PI_Q16=205887;
    function signed [31:0] wrap_theta_q16;
        input signed [31:0] value;
        reg [31:0] remainder_mag;
        reg signed [31:0] signed_remainder;
        integer bit_index;
        begin
            remainder_mag = value[31] ? (~value + 32'd1) : value;
            for (bit_index = 13; bit_index >= 0; bit_index = bit_index - 1) begin
                if (remainder_mag >= ($unsigned(TWO_PI_Q16) << bit_index))
                    remainder_mag = remainder_mag - ($unsigned(TWO_PI_Q16) << bit_index);
                // After each default-scale step the remainder is below the
                // divisor. Explicitly discard provably zero upper bits.
                if (TWO_PI_Q16 == 411775)
                    remainder_mag = remainder_mag & (32'hffffffff >> (13-bit_index));
            end
            signed_remainder = value[31] ? -$signed(remainder_mag) : $signed(remainder_mag);
            if (signed_remainder > PI_Q16)
                wrap_theta_q16 = signed_remainder - TWO_PI_Q16;
            else if (signed_remainder < -PI_Q16)
                wrap_theta_q16 = signed_remainder + TWO_PI_Q16;
            else
                wrap_theta_q16 = signed_remainder;
        end
    endfunction

reg signed [31:0] n,r; integer i;
initial begin
for(i=0;i<200000;i=i+1) begin
case(i) 0:n=32'sh80000000;1:n=32'sh7fffffff;default:n=$random;endcase
r=n%32'sd411775;
if(r>205887)r=r-411775;else if(r< -205887)r=r+411775;
if(wrap_theta_q16(n)!==r)$fatal(1,"wrap mismatch %d",n);
end
$display("PASS 200000 signed32 wrap comparisons including INT_MIN/MAX");
end endmodule