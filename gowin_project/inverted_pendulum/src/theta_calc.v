module theta_calc (
    input  wire signed [31:0] count_i,
    output wire signed [31:0] theta_mrad_o
);

    // theta[mrad] = count * 2*pi*1000 / 978
    // 2*pi*1000 ≈ 6283

    wire signed [63:0] mult_temp;

    assign mult_temp =
        $signed(count_i) * 64'sd6283;

    assign theta_mrad_o =
        mult_temp / 64'sd978;

endmodule