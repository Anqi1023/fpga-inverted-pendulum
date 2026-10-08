`default_nettype none
// duty_tenths=1 means 0.1%, 100 means 10%. Same-direction updates
// take effect at a period boundary; disable clears immediately.
module pwm_tenths #(
    parameter integer CLK_HZ=27000000, PWM_HZ=1000
)(
    input wire clk, rst_n, enable,
    input wire [8:0] duty_tenths,
    output wire pwm
);
    localparam integer PERIOD=CLK_HZ/PWM_HZ;
    localparam integer CW=(PERIOD>1)?$clog2(PERIOD):1;
    reg [CW-1:0] count=0;
    reg [CW:0] high_ticks=0;
    wire [31:0] requested=(duty_tenths<=300)?(PERIOD/1000)*duty_tenths:0;
    always @(posedge clk) begin
        if(!rst_n || !enable) begin count<=0; high_ticks<=0; end
        else if(count==PERIOD-1) begin count<=0; high_ticks<=requested[CW:0]; end
        else count<=count+1'b1;
    end
    assign pwm=rst_n && enable && count<high_ticks;
    // synthesis translate_off
    initial if(CLK_HZ<=0 || PWM_HZ<=0 || PERIOD<1000 ||
               CLK_HZ%PWM_HZ!=0 || PERIOD%1000!=0)
        $fatal(1,"Require exact PWM period and 0.1-percent resolution");
    // synthesis translate_on
endmodule
`default_nettype wire
