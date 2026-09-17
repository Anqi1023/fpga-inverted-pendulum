`default_nettype none
// Logical reset conditioner; external_reset_n pin/polarity are board TODO.
// Asynchronous assertion, two-stage synchronized release, stable release delay.
module reset_guard #(parameter integer RELEASE_CYCLES=270000)(
    input wire clk,
    input wire external_reset_n,
    output reg reset_n=0
);
    localparam integer CW=RELEASE_CYCLES>1 ? $clog2(RELEASE_CYCLES) : 1;
    reg [1:0] release_sync=0;
    reg [CW-1:0] count=0;
    always @(posedge clk or negedge external_reset_n) begin
        if(!external_reset_n) release_sync<=0;
        else release_sync<={release_sync[0],1'b1};
    end
    always @(posedge clk or negedge external_reset_n) begin
        if(!external_reset_n) begin reset_n<=0; count<=0; end
        else if(!release_sync[1]) begin reset_n<=0; count<=0; end
        else if(!reset_n) begin
            if(count==RELEASE_CYCLES-1) reset_n<=1;
            else count<=count+1'b1;
        end
    end
    // synthesis translate_off
    initial if(RELEASE_CYCLES<1) $fatal(1,"RELEASE_CYCLES must be positive");
    // synthesis translate_on
endmodule
`default_nettype wire
