`timescale 1ns/1ps
`default_nettype none
// Low speed: reciprocal encoder-edge period. High speed: original 5ms count speed.
// A 29-step sequential divider avoids a large combinational variable divider.
// Reversal/count discontinuity requires two fresh, same-direction edges.
module hybrid_arm_speed #(
    parameter integer CLK_HZ=27000000,
    parameter integer NUMERATOR_Q16_US=395940000,
    parameter integer HANDOFF_CONTROL=0
)(
    input wire clk,rst_n,sample_i,
    input wire signed [31:0] count_i,raw_speed_i,
    input wire [6:0] timeout_ms_i,
    output reg signed [31:0] speed_o,
    output reg valid_o,
    output wire [36:0] status_o,
    output wire trace_reverse_o,
    output wire [16:0] trace_period_o,
    output wire [4:0] trace_flags_o
);
    localparam integer US_DIV=CLK_HZ/1000000;
    localparam integer UW=US_DIV>1 ? $clog2(US_DIV) : 1;
    reg [UW-1:0] us_count;
    wire us_tick=us_count==US_DIV-1;
    wire [6:0] timeout_ms=timeout_ms_i<10 ? 7'd10 : timeout_ms_i>100 ? 7'd100 : timeout_ms_i;
    wire [16:0] timeout_us=timeout_ms*17'd1000;
    reg signed [31:0] previous_count;
    reg baseline,have_edge,negative,period_valid,fast_mode;
    reg [16:0] age_us,period_us;
    // status: [19:17] reason, [16:0] us since latest observed edge (saturating).
    // 0 acquiring,1 period,2 count,3 aging,4 timeout fade,5 stopped,6 reversal,7 discontinuity.
    reg [2:0] reason;
    reg [16:0] valid_age_us;
    reg [5:0] fade_steps;
    wire [31:0] magnitude=speed_o[31] ? -speed_o : speed_o;
    wire [31:0] fade_amount=(magnitude>>4)+((magnitude[3:0]!=0)?32'd1:32'd0);
    assign status_o={valid_age_us,reason,age_us};
    wire signed [31:0] delta=count_i-previous_count;
    wire edge_seen=baseline && delta!=0;
    wire single_edge=(delta==32'sd1 || delta==-32'sd1);
    wire direction_changed=have_edge && delta[31]!=negative;
    wire restart_edge=edge_seen && (!single_edge || !have_edge || direction_changed);
    wire short_period=HANDOFF_CONTROL && edge_seen && single_edge && have_edge &&
        !direction_changed && age_us==0;
    assign trace_reverse_o=edge_seen && single_edge && direction_changed;
    assign trace_period_o=period_us;
    assign trace_flags_o={fast_mode,period_valid,have_edge,negative,busy};
    wire [16:0] elapsed_period=age_us>period_us ? age_us : period_us;
    // R15K: an independently timestamped 0.5..2ms window must contain
    // exactly one valid encoder edge agreeing with the latest direction.
    // Multiple edges/jitter/discontinuities cannot seed provisional authority.
    // SV=1 with reason0/6 identifies PROVISIONAL, never confirmed/stopped.
    reg [10:0] window_age_us;
    reg window_seeded,window_one_edge,window_multiple,window_negative,temporary_active;
    reg [2:0] division_reason;
    wire window_proof=HANDOFF_CONTROL && window_seeded && have_edge &&
        !period_valid && window_age_us>=11'd500 && window_age_us<=11'd2000 &&
        window_one_edge && !window_multiple && window_negative==negative && age_us<17'd8000;
    // A single encoder count / at most 2ms is >= this conservative ceiling.
    // This is a compile-time scale bound, so no second division is needed.
    localparam integer TEMP_BASE=NUMERATOR_Q16_US/2000>16384 ? 16384:NUMERATOR_Q16_US/2000;
    // At Kv200 the 0.25rad/s ceiling contributes at most 50U = 5% PWM.
    // Withdraw that ceiling every 2ms; no temporary authority beyond 8ms.
    wire [14:0] temporary_ceiling=age_us<17'd2000 ? TEMP_BASE:
        age_us<17'd4000 ? (TEMP_BASE-(TEMP_BASE>>2)):age_us<17'd6000 ? (TEMP_BASE>>1):
        age_us<17'd8000 ? (TEMP_BASE>>2):15'd0;
    reg busy,division_negative;
    reg [4:0] steps;
    reg [28:0] dividend,quotient;
    reg [16:0] divisor;
    reg [17:0] remainder;
    wire [17:0] trial={remainder[16:0],dividend[28]};
    wire subtract=trial>={1'b0,divisor};
    wire [28:0] next_quotient={quotient[27:0],subtract};
    wire [17:0] next_remainder=subtract ? trial-{1'b0,divisor} : trial;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            us_count<=0;previous_count<=0;baseline<=0;have_edge<=0;negative<=0;
            period_valid<=0;fast_mode<=0;age_us<=0;period_us<=0;
            busy<=0;steps<=0;dividend<=0;quotient<=0;divisor<=0;remainder<=0;
            division_negative<=0;speed_o<=0;valid_o<=0;reason<=0;fade_steps<=0;valid_age_us<=17'h1ffff;
            window_age_us<=0;window_seeded<=0;window_one_edge<=0;window_multiple<=0;window_negative<=0;
            temporary_active<=0;division_reason<=0;
        end else begin
            if(us_tick) begin
                us_count<=0;
                if(valid_age_us<17'h1ffff)valid_age_us<=valid_age_us+1'b1;
                if(age_us<17'h1ffff)age_us<=age_us+1'b1;
            end else us_count<=us_count+1'b1;
            if(HANDOFF_CONTROL)begin
                if(us_tick && window_age_us!=11'h7ff)window_age_us<=window_age_us+1'b1;
                // A coincident restart retains the pre-edge window timestamp.
                if(sample_i && !restart_edge)begin
                    window_age_us<=0;window_seeded<=1;window_one_edge<=0;window_multiple<=0;
                end
                if(edge_seen)begin
                    if(!single_edge)window_multiple<=1;
                    else if(sample_i && !restart_edge)begin
                        window_one_edge<=1;window_multiple<=0;window_negative<=delta[31];
                    end else if(window_one_edge)window_multiple<=1;
                    else begin window_one_edge<=1;window_negative<=delta[31];end
                end
            end
            previous_count<=count_i;baseline<=1;
            if(edge_seen) begin
                age_us<=0;fade_steps<=0;
                if(single_edge)begin
                    negative<=delta[31];have_edge<=1;
                    if(have_edge && !direction_changed && age_us!=0)begin
                        period_us<=age_us;period_valid<=1;valid_age_us<=0;
                    end else begin period_us<=0;period_valid<=0;end
                end else begin have_edge<=0;period_valid<=0;period_us<=0;end
            end
            // Priority invalidation also cancels a division from the old direction.
            if(restart_edge || short_period) begin
                busy<=0;valid_o<=0;speed_o<=0;fast_mode<=0;
                temporary_active<=0;
                reason<=(!single_edge || short_period) ? 3'd7 : direction_changed ? 3'd6 : 3'd0;
                if(short_period)begin have_edge<=0;period_valid<=0;window_multiple<=1;end
            end else if(HANDOFF_CONTROL && temporary_active && !period_valid && age_us>=17'd8000)begin
                busy<=0;valid_o<=0;speed_o<=0;temporary_active<=0;
            end else if(!edge_seen && age_us>=timeout_us)begin
                have_edge<=0;period_valid<=0;busy<=0;fast_mode<=0;
                temporary_active<=0;
                // Bounded 64 estimator samples (1 kHz in top): decay 1/16 per sample, then exact zero.
                // Do not route coarse raw speed while fading or stopped.
                if(sample_i)begin
                    if(magnitude==0 || fade_steps==63)begin
                        valid_o<=0;speed_o<=0;reason<=5;
                    end else begin
                        fade_steps<=fade_steps+1'b1;reason<=4;
                        speed_o<=speed_o[31] ? -$signed(magnitude-fade_amount) : $signed(magnitude-fade_amount);
                    end
                end
            end else if(busy)begin
                dividend<={dividend[27:0],1'b0};quotient<=next_quotient;remainder<=next_remainder;
                if(steps==1)begin
                    busy<=0;valid_o<=1;
                    speed_o<=division_negative ? -$signed({3'd0,next_quotient}):$signed({3'd0,next_quotient});
                    temporary_active<=0;
                    if(HANDOFF_CONTROL)reason<=division_reason;
                end else steps<=steps-1'b1;
            end else if(sample_i)begin
                if(!period_valid || elapsed_period==0)begin
                    if(window_proof && temporary_ceiling!=0)begin
                        // Window evidence proves a speed magnitude above the
                        // conservative ceiling; only that capped authority enters.
                        speed_o<=negative ? -$signed({17'd0,temporary_ceiling}):$signed({17'd0,temporary_ceiling});
                        valid_o<=1;temporary_active<=1;
                    end else if(HANDOFF_CONTROL && temporary_active && temporary_ceiling!=0)begin
                        // Age is monotone until a new edge. A new opposite edge
                        // clears authority; a same-sign edge creates a period.
                        // Thus this ceiling can only decrease in this branch.
                        speed_o<=negative ? -$signed({17'd0,temporary_ceiling}):$signed({17'd0,temporary_ceiling});
                        valid_o<=1;
                    end else begin speed_o<=0;valid_o<=0;temporary_active<=0;end
                end
                // Raw 5ms data may still belong to the old direction after reversal.
                else if((elapsed_period<=17'd2048 || (fast_mode && elapsed_period<17'd4096)) &&
                        raw_speed_i!=0 && raw_speed_i[31]==negative)begin
                    speed_o<=raw_speed_i;valid_o<=1;fast_mode<=1;reason<=2;
                    temporary_active<=0;
                end else begin
                    // Keep the temporary marker until the formal result commits.
                    if(!HANDOFF_CONTROL || !temporary_active)reason<=age_us>period_us ? 3'd3:3'd1;
                    fast_mode<=0;busy<=1;steps<=29;dividend<=NUMERATOR_Q16_US;
                    quotient<=0;remainder<=0;divisor<=elapsed_period;division_negative<=negative;
                    division_reason<=age_us>period_us ? 3'd3:3'd1;
                end
            end
        end
    end
    // synthesis translate_off
    initial if(CLK_HZ<1000000 || CLK_HZ%1000000!=0 || NUMERATOR_Q16_US<1 || NUMERATOR_Q16_US>=536870912)
        $fatal(1,"Invalid hybrid speed clock/scale");
    // synthesis translate_on
endmodule
`default_nettype wire
