`timescale 1ns/1ps

// R3: calibrate while hanging still; retain upright alpha=0 and all Q16 interfaces.
// Circular averaging handles the raw-code seam. First valid angle is primed before AD.
module sensor_math #(
 
    
    parameter signed [31:0] ALPHA_ZERO_RAW = 32'sd20449,


    parameter signed [31:0] ALPHA_WRAP_RAW=32'sd6782,

    parameter signed [31:0] ALPHA_GAIN_Q24=32'sd3832,

    parameter integer      ALPHA_SPAN_RAW = 27512,

    parameter signed [31:0] TWO_PI_Q16=32'sd411775,

    parameter signed [31:0] THETA_GAIN_Q24=32'sd101360,
    parameter signed [31:0] SPEED_GAIN_Q16=32'sd79188,

    parameter signed [31:0] ALPHA_SETPOINT_Q16 = 32'sd0,
    parameter integer      ZERO_SAMPLES       = 64,
    parameter integer      CALIBRATE_DOWN     = 1, // R3: power on with pendulum hanging still

    parameter integer      SOFTRAMP_MS        = 300,

    parameter integer      UPDATE_HZ          = 200
) (

    input wire clk,
    input wire rst_n,

    input wire signed [31:0] encoder_count,

    input wire signed [15:0] adc_average,

    input wire update,

    input wire cal_gate,
    input wire rebase_velocity, // D05: ADC re-acquired; restart only pendulum derivative

    output reg signed [31:0] theta,
    output reg signed [31:0] theta_dot,

    output reg signed [31:0] alpha,
    output reg signed [31:0] alpha_dot,

    output reg valid,

    output wire signed [31:0] dbg_alpha_zero,
    output wire signed [31:0] dbg_alpha_wrap,
    output wire               dbg_zero_done
);


reg signed [31:0] last_encoder;
reg signed [31:0] last_alpha;

reg baseline;


reg signed [63:0] theta_calc;
reg signed [63:0] speed_calc;
reg signed [63:0] alpha_calc;


reg signed [31:0] alpha_next;


reg signed [31:0] theta_raw;
reg signed [31:0] theta_wrap;



reg signed [31:0] alpha_dot_filter;

reg signed [31:0] alpha_dot_raw;

reg signed [31:0] alpha_delta;

localparam signed [31:0] PI_Q16 = 32'sd205887;

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

    // Default scale has a proven signed21 bound; two corrections replace 14 wide stages.
    function signed [31:0] wrap_alpha_bounded;
        input signed [31:0] value;
        reg signed [20:0] v;
        integer k;
        begin
            v=value[20:0];
            for(k=0;k<2;k=k+1) begin
                if(v>21'sd205887) v=v-21'sd411775;
                else if(v < -21'sd205887) v=v+21'sd411775;
            end
            wrap_alpha_bounded={{11{v[20]}},v};
        end
    endfunction


reg signed [31:0] zero_acc;
reg signed [31:0] alpha_zero;
reg [7:0]         zero_cnt;
reg               zero_done;
// Calibration anchor and circular short-delta average; dbg_alpha_zero is still upright raw zero.
reg [14:0]        zero_anchor;
reg               cal_angle_primed;
wire signed [16:0] cal_delta=$signed({adc_average[15],adc_average})-
    $signed({2'b0,zero_anchor});
wire signed [16:0] cal_delta_short=(cal_delta>ALPHA_SPAN_RAW/2) ?
    cal_delta-ALPHA_SPAN_RAW : (cal_delta < -(ALPHA_SPAN_RAW/2)) ?
    cal_delta+ALPHA_SPAN_RAW : cal_delta;
wire signed [31:0] cal_top_unwrapped=$signed({1'b0,zero_anchor})+
    (($signed(zero_acc)+$signed(cal_delta_short))/ZERO_SAMPLES)+(ALPHA_SPAN_RAW/2);
wire signed [31:0] cal_top_raw=(cal_top_unwrapped>=ALPHA_SPAN_RAW) ?
    cal_top_unwrapped-ALPHA_SPAN_RAW : cal_top_unwrapped;

reg signed [31:0] enc_zero_acc;
reg [7:0]         enc_zero_cnt;
reg signed [31:0] enc_zero;
reg               enc_zero_done;


wire signed [31:0] alpha_wrap_raw =
    zero_done ? (alpha_zero - (ALPHA_SPAN_RAW / 2))
              : ALPHA_WRAP_RAW;

assign dbg_alpha_zero = alpha_zero;
assign dbg_alpha_wrap = alpha_wrap_raw;
assign dbg_zero_done  = zero_done;

localparam integer RAMP_STEPS =
    (SOFTRAMP_MS > 0) ? ((SOFTRAMP_MS * UPDATE_HZ) / 1000) : 1;
localparam integer RAMP_STEP_ABS =
    (SOFTRAMP_MS > 0)
        ? (((ALPHA_SETPOINT_Q16 >= 0 ? ALPHA_SETPOINT_Q16 : -ALPHA_SETPOINT_Q16)
            + RAMP_STEPS - 1) / RAMP_STEPS)
        : (ALPHA_SETPOINT_Q16 >= 0 ? ALPHA_SETPOINT_Q16 : -ALPHA_SETPOINT_Q16);
localparam signed [31:0] RAMP_STEP_Q16 =
    (ALPHA_SETPOINT_Q16 >= 0) ? RAMP_STEP_ABS : -RAMP_STEP_ABS;

reg signed [31:0] alpha_set_ramp;



always @(posedge clk or negedge rst_n)
begin

    if(!rst_n)
    begin

        theta <= 0;
        theta_dot <= 0;

        alpha <= 0;
        alpha_dot <= 0;

        last_encoder <= 0;
        last_alpha <= 0;

        baseline <= 0;

        valid <= 0;


        theta_calc <= 0;
        speed_calc <= 0;
        alpha_calc <= 0;

        alpha_next <= 0;


        alpha_dot_filter <= 0;
        alpha_dot_raw <= 0;
        alpha_delta <= 0;

        zero_acc   <= 0;
        alpha_zero <= 0;
        zero_cnt   <= 0;
        zero_done  <= 0;
        zero_anchor <= 0;
        cal_angle_primed <= 0;

        enc_zero_acc  <= 0;
        enc_zero_cnt  <= 0;
        enc_zero      <= 0;
        enc_zero_done <= 0;

        alpha_set_ramp <= 0;

    end


    else if(update)
    begin
        cal_angle_primed <= zero_done;



        theta_calc =
            $signed(encoder_count - enc_zero)
            *
            $signed(THETA_GAIN_Q24);


        speed_calc =
            $signed(encoder_count-last_encoder)
            *
            $signed(SPEED_GAIN_Q16);




        if(ALPHA_SETPOINT_Q16 >= 0)
        begin
            if(alpha_set_ramp + 2*RAMP_STEP_Q16 >= ALPHA_SETPOINT_Q16)
                alpha_set_ramp <= ALPHA_SETPOINT_Q16;
            else
                alpha_set_ramp <= alpha_set_ramp + RAMP_STEP_Q16;
        end
        else
        begin
            if(alpha_set_ramp + 2*RAMP_STEP_Q16 <= ALPHA_SETPOINT_Q16)
                alpha_set_ramp <= ALPHA_SETPOINT_Q16;
            else
                alpha_set_ramp <= alpha_set_ramp + RAMP_STEP_Q16;
        end



        if(!zero_done)
        begin
            if(CALIBRATE_DOWN!=0 && cal_gate)
            begin
                if(zero_cnt==0) begin
                    zero_anchor<=adc_average[14:0]; zero_acc<=0; zero_cnt<=1;
                end else if(zero_cnt==ZERO_SAMPLES-1) begin
                    alpha_zero<=cal_top_raw; zero_done<=1'b1;
                end else begin
                    zero_acc<=zero_acc+$signed(cal_delta_short);
                    zero_cnt<=zero_cnt+1'b1;
                end
            end
            else if(CALIBRATE_DOWN==0 && cal_gate && (zero_cnt == ZERO_SAMPLES - 1))
            begin
                alpha_zero <= (zero_acc + $signed(adc_average)) / ZERO_SAMPLES;
                zero_done  <= 1'b1;
            end
            else if(CALIBRATE_DOWN==0 && cal_gate)
            begin
                zero_acc <= zero_acc + $signed(adc_average);
                zero_cnt <= zero_cnt + 1'b1;
            end
        end


        if(!enc_zero_done)
        begin
            if(cal_gate && (enc_zero_cnt == ZERO_SAMPLES - 1))
            begin
                enc_zero      <= (enc_zero_acc + encoder_count) / ZERO_SAMPLES;
                enc_zero_done <= 1'b1;
            end
            else if(cal_gate)
            begin
                enc_zero_acc <= enc_zero_acc + encoder_count;
                enc_zero_cnt <= enc_zero_cnt + 1'b1;
            end
        end



        alpha_calc =
            ($signed({{48{adc_average[15]}},adc_average})
            -
            alpha_zero)
            *
            ALPHA_GAIN_Q24;


        alpha_next = alpha_calc[39:8] - alpha_set_ramp;


        if(!zero_done)
            alpha_next = 32'sd0;


        if(CALIBRATE_DOWN!=0 && ALPHA_GAIN_Q24>0 && ALPHA_GAIN_Q24<=3832 &&
           ALPHA_SPAN_RAW>0 && ALPHA_SPAN_RAW<=32767 &&
           ALPHA_SETPOINT_Q16>=-10000 && ALPHA_SETPOINT_Q16<=10000 && TWO_PI_Q16==411775)
            alpha_next = wrap_alpha_bounded(alpha_next);
        else alpha_next = wrap_theta_q16(alpha_next);




        alpha_delta = alpha_next - last_alpha;

        if (alpha_delta > PI_Q16)
            alpha_delta = alpha_delta - TWO_PI_Q16;
        else if (alpha_delta < -PI_Q16)
            alpha_delta = alpha_delta + TWO_PI_Q16;

        alpha_dot_raw =
            alpha_delta
            *
            UPDATE_HZ;




        if(baseline && !rebase_velocity && (CALIBRATE_DOWN==0 || cal_angle_primed))
        begin

            alpha_dot_filter <=
                (alpha_dot_raw + alpha_dot_filter)
                >>> 1;


            alpha_dot <= alpha_dot_filter;

        end

        else
        begin

            alpha_dot_filter <= 0;

            alpha_dot <= 0;

        end





        if(enc_zero_done)
        begin
            theta_raw  = theta_calc[39:8];
            theta_wrap = wrap_theta_q16(theta_raw);
            theta <= theta_wrap;
        end
        else
            theta <= 0;


        if(baseline)
            theta_dot <= speed_calc[31:0];

        else
            theta_dot <= 0;



        alpha <= alpha_next;


        last_encoder <= encoder_count;

        last_alpha <= alpha_next;


        baseline <= 1;


        valid <= baseline && zero_done && enc_zero_done && (CALIBRATE_DOWN==0 || cal_angle_primed);


    end


end


// synthesis translate_off
initial begin
    if(CALIBRATE_DOWN!=0 && (ZERO_SAMPLES<2 || ZERO_SAMPLES>256 || ALPHA_SPAN_RAW<2 || ALPHA_SPAN_RAW>32767 || ALPHA_SPAN_RAW%2!=0))
        $fatal(1,"Invalid hanging calibration parameters");
    if (ALPHA_SPAN_RAW > 0 && ALPHA_GAIN_Q24 > 0) begin
        $display("[sensor_math] ALPHA scale: 1 code = %0.6f rad; %0d codes = %0.3f rad (%0.1f deg)",
                 ALPHA_GAIN_Q24 / 256.0 / 65536.0,
                 ALPHA_SPAN_RAW,
                 ALPHA_SPAN_RAW * ALPHA_GAIN_Q24 / 256.0 / 65536.0,
                 ALPHA_SPAN_RAW * ALPHA_GAIN_Q24 / 256.0 / 65536.0 * 57.29578);
        $display("[sensor_math] wrap boundary = zero - %0d codes (runtime)", ALPHA_SPAN_RAW / 2);
    end
end

// synthesis translate_on
endmodule

