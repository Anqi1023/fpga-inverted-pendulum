`timescale 1ns/1ps

module sensor_state(
    input wire clk,
    input wire rst_n,

    input wire encoder_a,
    input wire encoder_b,

    input wire signed [15:0] adc_sample,
    input wire adc_sample_valid,

    input wire adc_ready,
    input wire adc_fault,

    output wire signed [31:0] theta,
    output wire signed [31:0] theta_dot,

    output wire signed [31:0] alpha,
    output wire signed [31:0] alpha_dot,

    output wire sensor_valid,

    output reg state_update,

    output reg signed [31:0] raw_encoder,

    output reg signed [15:0] raw_adc,
    output reg signed [15:0] raw_adc_average
);


wire signed [31:0] encoder_count;
wire encoder_ready;
wire encoder_error;


reg signed [31:0] adc_sum;
reg signed [31:0] next_sum;

reg [4:0] adc_count;

reg signed [15:0] adc_average;


wire math_valid;

reg fault_latched;


// ======================================================
// Encoder
// ======================================================

quadrature_decoder #(
    .COUNT_WIDTH(32)
)
enc
(
    .clk(clk),
    .rst_n(rst_n),

    .enc_a(encoder_a),
    .enc_b(encoder_b),

    .position_count(encoder_count),

    .ready(encoder_ready),

    .illegal_transition(encoder_error)
);


// ======================================================
// Math
// ======================================================

sensor_math math
(
    .clk(clk),
    .rst_n(rst_n),

    .encoder_count(encoder_count),

    .adc_average(adc_average),

    .update(state_update),

    .theta(theta),
    .theta_dot(theta_dot),

    .alpha(alpha),
    .alpha_dot(alpha_dot),

    .valid(math_valid)
);



// ======================================================
// Generate update
// Debug mode:
// encoder does not wait ADC
// ======================================================

reg [19:0] update_counter;


always @(posedge clk or negedge rst_n)
begin

    if(!rst_n)
    begin

        adc_sum <= 0;
        next_sum <= 0;

        adc_count <= 0;

        adc_average <= 0;

        state_update <= 0;

        fault_latched <= 0;

        update_counter <= 0;

    end


    else
    begin

        state_update <= 0;


        // 约50Hz更新
        if(update_counter == 20'd540000)
        begin

            update_counter <= 0;

            state_update <= 1'b1;

        end

        else
        begin

            update_counter <= update_counter + 1'b1;

        end



        if(encoder_error || adc_fault)
            fault_latched <= 1;



        // ADC正常时进行平均

        if(adc_ready && adc_sample_valid)
        begin

            if(adc_count == 15)
            begin

                next_sum = adc_sum + adc_sample;

                adc_average <= next_sum[19:4];

                adc_sum <= 0;

                adc_count <= 0;

            end

            else
            begin

                adc_sum <= adc_sum + adc_sample;

                adc_count <= adc_count + 1'b1;

            end

        end


    end

end




// ======================================================
// Debug raw output
// ======================================================

always @(posedge clk or negedge rst_n)
begin

    if(!rst_n)
    begin

        raw_encoder <= 0;

        raw_adc <= 0;

        raw_adc_average <= 0;

    end


    else
    begin

        if(state_update)
        begin

            raw_encoder <= encoder_count;

            raw_adc <= adc_sample;

            raw_adc_average <= adc_average;

        end

    end

end



assign sensor_valid =
        encoder_ready &
        math_valid &
        ~fault_latched;


endmodule