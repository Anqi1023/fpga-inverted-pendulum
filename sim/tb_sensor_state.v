`timescale 1ns/1ps
module tb_sensor_state;
    reg clk=0,rst_n=0,encoder_a=0,encoder_b=0;
    reg signed[15:0] adc_sample=19985;reg adc_sample_valid=0,adc_ready=1,adc_fault=0;
    wire signed[31:0] theta,theta_dot,alpha,alpha_dot;wire sensor_valid,state_update;integer errors=0,i;
    wire signed [31:0] raw_encoder;wire signed [15:0] raw_adc,raw_adc_average;
    sensor_state dut(.clk(clk),.rst_n(rst_n),.encoder_a(encoder_a),.encoder_b(encoder_b),.adc_sample(adc_sample),
        .adc_sample_valid(adc_sample_valid),.adc_ready(adc_ready),.adc_fault(adc_fault),.theta(theta),.theta_dot(theta_dot),
        .alpha(alpha),.alpha_dot(alpha_dot),.sensor_valid(sensor_valid),.state_update(state_update),.raw_encoder(raw_encoder),.raw_adc(raw_adc),.raw_adc_average(raw_adc_average));
    always #5 clk=~clk;
    task adc_window;input signed[15:0] value;begin
        for(i=0;i<16;i=i+1)begin @(negedge clk);adc_sample=value;adc_sample_valid=1;@(negedge clk);adc_sample_valid=0;end
        repeat(3)@(negedge clk);
    end endtask
    task qstep;input[1:0]ab;begin @(negedge clk);{encoder_a,encoder_b}=ab;repeat(4)@(negedge clk);end endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;repeat(5)@(negedge clk);
        adc_window(19985);if(sensor_valid)begin $display("FAIL early valid");errors=errors+1;end
        qstep(2'b01);qstep(2'b11);qstep(2'b10);qstep(2'b00);
        adc_window(20241);
        if(raw_encoder!==4 || raw_adc!==20241 || raw_adc_average!==20241) begin $display("FAIL raw snapshot");errors=errors+1;end
        if(!sensor_valid)begin $display("FAIL valid");errors=errors+1;end
        if(theta<1580||theta>1590)begin $display("FAIL theta=%0d",theta);errors=errors+1;end
        if(alpha<3990||alpha>3994)begin $display("FAIL alpha=%0d",alpha);errors=errors+1;end
        // Distinguish the last sample from the mean; changing ADC afterward must not alter the snapshot.
        for(i=0;i<16;i=i+1)begin @(negedge clk);adc_sample=100+i;adc_sample_valid=1;@(negedge clk);adc_sample_valid=0;end
        repeat(3)@(negedge clk);adc_sample=-1000;
        if(raw_adc!==115 || raw_adc_average!==107 || raw_encoder!==4)begin $display("FAIL window snapshot");errors=errors+1;end
        adc_fault=1;@(negedge clk);adc_fault=0;@(negedge clk);
        if(sensor_valid)begin $display("FAIL fault latch");errors=errors+1;end
        if(errors==0)$display("PASS tb_sensor_state");else $display("FAIL tb_sensor_state errors=%0d",errors);
        $finish;
    end
endmodule
