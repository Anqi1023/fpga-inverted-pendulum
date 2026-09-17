`timescale 1ns/1ps
module tb_sensor_math;
    reg clk=0,rst_n=0,update=0;reg signed[31:0] encoder_count=0;reg signed[15:0] adc_average=19985;
    wire signed[31:0] theta,theta_dot,alpha,alpha_dot;wire valid;integer errors=0;
    sensor_math dut(.clk(clk),.rst_n(rst_n),.encoder_count(encoder_count),.adc_average(adc_average),.update(update),
        .theta(theta),.theta_dot(theta_dot),.alpha(alpha),.alpha_dot(alpha_dot),.valid(valid));
    always #5 clk=~clk;
    task pulse;begin @(negedge clk);update=1;@(negedge clk);update=0;end endtask
    task near;input signed[31:0] got;input signed[31:0] expected;input integer tol;begin
        if((got>expected+tol)||(got<expected-tol))begin $display("FAIL got=%0d expected=%0d",got,expected);errors=errors+1;end end endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1; pulse; near(alpha,0,2);if(valid)errors=errors+1;
        encoder_count=260;adc_average=20241;pulse;
        near(theta,102944,2);near(theta_dot,5147220,2);near(alpha,3992,2);near(alpha_dot,199600,20);
        if(!valid)errors=errors+1;
        adc_average=6782;pulse;near(alpha,205890,4); // about +pi after wrap
        if(errors==0)$display("PASS tb_sensor_math");else $display("FAIL tb_sensor_math errors=%0d",errors);
        $finish;
    end
endmodule
