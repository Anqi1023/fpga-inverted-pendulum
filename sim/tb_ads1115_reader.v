`timescale 1ns/1ps
module tb_ads1115_reader;
    reg clk=0,rst_n=0,ack_enable=1;tri1 scl;tri1 sda;integer errors=0;integer timeout;
    wire signed[15:0] sample;wire sample_valid,ready,fault;
    ads1115_reader #(.SAMPLE_INTERVAL_CYCLES(220),.I2C_HALF_PERIOD_CYCLES(2)) dut(
        .clk(clk),.rst_n(rst_n),.scl(scl),.sda(sda),.sample(sample),.sample_valid(sample_valid),.ready(ready),.fault(fault));
    always #5 clk=~clk;
    wire ack_phase=(dut.engine.state==10)||(dut.engine.state==11);
    wire read_phase=(dut.engine.state==12)||(dut.engine.state==13);
    wire [7:0] reply=(dut.state==12)?8'h4E:8'h20;
    wire reply_zero=~reply[dut.engine.bit_index];
    assign sda=(ack_enable&&ack_phase)?1'b0:((read_phase&&reply_zero)?1'b0:1'bz);
    initial begin
        repeat(4)@(negedge clk);rst_n=1;timeout=0;
        while(!sample_valid&&timeout<3000)begin @(negedge clk);timeout=timeout+1;end
        if(!sample_valid||sample!==16'sh4E20||!ready||fault)begin
            $display("FAIL first sample=%h ready=%b fault=%b",sample,ready,fault);errors=errors+1;end
        ack_enable=0;timeout=0;
        while(!fault&&timeout<2000)begin @(negedge clk);timeout=timeout+1;end
        if(!fault||ready)begin $display("FAIL NACK handling ready=%b fault=%b",ready,fault);errors=errors+1;end
        if(errors==0)$display("PASS tb_ads1115_reader");else $display("FAIL tb_ads1115_reader errors=%0d",errors);
        $finish;
    end
endmodule
