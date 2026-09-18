`timescale 1ns/1ps
module tb_sensor_uart_packet;
`ifdef RAW_DEBUG_TEST
    localparam integer RAW=1;
`else
    localparam integer RAW=0;
`endif
    localparam integer CPB=4;
    reg clk=0,rst_n=0,send=0;wire tx,busy;integer errors=0,i;reg[7:0] got[0:19];reg[7:0] expected[0:19];
    always #5 clk=~clk;
    sensor_uart_packet #(.CLKS_PER_BIT(CPB),.RAW_DEBUG(RAW)) dut(.clk(clk),.rst_n(rst_n),.send(send),
        .theta(32'h12345678),.theta_dot(32'h89ABCDEF),.alpha(32'h01020304),.alpha_dot(32'hFFEEDDCC),
        .sensor_valid(1'b1),.raw_encoder(-32'sd12345),.raw_adc(-16'sd1234),.raw_adc_average(16'sd19985),.tx(tx),.busy(busy));
    task receive_byte;output[7:0] value;integer bitno;begin
        @(negedge tx);repeat(CPB+CPB/2)@(posedge clk);
        for(bitno=0;bitno<8;bitno=bitno+1)begin value[bitno]=tx;repeat(CPB)@(posedge clk);end
        if(tx!==1'b1)errors=errors+1;
    end endtask
    initial begin #100000; $display("FAIL UART timeout"); errors=errors+1; $finish; end
    initial begin
        expected[0]=8'hA5;expected[1]=8'h5A;
        expected[2]=8'h78;expected[3]=8'h56;expected[4]=8'h34;expected[5]=8'h12;
        expected[6]=8'hEF;expected[7]=8'hCD;expected[8]=8'hAB;expected[9]=8'h89;
        expected[10]=8'h04;expected[11]=8'h03;expected[12]=8'h02;expected[13]=8'h01;
        expected[14]=8'hCC;expected[15]=8'hDD;expected[16]=8'hEE;expected[17]=8'hFF;expected[18]=8'h01;
        if(RAW) begin
            expected[1]=8'h5B;
            expected[2]=8'hC7;expected[3]=8'hCF;expected[4]=8'hFF;expected[5]=8'hFF;
            expected[6]=8'h2E;expected[7]=8'hFB;expected[8]=8'h11;expected[9]=8'h4E;
            for(i=10;i<18;i=i+1)expected[i]=0;
        end
        expected[19]=8'hFF;for(i=0;i<19;i=i+1)expected[19]=expected[19]^expected[i];
        repeat(3)@(negedge clk);rst_n=1;@(negedge clk);send=1;@(negedge clk);send=0;
        for(i=0;i<20;i=i+1)receive_byte(got[i]);
        for(i=0;i<20;i=i+1)if(got[i]!==expected[i])begin $display("byte %0d got %02x expected %02x",i,got[i],expected[i]);errors=errors+1;end
        wait(!busy);if(errors==0)$display("PASS tb_sensor_uart_packet");else $display("FAIL tb_sensor_uart_packet errors=%0d",errors);
        #20;$finish;
    end
endmodule
