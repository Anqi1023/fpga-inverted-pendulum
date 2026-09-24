module sensor_packet(

input wire clk,
input wire rst_n,

input wire send_i,

input wire signed [31:0] theta,
input wire signed [31:0] theta_dot,

input wire signed [31:0] alpha,
input wire signed [31:0] alpha_dot,

input wire [2:0] state_i,

input wire uart_busy_i,

output reg [7:0] uart_data_o,
output reg uart_start_o

);


always @(posedge clk or negedge rst_n) begin

    if(!rst_n) begin

        uart_data_o  <= 8'd0;
        uart_start_o <= 1'b0;

    end
    else begin

        uart_start_o <= 1'b0;

    end

end


endmodule