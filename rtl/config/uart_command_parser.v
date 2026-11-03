`timescale 1ns/1ps
`default_nettype none
// Uppercase ASCII, decimal IDs and signed decimal values. CR/LF/CRLF accepted.
// Maximum 3 tokens, bounded digits. An invalid line never becomes a partial SET.
// Opcodes: INFO=0 GET_ALL=1 SET=2 APPLY=3 STOP=4 START=5 DEFAULT=6 REARM=7 BAD=15.
module uart_command_parser(
    input wire clk, rst_n,
    input wire [7:0] data_i,
    input wire valid_i, error_i,
    output reg [3:0] command_o,
    output reg [7:0] id_o,
    output reg signed [31:0] value_o,
    output reg command_valid_o
);
    reg [63:0] word0,word1;
    reg [1:0] token;
    reg in_token, have_line, bad, id_bad, value_bad, negative;
    reg [6:0] length0,length1,length2,line_length;
    reg [7:0] id_value;
    reg [31:0] magnitude;
    wire space=(data_i==8'd32 || data_i==8'd9);
    wire digit=(data_i>="0" && data_i<="9");
    wire [3:0] decimal_digit=data_i-8'd48;
    task reset_line;
        begin
            word0<=0; word1<=0; token<=0; in_token<=0; have_line<=0;
            bad<=0; id_bad<=0; value_bad<=0; negative<=0;
            length0<=0; length1<=0; length2<=0; line_length<=0;
            id_value<=0; magnitude<=0;
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_line; command_o<=15; command_valid_o<=0; id_o<=0; value_o<=0;
        end else begin
            command_valid_o<=0;
            if (error_i) bad<=1;
            if (valid_i) begin
                if (data_i==13 || data_i==10) begin
                    if (have_line || bad) begin
                        command_valid_o<=1; command_o<=15;
                        id_o<=id_value; value_o<=negative ? -$signed(magnitude) : $signed(magnitude);
                        if (!bad) begin
                            if (word0=="STOP" && length1==0 && length2==0) command_o<=4;
                            else if (word0=="START" && length1==0 && length2==0) command_o<=5;
                            else if (word0=="REARM" && length1==0 && length2==0) command_o<=7;
                            else if (word0=="APPLY" && length1==0 && length2==0) command_o<=3;
                            else if (word0=="DEFAULT" && length1==0 && length2==0) command_o<=6;
                            else if (word0=="GET" && word1=="INFO" && length2==0) command_o<=0;
                            else if (word0=="GET" && word1=="ALL" && length2==0) command_o<=1;
                            else if (word0=="GET" && word1=="POS" && length2==0) command_o<=8;
                            else if (word0=="TRACE" && word1=="ARM" && length2==0) command_o<=9;
                            else if (word0=="TRACE" && word1=="GET" && length2==0) command_o<=10;
                            else if (word0=="TRACE" && word1=="STAT" && length2==0) command_o<=11;
                            else if (word0=="TRACE" && word1=="FREEZE" && length2==0) command_o<=12;
                            else if (word0=="TRACE" && word1=="REF" && length2==0) command_o<=13;
                            else if (word0=="SET" && length1!=0 && !id_bad &&
                                     length2!=0 && !value_bad && !(negative && length2==1)) command_o<=2;
                        end
                    end
                    reset_line;
                end else if (space) begin
                    if (in_token) begin
                        in_token<=0;
                        if (token==2) token<=3; else token<=token+1'b1;
                    end
                end else begin
                    have_line<=1; in_token<=1;
                    if (line_length==63) bad<=1; else line_length<=line_length+1'b1;
                    if (data_i<33 || data_i>126) bad<=1;
                    case(token)
                        0: if (length0>=8) bad<=1;
                           else begin word0<={word0[55:0],data_i}; length0<=length0+1'b1; end
                        1: begin
                            if (length1>=8) bad<=1;
                            else begin word1<={word1[55:0],data_i}; length1<=length1+1'b1; end
                            if (!digit || id_value>25 || (id_value==25 && decimal_digit>5)) id_bad<=1;
                            else id_value<=id_value*8'd10+decimal_digit;
                        end
                        2: begin
                            if (length2>=11) bad<=1; else length2<=length2+1'b1;
                            if (data_i=="-" && length2==0) negative<=1;
                            else if (!digit || magnitude>32'd214748364 ||
                                     (magnitude==32'd214748364 && decimal_digit>7)) value_bad<=1;
                            else magnitude<=magnitude*32'd10+decimal_digit;
                        end
                        default: bad<=1;
                    endcase
                end
            end
        end
    end
endmodule
`default_nettype wire
