module encoder (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enc_a,
    input  wire               enc_b,
    output reg signed [31:0]  count
);

    // 两级同步
    reg [1:0] a_sync;
    reg [1:0] b_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 2'b00;
            b_sync <= 2'b00;
        end else begin
            a_sync <= {a_sync[0], enc_a};
            b_sync <= {b_sync[0], enc_b};
        end
    end


    // 正交解码
    reg [1:0] prev_state;

    wire [1:0] curr_state;

    assign curr_state = {
        a_sync[1],
        b_sync[1]
    };


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            prev_state <= 2'b00;
            count      <= 32'sd0;

        end else begin

            case ({prev_state, curr_state})

                // 一个方向
                4'b0001,
                4'b0111,
                4'b1110,
                4'b1000:
                    count <= count + 1;


                // 另一个方向
                4'b0010,
                4'b1011,
                4'b1101,
                4'b0100:
                    count <= count - 1;


                default:
                    count <= count;

            endcase

            prev_state <= curr_state;

        end

    end

endmodule