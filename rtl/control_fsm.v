module control_fsm (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick_i,
    output reg [2:0]  state_o
);

    localparam IDLE        = 3'd0;
    localparam CALIBRATION = 3'd1;
    localparam READY       = 3'd2;
    localparam RUN         = 3'd3;
    localparam FAULT       = 3'd4;

    reg [15:0] state_timer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_o     <= IDLE;
            state_timer <= 16'd0;
        end
        else if (tick_i) begin

            case (state_o)

                IDLE: begin
                    if (state_timer >= 16'd999) begin
                        state_o     <= CALIBRATION;
                        state_timer <= 16'd0;
                    end
                    else begin
                        state_timer <= state_timer + 1'b1;
                    end
                end

                CALIBRATION: begin
                    if (state_timer >= 16'd999) begin
                        state_o     <= READY;
                        state_timer <= 16'd0;
                    end
                    else begin
                        state_timer <= state_timer + 1'b1;
                    end
                end

                READY: begin
                    if (state_timer >= 16'd999) begin
                        state_o     <= RUN;
                        state_timer <= 16'd0;
                    end
                    else begin
                        state_timer <= state_timer + 1'b1;
                    end
                end

                RUN: begin
                    state_o     <= RUN;
                    state_timer <= 16'd0;
                end

                FAULT: begin
                    state_o     <= FAULT;
                    state_timer <= 16'd0;
                end

                default: begin
                    state_o     <= FAULT;
                    state_timer <= 16'd0;
                end

            endcase
        end
    end

endmodule