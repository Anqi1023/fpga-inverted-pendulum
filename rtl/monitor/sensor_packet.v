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


// =========================================================
// Snapshot registers
// =========================================================

reg signed [31:0] theta_r;
reg signed [31:0] theta_dot_r;
reg signed [31:0] alpha_r;
reg signed [31:0] alpha_dot_r;


// =========================================================
// Absolute value for display
// =========================================================

wire [31:0] theta_mag =
    theta_r[31] ? (~theta_r + 32'd1) : theta_r;

wire [31:0] theta_dot_mag =
    theta_dot_r[31] ? (~theta_dot_r + 32'd1) : theta_dot_r;

wire [31:0] alpha_mag =
    alpha_r[31] ? (~alpha_r + 32'd1) : alpha_r;

wire [31:0] alpha_dot_mag =
    alpha_dot_r[31] ? (~alpha_dot_r + 32'd1) : alpha_dot_r;


// =========================================================
// Convert one hex nibble to ASCII
// =========================================================

function [7:0] hex_ascii;

    input [3:0] value;

    begin

        if(value < 10)
            hex_ascii = "0" + value;
        else
            hex_ascii = "A" + (value - 10);

    end

endfunction


// =========================================================
// Get one hex character from a 32-bit value
// pos = 0 -> highest nibble
// pos = 7 -> lowest nibble
// =========================================================

function [7:0] hex32_ascii;

    input [31:0] value;
    input [2:0] pos;

    begin

        case(pos)

            3'd0: hex32_ascii = hex_ascii(value[31:28]);
            3'd1: hex32_ascii = hex_ascii(value[27:24]);
            3'd2: hex32_ascii = hex_ascii(value[23:20]);
            3'd3: hex32_ascii = hex_ascii(value[19:16]);
            3'd4: hex32_ascii = hex_ascii(value[15:12]);
            3'd5: hex32_ascii = hex_ascii(value[11:8]);
            3'd6: hex32_ascii = hex_ascii(value[7:4]);
            3'd7: hex32_ascii = hex_ascii(value[3:0]);

            default:
                hex32_ascii = "?";

        endcase

    end

endfunction


// =========================================================
// Send control
//
// 27 MHz:
// 13,500,000 clocks ≈ 500 ms
// =========================================================

reg [23:0] timer;
reg [5:0]  index;

reg [2:0] tx_state;

localparam TX_WAIT      = 3'd0;
localparam TX_LOAD      = 3'd1;
localparam TX_START     = 3'd2;
localparam TX_WAIT_BUSY = 3'd3;
localparam TX_WAIT_DONE = 3'd4;


// =========================================================
// Current character
// =========================================================

reg [7:0] next_char;

always @*
begin

    next_char = "?";


    // -----------------------------------------------------
    // TH=+XXXXXXXX
    // -----------------------------------------------------

    if(index == 6'd0)
        next_char = "T";

    else if(index == 6'd1)
        next_char = "H";

    else if(index == 6'd2)
        next_char = "=";

    else if(index == 6'd3)
        next_char = theta_r[31] ? "-" : "+";

    else if((index >= 6'd4) && (index <= 6'd11))
        next_char = hex32_ascii(
            theta_mag,
            index - 6'd4
        );


    // space
    else if(index == 6'd12)
        next_char = " ";


    // -----------------------------------------------------
    // TD=+XXXXXXXX
    // -----------------------------------------------------

    else if(index == 6'd13)
        next_char = "T";

    else if(index == 6'd14)
        next_char = "D";

    else if(index == 6'd15)
        next_char = "=";

    else if(index == 6'd16)
        next_char = theta_dot_r[31] ? "-" : "+";

    else if((index >= 6'd17) && (index <= 6'd24))
        next_char = hex32_ascii(
            theta_dot_mag,
            index - 6'd17
        );


    // space
    else if(index == 6'd25)
        next_char = " ";


    // -----------------------------------------------------
    // AL=+XXXXXXXX
    // -----------------------------------------------------

    else if(index == 6'd26)
        next_char = "A";

    else if(index == 6'd27)
        next_char = "L";

    else if(index == 6'd28)
        next_char = "=";

    else if(index == 6'd29)
        next_char = alpha_r[31] ? "-" : "+";

    else if((index >= 6'd30) && (index <= 6'd37))
        next_char = hex32_ascii(
            alpha_mag,
            index - 6'd30
        );


    // space
    else if(index == 6'd38)
        next_char = " ";


    // -----------------------------------------------------
    // AD=+XXXXXXXX
    // -----------------------------------------------------

    else if(index == 6'd39)
        next_char = "A";

    else if(index == 6'd40)
        next_char = "D";

    else if(index == 6'd41)
        next_char = "=";

    else if(index == 6'd42)
        next_char = alpha_dot_r[31] ? "-" : "+";

    else if((index >= 6'd43) && (index <= 6'd50))
        next_char = hex32_ascii(
            alpha_dot_mag,
            index - 6'd43
        );


    // CR LF
    else if(index == 6'd51)
        next_char = 8'h0D;

    else if(index == 6'd52)
        next_char = 8'h0A;

end


// =========================================================
// UART state machine
// =========================================================

always @(posedge clk or negedge rst_n)
begin

    if(!rst_n)
    begin

        timer <= 24'd0;
        index <= 6'd0;

        tx_state <= TX_WAIT;

        uart_data_o  <= 8'd0;
        uart_start_o <= 1'b0;

        theta_r     <= 32'sd0;
        theta_dot_r <= 32'sd0;
        alpha_r     <= 32'sd0;
        alpha_dot_r <= 32'sd0;

    end

    else
    begin

        // start only lasts one clock
        uart_start_o <= 1'b0;


        case(tx_state)


            // =================================================
            // Wait about 500 ms
            // =================================================

            TX_WAIT:
            begin

                if(timer >= 24'd13_499_999)
                begin

                    timer <= 24'd0;

                    // Snapshot the four sensor states
                    theta_r     <= theta;
                    theta_dot_r <= theta_dot;
                    alpha_r     <= alpha;
                    alpha_dot_r <= alpha_dot;

                    index <= 6'd0;

                    tx_state <= TX_LOAD;

                end

                else
                begin

                    timer <= timer + 1'b1;

                end

            end


            // =================================================
            // Put data on uart_data_o first
            // =================================================

            TX_LOAD:
            begin

                if(!uart_busy_i)
                begin

                    uart_data_o <= next_char;

                    tx_state <= TX_START;

                end

            end


            // =================================================
            // One-clock start pulse
            // =================================================

            TX_START:
            begin

                uart_start_o <= 1'b1;

                tx_state <= TX_WAIT_BUSY;

            end


            // =================================================
            // Wait until uart_tx accepts this byte
            // =================================================

            TX_WAIT_BUSY:
            begin

                if(uart_busy_i)
                begin

                    tx_state <= TX_WAIT_DONE;

                end

            end


            // =================================================
            // Wait until byte completely sent
            // =================================================

            TX_WAIT_DONE:
            begin

                if(!uart_busy_i)
                begin

                    if(index == 6'd52)
                    begin

                        index <= 6'd0;

                        tx_state <= TX_WAIT;

                    end

                    else
                    begin

                        index <= index + 1'b1;

                        tx_state <= TX_LOAD;

                    end

                end

            end


            default:
            begin

                tx_state <= TX_WAIT;
                timer <= 24'd0;
                index <= 6'd0;

            end


        endcase

    end

end


endmodule