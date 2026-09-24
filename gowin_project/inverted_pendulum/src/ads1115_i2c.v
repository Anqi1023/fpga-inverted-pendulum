module ads1115_i2c (
    input  wire              clk,
    input  wire              rst_n,

    inout  wire              i2c_sda,
    output wire              i2c_scl,

    output reg signed [15:0] adc_raw,
    output reg               data_valid,
    output reg               ack_error
);

    // ============================================================
    // FPGA clock = 27 MHz
    // I2C phase divider
    // ============================================================

    localparam integer DIVIDER = 68;

    reg [7:0] div_cnt;
    reg       tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 8'd0;
            tick    <= 1'b0;
        end else begin
            if (div_cnt == DIVIDER - 1) begin
                div_cnt <= 8'd0;
                tick    <= 1'b1;
            end else begin
                div_cnt <= div_cnt + 1'b1;
                tick    <= 1'b0;
            end
        end
    end


    // ============================================================
    // I2C open-drain
    // ============================================================

    reg sda_low;
    reg scl_low;

    assign i2c_sda = sda_low ? 1'b0 : 1'bz;
    assign i2c_scl = scl_low ? 1'b0 : 1'bz;

    wire sda_in;
    assign sda_in = i2c_sda;


    // ============================================================
    // ADS1115
    //
    // ADDR -> GND
    // 7-bit address = 0x48
    //
    // Write = 0x90
    // Read  = 0x91
    // ============================================================

    localparam [7:0] ADS_WRITE = 8'h90;
    localparam [7:0] ADS_READ  = 8'h91;


    // ============================================================
    // Configuration = 0xC283
    //
    // OS   = 1
    // MUX  = 100 : AIN0 vs GND
    // PGA  = 001 : +/-4.096 V
    // MODE = 0   : CONTINUOUS conversion
    // DR   = 100 : 128 SPS
    // COMP = disabled
    //
    // Previous:
    // C383 -> single-shot
    //
    // Now:
    // C283 -> continuous
    // ============================================================

    localparam [7:0] CONFIG_H = 8'hC2;
    localparam [7:0] CONFIG_L = 8'h83;


    // ============================================================
    // States
    // ============================================================

    localparam [6:0]

        POWER_WAIT       = 7'd0,

        // Configure once
        CFG_START        = 7'd1,
        CFG_ADDR         = 7'd2,
        CFG_PTR          = 7'd3,
        CFG_HIGH         = 7'd4,
        CFG_LOW          = 7'd5,
        CFG_STOP         = 7'd6,

        FIRST_WAIT       = 7'd7,

        // Set pointer to conversion register
        PTR_START        = 7'd8,
        PTR_ADDR         = 7'd9,
        PTR_CONVERSION   = 7'd10,
        PTR_STOP         = 7'd11,

        // Read ADC
        READ_START       = 7'd12,
        READ_ADDR        = 7'd13,
        READ_HIGH        = 7'd14,
        READ_LOW         = 7'd15,
        READ_STOP        = 7'd16,

        // Store ADC / wait before next read
        SAMPLE_WAIT      = 7'd17,

        // START primitive
        START_RELEASE    = 7'd20,
        START_SDA_LOW    = 7'd21,
        START_SCL_LOW    = 7'd22,

        // TX primitive
        TX_SETUP         = 7'd30,
        TX_SCL_HIGH      = 7'd31,
        TX_HIGH_WAIT     = 7'd32,
        TX_SCL_LOW       = 7'd33,

        // Slave ACK
        ACK_SETUP        = 7'd40,
        ACK_SCL_HIGH     = 7'd41,
        ACK_SAMPLE       = 7'd42,
        ACK_SCL_LOW      = 7'd43,

        // RX primitive
        RX_SETUP         = 7'd50,
        RX_SCL_HIGH      = 7'd51,
        RX_SAMPLE        = 7'd52,
        RX_SCL_LOW       = 7'd53,

        // Master ACK/NACK
        MASTER_ACK_SETUP = 7'd60,
        MASTER_ACK_HIGH  = 7'd61,
        MASTER_ACK_WAIT  = 7'd62,
        MASTER_ACK_LOW   = 7'd63,

        // STOP primitive
        STOP_SDA_LOW     = 7'd70,
        STOP_SCL_HIGH    = 7'd71,
        STOP_HIGH_WAIT   = 7'd72,
        STOP_SDA_HIGH    = 7'd73;


    reg [6:0] state;
    reg [6:0] next_state;

    reg [7:0] tx_byte;
    reg [7:0] rx_byte;

    reg [2:0] bit_cnt;

    reg master_send_ack;

    reg [15:0] wait_cnt;

    reg [7:0] read_high_byte;
    reg [7:0] read_low_byte;


    // ============================================================
    // Main FSM
    // ============================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            state           <= POWER_WAIT;
            next_state      <= POWER_WAIT;

            sda_low         <= 1'b0;
            scl_low         <= 1'b0;

            tx_byte         <= 8'h00;
            rx_byte         <= 8'h00;

            bit_cnt         <= 3'd7;

            master_send_ack <= 1'b0;

            wait_cnt        <= 16'd0;

            read_high_byte  <= 8'h00;
            read_low_byte   <= 8'h00;

            adc_raw         <= 16'sd0;
            data_valid      <= 1'b0;

            ack_error       <= 1'b0;

        end else if (tick) begin

            data_valid <= 1'b0;

            case (state)

                // =================================================
                // Power-up delay
                // =================================================

                POWER_WAIT: begin

                    sda_low <= 1'b0;
                    scl_low <= 1'b0;

                    if (wait_cnt >= 16'd8000) begin

                        wait_cnt  <= 16'd0;
                        ack_error <= 1'b0;

                        state <= CFG_START;

                    end else begin

                        wait_cnt <= wait_cnt + 1'b1;

                    end

                end


                // =================================================
                // Configure ADS1115 ONCE
                //
                // START
                // 90
                // 01
                // C2
                // 83
                // STOP
                //
                // Config register = C283
                // Continuous conversion
                // =================================================

                CFG_START: begin

                    next_state <= CFG_ADDR;
                    state      <= START_RELEASE;

                end


                CFG_ADDR: begin

                    tx_byte    <= ADS_WRITE;
                    bit_cnt    <= 3'd7;

                    next_state <= CFG_PTR;
                    state      <= TX_SETUP;

                end


                CFG_PTR: begin

                    tx_byte    <= 8'h01;
                    bit_cnt    <= 3'd7;

                    next_state <= CFG_HIGH;
                    state      <= TX_SETUP;

                end


                CFG_HIGH: begin

                    tx_byte    <= CONFIG_H;
                    bit_cnt    <= 3'd7;

                    next_state <= CFG_LOW;
                    state      <= TX_SETUP;

                end


                CFG_LOW: begin

                    tx_byte    <= CONFIG_L;
                    bit_cnt    <= 3'd7;

                    next_state <= CFG_STOP;
                    state      <= TX_SETUP;

                end


                CFG_STOP: begin

                    next_state <= FIRST_WAIT;
                    state      <= STOP_SDA_LOW;

                end


                // =================================================
                // Wait for first continuous conversion
                // =================================================

                FIRST_WAIT: begin

                    sda_low <= 1'b0;
                    scl_low <= 1'b0;

                    if (wait_cnt >= 16'd5000) begin

                        wait_cnt <= 16'd0;
                        state    <= PTR_START;

                    end else begin

                        wait_cnt <= wait_cnt + 1'b1;

                    end

                end


                // =================================================
                // Set pointer = conversion register 0x00
                //
                // START
                // 90
                // 00
                // STOP
                //
                // This is done once.
                // =================================================

                PTR_START: begin

                    next_state <= PTR_ADDR;
                    state      <= START_RELEASE;

                end


                PTR_ADDR: begin

                    tx_byte    <= ADS_WRITE;
                    bit_cnt    <= 3'd7;

                    next_state <= PTR_CONVERSION;
                    state      <= TX_SETUP;

                end


                PTR_CONVERSION: begin

                    tx_byte    <= 8'h00;
                    bit_cnt    <= 3'd7;

                    next_state <= PTR_STOP;
                    state      <= TX_SETUP;

                end


                PTR_STOP: begin

                    next_state <= READ_START;
                    state      <= STOP_SDA_LOW;

                end


                // =================================================
                // Read conversion register
                //
                // Pointer remains at 0x00.
                //
                // START
                // 91
                // MSB
                // ACK
                // LSB
                // NACK
                // STOP
                // =================================================

                READ_START: begin

                    next_state <= READ_ADDR;
                    state      <= START_RELEASE;

                end


                READ_ADDR: begin

                    tx_byte    <= ADS_READ;
                    bit_cnt    <= 3'd7;

                    next_state <= READ_HIGH;
                    state      <= TX_SETUP;

                end


                READ_HIGH: begin

                    rx_byte <= 8'h00;
                    bit_cnt <= 3'd7;

                    // ACK after first byte
                    master_send_ack <= 1'b1;

                    next_state <= READ_LOW;
                    state      <= RX_SETUP;

                end


                READ_LOW: begin

                    // First byte = MSB
                    read_high_byte <= rx_byte;

                    rx_byte <= 8'h00;
                    bit_cnt <= 3'd7;

                    // NACK after second byte
                    master_send_ack <= 1'b0;

                    next_state <= READ_STOP;
                    state      <= RX_SETUP;

                end


                READ_STOP: begin

                    // Second byte = LSB
                    read_low_byte <= rx_byte;

                    next_state <= SAMPLE_WAIT;
                    state      <= STOP_SDA_LOW;

                end


                // =================================================
                // Store result
                //
                // ADS1115 continues converting automatically.
                // =================================================

                SAMPLE_WAIT: begin

                    adc_raw <= $signed({
                        read_high_byte,
                        read_low_byte
                    });

                    data_valid <= 1'b1;

                    // About 10 ms between reads.
                    //
                    // tick ≈ 2.52 us
                    // 4000 ticks ≈ 10.1 ms
                    //
                    // ADS1115 = 128 SPS
                    // one conversion ≈ 7.8 ms

                    if (wait_cnt >= 16'd4000) begin

                        wait_cnt <= 16'd0;

                        // Pointer already = 0x00
                        // Directly read next conversion.
                        state <= READ_START;

                    end else begin

                        wait_cnt <= wait_cnt + 1'b1;

                    end

                end


                // =================================================
                // I2C START
                // =================================================

                START_RELEASE: begin

                    sda_low <= 1'b0;
                    scl_low <= 1'b0;

                    state <= START_SDA_LOW;

                end


                START_SDA_LOW: begin

                    // SDA HIGH -> LOW while SCL HIGH
                    sda_low <= 1'b1;
                    scl_low <= 1'b0;

                    state <= START_SCL_LOW;

                end


                START_SCL_LOW: begin

                    sda_low <= 1'b1;
                    scl_low <= 1'b1;

                    state <= next_state;

                end


                // =================================================
                // TX one bit
                // =================================================

                TX_SETUP: begin

                    scl_low <= 1'b1;

                    if (tx_byte[bit_cnt] == 1'b0)
                        sda_low <= 1'b1;
                    else
                        sda_low <= 1'b0;

                    state <= TX_SCL_HIGH;

                end


                TX_SCL_HIGH: begin

                    scl_low <= 1'b0;

                    state <= TX_HIGH_WAIT;

                end


                TX_HIGH_WAIT: begin

                    // Hold SCL HIGH for one phase
                    scl_low <= 1'b0;

                    state <= TX_SCL_LOW;

                end


                TX_SCL_LOW: begin

                    scl_low <= 1'b1;

                    if (bit_cnt == 3'd0) begin

                        // Release SDA for slave ACK
                        sda_low <= 1'b0;

                        state <= ACK_SETUP;

                    end else begin

                        bit_cnt <= bit_cnt - 1'b1;

                        state <= TX_SETUP;

                    end

                end


                // =================================================
                // Slave ACK
                // =================================================

                ACK_SETUP: begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b0;

                    state <= ACK_SCL_HIGH;

                end


                ACK_SCL_HIGH: begin

                    scl_low <= 1'b0;
                    sda_low <= 1'b0;

                    state <= ACK_SAMPLE;

                end


                ACK_SAMPLE: begin

                    // ACK  = SDA LOW
                    // NACK = SDA HIGH

                    if (sda_in == 1'b1)
                        ack_error <= 1'b1;

                    state <= ACK_SCL_LOW;

                end


                ACK_SCL_LOW: begin

                    scl_low <= 1'b1;

                    state <= next_state;

                end


                // =================================================
                // RX one bit
                // =================================================

                RX_SETUP: begin

                    scl_low <= 1'b1;

                    // Release SDA for ADS1115
                    sda_low <= 1'b0;

                    state <= RX_SCL_HIGH;

                end


                RX_SCL_HIGH: begin

                    // Raise SCL first
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;

                    state <= RX_SAMPLE;

                end


                RX_SAMPLE: begin

                    // SCL has already been HIGH for one full phase.
                    rx_byte[bit_cnt] <= sda_in;

                    state <= RX_SCL_LOW;

                end


                RX_SCL_LOW: begin

                    scl_low <= 1'b1;

                    if (bit_cnt == 3'd0) begin

                        state <= MASTER_ACK_SETUP;

                    end else begin

                        bit_cnt <= bit_cnt - 1'b1;

                        state <= RX_SETUP;

                    end

                end


                // =================================================
                // Master ACK / NACK
                // =================================================

                MASTER_ACK_SETUP: begin

                    scl_low <= 1'b1;

                    if (master_send_ack)
                        sda_low <= 1'b1; // ACK = SDA LOW
                    else
                        sda_low <= 1'b0; // NACK = release SDA

                    state <= MASTER_ACK_HIGH;

                end


                MASTER_ACK_HIGH: begin

                    scl_low <= 1'b0;

                    state <= MASTER_ACK_WAIT;

                end


                MASTER_ACK_WAIT: begin

                    scl_low <= 1'b0;

                    state <= MASTER_ACK_LOW;

                end


                MASTER_ACK_LOW: begin

                    scl_low <= 1'b1;

                    // Release SDA
                    sda_low <= 1'b0;

                    state <= next_state;

                end


                // =================================================
                // STOP
                // =================================================

                STOP_SDA_LOW: begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b1;

                    state <= STOP_SCL_HIGH;

                end


                STOP_SCL_HIGH: begin

                    scl_low <= 1'b0;
                    sda_low <= 1'b1;

                    state <= STOP_HIGH_WAIT;

                end


                STOP_HIGH_WAIT: begin

                    scl_low <= 1'b0;
                    sda_low <= 1'b1;

                    state <= STOP_SDA_HIGH;

                end


                STOP_SDA_HIGH: begin

                    // SDA LOW -> HIGH while SCL HIGH
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;

                    state <= next_state;

                end


                // =================================================
                // Fallback
                // =================================================

                default: begin

                    state      <= POWER_WAIT;
                    next_state <= POWER_WAIT;

                    wait_cnt <= 16'd0;

                    sda_low <= 1'b0;
                    scl_low <= 1'b0;

                end

            endcase
        end
    end

endmodule