`timescale 1ns/1ps
`default_nettype none
// Read-only telemetry at 460800 baud. Deployed R14H: 100Hz, 398 ASCII bytes.
// A busy transmitter drops a complete sample; N/T make gaps visible.
module sensor_packet #(parameter integer MOTION_REPORT=0, LIVE_BRAKE_CONFIG=0, BRAKE_DIAGNOSTICS=0, BRAKE_DISTANCE_GAIN_Q8=384, LOOP_DIAGNOSTICS=0, COMPACT_DIAGNOSTICS=0, ARM_DIAGNOSTICS=0, AGE_DIAGNOSTICS=0, SPEED_DIAGNOSTICS=0, SETTLE_DIAGNOSTICS=0, PID_DIAGNOSTICS=0, HOLD_DIAGNOSTICS=0)(
    input wire [9:0] brake_gain_i,
    input wire [18:0] brake_accel_i,
    input wire [16:0] approach_speed_i,
    input wire [9:0] release_rate_i,
    input wire [31:0] brake_distance_i,
    input wire signed [32:0] velocity_error_i,
    input wire [3:0] brake_flags_i,
    input wire [7:0] brake_status_i,
    input wire signed [31:0] velocity_ref_i,angle_ref_i,
    input wire [31:0] hold_control_i,
    input wire signed [31:0] pid_integral_i,
    input wire [31:0] settle_status_i,boost_settings_i,
    input wire signed [31:0] capture_travel_i,lock_origin_i,hold_position_i,
    input wire signed [31:0] raw_speed_i,est_speed_i,
    input wire speed_source_i,est_valid_i,
    input wire [63:0] speed_status_i,
    input wire signed [31:0] motion_position_i,motion_velocity_i,outer_i_i,
    input wire signed [31:0] original_arm_i,arm_request_i,
    input wire signed [31:0] loop_reference_i,
    input wire [3:0] loop_state_i,
    input wire clk, rst_n, send_i,
    input wire [31:0] version_i,
    input wire run_i,
    output wire frame_busy_o,
    input wire [31:0] ms_i,
    input wire [15:0] seq_i,
    input wire signed [31:0] theta, theta_dot, alpha, alpha_dot,
    input wire signed [15:0] control_u, safe_u,
    input wire signed [31:0] pa_i, pd_i, pt_i, pv_i,
    input wire [31:0] exec_i, flags_i,
    input wire signed [15:0] raw_i,
    input wire signed [15:0] adc_i,
    input wire [7:0] reseeds_i,
    input wire signed [31:0] zero_i,
    input wire cal_i, go_i,
    input wire uart_busy_i,
    output reg [7:0] uart_data_o,
    output reg uart_start_o
);
    // synthesis translate_off
    initial if(COMPACT_DIAGNOSTICS && !(ARM_DIAGNOSTICS && AGE_DIAGNOSTICS && SPEED_DIAGNOSTICS && SETTLE_DIAGNOSTICS && PID_DIAGNOSTICS && HOLD_DIAGNOSTICS))
        $fatal(1,"Compact telemetry requires the complete diagnostic source layout");
    // synthesis translate_on
    reg signed [31:0] motion_position_r;
    reg signed [18:0] motion_velocity_r;reg signed [13:0] outer_i_r;
    reg [7:0] brake_status_r;
    reg [15:0] GAIN_Q8;
    reg [31:0] brake_accel_r,approach_speed_r,brake_distance_r;
    reg [15:0] release_rate_r;
    reg signed [32:0] velocity_error_r;
    reg [3:0] brake_flags_r;

    reg signed [31:0] velocity_ref_r,angle_ref_r;
    reg [31:0] hold_control_r;

    reg signed [31:0] pid_integral_r;

    reg [31:0] settle_status_r,boost_settings_r;
    reg signed [31:0] capture_travel_r,lock_origin_r,hold_position_r;

    reg signed [31:0] raw_speed_r,est_speed_r;
    reg speed_source_r,est_valid_r;
    reg [63:0] speed_status_r;
    reg signed [31:0] loop_reference_r;
    reg [3:0] loop_state_r;

    reg signed [31:0] original_arm_r,arm_request_r;

    reg [31:0] version_r;
    reg run_r;
    assign frame_busy_o=(tx_state!=IDLE);
    reg [31:0] ms_i_r;
    reg [15:0] seq_i_r;
    reg [31:0] theta_r;

    reg [31:0] theta_dot_r;

    reg [31:0] alpha_r;

    reg [31:0] alpha_dot_r;

    reg [15:0] control_u_r;

    reg [15:0] safe_u_r;

    reg [31:0] pa_i_r;

    reg [31:0] pd_i_r;

    reg [31:0] pt_i_r;

    reg [31:0] pv_i_r;

    reg [31:0] exec_i_r;
    reg [31:0] flags_i_r;
    reg [15:0] raw_i_r;

    reg [31:0] zero_i_r;

    reg signed [15:0] adc_r;
    reg [7:0] reseeds_r;

    reg cal_r, go_r;
    reg [9:0] index;
    reg [2:0] tx_state;
    reg [7:0] next_char;
    localparam integer BASE_LAST=BRAKE_DIAGNOSTICS ? 443:COMPACT_DIAGNOSTICS ?
        (LOOP_DIAGNOSTICS ? 366:348):ARM_DIAGNOSTICS ? 423:AGE_DIAGNOSTICS ? 397:
        HOLD_DIAGNOSTICS ? 373:PID_DIAGNOSTICS ? 335:SETTLE_DIAGNOSTICS ? 322:SPEED_DIAGNOSTICS ? 259:223;
    wire [5:0] append_index=index-(BASE_LAST-1);
    wire [3:0] append_offset=append_index>=24 ? append_index-24:append_index>=12 ? append_index-12:append_index;
    wire [31:0] append_word=append_index<12 ? motion_position_r:
        append_index<24 ? {{13{motion_velocity_r[18]}},motion_velocity_r}:{{18{outer_i_r[13]}},outer_i_r};

    localparam IDLE=0, LOAD=1, START=2, WAIT_BUSY=3, WAIT_DONE=4;
    function [7:0] hex_ascii;
        input [3:0] nib;
        begin
            hex_ascii = (nib < 10) ? (8'd48 + nib) : (8'd65 + nib - 10);
        end
    endfunction
    // Compact layout retains original encodings and snapshot semantics.
    // Keep [0:163], C=[184:187], [205:282], [321:423].
    wire [9:0] format_index = BRAKE_DIAGNOSTICS && index>=365 ? index+10'd77 : !COMPACT_DIAGNOSTICS ? index :
        index<10'd164 ? index : index<10'd168 ? index+10'd20 :
        index<10'd246 ? index+10'd37 :
        LOOP_DIAGNOSTICS && index>=347 ? (index<365 ? index+77:index+57):index+10'd75;
    // Compile-time snapshot trimming below drops only fields absent from the
    // selected UART layout. Non-compact / brake-enabled layouts retain them.
    // R15I route fix: UART-only field formatter. Select one signed snapshot
    // word before taking its magnitude, instead of 25 parallel negation buses.
    // Two small register stages isolate the word mux from the carry chain.
    // Every signed field has a sign byte before its first magnitude nibble;
    // the existing UART handshake leaves >=4 clocks between byte loads.
    // Therefore both stages settle before use, without delaying any UART byte.
    reg signed [32:0] magnitude_source,magnitude_word;
    reg [32:0] magnitude_value;
    wire [35:0] display_magnitude={3'd0,magnitude_value};
    always @* begin
        magnitude_source=0;
        case(format_index)
            10'd25,10'd26,10'd27,10'd28,10'd29,10'd30,10'd31,10'd32,10'd33:magnitude_source=$signed({{1{theta_r[31]}},theta_r});
            10'd38,10'd39,10'd40,10'd41,10'd42,10'd43,10'd44,10'd45,10'd46:magnitude_source=$signed({{1{theta_dot_r[31]}},theta_dot_r});
            10'd51,10'd52,10'd53,10'd54,10'd55,10'd56,10'd57,10'd58,10'd59:magnitude_source=$signed({{1{alpha_r[31]}},alpha_r});
            10'd64,10'd65,10'd66,10'd67,10'd68,10'd69,10'd70,10'd71,10'd72:magnitude_source=$signed({{1{alpha_dot_r[31]}},alpha_dot_r});
            10'd76,10'd77,10'd78,10'd79,10'd80:magnitude_source=$signed({{17{control_u_r[15]}},control_u_r});
            10'd85,10'd86,10'd87,10'd88,10'd89:magnitude_source=$signed({{17{safe_u_r[15]}},safe_u_r});
            10'd94,10'd95,10'd96,10'd97,10'd98,10'd99,10'd100,10'd101,10'd102:magnitude_source=$signed({{1{pa_i_r[31]}},pa_i_r});
            10'd107,10'd108,10'd109,10'd110,10'd111,10'd112,10'd113,10'd114,10'd115:magnitude_source=$signed({{1{pd_i_r[31]}},pd_i_r});
            10'd120,10'd121,10'd122,10'd123,10'd124,10'd125,10'd126,10'd127,10'd128:magnitude_source=$signed({{1{pt_i_r[31]}},pt_i_r});
            10'd133,10'd134,10'd135,10'd136,10'd137,10'd138,10'd139,10'd140,10'd141:magnitude_source=$signed({{1{pv_i_r[31]}},pv_i_r});
            10'd167,10'd168,10'd169,10'd170,10'd171:magnitude_source=$signed({{17{raw_i_r[15]}},raw_i_r});
            10'd175,10'd176,10'd177,10'd178,10'd179,10'd180,10'd181,10'd182,10'd183:magnitude_source=$signed({{1{zero_i_r[31]}},zero_i_r});
            10'd195,10'd196,10'd197,10'd198,10'd199:magnitude_source=$signed({{17{adc_r[15]}},adc_r});
            10'd226,10'd227,10'd228,10'd229,10'd230,10'd231,10'd232,10'd233,10'd234:magnitude_source=$signed({{1{raw_speed_r[31]}},raw_speed_r});
            10'd239,10'd240,10'd241,10'd242,10'd243,10'd244,10'd245,10'd246,10'd247:magnitude_source=$signed({{1{est_speed_r[31]}},est_speed_r});
            10'd274,10'd275,10'd276,10'd277,10'd278,10'd279,10'd280,10'd281,10'd282:magnitude_source=$signed({{1{capture_travel_r[31]}},capture_travel_r});
            10'd287,10'd288,10'd289,10'd290,10'd291,10'd292,10'd293,10'd294,10'd295:magnitude_source=$signed({{1{lock_origin_r[31]}},lock_origin_r});
            10'd300,10'd301,10'd302,10'd303,10'd304,10'd305,10'd306,10'd307,10'd308:magnitude_source=$signed({{1{hold_position_r[31]}},hold_position_r});
            10'd325,10'd326,10'd327,10'd328,10'd329,10'd330,10'd331,10'd332,10'd333:magnitude_source=$signed({{1{pid_integral_r[31]}},pid_integral_r});
            10'd338,10'd339,10'd340,10'd341,10'd342,10'd343,10'd344,10'd345,10'd346:magnitude_source=$signed({{1{velocity_ref_r[31]}},velocity_ref_r});
            10'd351,10'd352,10'd353,10'd354,10'd355,10'd356,10'd357,10'd358,10'd359:magnitude_source=$signed({{1{angle_ref_r[31]}},angle_ref_r});
            10'd400,10'd401,10'd402,10'd403,10'd404,10'd405,10'd406,10'd407,10'd408:magnitude_source=$signed({{1{original_arm_r[31]}},original_arm_r});
            10'd413,10'd414,10'd415,10'd416,10'd417,10'd418,10'd419,10'd420,10'd421:magnitude_source=$signed({{1{arm_request_r[31]}},arm_request_r});
            10'd428,10'd429,10'd430,10'd431,10'd432,10'd433,10'd434,10'd435,10'd436:magnitude_source=$signed({{1{loop_reference_r[31]}},loop_reference_r});
            10'd509,10'd510,10'd511,10'd512,10'd513,10'd514,10'd515,10'd516,10'd517,10'd518:magnitude_source=$signed(velocity_error_r);
            default:magnitude_source=0;
        endcase
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin magnitude_word<=0;magnitude_value<=0;end
        else begin
            magnitude_word<=magnitude_source;
            magnitude_value<=magnitude_word[32] ? -magnitude_word:magnitude_word;
        end
    end
    // Select the nibble BEFORE ASCII conversion: one shared encoder.
    // All frame offsets, transmitted snapshot values and UART timing are unchanged.
    reg [3:0] hex_nibble;
    reg is_hex;
    reg [7:0] literal_char;
    always @* begin
        hex_nibble=0; is_hex=0; literal_char=8'd32;
        case (format_index)
            8'd0: literal_char = 8'd68;
            8'd1: literal_char = 8'd51;
            8'd2: literal_char = 8'd52;
            8'd3: literal_char = 8'd32;
            8'd4: literal_char = 8'd84;
            8'd5: literal_char = 8'd61;
            8'd6: begin hex_nibble = ms_i_r[31:28]; is_hex = 1'b1; end
            8'd7: begin hex_nibble = ms_i_r[27:24]; is_hex = 1'b1; end
            8'd8: begin hex_nibble = ms_i_r[23:20]; is_hex = 1'b1; end
            8'd9: begin hex_nibble = ms_i_r[19:16]; is_hex = 1'b1; end
            8'd10: begin hex_nibble = ms_i_r[15:12]; is_hex = 1'b1; end
            8'd11: begin hex_nibble = ms_i_r[11:8]; is_hex = 1'b1; end
            8'd12: begin hex_nibble = ms_i_r[7:4]; is_hex = 1'b1; end
            8'd13: begin hex_nibble = ms_i_r[3:0]; is_hex = 1'b1; end
            8'd14: literal_char = 8'd32;
            8'd15: literal_char = 8'd78;
            8'd16: literal_char = 8'd61;
            8'd17: begin hex_nibble = seq_i_r[15:12]; is_hex = 1'b1; end
            8'd18: begin hex_nibble = seq_i_r[11:8]; is_hex = 1'b1; end
            8'd19: begin hex_nibble = seq_i_r[7:4]; is_hex = 1'b1; end
            8'd20: begin hex_nibble = seq_i_r[3:0]; is_hex = 1'b1; end
            8'd21: literal_char = 8'd32;
            8'd22: literal_char = 8'd84;
            8'd23: literal_char = 8'd72;
            8'd24: literal_char = 8'd61;
            8'd25: literal_char = theta_r[31] ? "-" : "+";
            8'd26: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd27: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd28: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd29: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd30: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd31: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd32: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd33: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd34: literal_char = 8'd32;
            8'd35: literal_char = 8'd84;
            8'd36: literal_char = 8'd68;
            8'd37: literal_char = 8'd61;
            8'd38: literal_char = theta_dot_r[31] ? "-" : "+";
            8'd39: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd40: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd41: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd42: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd43: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd44: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd45: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd46: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd47: literal_char = 8'd32;
            8'd48: literal_char = 8'd65;
            8'd49: literal_char = 8'd76;
            8'd50: literal_char = 8'd61;
            8'd51: literal_char = alpha_r[31] ? "-" : "+";
            8'd52: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd53: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd54: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd55: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd56: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd57: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd58: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd59: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd60: literal_char = 8'd32;
            8'd61: literal_char = 8'd65;
            8'd62: literal_char = 8'd68;
            8'd63: literal_char = 8'd61;
            8'd64: literal_char = alpha_dot_r[31] ? "-" : "+";
            8'd65: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd66: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd67: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd68: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd69: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd70: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd71: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd72: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd73: literal_char = 8'd32;
            8'd74: literal_char = 8'd85;
            8'd75: literal_char = 8'd61;
            8'd76: literal_char = control_u_r[15] ? "-" : "+";
            8'd77: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd78: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd79: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd80: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd81: literal_char = 8'd32;
            8'd82: literal_char = 8'd85;
            8'd83: literal_char = 8'd83;
            8'd84: literal_char = 8'd61;
            8'd85: literal_char = safe_u_r[15] ? "-" : "+";
            8'd86: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd87: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd88: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd89: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd90: literal_char = 8'd32;
            8'd91: literal_char = 8'd80;
            8'd92: literal_char = 8'd65;
            8'd93: literal_char = 8'd61;
            8'd94: literal_char = pa_i_r[31] ? "-" : "+";
            8'd95: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd96: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd97: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd98: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd99: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd100: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd101: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd102: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd103: literal_char = 8'd32;
            8'd104: literal_char = 8'd80;
            8'd105: literal_char = 8'd68;
            8'd106: literal_char = 8'd61;
            8'd107: literal_char = pd_i_r[31] ? "-" : "+";
            8'd108: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd109: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd110: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd111: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd112: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd113: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd114: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd115: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd116: literal_char = 8'd32;
            8'd117: literal_char = 8'd80;
            8'd118: literal_char = 8'd84;
            8'd119: literal_char = 8'd61;
            8'd120: literal_char = pt_i_r[31] ? "-" : "+";
            8'd121: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd122: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd123: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd124: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd125: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd126: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd127: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd128: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd129: literal_char = 8'd32;
            8'd130: literal_char = 8'd80;
            8'd131: literal_char = 8'd86;
            8'd132: literal_char = 8'd61;
            8'd133: literal_char = pv_i_r[31] ? "-" : "+";
            8'd134: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd135: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd136: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd137: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd138: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd139: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd140: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd141: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd142: literal_char = 8'd32;
            8'd143: literal_char = 8'd88;
            8'd144: literal_char = 8'd61;
            8'd145: begin hex_nibble = exec_i_r[31:28]; is_hex = 1'b1; end
            8'd146: begin hex_nibble = exec_i_r[27:24]; is_hex = 1'b1; end
            8'd147: begin hex_nibble = exec_i_r[23:20]; is_hex = 1'b1; end
            8'd148: begin hex_nibble = exec_i_r[19:16]; is_hex = 1'b1; end
            8'd149: begin hex_nibble = exec_i_r[15:12]; is_hex = 1'b1; end
            8'd150: begin hex_nibble = exec_i_r[11:8]; is_hex = 1'b1; end
            8'd151: begin hex_nibble = exec_i_r[7:4]; is_hex = 1'b1; end
            8'd152: begin hex_nibble = exec_i_r[3:0]; is_hex = 1'b1; end
            8'd153: literal_char = 8'd32;
            8'd154: literal_char = 8'd70;
            8'd155: literal_char = 8'd61;
            8'd156: begin hex_nibble = flags_i_r[31:28]; is_hex = 1'b1; end
            8'd157: begin hex_nibble = flags_i_r[27:24]; is_hex = 1'b1; end
            8'd158: begin hex_nibble = flags_i_r[23:20]; is_hex = 1'b1; end
            8'd159: begin hex_nibble = flags_i_r[19:16]; is_hex = 1'b1; end
            8'd160: begin hex_nibble = flags_i_r[15:12]; is_hex = 1'b1; end
            8'd161: begin hex_nibble = flags_i_r[11:8]; is_hex = 1'b1; end
            8'd162: begin hex_nibble = flags_i_r[7:4]; is_hex = 1'b1; end
            8'd163: begin hex_nibble = flags_i_r[3:0]; is_hex = 1'b1; end
            8'd164: literal_char = 8'd32;
            8'd165: literal_char = 8'd82;
            8'd166: literal_char = 8'd61;
            8'd167: literal_char = raw_i_r[15] ? "-" : "+";
            8'd168: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd169: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd170: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd171: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd172: literal_char = 8'd32;
            8'd173: literal_char = 8'd90;
            8'd174: literal_char = 8'd61;
            8'd175: literal_char = zero_i_r[31] ? "-" : "+";
            8'd176: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            8'd177: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            8'd178: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            8'd179: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            8'd180: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd181: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd182: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd183: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd184: literal_char = 8'd32;
            8'd185: literal_char = 8'd67;
            8'd186: literal_char = 8'd61;
            8'd187: literal_char = cal_r ? "1" : "0";
            8'd188: literal_char = 8'd32;
            8'd189: literal_char = 8'd71;
            8'd190: literal_char = 8'd61;
            8'd191: literal_char = go_r ? "1" : "0";
            8'd192: literal_char = " ";
            8'd193: literal_char = "A";
            8'd194: literal_char = "=";
            8'd195: literal_char = adc_r[15] ? "-" : "+";
            8'd196: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            8'd197: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            8'd198: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            8'd199: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            8'd200: literal_char = " ";
            8'd201: literal_char = "J";
            8'd202: literal_char = "=";
            8'd203: begin hex_nibble = reseeds_r[7:4]; is_hex = 1'b1; end
            8'd204: begin hex_nibble = reseeds_r[3:0]; is_hex = 1'b1; end
            8'd205: literal_char = 8'd32;
            8'd206: literal_char = 8'd81;
            8'd207: literal_char = 8'd61;
            8'd208: begin hex_nibble = version_r[31:28]; is_hex = 1'b1; end
            8'd209: begin hex_nibble = version_r[27:24]; is_hex = 1'b1; end
            8'd210: begin hex_nibble = version_r[23:20]; is_hex = 1'b1; end
            8'd211: begin hex_nibble = version_r[19:16]; is_hex = 1'b1; end
            8'd212: begin hex_nibble = version_r[15:12]; is_hex = 1'b1; end
            8'd213: begin hex_nibble = version_r[11:8]; is_hex = 1'b1; end
            8'd214: begin hex_nibble = version_r[7:4]; is_hex = 1'b1; end
            8'd215: begin hex_nibble = version_r[3:0]; is_hex = 1'b1; end
            8'd216: literal_char = 8'd32;
            8'd217: literal_char = 8'd82;
            8'd218: literal_char = 8'd85;
            8'd219: literal_char = 8'd78;
            8'd220: literal_char = 8'd61;
            8'd221: literal_char = run_r ? "1" : "0";
            8'd222: literal_char = SPEED_DIAGNOSTICS ? 8'd32 : 8'd13;
            8'd223: literal_char = SPEED_DIAGNOSTICS ? 8'd84 : 8'd10;
            10'd224: literal_char = "R";
            10'd225: literal_char = "=";
            10'd226: literal_char = raw_speed_r[31] ? "-" : "+";
            10'd227: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd228: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd229: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd230: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd231: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd232: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd233: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd234: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd235: literal_char = " ";
            10'd236: literal_char = "T";
            10'd237: literal_char = "E";
            10'd238: literal_char = "=";
            10'd239: literal_char = est_speed_r[31] ? "-" : "+";
            10'd240: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd241: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd242: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd243: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd244: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd245: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd246: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd247: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd248: literal_char = " ";
            10'd249: literal_char = "S";
            10'd250: literal_char = "M";
            10'd251: literal_char = "=";
            10'd252: literal_char = speed_source_r ? "1" : "0";
            10'd253: literal_char = " ";
            10'd254: literal_char = "S";
            10'd255: literal_char = "V";
            10'd256: literal_char = "=";
            10'd257: literal_char = est_valid_r ? "1" : "0";
            10'd258: literal_char = SETTLE_DIAGNOSTICS ? " " : 8'd13;
            10'd259: literal_char = SETTLE_DIAGNOSTICS ? "S" : 8'd10;
            10'd260: literal_char = "S";
            10'd261: literal_char = "=";
            10'd262: begin hex_nibble = settle_status_r[31:28]; is_hex = 1'b1; end
            10'd263: begin hex_nibble = settle_status_r[27:24]; is_hex = 1'b1; end
            10'd264: begin hex_nibble = settle_status_r[23:20]; is_hex = 1'b1; end
            10'd265: begin hex_nibble = settle_status_r[19:16]; is_hex = 1'b1; end
            10'd266: begin hex_nibble = settle_status_r[15:12]; is_hex = 1'b1; end
            10'd267: begin hex_nibble = settle_status_r[11:8]; is_hex = 1'b1; end
            10'd268: begin hex_nibble = settle_status_r[7:4]; is_hex = 1'b1; end
            10'd269: begin hex_nibble = settle_status_r[3:0]; is_hex = 1'b1; end
            10'd270: literal_char = " ";
            10'd271: literal_char = "C";
            10'd272: literal_char = "T";
            10'd273: literal_char = "=";
            10'd274: literal_char = capture_travel_r[31] ? "-" : "+";
            10'd275: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd276: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd277: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd278: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd279: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd280: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd281: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd282: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd283: literal_char = " ";
            10'd284: literal_char = "L";
            10'd285: literal_char = "O";
            10'd286: literal_char = "=";
            10'd287: literal_char = lock_origin_r[31] ? "-" : "+";
            10'd288: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd289: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd290: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd291: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd292: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd293: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd294: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd295: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd296: literal_char = " ";
            10'd297: literal_char = "X";
            10'd298: literal_char = "P";
            10'd299: literal_char = "=";
            10'd300: literal_char = hold_position_r[31] ? "-" : "+";
            10'd301: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd302: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd303: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd304: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd305: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd306: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd307: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd308: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd309: literal_char = " ";
            10'd310: literal_char = "B";
            10'd311: literal_char = "C";
            10'd312: literal_char = "=";
            10'd313: begin hex_nibble = boost_settings_r[31:28]; is_hex = 1'b1; end
            10'd314: begin hex_nibble = boost_settings_r[27:24]; is_hex = 1'b1; end
            10'd315: begin hex_nibble = boost_settings_r[23:20]; is_hex = 1'b1; end
            10'd316: begin hex_nibble = boost_settings_r[19:16]; is_hex = 1'b1; end
            10'd317: begin hex_nibble = boost_settings_r[15:12]; is_hex = 1'b1; end
            10'd318: begin hex_nibble = boost_settings_r[11:8]; is_hex = 1'b1; end
            10'd319: begin hex_nibble = boost_settings_r[7:4]; is_hex = 1'b1; end
            10'd320: begin hex_nibble = boost_settings_r[3:0]; is_hex = 1'b1; end
            10'd321: literal_char = PID_DIAGNOSTICS ? 8'd32 : 8'd13;
            10'd322: literal_char = PID_DIAGNOSTICS ? "P" : 8'd10;
            10'd323: literal_char = "I";
            10'd324: literal_char = "=";
            10'd325: literal_char = pid_integral_r[31] ? "-" : "+";
            10'd326: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd327: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd328: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd329: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd330: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd331: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd332: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd333: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd334: literal_char = HOLD_DIAGNOSTICS ? " " : 8'd13;
            10'd335: literal_char = HOLD_DIAGNOSTICS ? "V" : 8'd10;
            10'd336: literal_char = "R";
            10'd337: literal_char = "=";
            10'd338: literal_char = velocity_ref_r[31] ? "-" : "+";
            10'd339: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd340: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd341: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd342: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd343: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd344: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd345: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd346: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd347: literal_char = " ";
            10'd348: literal_char = "A";
            10'd349: literal_char = "R";
            10'd350: literal_char = "=";
            10'd351: literal_char = angle_ref_r[31] ? "-" : "+";
            10'd352: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd353: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd354: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd355: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd356: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd357: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd358: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd359: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd360: literal_char = " ";
            10'd361: literal_char = "H";
            10'd362: literal_char = "C";
            10'd363: literal_char = "=";
            10'd364: begin hex_nibble = hold_control_r[31:28]; is_hex = 1'b1; end
            10'd365: begin hex_nibble = hold_control_r[27:24]; is_hex = 1'b1; end
            10'd366: begin hex_nibble = hold_control_r[23:20]; is_hex = 1'b1; end
            10'd367: begin hex_nibble = hold_control_r[19:16]; is_hex = 1'b1; end
            10'd368: begin hex_nibble = hold_control_r[15:12]; is_hex = 1'b1; end
            10'd369: begin hex_nibble = hold_control_r[11:8]; is_hex = 1'b1; end
            10'd370: begin hex_nibble = hold_control_r[7:4]; is_hex = 1'b1; end
            10'd371: begin hex_nibble = hold_control_r[3:0]; is_hex = 1'b1; end
            10'd372: literal_char = AGE_DIAGNOSTICS ? " " : 8'd13;
            10'd373: literal_char = AGE_DIAGNOSTICS ? "S" : 8'd10;
            10'd374: literal_char = "D";
            10'd375: literal_char = "=";
            10'd376: begin hex_nibble = speed_status_r[31:28]; is_hex = 1'b1; end
            10'd377: begin hex_nibble = speed_status_r[27:24]; is_hex = 1'b1; end
            10'd378: begin hex_nibble = speed_status_r[23:20]; is_hex = 1'b1; end
            10'd379: begin hex_nibble = speed_status_r[19:16]; is_hex = 1'b1; end
            10'd380: begin hex_nibble = speed_status_r[15:12]; is_hex = 1'b1; end
            10'd381: begin hex_nibble = speed_status_r[11:8]; is_hex = 1'b1; end
            10'd382: begin hex_nibble = speed_status_r[7:4]; is_hex = 1'b1; end
            10'd383: begin hex_nibble = speed_status_r[3:0]; is_hex = 1'b1; end
            10'd384: literal_char = " ";
            10'd385: literal_char = "S";
            10'd386: literal_char = "A";
            10'd387: literal_char = "=";
            10'd388: begin hex_nibble = speed_status_r[63:60]; is_hex = 1'b1; end
            10'd389: begin hex_nibble = speed_status_r[59:56]; is_hex = 1'b1; end
            10'd390: begin hex_nibble = speed_status_r[55:52]; is_hex = 1'b1; end
            10'd391: begin hex_nibble = speed_status_r[51:48]; is_hex = 1'b1; end
            10'd392: begin hex_nibble = speed_status_r[47:44]; is_hex = 1'b1; end
            10'd393: begin hex_nibble = speed_status_r[43:40]; is_hex = 1'b1; end
            10'd394: begin hex_nibble = speed_status_r[39:36]; is_hex = 1'b1; end
            10'd395: begin hex_nibble = speed_status_r[35:32]; is_hex = 1'b1; end
            10'd396: literal_char = ARM_DIAGNOSTICS ? " " : 8'd13;
            10'd397: literal_char = ARM_DIAGNOSTICS ? "O" : 8'd10;
            10'd398: literal_char = "A";
            10'd399: literal_char = "=";
            10'd400: literal_char = original_arm_r[31] ? "-" : "+";
            10'd401: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd402: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd403: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd404: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd405: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd406: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd407: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd408: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd409: literal_char = " ";
            10'd410: literal_char = "A";
            10'd411: literal_char = "Q";
            10'd412: literal_char = "=";
            10'd413: literal_char = arm_request_r[31] ? "-" : "+";
            10'd414: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd415: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd416: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd417: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd418: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd419: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd420: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd421: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd422: literal_char = 8'd13;
            10'd423: literal_char = 8'd10;
            10'd424: literal_char = " ";
            10'd425: literal_char = "N";
            10'd426: literal_char = "V";
            10'd427: literal_char = "=";
            10'd428: literal_char = loop_reference_r[31] ? "-":"+";
            10'd429: begin hex_nibble = display_magnitude[31:28]; is_hex = 1'b1; end
            10'd430: begin hex_nibble = display_magnitude[27:24]; is_hex = 1'b1; end
            10'd431: begin hex_nibble = display_magnitude[23:20]; is_hex = 1'b1; end
            10'd432: begin hex_nibble = display_magnitude[19:16]; is_hex = 1'b1; end
            10'd433: begin hex_nibble = display_magnitude[15:12]; is_hex = 1'b1; end
            10'd434: begin hex_nibble = display_magnitude[11:8]; is_hex = 1'b1; end
            10'd435: begin hex_nibble = display_magnitude[7:4]; is_hex = 1'b1; end
            10'd436: begin hex_nibble = display_magnitude[3:0]; is_hex = 1'b1; end
            10'd437: literal_char = " ";
            10'd438: literal_char = "N";
            10'd439: literal_char = "S";
            10'd440: literal_char = "=";
            10'd441: begin hex_nibble = loop_state_r; is_hex = 1'b1; end
            10'd442: literal_char = 8'd32;
            10'd443: literal_char = 8'd66;
            10'd444: literal_char = 8'd71;
            10'd445: literal_char = 8'd61;
            10'd446: begin hex_nibble = GAIN_Q8[15:12]; is_hex=1'b1; end
            10'd447: begin hex_nibble = GAIN_Q8[11:8]; is_hex=1'b1; end
            10'd448: begin hex_nibble = GAIN_Q8[7:4]; is_hex=1'b1; end
            10'd449: begin hex_nibble = GAIN_Q8[3:0]; is_hex=1'b1; end
            10'd450: literal_char = 8'd32;
            10'd451: literal_char = 8'd66;
            10'd452: literal_char = 8'd83;
            10'd453: literal_char = 8'd61;
            10'd454: begin hex_nibble = brake_status_r[7:4]; is_hex=1'b1; end
            10'd455: begin hex_nibble = brake_status_r[3:0]; is_hex=1'b1; end
            10'd456: literal_char = 8'd32;
            10'd457: literal_char = 8'd66;
            10'd458: literal_char = 8'd65;
            10'd459: literal_char = 8'd61;
            10'd460: begin hex_nibble=brake_accel_r[31:28];is_hex=1;end
            10'd461: begin hex_nibble=brake_accel_r[27:24];is_hex=1;end
            10'd462: begin hex_nibble=brake_accel_r[23:20];is_hex=1;end
            10'd463: begin hex_nibble=brake_accel_r[19:16];is_hex=1;end
            10'd464: begin hex_nibble=brake_accel_r[15:12];is_hex=1;end
            10'd465: begin hex_nibble=brake_accel_r[11:8];is_hex=1;end
            10'd466: begin hex_nibble=brake_accel_r[7:4];is_hex=1;end
            10'd467: begin hex_nibble=brake_accel_r[3:0];is_hex=1;end
            10'd468: literal_char=8'd32;
            10'd469: literal_char=8'd65;
            10'd470: literal_char=8'd83;
            10'd471: literal_char=8'd61;
            10'd472: begin hex_nibble=approach_speed_r[31:28];is_hex=1;end
            10'd473: begin hex_nibble=approach_speed_r[27:24];is_hex=1;end
            10'd474: begin hex_nibble=approach_speed_r[23:20];is_hex=1;end
            10'd475: begin hex_nibble=approach_speed_r[19:16];is_hex=1;end
            10'd476: begin hex_nibble=approach_speed_r[15:12];is_hex=1;end
            10'd477: begin hex_nibble=approach_speed_r[11:8];is_hex=1;end
            10'd478: begin hex_nibble=approach_speed_r[7:4];is_hex=1;end
            10'd479: begin hex_nibble=approach_speed_r[3:0];is_hex=1;end
            10'd480: literal_char=8'd32;
            10'd481: literal_char=8'd66;
            10'd482: literal_char=8'd82;
            10'd483: literal_char=8'd61;
            10'd484: begin hex_nibble=release_rate_r[15:12];is_hex=1;end
            10'd485: begin hex_nibble=release_rate_r[11:8];is_hex=1;end
            10'd486: begin hex_nibble=release_rate_r[7:4];is_hex=1;end
            10'd487: begin hex_nibble=release_rate_r[3:0];is_hex=1;end
            10'd488: literal_char=8'd32;
            10'd489: literal_char=8'd66;
            10'd490: literal_char=8'd68;
            10'd491: literal_char=8'd61;
            10'd492: begin hex_nibble=brake_distance_r[31:28];is_hex=1;end
            10'd493: begin hex_nibble=brake_distance_r[27:24];is_hex=1;end
            10'd494: begin hex_nibble=brake_distance_r[23:20];is_hex=1;end
            10'd495: begin hex_nibble=brake_distance_r[19:16];is_hex=1;end
            10'd496: begin hex_nibble=brake_distance_r[15:12];is_hex=1;end
            10'd497: begin hex_nibble=brake_distance_r[11:8];is_hex=1;end
            10'd498: begin hex_nibble=brake_distance_r[7:4];is_hex=1;end
            10'd499: begin hex_nibble=brake_distance_r[3:0];is_hex=1;end
            10'd500: literal_char=8'd32;
            10'd501: literal_char=8'd66;
            10'd502: literal_char=8'd70;
            10'd503: literal_char=8'd61;
            10'd504: begin hex_nibble=brake_flags_r[3:0];is_hex=1;end
            10'd505: literal_char=8'd32;
            10'd506: literal_char=8'd86;
            10'd507: literal_char=8'd69;
            10'd508: literal_char=8'd61;
            10'd509: literal_char=velocity_error_r[32]?"-":"+";
            10'd510: begin hex_nibble=display_magnitude[35:32];is_hex=1;end
            10'd511: begin hex_nibble=display_magnitude[31:28];is_hex=1;end
            10'd512: begin hex_nibble=display_magnitude[27:24];is_hex=1;end
            10'd513: begin hex_nibble=display_magnitude[23:20];is_hex=1;end
            10'd514: begin hex_nibble=display_magnitude[19:16];is_hex=1;end
            10'd515: begin hex_nibble=display_magnitude[15:12];is_hex=1;end
            10'd516: begin hex_nibble=display_magnitude[11:8];is_hex=1;end
            10'd517: begin hex_nibble=display_magnitude[7:4];is_hex=1;end
            10'd518: begin hex_nibble=display_magnitude[3:0];is_hex=1;end
            10'd519: literal_char=8'd13;
            10'd520: literal_char=8'd10;
            default: literal_char = 8'h3F;
        endcase
        next_char = is_hex ? hex_ascii(hex_nibble) : literal_char;
        if(MOTION_REPORT && index>=BASE_LAST-1)begin
            if(append_index==36)next_char=13;
            else if(append_index==37)next_char=10;
            else case(append_offset)
                0:next_char=" ";
                1:next_char=append_index<24 ? "R":"A";
                2:next_char=append_index<12 ? "P":append_index<24 ? "V":"I";
                3:next_char="=";
                default:next_char=hex_ascii((append_word>>((11-append_offset)*4)) & 4'hf);
            endcase
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motion_position_r<=0;motion_velocity_r<=0;outer_i_r<=0;
            brake_accel_r<=0;approach_speed_r<=0;release_rate_r<=0;brake_distance_r<=0;velocity_error_r<=0;brake_flags_r<=0;GAIN_Q8<=BRAKE_DISTANCE_GAIN_Q8;brake_status_r<=0; index <= 0; tx_state <= IDLE; version_r<=0; run_r<=0;
            velocity_ref_r<=0;angle_ref_r<=0;hold_control_r<=0;pid_integral_r<=0;settle_status_r<=0;boost_settings_r<=0;capture_travel_r<=0;lock_origin_r<=0;hold_position_r<=0;
            loop_reference_r<=0;loop_state_r<=0;original_arm_r<=0;arm_request_r<=0;speed_status_r<=0;raw_speed_r<=0;est_speed_r<=0;speed_source_r<=0;est_valid_r<=0;
            uart_data_o <= 0; uart_start_o <= 0; cal_r <= 0; go_r <= 0; adc_r <= 0; reseeds_r <= 0;
            ms_i_r <= 0;
            seq_i_r <= 0;
            theta_r <= 0;
            theta_dot_r <= 0;
            alpha_r <= 0;
            alpha_dot_r <= 0;
            control_u_r <= 0;
            safe_u_r <= 0;
            pa_i_r <= 0;
            pd_i_r <= 0;
            pt_i_r <= 0;
            pv_i_r <= 0;
            exec_i_r <= 0;
            flags_i_r <= 0;
            raw_i_r <= 0;
            zero_i_r <= 0;
        end else begin
            uart_start_o <= 0;
            case (tx_state)
                IDLE: if (send_i && !uart_busy_i) begin
                        motion_position_r<=MOTION_REPORT ? motion_position_i:32'sd0;
                        motion_velocity_r<=MOTION_REPORT ? motion_velocity_i[18:0]:19'sd0;
                        outer_i_r<=MOTION_REPORT ? outer_i_i[13:0]:14'sd0;
                        brake_accel_r<=BRAKE_DIAGNOSTICS ? {13'd0,brake_accel_i}:32'd0;approach_speed_r<=BRAKE_DIAGNOSTICS ? {15'd0,approach_speed_i}:32'd0;release_rate_r<=BRAKE_DIAGNOSTICS ? {6'd0,release_rate_i}:16'd0;brake_distance_r<=BRAKE_DIAGNOSTICS ? brake_distance_i:32'd0;velocity_error_r<=BRAKE_DIAGNOSTICS ? velocity_error_i:33'd0;brake_flags_r<=BRAKE_DIAGNOSTICS ? brake_flags_i:4'd0;GAIN_Q8<=BRAKE_DIAGNOSTICS ? (LIVE_BRAKE_CONFIG?{6'd0,brake_gain_i}:BRAKE_DISTANCE_GAIN_Q8):BRAKE_DISTANCE_GAIN_Q8;brake_status_r<=BRAKE_DIAGNOSTICS ? brake_status_i:8'd0; version_r<=version_i; run_r<=run_i;
                        velocity_ref_r<=velocity_ref_i;angle_ref_r<=angle_ref_i;hold_control_r<=hold_control_i;pid_integral_r<=pid_integral_i;settle_status_r<=settle_status_i;boost_settings_r<=COMPACT_DIAGNOSTICS ? 32'd0:boost_settings_i;
                        capture_travel_r<=capture_travel_i;lock_origin_r<=COMPACT_DIAGNOSTICS ? 32'd0:lock_origin_i;hold_position_r<=COMPACT_DIAGNOSTICS ? 32'd0:hold_position_i;
                        loop_reference_r<=loop_reference_i;loop_state_r<=loop_state_i;original_arm_r<=original_arm_i;arm_request_r<=arm_request_i;speed_status_r<=speed_status_i;raw_speed_r<=raw_speed_i;est_speed_r<=est_speed_i;speed_source_r<=speed_source_i;est_valid_r<=est_valid_i;
                    ms_i_r <= ms_i;
                        seq_i_r <= seq_i;
                        theta_r <= theta;
                        theta_dot_r <= theta_dot;
                        alpha_r <= alpha;
                        alpha_dot_r <= alpha_dot;
                        control_u_r <= control_u;
                        safe_u_r <= safe_u;
                        pa_i_r <= pa_i;
                        pd_i_r <= pd_i;
                        pt_i_r <= pt_i;
                        pv_i_r <= pv_i;
                        exec_i_r <= exec_i;
                        flags_i_r <= flags_i;
                        raw_i_r <= COMPACT_DIAGNOSTICS ? 16'd0:raw_i;
                        zero_i_r <= COMPACT_DIAGNOSTICS ? 32'd0:zero_i;
                    cal_r <= cal_i; go_r <= COMPACT_DIAGNOSTICS ? 1'b0:go_i; adc_r <= COMPACT_DIAGNOSTICS ? 16'd0:adc_i; reseeds_r <= COMPACT_DIAGNOSTICS ? 8'd0:reseeds_i;
                    index <= 0; tx_state <= LOAD;
                end
                LOAD: if (!uart_busy_i) begin
                    uart_data_o <= next_char;
                    tx_state <= START;
                end
                START: begin
                    uart_start_o <= 1;
                    tx_state <= WAIT_BUSY;
                end
                WAIT_BUSY: if (uart_busy_i) tx_state <= WAIT_DONE;
                WAIT_DONE: if (!uart_busy_i) begin
                    if (index == BASE_LAST+(MOTION_REPORT ? 36:0)) tx_state <= IDLE;
                    else begin index <= index + 1'b1; tx_state <= LOAD; end
                end
                default: tx_state <= IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire


