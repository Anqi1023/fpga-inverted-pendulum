// R14B profile14: return coordination IDs70..72, STOP-only atomic APPLY.
// R14A profile13 adds ID64 mode2; original R14 mode1 remains available.
// Retired IDs46..53/58..63 remain read-only defaults.
// R13 profile10, fine drive57 + bounded arm reference58..63.
// R12: EXT_POSITION=8, profile9, PID IDs54..56.
// R11: EXT_POSITION=7, INFO A=8; settle parameters46..53; ID40 position status.
// APPLY remains stop-only. STOP has priority while any response is transmitting.
`timescale 1ns/1ps
`default_nettype none
module serial_parameter_bank #(parameter integer EXT_POSITION=0, DEFAULT_KAD=55, TARGET_BITS=(EXT_POSITION>=33 ? 32:19))(
    input wire clk,rst_n,tick_i,
    input wire [3:0] command_i,
    input wire command_valid_i,
    input wire [7:0] id_i,
    input wire signed [31:0] value_i,
    input wire stopped_i,ready_i,
    input wire position_ready_i,
    input wire signed [31:0] position_actual_i,
    input wire signed [TARGET_BITS-1:0] position_target_i,
    output reg position_set_o,
    output reg signed [TARGET_BITS-1:0] position_value_o,
    input wire [2:0] phase_i,
    input wire [3:0] reason_i,
    input wire tx_busy_i,
    output reg [7:0] tx_data_o,
    output reg tx_start_o,
    output wire response_pending_o,
    output wire [1279:0] config_o,
    output wire [63:0] speed_config_o,
    output wire [95:0] boost_config_o,
    output wire [255:0] settle_config_o,
    output wire [95:0] pid_config_o,
    output wire [223:0] motion_config_o,
    output wire [191:0] position_i_config_o,
    output wire [95:0] return_config_o,
    output wire [95:0] near_config_o,
    output wire [1:0] speed_filter_mode_o,
    output wire signed [10:0] k_theta_x10_o,
    output wire [6:0] return_position_cap_o,
    output wire manual_start_o,
    output wire [127:0] arm_loop_config_o,
    output wire [63:0] brake_config_o,
    output wire [95:0] coordinate_config_o,
    output wire velocity_slew_bypass_o,
    output wire [63:0] point_config_o,
    output wire [63:0] motion_pi_config_o,
    output wire [223:0] cruise_config_o,
    output wire [63:0] cruise_gains_o,
    input wire [31:0] trace_status_i,trace_reference_cycle_i,trace_capture_version_i,
    input wire [31:0] trace_arm_ms_i,trace_reference_ms_i,
    input wire [415:0] trace_data_i,
    output wire [6:0] trace_read_index_o,
    output wire trace_arm_o,trace_freeze_o,trace_reference_mode_o,
    output reg [31:0] version_o,
    output reg run_o,rearm_o,applied_o
);
    wire signed [31:0] active[0:109];
    wire signed [31:0] pending[0:109];
    function signed [31:0] default_value;
        input [7:0] id;
        begin case(id)
            8'd0:default_value=32'sd2000;
            8'd1:default_value=DEFAULT_KAD;
            8'd2:default_value=(EXT_POSITION>=3) ? 32'sd8 : 32'sd3;
            8'd3:default_value=32'sd35;
            8'd4:default_value=-32'sd500;
            8'd5:default_value=32'sd550;
            8'd6:default_value=32'sd80;
            8'd7:default_value=32'sd262144;
            8'd8:default_value=32'sd40;
            8'd9:default_value=32'sd1;
            8'd10:default_value=32'sd8;
            8'd11:default_value=32'sd262144;
            8'd12:default_value=32'sd3;
            8'd13:default_value=32'sd6;
            8'd14:default_value=32'sd32768;
            8'd15:default_value=32'sd1;
            8'd16:default_value=32'sd34315;
            8'd17:default_value=32'sd12;
            8'd18:default_value=32'sd11796;
            8'd19:default_value=32'sd163840;
            8'd20:default_value=32'sd11469;
            8'd21:default_value=32'sd45753;
            8'd22:default_value=32'sd550;
            8'd23:default_value=32'sd32768;
            8'd24:default_value=32'sd524288;
            8'd25:default_value=32'sd1;
            8'd26:default_value=32'sd20;
            8'd27:default_value=32'sd60;
            8'd28:default_value=32'sd327680;
            8'd29:default_value=(EXT_POSITION>=3) ? 32'sd163840 : 32'sd400000;
            8'd30:default_value=32'sd280;
            8'd31:default_value=32'sd655;
            8'd32:default_value=32'sd100;
            8'd33:default_value=32'sd1000;
            8'd34:default_value=32'sd39322;
            8'd35:default_value=32'sd0;
            8'd36:default_value=32'sd393216;
            8'd37:default_value=32'sd262144;
            8'd38:default_value=32'sd150;
            8'd39:default_value=32'sd2400;
            8'd41:default_value=0;
            8'd42:default_value=40;
            8'd43:default_value=0;
            8'd44:default_value=150;
            8'd45:default_value=4;
            8'd46:default_value=0;
            8'd47:default_value=35;
            8'd48:default_value=19661;
            8'd49:default_value=2288;
            8'd50:default_value=19661;
            8'd51:default_value=60;
            8'd52:default_value=4575;
            8'd53:default_value=65536;
            8'd54:default_value=30;
            8'd55:default_value=20;
            8'd56:default_value=3431;
            8'd57:default_value=0;
            8'd58:default_value=0;
            8'd59:default_value=14980;
            8'd60:default_value=1147;
            8'd61:default_value=65536;
            8'd62:default_value=2288;
            8'd63:default_value=11438;
            8'd64:default_value=0;
            8'd65:default_value=0;
            8'd66:default_value=10;
            8'd67:default_value=2288;
            8'd68:default_value=32768;
            8'd70:default_value=0;
            8'd71:default_value=187246;
            8'd72:default_value=35;
            8'd69:default_value=131072;
            8'd73:default_value=0;
            8'd74:default_value=0;
            8'd75:default_value=11438;
            8'd76:default_value=4;
            8'd77:default_value=32'sd80;
            8'd78:default_value=32'sd60;
            8'd79:default_value=32'sd0;
            8'd80:default_value=(EXT_POSITION>=23)?32'sd1:32'sd0;
            8'd81:default_value=32'sd105;
            8'd82:default_value=32'sd300;
            8'd83:default_value=32'sd0;
            8'd84:default_value=32'sd384;
            8'd85:default_value=32'sd20;
            8'd86:default_value=1;
            8'd87:default_value=65536;
            8'd88:default_value=23400;
            8'd89:default_value=1000;
            8'd90:default_value=(EXT_POSITION>=28)?32'sd1:32'sd0;
            8'd91:default_value=1;
            8'd92:default_value=22876;
            8'd93:default_value=23400;
            8'd94:default_value=160;
            8'd95:default_value=1;
            8'd96:default_value=29952;
            8'd97:default_value=210;
            8'd98:default_value=0;
            8'd99:default_value=10;
            8'd100:default_value=286;
            8'd101:default_value=11438;

            8'd102:default_value=32'sd23400;
            8'd103:default_value=32'sd128;
            8'd104:default_value=32'sd3431;
            8'd105:default_value=32'sd65536;
            8'd106:default_value=32'sd100;
            8'd107:default_value=32'sd100;
            8'd108:default_value=32'sd2400;
            8'd109:default_value=32'sd100;
            default:default_value=0;
        endcase
        // R14I: exact uploaded operating defaults; historical profiles unchanged.
        if(EXT_POSITION>=20)begin case(id)
            8'd0:default_value=32'sd2300;
            8'd1:default_value=32'sd150;
            8'd2:default_value=32'sd8;
            8'd3:default_value=32'sd35;
            8'd4:default_value=-32'sd500;
            8'd5:default_value=32'sd550;
            8'd6:default_value=32'sd80;
            8'd7:default_value=32'sd262144;
            8'd8:default_value=32'sd40;
            8'd9:default_value=32'sd1;
            8'd10:default_value=32'sd8;
            8'd11:default_value=32'sd262144;
            8'd12:default_value=32'sd3;
            8'd13:default_value=32'sd6;
            8'd14:default_value=32'sd32768;
            8'd15:default_value=32'sd1;
            8'd16:default_value=32'sd34315;
            8'd17:default_value=32'sd12;
            8'd18:default_value=32'sd11796;
            8'd19:default_value=32'sd163840;
            8'd20:default_value=32'sd11469;
            8'd21:default_value=32'sd45753;
            8'd22:default_value=32'sd550;
            8'd23:default_value=32'sd32768;
            8'd24:default_value=32'sd524288;
            8'd25:default_value=32'sd1;
            8'd26:default_value=32'sd20;
            8'd27:default_value=32'sd60;
            8'd28:default_value=32'sd327680;
            8'd29:default_value=32'sd163840;
            8'd30:default_value=32'sd280;
            8'd31:default_value=32'sd655;
            8'd32:default_value=32'sd100;
            8'd33:default_value=32'sd1000;
            8'd34:default_value=32'sd39322;
            8'd35:default_value=32'sd0;
            8'd36:default_value=32'sd393216;
            8'd37:default_value=32'sd262144;
            8'd38:default_value=32'sd150;
            8'd39:default_value=32'sd2400;
            8'd41:default_value=32'sd1;
            8'd42:default_value=32'sd40;
            8'd43:default_value=32'sd0;
            8'd44:default_value=32'sd150;
            8'd45:default_value=32'sd4;
            8'd46:default_value=32'sd0;
            8'd47:default_value=32'sd35;
            8'd48:default_value=32'sd19661;
            8'd49:default_value=32'sd2288;
            8'd50:default_value=32'sd19661;
            8'd51:default_value=32'sd60;
            8'd52:default_value=32'sd4575;
            8'd53:default_value=32'sd65536;
            8'd54:default_value=32'sd30;
            8'd55:default_value=32'sd20;
            8'd56:default_value=32'sd3431;
            8'd57:default_value=32'sd1;
            8'd58:default_value=32'sd0;
            8'd59:default_value=32'sd14980;
            8'd60:default_value=32'sd1147;
            8'd61:default_value=32'sd65536;
            8'd62:default_value=32'sd2288;
            8'd63:default_value=32'sd11438;
            8'd64:default_value=32'sd1;
            8'd65:default_value=32'sd0;
            8'd66:default_value=32'sd10;
            8'd67:default_value=32'sd2288;
            8'd68:default_value=32'sd32768;
            8'd69:default_value=32'sd131072;
            8'd70:default_value=32'sd1;
            8'd71:default_value=32'sd234000;
            8'd72:default_value=32'sd105;
            8'd73:default_value=32'sd0;
            8'd74:default_value=32'sd1;
            8'd75:default_value=32'sd11438;
            8'd76:default_value=32'sd4;
            8'd77:default_value=32'sd80;
            8'd78:default_value=32'sd60;
            8'd79:default_value=32'sd1;
            8'd80:default_value=(EXT_POSITION>=23)?32'sd1:32'sd0;
            8'd81:default_value=32'sd105;
            8'd82:default_value=32'sd300;
            8'd83:default_value=32'sd0;
            8'd84:default_value=32'sd384;
            8'd85:default_value=32'sd20;
            8'd86:default_value=1;
            8'd87:default_value=65536;
            8'd88:default_value=23400;
            8'd89:default_value=1000;
            8'd90:default_value=(EXT_POSITION>=28)?32'sd1:32'sd0;
            8'd91:default_value=1;
            8'd92:default_value=22876;
            8'd93:default_value=23400;
            8'd94:default_value=160;
            8'd95:default_value=1;
            8'd96:default_value=29952;
            8'd97:default_value=210;
            8'd98:default_value=0;
            8'd99:default_value=10;
            8'd100:default_value=286;
            8'd101:default_value=11438;
            8'd102:default_value=32'sd23400;
            8'd103:default_value=32'sd128;
            8'd104:default_value=32'sd3431;
            8'd105:default_value=32'sd65536;
            8'd106:default_value=32'sd100;
            8'd107:default_value=32'sd100;
            8'd108:default_value=32'sd2400;
            8'd109:default_value=32'sd100;
            default:default_value=0;
        endcase end
            if(EXT_POSITION>=24)begin case(id)
                8'd81:default_value=32'sd1311;
                8'd82:default_value=32'sd2621;
                8'd83:default_value=32'sd0;
                8'd84:default_value=32'sd2288;
                8'd85:default_value=32'sd128;
                8'd86:default_value=32'sd11438;
                8'd87:default_value=32'sd3431;
                8'd88:default_value=32'sd32768;
                8'd89:default_value=32'sd1144;
                default:begin end
            endcase end
            if(EXT_POSITION>=25)begin case(id)
                8'd1:default_value=32'sd105;
                8'd81:default_value=32'sd0;
                8'd82:default_value=32'sd160;
                8'd89:default_value=32'sd300;
                default:begin end
            endcase end
        end
    endfunction
    function signed [31:0] minimum_value;
        input [7:0] id;
        begin case(id)
            8'd0:minimum_value=32'sd0;
            8'd1:minimum_value=-32'sd500;
            8'd2:minimum_value=-32'sd100;
            8'd3:minimum_value=-32'sd200;
            8'd4:minimum_value=-32'sd10000;
            8'd5:minimum_value=32'sd100;
            8'd6:minimum_value=32'sd1;
            8'd7:minimum_value=32'sd32768;
            8'd8:minimum_value=32'sd0;
            8'd9:minimum_value=32'sd0;
            8'd10:minimum_value=32'sd1;
            8'd11:minimum_value=32'sd32768;
            8'd12:minimum_value=32'sd1;
            8'd13:minimum_value=32'sd1;
            8'd14:minimum_value=32'sd6554;
            8'd15:minimum_value=32'sd0;
            8'd16:minimum_value=32'sd5719;
            8'd17:minimum_value=32'sd1;
            8'd18:minimum_value=32'sd2288;
            8'd19:minimum_value=32'sd32768;
            8'd20:minimum_value=32'sd4096;
            8'd21:minimum_value=32'sd6863;
            8'd22:minimum_value=32'sd100;
            8'd23:minimum_value=32'sd6554;
            8'd24:minimum_value=32'sd65536;
            8'd25:minimum_value=32'sd0;
            8'd26:minimum_value=32'sd1;
            8'd27:minimum_value=32'sd1;
            8'd28:minimum_value=32'sd65536;
            8'd29:minimum_value=32'sd65536;
            8'd30:minimum_value=32'sd50;
            8'd31:minimum_value=32'sd0;
            8'd32:minimum_value=32'sd0;
            8'd33:minimum_value=32'sd100;
            8'd34:minimum_value=32'sd11796;
            8'd35:minimum_value=32'sd0;
            8'd36:minimum_value=32'sd196608;
            8'd37:minimum_value=32'sd32768;
            8'd38:minimum_value=32'sd100;
            8'd39:minimum_value=32'sd200;
            8'd41:minimum_value=0;
            8'd42:minimum_value=10;
            8'd43:minimum_value=0;
            8'd44:minimum_value=80;
            8'd45:minimum_value=1;
            8'd46:minimum_value=0;
            8'd47:minimum_value=-200;
            8'd48:minimum_value=3277;
            8'd49:minimum_value=572;
            8'd50:minimum_value=3277;
            8'd51:minimum_value=20;
            8'd52:minimum_value=1144;
            8'd53:minimum_value=6554;
            8'd54:minimum_value=0;
            8'd55:minimum_value=0;
            8'd56:minimum_value=572;
            8'd57:minimum_value=0;
            8'd58:minimum_value=0;
            8'd59:minimum_value=0;
            8'd60:minimum_value=-32768;
            8'd61:minimum_value=3277;
            8'd62:minimum_value=572;
            8'd63:minimum_value=1144;
            8'd64:minimum_value=0;
            8'd65:minimum_value=-64;
            8'd66:minimum_value=0;
            8'd67:minimum_value=572;
            8'd68:minimum_value=3277;
            8'd70:minimum_value=0;
            8'd71:minimum_value=16384;
            8'd72:minimum_value=1;
            8'd69:minimum_value=3277;
            8'd73:minimum_value=0;
            8'd74:minimum_value=0;
            8'd75:minimum_value=1144;
            8'd76:minimum_value=0;
            8'd77:minimum_value=-32'sd1000;
            8'd78:minimum_value=32'sd1;
            8'd79:minimum_value=32'sd0;
            8'd80:minimum_value=32'sd0;
            8'd81:minimum_value=32'sd0;
            8'd82:minimum_value=32'sd1;
            8'd83:minimum_value=32'sd0;
            8'd84:minimum_value=32'sd256;
            8'd85:minimum_value=32'sd10;
            8'd86:minimum_value=0;
            8'd87:minimum_value=6554;
            8'd88:minimum_value=3277;
            8'd89:minimum_value=10;

            8'd91:minimum_value=0;
            8'd92:minimum_value=2288;
            8'd93:minimum_value=3277;
            8'd94:minimum_value=0;
            8'd95:minimum_value=0;
            8'd96:minimum_value=4194;
            8'd97:minimum_value=16;
            8'd98:minimum_value=0;
            8'd99:minimum_value=6;
            8'd100:minimum_value=0;
            8'd101:minimum_value=2288;
            8'd102:minimum_value=-32'sd65536;
            8'd103:minimum_value=32'sd16;
            8'd104:minimum_value=32'sd1144;
            8'd105:minimum_value=32'sd6554;
            8'd106:minimum_value=32'sd20;
            8'd107:minimum_value=32'sd10;
            8'd108:minimum_value=32'sd0;
            8'd109:minimum_value=-32'sd500;
            default:minimum_value=0;
        endcase
            if(EXT_POSITION>=10 && ((id>=46 && id<=53) || (id>=58 && id<=63)))
                minimum_value=default_value(id);
            if(EXT_POSITION>=36 && (id==2 || (id>=74 && id<=78))) minimum_value=default_value(id);
            if(EXT_POSITION>=24)begin case(id)
                8'd81:minimum_value=-32'sd8192;
                8'd82:minimum_value=-32'sd8192;
                8'd84:minimum_value=32'sd572;
                8'd85:minimum_value=32'sd16;
                8'd86:minimum_value=32'sd2288;
                8'd87:minimum_value=32'sd1144;
                8'd88:minimum_value=32'sd6554;
                8'd89:minimum_value=32'sd286;
                default:begin end
            endcase end
            if(EXT_POSITION>=25)begin case(id)
                8'd82:minimum_value=32'sd0;
                8'd89:minimum_value=32'sd1;
                default:begin end
            endcase end
            if(EXT_POSITION>=35 && ((id>=64 && id<=69)|| (EXT_POSITION<37 && ((id>=54 && id<=56)||id==80))))
                minimum_value=default_value(id);
            if(EXT_POSITION>=36 && (id==2 || (id>=74 && id<=78))) minimum_value=default_value(id);
        end
    endfunction
    function signed [31:0] maximum_value;
        input [7:0] id;
        begin case(id)
            8'd0:maximum_value=32'sd5000;
            8'd1:maximum_value=32'sd500;
            8'd2:maximum_value=32'sd100;
            8'd3:maximum_value=32'sd200;
            8'd4:maximum_value=32'sd10000;
            8'd5:maximum_value=32'sd800;
            8'd6:maximum_value=32'sd200;
            8'd7:maximum_value=32'sd458752;
            8'd8:maximum_value=32'sd200;
            8'd9:maximum_value=32'sd1;
            8'd10:maximum_value=32'sd32;
            8'd11:maximum_value=32'sd655360;
            8'd12:maximum_value=32'sd32;
            8'd13:maximum_value=32'sd40;
            8'd14:maximum_value=32'sd131072;
            8'd15:maximum_value=32'sd1;
            8'd16:maximum_value=32'sd68629;
            8'd17:maximum_value=32'sd40;
            8'd18:maximum_value=32'sd22876;
            8'd19:maximum_value=32'sd327680;
            8'd20:maximum_value=32'sd32768;
            8'd21:maximum_value=32'sd91506;
            8'd22:maximum_value=32'sd800;
            8'd23:maximum_value=32'sd131072;
            8'd24:maximum_value=32'sd1310720;
            8'd25:maximum_value=32'sd1;
            8'd26:maximum_value=32'sd100;
            8'd27:maximum_value=32'sd200;
            8'd28:maximum_value=32'sd393216;
            8'd29:maximum_value=32'sd524288;
            8'd30:maximum_value=32'sd1000;
            8'd31:maximum_value=32'sd6554;
            8'd32:maximum_value=32'sd200;
            8'd33:maximum_value=32'sd1000;
            8'd34:maximum_value=32'sd39322;
            8'd35:maximum_value=32'sd1;
            8'd36:maximum_value=32'sd458752;
            8'd37:maximum_value=32'sd393216;
            8'd38:maximum_value=32'sd550;
            8'd39:maximum_value=32'sd2400;
            8'd41:maximum_value=1;
            8'd42:maximum_value=100;
            8'd43:maximum_value=1;
            8'd44:maximum_value=250;
            8'd45:maximum_value=10;
            8'd46:maximum_value=1;
            8'd47:maximum_value=200;
            8'd48:maximum_value=65536;
            8'd49:maximum_value=5719;
            8'd50:maximum_value=65536;
            8'd51:maximum_value=400;
            8'd52:maximum_value=11438;
            8'd53:maximum_value=131072;
            8'd54:maximum_value=500;
            8'd55:maximum_value=100;
            8'd56:maximum_value=11438;
            8'd57:maximum_value=1;
            8'd58:maximum_value=1;
            8'd59:maximum_value=131072;
            8'd60:maximum_value=32767;
            8'd61:maximum_value=196608;
            8'd62:maximum_value=5719;
            8'd63:maximum_value=57190;
            8'd64:maximum_value=(EXT_POSITION>=12)?2:1;
            8'd65:maximum_value=64;
            8'd66:maximum_value=50;
            8'd67:maximum_value=5719;
            8'd68:maximum_value=131072;
            8'd70:maximum_value=1;
            8'd71:maximum_value=262144;
            8'd72:maximum_value=200;
            8'd73:maximum_value=2;
            8'd74:maximum_value=1;
            8'd75:maximum_value=34315;
            8'd76:maximum_value=100;
            8'd69:maximum_value=196608;
            8'd77:maximum_value=32'sd1000;
            8'd78:maximum_value=32'sd100;
            8'd79:maximum_value=32'sd1;
            8'd80:maximum_value=32'sd1;
            8'd81:maximum_value=32'sd200;
            8'd82:maximum_value=32'sd500;
            8'd83:maximum_value=32'sd1;
            8'd84:maximum_value=32'sd768;
            8'd85:maximum_value=32'sd40;
            8'd86:maximum_value=1;
            8'd87:maximum_value=262144;
            8'd88:maximum_value=65536;
            8'd89:maximum_value=1000;
            8'd90:maximum_value=(EXT_POSITION>=28)?32'sd1:32'sd0;

            8'd91:maximum_value=1;
            8'd92:maximum_value=102944;
            8'd93:maximum_value=65536;
            8'd94:maximum_value=200;
            8'd95:maximum_value=1;
            8'd96:maximum_value=167772;
            8'd97:maximum_value=1024;
            8'd98:maximum_value=1;
            8'd99:maximum_value=12;
            8'd100:maximum_value=1144;
            8'd101:maximum_value=34315;
            8'd102:maximum_value=32'sd65536;
            8'd103:maximum_value=32'sd1024;
            8'd104:maximum_value=32'sd11438;
            8'd105:maximum_value=32'sd131072;
            8'd106:maximum_value=32'sd400;
            8'd107:maximum_value=32'sd200;
            8'd108:maximum_value=32'sd5000;
            8'd109:maximum_value=32'sd500;
            default:maximum_value=0;
        endcase
            if(EXT_POSITION>=10 && ((id>=46 && id<=53) || (id>=58 && id<=63)))
                maximum_value=default_value(id);
            if(EXT_POSITION>=36 && (id==2 || (id>=74 && id<=78))) maximum_value=default_value(id);
            if(EXT_POSITION>=24)begin case(id)
                8'd81:maximum_value=32'sd8192;
                8'd82:maximum_value=32'sd8192;
                8'd84:maximum_value=32'sd5719;
                8'd85:maximum_value=32'sd512;
                8'd86:maximum_value=32'sd22876;
                8'd87:maximum_value=32'sd11438;
                8'd88:maximum_value=32'sd131072;
                8'd89:maximum_value=32'sd5719;
                default:begin end
            endcase end
            if(EXT_POSITION>=25)begin case(id)
                8'd82:maximum_value=32'sd200;
                8'd89:maximum_value=32'sd500;
                default:begin end
            endcase end
            if(EXT_POSITION>=35 && ((id>=64 && id<=69)|| (EXT_POSITION<37 && ((id>=54 && id<=56)||id==80))))
                maximum_value=default_value(id);
            if(EXT_POSITION>=36 && (id==2 || (id>=74 && id<=78))) maximum_value=default_value(id);
        end
    endfunction
    wire signed [31:0] selected_minimum=minimum_value(id_i);
    wire signed [31:0] selected_maximum=maximum_value(id_i);
    wire value_in_range=(value_i>=selected_minimum) && (value_i<=selected_maximum);
    wire write_pending=command_valid_i && !serial_busy && command_i==2 &&
                       (id_i<40 || (EXT_POSITION>=5 && (id_i==41 || id_i==42)) || (EXT_POSITION>=6 && id_i>=43 && id_i<=45) || (EXT_POSITION>=7 && id_i>=46 && id_i<=53) || (EXT_POSITION>=8 && id_i>=54 && id_i<=56) || (EXT_POSITION>=9 && id_i>=57 && id_i<=63) || (EXT_POSITION>=10 && id_i>=64 && id_i<=69) || (EXT_POSITION>=13 && id_i>=70 && id_i<=72) || (EXT_POSITION>=14 && id_i==73) || (EXT_POSITION>=15 && id_i>=74 && id_i<=76) || (EXT_POSITION>=16 && id_i==77) || (EXT_POSITION>=17 && id_i==78) || (EXT_POSITION>=18 && id_i==79) || (EXT_POSITION>=21 && id_i>=80 && id_i<=83) || (EXT_POSITION>=22 && id_i>=84 && id_i<=85) || (EXT_POSITION>=23 && id_i>=86 && id_i<=89) || (EXT_POSITION>=28 && id_i==90) || (EXT_POSITION>=32 && id_i>=91 && id_i<=94) || (EXT_POSITION>=33 && id_i>=95 && id_i<=101) || (EXT_POSITION>=37 && id_i>=102 && id_i<=109)) && value_in_range;
    assign trace_arm_o=EXT_POSITION>=29 && command_valid_i && !serial_busy &&
        (command_i==9 || (EXT_POSITION>=30 && command_i==13));
    assign trace_reference_mode_o=command_i==13;
    assign trace_freeze_o=EXT_POSITION>=29 && command_valid_i && !serial_busy && command_i==12;
    assign trace_read_index_o=response_id[6:0];
    wire restore_pending=command_valid_i && !serial_busy && command_i==6;
    wire apply_registers=apply_wait && tick_i && !run_o && stopped_i && stopped_ticks>=2;
    function integer storage_width;
        input integer id;
        begin case(id)
            0:storage_width=13;
            1:storage_width=10;
            2:storage_width=8;
            3:storage_width=9;
            4:storage_width=15;
            5:storage_width=10;
            6:storage_width=8;
            7:storage_width=19;
            8:storage_width=8;
            9:storage_width=1;
            10:storage_width=6;
            11:storage_width=20;
            12:storage_width=6;
            13:storage_width=6;
            14:storage_width=18;
            15:storage_width=1;
            16:storage_width=17;
            17:storage_width=6;
            18:storage_width=15;
            19:storage_width=19;
            20:storage_width=16;
            21:storage_width=17;
            22:storage_width=10;
            23:storage_width=18;
            24:storage_width=21;
            25:storage_width=1;
            26:storage_width=7;
            27:storage_width=8;
            28:storage_width=19;
            29:storage_width=20;
            30:storage_width=10;
            31:storage_width=13;
            32:storage_width=8;
            33:storage_width=10;
            34:storage_width=16;
            35:storage_width=1;
            36:storage_width=19;
            37:storage_width=19;
            38:storage_width=10;
            39:storage_width=12;
            default:storage_width=32;
        endcase end
    endfunction
    function signed_storage;
        input integer id;
        begin case(id)
            0:signed_storage=1'b0;
            1:signed_storage=1'b1;
            2:signed_storage=1'b1;
            3:signed_storage=1'b1;
            4:signed_storage=1'b1;
            5:signed_storage=1'b0;
            6:signed_storage=1'b0;
            7:signed_storage=1'b0;
            8:signed_storage=1'b0;
            9:signed_storage=1'b0;
            10:signed_storage=1'b0;
            11:signed_storage=1'b0;
            12:signed_storage=1'b0;
            13:signed_storage=1'b0;
            14:signed_storage=1'b0;
            15:signed_storage=1'b0;
            16:signed_storage=1'b0;
            17:signed_storage=1'b0;
            18:signed_storage=1'b0;
            19:signed_storage=1'b0;
            20:signed_storage=1'b0;
            21:signed_storage=1'b0;
            22:signed_storage=1'b0;
            23:signed_storage=1'b0;
            24:signed_storage=1'b0;
            25:signed_storage=1'b0;
            26:signed_storage=1'b0;
            27:signed_storage=1'b0;
            28:signed_storage=1'b0;
            29:signed_storage=1'b0;
            30:signed_storage=1'b0;
            31:signed_storage=1'b0;
            32:signed_storage=1'b0;
            33:signed_storage=1'b0;
            34:signed_storage=1'b0;
            35:signed_storage=1'b0;
            36:signed_storage=1'b0;
            37:signed_storage=1'b0;
            38:signed_storage=1'b0;
            39:signed_storage=1'b0;
            default:signed_storage=0;
        endcase end
    endfunction
    genvar g;
    generate for(g=0;g<40;g=g+1) begin: export_config
        if(g==2 && EXT_POSITION>=36)begin: retired_theta_alias
            assign active[g]=default_value(g);assign pending[g]=default_value(g);
            assign config_o[g*32 +: 32]=active[g];
        end else if(g==2 && EXT_POSITION>=16) begin: decimal_alias
        // ID2 SET retains integer semantics. Fractional readback uses an explicit
        // out-of-range sentinel, NEVER a silently rounded integer. ID77 is exact.
        wire signed [10:0] aq=$signed(active[77][10:0])/11'sd10;
        wire signed [10:0] pq=$signed(pending[77][10:0])/11'sd10;
        wire signed [10:0] ar=$signed(active[77][10:0])%11'sd10;
        wire signed [10:0] pr=$signed(pending[77][10:0])%11'sd10;
        assign active[g]=(ar==0) ? {{21{aq[10]}},aq} : 32'sh80000000;
        assign pending[g]=(pr==0) ? {{21{pq[10]}},pq} : 32'sh80000000;
        assign config_o[g*32 +: 32]=active[g];
        end else begin: legacy_storage
        localparam integer WIDTH=storage_width(g);
        reg [WIDTH-1:0] active_bits, pending_bits;
        if(signed_storage(g)) begin: signed_field
            assign active[g]={{(32-WIDTH){active_bits[WIDTH-1]}},active_bits};
            assign pending[g]={{(32-WIDTH){pending_bits[WIDTH-1]}},pending_bits};
        end else begin: unsigned_field
            assign active[g]={{(32-WIDTH){1'b0}},active_bits};
            assign pending[g]={{(32-WIDTH){1'b0}},pending_bits};
        end
        assign config_o[g*32 +: 32]=active[g];
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                active_bits<=default_value(g); pending_bits<=default_value(g);
            end else begin
                if(apply_registers) active_bits<=pending_bits;
                if(restore_pending) pending_bits<=default_value(g);
                else if(write_pending && id_i==g) pending_bits<=value_i[WIDTH-1:0];
            end
        end
        end
    end endgenerate
    assign active[40]=0; assign pending[40]=0;
    assign speed_config_o={active[42],active[41]};
    generate if(EXT_POSITION>=5) begin: speed_parameters
        reg mode_active, mode_pending;
        reg [6:0] timeout_active, timeout_pending;
        assign active[41]={31'd0,mode_active}; assign pending[41]={31'd0,mode_pending};
        assign active[42]={25'd0,timeout_active}; assign pending[42]={25'd0,timeout_pending};
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin mode_active<=default_value(41);mode_pending<=default_value(41);timeout_active<=default_value(42);timeout_pending<=default_value(42);end
            else begin
                if(apply_registers) begin mode_active<=mode_pending;timeout_active<=timeout_pending;end
                if(restore_pending) begin mode_pending<=default_value(41);timeout_pending<=default_value(42);end
                else if(write_pending) begin
                    if(id_i==41) mode_pending<=value_i[0];
                    if(id_i==42) timeout_pending<=value_i[6:0];
                end
            end
        end
    end else begin: no_speed_parameters
        assign active[41]=0;assign pending[41]=0;assign active[42]=0;assign pending[42]=0;
    end endgenerate
    assign boost_config_o={active[45],active[44],active[43]};
    genvar b;
    generate for(b=43;b<=45;b=b+1) begin: boost_parameters
        if(EXT_POSITION>=6) begin: enabled
            localparam integer BW=(b==43) ? 1 : ((b==44) ? 8 : 4);
            reg [BW-1:0] av,pv;
            assign active[b]={{(32-BW){1'b0}},av};
            assign pending[b]={{(32-BW){1'b0}},pv};
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin av<=default_value(b);pv<=default_value(b);end
                else begin
                    if(apply_registers) av<=pv;
                    if(restore_pending) pv<=default_value(b);
                    else if(write_pending && id_i==b) pv<=value_i[BW-1:0];
                end
            end
        end else begin: disabled
            assign active[b]=0;assign pending[b]=0;
        end
    end endgenerate
    genvar c;
    generate for(c=46;c<=53;c=c+1) begin: settle_parameters
        assign settle_config_o[(c-46)*32 +: 32]=active[c];
        if(EXT_POSITION>=10)begin: retired
            assign active[c]=default_value(c);assign pending[c]=default_value(c);
        end else if(EXT_POSITION>=7) begin: enabled
            localparam integer CW=(c==46) ? 1 : ((c==47 || c==51) ? 9 :
                ((c==49) ? 13 : ((c==52) ? 14 : ((c==53) ? 18 : 17))));
            reg [CW-1:0] av,pv;
            if(c==47) begin: signed_gain
                assign active[c]={{(32-CW){av[CW-1]}},av};
                assign pending[c]={{(32-CW){pv[CW-1]}},pv};
            end else begin: unsigned_field
                assign active[c]={{(32-CW){1'b0}},av};
                assign pending[c]={{(32-CW){1'b0}},pv};
            end
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin av<=default_value(c);pv<=default_value(c);end
                else begin
                    if(apply_registers) av<=pv;
                    if(restore_pending) pv<=default_value(c);
                    else if(write_pending && id_i==c) pv<=value_i[CW-1:0];
                end
            end
        end else begin: disabled
            assign active[c]=0;assign pending[c]=0;
        end
    end endgenerate
    genvar ip;
    generate for(ip=54;ip<=56;ip=ip+1) begin: pid_parameters
        assign pid_config_o[(ip-54)*32 +: 32]=active[ip];
        if(EXT_POSITION>=35 && EXT_POSITION<37)begin: retired
            assign active[ip]=default_value(ip);assign pending[ip]=default_value(ip);
        end else if(EXT_POSITION>=8) begin: enabled
            localparam integer W=(ip==54)?9:((ip==55)?7:14);
            reg [W-1:0] av,pv;
            assign active[ip]={{(32-W){1'b0}},av};
            assign pending[ip]={{(32-W){1'b0}},pv};
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin av<=default_value(ip);pv<=default_value(ip);end
                else begin
                    if(apply_registers) av<=pv;
                    if(restore_pending) pv<=default_value(ip);
                    else if(write_pending && id_i==ip) pv<=value_i[W-1:0];
                end
            end
        end else begin: disabled
            assign active[ip]=0;assign pending[ip]=0;
        end
    end endgenerate
    genvar im;
    generate for(im=57;im<=63;im=im+1) begin: motion_parameters
        assign motion_config_o[(im-57)*32 +: 32]=active[im];
        if(EXT_POSITION>=10 && im>=58)begin: retired
            assign active[im]=default_value(im);assign pending[im]=default_value(im);
        end else if(EXT_POSITION>=9) begin: enabled
            localparam integer W=(im<=58)?1:((im==59 || im==61)?18:((im==60 || im==63)?16:13));
            reg [W-1:0] av,pv;
            if(im==60) begin: signed_gain
                assign active[im]={{(32-W){av[W-1]}},av};assign pending[im]={{(32-W){pv[W-1]}},pv};
            end else begin: unsigned_value
                assign active[im]={{(32-W){1'b0}},av};assign pending[im]={{(32-W){1'b0}},pv};
            end
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n)begin av<=default_value(im);pv<=default_value(im);end
                else begin
                    if(apply_registers)av<=pv;
                    if(restore_pending)pv<=default_value(im);
                    else if(write_pending && id_i==im)pv<=value_i[W-1:0];
                end
            end
        end else begin: disabled
            assign active[im]=0;assign pending[im]=0;
        end
    end endgenerate
    genvar ix;
    generate for(ix=64;ix<=69;ix=ix+1)begin: position_i_parameters
        assign position_i_config_o[(ix-64)*32 +: 32]=active[ix];
        if(EXT_POSITION>=35)begin: retired
            assign active[ix]=default_value(ix);assign pending[ix]=default_value(ix);
        end else if(EXT_POSITION>=10)begin: enabled
            localparam integer W=(ix==64)?2:((ix==65)?8:((ix==66)?6:((ix==67)?13:18)));
            reg [W-1:0] av,pv;
            if(ix==65)begin: signed_gain
                assign active[ix]={{(32-W){av[W-1]}},av};assign pending[ix]={{(32-W){pv[W-1]}},pv};
            end else begin: unsigned_field
                assign active[ix]={{(32-W){1'b0}},av};assign pending[ix]={{(32-W){1'b0}},pv};
            end
            always @(posedge clk or negedge rst_n)begin
                if(!rst_n)begin av<=default_value(ix);pv<=default_value(ix);end
                else begin
                    if(apply_registers)av<=pv;
                    if(restore_pending)pv<=default_value(ix);
                    else if(write_pending && id_i==ix)pv<=value_i[W-1:0];
                end
            end
        end else begin: disabled
            assign active[ix]=0;assign pending[ix]=0;
        end
    end endgenerate
    genvar rc;
    generate for(rc=70;rc<=72;rc=rc+1)begin: return_parameters
        assign return_config_o[(rc-70)*32 +: 32]=active[rc];
        if(EXT_POSITION>=13)begin: enabled
            localparam integer W=(rc==70)?1:((rc==71)?19:8);
            reg [W-1:0] av,pv;
            assign active[rc]={{(32-W){1'b0}},av};assign pending[rc]={{(32-W){1'b0}},pv};
            always @(posedge clk or negedge rst_n)begin
                if(!rst_n)begin av<=default_value(rc);pv<=default_value(rc);end
                else begin
                    if(apply_registers)av<=pv;
                    if(restore_pending)pv<=default_value(rc);
                    else if(write_pending && id_i==rc)pv<=value_i[W-1:0];
                end
            end
        end else begin: disabled
            assign active[rc]=0;assign pending[rc]=0;
        end
    end endgenerate
    assign speed_filter_mode_o=active[73][1:0];
    generate if(EXT_POSITION>=14)begin: speed_filter_parameter
        reg [1:0] av,pv;
        assign active[73]={30'd0,av};assign pending[73]={30'd0,pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=0;pv<=0;end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=0;
                else if(write_pending && id_i==73)pv<=value_i[1:0];
            end
        end
    end else begin: no_speed_filter_parameter
        assign active[73]=0;assign pending[73]=0;
    end endgenerate
    genvar nc;
    generate for(nc=74;nc<=76;nc=nc+1)begin: near_parameters
        assign near_config_o[(nc-74)*32 +: 32]=active[nc];
        if(EXT_POSITION>=36)begin: retired
            assign active[nc]=default_value(nc);assign pending[nc]=default_value(nc);
        end else if(EXT_POSITION>=15)begin: enabled
            localparam integer W=(nc==74)?1:((nc==75)?16:7);
            reg [W-1:0] av,pv;
            assign active[nc]={{(32-W){1'b0}},av};assign pending[nc]={{(32-W){1'b0}},pv};
            always @(posedge clk or negedge rst_n)begin
                if(!rst_n)begin av<=default_value(nc);pv<=default_value(nc);end
                else begin
                    if(apply_registers)av<=pv;
                    if(restore_pending)pv<=default_value(nc);
                    else if(write_pending && id_i==nc)pv<=value_i[W-1:0];
                end
            end
        end else begin: disabled
            assign active[nc]=0;assign pending[nc]=0;
        end
    end endgenerate
    // Profile17: one canonical active/pending gain. Either alias writes the
    // same pending register; last accepted SET wins; STOP-only APPLY is atomic.
    generate if(EXT_POSITION>=36)begin: fixed_decimal_theta
        assign active[77]=default_value(77);assign pending[77]=default_value(77);assign k_theta_x10_o=11'sd80;
    end else if(EXT_POSITION>=16) begin: decimal_theta
        reg signed [10:0] av,pv;
        wire signed [14:0] legacy_scaled=$signed(value_i[7:0])*15'sd10;
        assign active[77]={{21{av[10]}},av};
        assign pending[77]={{21{pv[10]}},pv};
        assign k_theta_x10_o=av;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin av<=default_value(77);pv<=default_value(77);end
            else begin
                if(apply_registers) av<=pv;
                if(restore_pending) pv<=default_value(77);
                else if(write_pending && id_i==2) pv<=legacy_scaled[10:0];
                else if(write_pending && id_i==77) pv<=value_i[10:0];
            end
        end
    end else begin: no_decimal_theta
        assign active[77]=0;assign pending[77]=0;assign k_theta_x10_o=0;
    end endgenerate
    // R14F: independent bounded position-P return cap, U (1U = 0.1%).
    // ID78 shares STOP-only APPLY with every existing parameter.
    generate if(EXT_POSITION>=36)begin: fixed_legacy_return_cap
        assign active[78]=default_value(78);assign pending[78]=default_value(78);assign return_position_cap_o=7'd60;
    end else if(EXT_POSITION>=17) begin: independent_return_cap
        reg [6:0] av,pv;
        assign active[78]={25'd0,av};assign pending[78]={25'd0,pv};
        assign return_position_cap_o=av;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin av<=default_value(78);pv<=default_value(78);end
            else begin
                if(apply_registers) av<=pv;
                if(restore_pending) pv<=default_value(78);
                else if(write_pending && id_i==78) pv<=value_i[6:0];
            end
        end
    end else begin: no_independent_return_cap
        assign active[78]=0;assign pending[78]=0;assign return_position_cap_o=0;
    end endgenerate
    // R14G: ID79 START_MODE, 0 original automatic, 1 manual upright capture.
    generate if(EXT_POSITION>=18) begin: start_mode_parameter
        reg av,pv;
        assign active[79]={31'd0,av};assign pending[79]={31'd0,pv};
        assign manual_start_o=av;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin av<=default_value(79);pv<=default_value(79);end
            else begin
                if(apply_registers) av<=pv;
                if(restore_pending) pv<=default_value(79);
                else if(write_pending && id_i==79) pv<=value_i[0];
            end
        end
    end else begin: no_start_mode
        assign active[79]=0;assign pending[79]=0;assign manual_start_o=0;
    end endgenerate
    // Compact physical widths; R15 reuses IDs81..89 under firmware profile25.
    generate if(EXT_POSITION>=35 && EXT_POSITION<37)begin: fixed_mode_80
        assign active[80]=32'd1;assign pending[80]=32'd1;
    end else if(EXT_POSITION>=21)begin: parameter_80
        localparam integer W=(EXT_POSITION>=24 ? 1:1);
        reg [W-1:0] av,pv;
        assign active[80]={{(32-W){1'b0}},av};assign pending[80]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(80);pv<=default_value(80);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(80);
                else if(write_pending && id_i==80)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[80]=default_value(80);assign pending[80]=default_value(80);end endgenerate
    generate if(EXT_POSITION>=21)begin: parameter_81
        localparam integer W=(EXT_POSITION>=24 ? 15:8);
        reg [W-1:0] av,pv;
        assign active[81]=EXT_POSITION>=24 ? {{(32-W){av[W-1]}},av}:{{(32-W){1'b0}},av};
        assign pending[81]=EXT_POSITION>=24 ? {{(32-W){pv[W-1]}},pv}:{{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(81);pv<=default_value(81);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(81);
                else if(write_pending && id_i==81)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[81]=default_value(81);assign pending[81]=default_value(81);end endgenerate
    generate if(EXT_POSITION>=21)begin: parameter_82
        localparam integer W=(EXT_POSITION>=25 ? 8:EXT_POSITION>=24 ? 15:9);
        reg [W-1:0] av,pv;
        assign active[82]=EXT_POSITION==24 ? {{(32-W){av[W-1]}},av}:{{(32-W){1'b0}},av};
        assign pending[82]=EXT_POSITION==24 ? {{(32-W){pv[W-1]}},pv}:{{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(82);pv<=default_value(82);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(82);
                else if(write_pending && id_i==82)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[82]=default_value(82);assign pending[82]=default_value(82);end endgenerate
    generate if(EXT_POSITION>=21)begin: parameter_83
        localparam integer W=(EXT_POSITION>=24 ? 1:1);
        reg [W-1:0] av,pv;
        assign active[83]={{(32-W){1'b0}},av};assign pending[83]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(83);pv<=default_value(83);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(83);
                else if(write_pending && id_i==83)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[83]=default_value(83);assign pending[83]=default_value(83);end endgenerate
    generate if(EXT_POSITION>=22)begin: parameter_84
        localparam integer W=(EXT_POSITION>=24 ? 13:10);
        reg [W-1:0] av,pv;
        assign active[84]={{(32-W){1'b0}},av};assign pending[84]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(84);pv<=default_value(84);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(84);
                else if(write_pending && id_i==84)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[84]=default_value(84);assign pending[84]=default_value(84);end endgenerate
    generate if(EXT_POSITION>=22)begin: parameter_85
        localparam integer W=(EXT_POSITION>=24 ? 10:6);
        reg [W-1:0] av,pv;
        assign active[85]={{(32-W){1'b0}},av};assign pending[85]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(85);pv<=default_value(85);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(85);
                else if(write_pending && id_i==85)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[85]=default_value(85);assign pending[85]=default_value(85);end endgenerate
    generate if(EXT_POSITION>=23)begin: parameter_86
        localparam integer W=(EXT_POSITION>=24 ? 15:1);
        reg [W-1:0] av,pv;
        assign active[86]={{(32-W){1'b0}},av};assign pending[86]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(86);pv<=default_value(86);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(86);
                else if(write_pending && id_i==86)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[86]=default_value(86);assign pending[86]=default_value(86);end endgenerate
    generate if(EXT_POSITION>=23)begin: parameter_87
        localparam integer W=(EXT_POSITION>=24 ? 14:19);
        reg [W-1:0] av,pv;
        assign active[87]={{(32-W){1'b0}},av};assign pending[87]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(87);pv<=default_value(87);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(87);
                else if(write_pending && id_i==87)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[87]=default_value(87);assign pending[87]=default_value(87);end endgenerate
    generate if(EXT_POSITION>=23)begin: parameter_88
        localparam integer W=(EXT_POSITION>=24 ? 18:17);
        reg [W-1:0] av,pv;
        assign active[88]={{(32-W){1'b0}},av};assign pending[88]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(88);pv<=default_value(88);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(88);
                else if(write_pending && id_i==88)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[88]=default_value(88);assign pending[88]=default_value(88);end endgenerate
    generate if(EXT_POSITION>=23)begin: parameter_89
        localparam integer W=(EXT_POSITION>=25 ? 9:EXT_POSITION>=24 ? 13:10);
        reg [W-1:0] av,pv;
        assign active[89]={{(32-W){1'b0}},av};assign pending[89]={{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(89);pv<=default_value(89);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(89);
                else if(write_pending && id_i==89)pv<=value_i[W-1:0];
            end
        end
    end else begin assign active[89]=default_value(89);assign pending[89]=default_value(89);end endgenerate
    // R15E ID90: one active bit + one pending bit, same STOP-only atomic APPLY.
    generate if(EXT_POSITION>=28)begin: parameter_90
        reg av,pv;
        assign active[90]={31'd0,av};assign pending[90]={31'd0,pv};
        assign velocity_slew_bypass_o=av;
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin av<=default_value(90);pv<=default_value(90);end
            else begin
                if(apply_registers)av<=pv;
                if(restore_pending)pv<=default_value(90);
                else if(write_pending && id_i==90)pv<=value_i[0];
            end
        end
    end else begin
        assign active[90]=0;assign pending[90]=0;assign velocity_slew_bypass_o=0;
    end endgenerate
    genvar pi;
    generate for(pi=91;pi<=94;pi=pi+1)begin: point_parameters
        if(EXT_POSITION>=32)begin: enabled
            localparam integer W=pi==91 ? 1:pi==94 ? 8:17;
            reg [W-1:0] av,pv;
            assign active[pi]={{(32-W){1'b0}},av};assign pending[pi]={{(32-W){1'b0}},pv};
            always @(posedge clk or negedge rst_n)begin
                if(!rst_n)begin av<=default_value(pi);pv<=default_value(pi);end
                else begin
                    if(apply_registers)av<=pv;
                    if(restore_pending)pv<=default_value(pi);
                    else if(write_pending && id_i==pi)pv<=value_i[W-1:0];
                end
            end
        end else begin: disabled
            assign active[pi]=0;assign pending[pi]=0;
        end
    end endgenerate
    genvar mp;
    generate for(mp=95;mp<=101;mp=mp+1)begin: motion_pi_parameters
        if(EXT_POSITION>=33)begin: enabled
            localparam integer W=(mp==95 || mp==98) ? 1:mp==96 ? 18:mp==99 ? 4:mp==101 ? 16:11;
            reg [W-1:0] av,pv;
            assign active[mp]={{(32-W){1'b0}},av};assign pending[mp]={{(32-W){1'b0}},pv};
            always @(posedge clk or negedge rst_n)begin
                if(!rst_n)begin av<=default_value(mp);pv<=default_value(mp);end
                else begin
                    if(apply_registers)av<=pv;
                    if(restore_pending)pv<=default_value(mp);
                    else if(write_pending && id_i==mp)pv<=value_i[W-1:0];
                end
            end
        end else begin: disabled assign active[mp]=0;assign pending[mp]=0;end
    end endgenerate
    genvar cg;
    generate for(cg=108;cg<=109;cg=cg+1)begin: cruise_inner_gains
      reg signed [15:0] av,pv;
      assign active[cg]={{16{av[15]}},av};assign pending[cg]={{16{pv[15]}},pv};
      assign cruise_gains_o[(cg-108)*32 +: 32]=active[cg];
      always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin av<=default_value(cg);pv<=default_value(cg);end
        else begin
          if(apply_registers)av<=pv;
          if(restore_pending)pv<=default_value(cg);
          else if(write_pending && id_i==cg)pv<=value_i[15:0];
        end
      end
    end endgenerate
    genvar cp;
    generate for(cp=102;cp<=107;cp=cp+1)begin: dual_cruise_parameters
      if(EXT_POSITION>=37)begin: enabled
        localparam integer W=cp==102 ? 18:cp==103 ? 11:cp==104 ? 14:cp==105 ? 18:cp==106 ? 9:8;
        reg [W-1:0] av,pv;
        assign active[cp]=cp==102 ? {{(32-W){av[W-1]}},av}:{{(32-W){1'b0}},av};
        assign pending[cp]=cp==102 ? {{(32-W){pv[W-1]}},pv}:{{(32-W){1'b0}},pv};
        always @(posedge clk or negedge rst_n)begin
          if(!rst_n)begin av<=default_value(cp);pv<=default_value(cp);end
          else begin
            if(apply_registers)av<=pv;
            if(restore_pending)pv<=default_value(cp);
            else if(write_pending && id_i==cp)pv<=value_i[W-1:0];
          end
        end
      end else begin assign active[cp]=default_value(cp);assign pending[cp]=default_value(cp);end
      assign cruise_config_o[(cp-101)*32 +: 32]=active[cp];
    end endgenerate
    assign cruise_config_o[31:0]=EXT_POSITION>=37 && active[80]==0 ? 32'd1:32'd0;
    assign motion_pi_config_o=EXT_POSITION>=33 ? {2'd0,active[101][15:0],active[100][10:0],
        active[99][3:0],active[98][0],active[97][10:0],active[96][17:0],active[95][0]}:64'd0;
    assign point_config_o=EXT_POSITION>=32 ?
        {21'd0,active[94][7:0],active[93][16:0],active[92][16:0],active[91][0]}:64'd0;
    assign brake_config_o=EXT_POSITION>=24 ? 64'd0:
        {1'b0,active[89][9:0],active[88][16:0],active[87][18:0],active[86][0],active[85][5:0],active[84][9:0]};
    assign coordinate_config_o=EXT_POSITION>=24 ?
        {13'd0,active[89][12:0],active[88][17:0],active[87][13:0],active[86][14:0],active[85][9:0],active[84][12:0]}:96'd0;
    assign arm_loop_config_o={active[83],active[82],active[81],active[80]};
    wire signed [31:0] near_far_x10=(EXT_POSITION>=16) ? pending[77] : pending[2]*32'sd10;
    wire near_combination_ok=(EXT_POSITION>=24 && pending[80]==1) || (EXT_POSITION<15) || pending[74]==0 ||
        (pending[70]==1 && near_far_x10>0 && pending[76]*32'sd10<=near_far_x10);
    wire motion_combination_ok=(EXT_POSITION<9) || !(pending[46]!=0 && pending[58]!=0);
    wire settle_combination_ok=(EXT_POSITION<7) || (EXT_POSITION>=10) ||
        ((pending[52]>=pending[49]) && (pending[53]>=pending[50]) &&
        (((pending[3]>0) && (pending[47]>=0)) ||
         ((pending[3]<0) && (pending[47]<=0)) ||
         ((pending[3]==0) && (pending[47]==0))));
    wire coordinate_combination_ok=EXT_POSITION<24 || pending[80]==0 ||
        (pending[0]>0 && (EXT_POSITION>=25 || pending[89]<=pending[84]) && pending[87]<pending[34]);
    wire combination_ok=(EXT_POSITION<33 || pending[80]==0 || pending[98]==0 || pending[81]!=0) && coordinate_combination_ok && ((EXT_POSITION<21) || pending[80]==0 || (pending[41]==1 && pending[46]==0 && pending[58]==0)) && near_combination_ok && motion_combination_ok && settle_combination_ok && (pending[21]>pending[16]) &&
        (pending[24]>=pending[23]) && (pending[37]<pending[36]) &&
        (pending[38]<=pending[5]) && (pending[32]<=pending[33]) &&
        (pending[18]<=pending[34]) && (pending[27]>=pending[26]);
    reg response_pending, all_mode, deferred_stop, deferred_busy, apply_wait;
    reg trace_dump_mode,trace_read_wait;reg [2:0] trace_part;reg [31:0] trace_xor;
    localparam integer TRACE_LAST_PART=EXT_POSITION>=30 ? 6:4;
    wire trace_data_frame=EXT_POSITION>=29 && response_code>=8'h10 && response_code<=8'h10+TRACE_LAST_PART;
    reg [31:0] trace_a,trace_p;
    always @* begin
        trace_a=0;trace_p=0;
        if(response_code==8'h0C && response_id==8'hFD)begin
            trace_a=trace_capture_version_i;trace_p=32'd27000000;
        end else if(response_code==8'h0C && response_id==8'hFC)begin
            trace_a=trace_arm_ms_i;trace_p=trace_reference_ms_i;
        end else if(response_code==8'h0E)begin
            trace_a=trace_xor;trace_p={24'd0,trace_status_i[7:0]};
        end else if(trace_data_frame)begin
            case(trace_part)
                0:begin trace_a=trace_data_i[31:0];trace_p=trace_data_i[63:32];end
                1:begin trace_a=trace_data_i[95:64];trace_p=trace_data_i[127:96];end
                2:begin trace_a=trace_data_i[159:128];trace_p=trace_data_i[191:160];end
                3:begin trace_a=trace_data_i[223:192];trace_p=trace_data_i[255:224];end
                4:begin trace_a=trace_data_i[287:256];trace_p=EXT_POSITION>=30 ? trace_data_i[319:288]:32'd0;end
                5:begin trace_a=trace_data_i[351:320];trace_p=trace_data_i[383:352];end
                default:begin trace_a=trace_data_i[415:384];trace_p=0;end
            endcase
        end else begin trace_a=trace_status_i;trace_p=trace_reference_cycle_i;end
    end
    wire trace_reply=EXT_POSITION>=29 && ((response_code>=8'h0B && response_code<=8'h0F) || trace_data_frame);
    reg [7:0] response_code,response_id;
    reg [2:0] state;
    reg [6:0] byte_index;
    reg [7:0] frame_code,frame_id;
    reg [31:0] frame_active,frame_pending,frame_version;
    reg frame_run;
    reg [2:0] frame_phase;
    reg [3:0] frame_reason;
    reg [2:0] stopped_ticks;
    assign response_pending_o=response_pending || state!=0 || apply_wait;
    wire serial_busy=response_pending_o;
    function [7:0] hex_char;
        input [3:0] n;
        begin hex_char=n<10 ? 8'd48+n : 8'd65+n-10; end
    endfunction
    function [7:0] frame_char;
        input [6:0] index;
        reg [3:0] nibble;
        reg is_hex;
        reg [7:0] literal_char;
        begin
        nibble=0;is_hex=0;literal_char=8'd63;
        case(index)
            7'd0:literal_char=8'd64;
            7'd1:literal_char=8'd68;
            7'd2:literal_char=8'd51;
            7'd3:literal_char=8'd52;
            7'd4:literal_char=8'd32;
            7'd5:literal_char=8'd67;
            7'd6:literal_char=8'd61;
            7'd7:begin nibble=frame_code[7:4];is_hex=1'b1;end
            7'd8:begin nibble=frame_code[3:0];is_hex=1'b1;end
            7'd9:literal_char=8'd32;
            7'd10:literal_char=8'd73;
            7'd11:literal_char=8'd61;
            7'd12:begin nibble=frame_id[7:4];is_hex=1'b1;end
            7'd13:begin nibble=frame_id[3:0];is_hex=1'b1;end
            7'd14:literal_char=8'd32;
            7'd15:literal_char=8'd65;
            7'd16:literal_char=8'd61;
            7'd17:begin nibble=frame_active[31:28];is_hex=1'b1;end
            7'd18:begin nibble=frame_active[27:24];is_hex=1'b1;end
            7'd19:begin nibble=frame_active[23:20];is_hex=1'b1;end
            7'd20:begin nibble=frame_active[19:16];is_hex=1'b1;end
            7'd21:begin nibble=frame_active[15:12];is_hex=1'b1;end
            7'd22:begin nibble=frame_active[11:8];is_hex=1'b1;end
            7'd23:begin nibble=frame_active[7:4];is_hex=1'b1;end
            7'd24:begin nibble=frame_active[3:0];is_hex=1'b1;end
            7'd25:literal_char=8'd32;
            7'd26:literal_char=8'd80;
            7'd27:literal_char=8'd61;
            7'd28:begin nibble=frame_pending[31:28];is_hex=1'b1;end
            7'd29:begin nibble=frame_pending[27:24];is_hex=1'b1;end
            7'd30:begin nibble=frame_pending[23:20];is_hex=1'b1;end
            7'd31:begin nibble=frame_pending[19:16];is_hex=1'b1;end
            7'd32:begin nibble=frame_pending[15:12];is_hex=1'b1;end
            7'd33:begin nibble=frame_pending[11:8];is_hex=1'b1;end
            7'd34:begin nibble=frame_pending[7:4];is_hex=1'b1;end
            7'd35:begin nibble=frame_pending[3:0];is_hex=1'b1;end
            7'd36:literal_char=8'd32;
            7'd37:literal_char=8'd86;
            7'd38:literal_char=8'd61;
            7'd39:begin nibble=frame_version[31:28];is_hex=1'b1;end
            7'd40:begin nibble=frame_version[27:24];is_hex=1'b1;end
            7'd41:begin nibble=frame_version[23:20];is_hex=1'b1;end
            7'd42:begin nibble=frame_version[19:16];is_hex=1'b1;end
            7'd43:begin nibble=frame_version[15:12];is_hex=1'b1;end
            7'd44:begin nibble=frame_version[11:8];is_hex=1'b1;end
            7'd45:begin nibble=frame_version[7:4];is_hex=1'b1;end
            7'd46:begin nibble=frame_version[3:0];is_hex=1'b1;end
            7'd47:literal_char=8'd32;
            7'd48:literal_char=8'd82;
            7'd49:literal_char=8'd85;
            7'd50:literal_char=8'd78;
            7'd51:literal_char=8'd61;
            7'd52:literal_char=frame_run ? "1" : "0";
            7'd53:literal_char=8'd32;
            7'd54:literal_char=8'd80;
            7'd55:literal_char=8'd72;
            7'd56:literal_char=8'd61;
            7'd57:begin nibble={1'b0,frame_phase};is_hex=1'b1;end
            7'd58:literal_char=8'd32;
            7'd59:literal_char=8'd82;
            7'd60:literal_char=8'd83;
            7'd61:literal_char=8'd61;
            7'd62:begin nibble=frame_reason;is_hex=1'b1;end
            7'd63:literal_char=8'd13;
            7'd64:literal_char=8'd10;
            default:literal_char=8'd63;
        endcase
        // Select one nibble BEFORE converting it to ASCII. One shared encoder
        // serves every code/ID/value/version field; the UART frame is unchanged.
        frame_char=is_hex ? hex_char(nibble):literal_char;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            version_o<=0; run_o<=0; rearm_o<=0; applied_o<=0; position_set_o<=0; position_value_o<=0;
            response_pending<=0; all_mode<=0; response_code<=0; response_id<=0;
            deferred_stop<=0; deferred_busy<=0; apply_wait<=0; stopped_ticks<=0;
            trace_dump_mode<=0;trace_read_wait<=0;trace_part<=0;trace_xor<=0;
            state<=0; byte_index<=0; tx_start_o<=0; tx_data_o<=0;
            frame_code<=0; frame_id<=0; frame_active<=0; frame_pending<=0;
            frame_version<=0; frame_run<=0; frame_phase<=0; frame_reason<=0;
        end else begin
            tx_start_o<=0; rearm_o<=0; applied_o<=0; position_set_o<=0;
            if(run_o || !stopped_i) stopped_ticks<=0;
            else if(tick_i && stopped_ticks<3) stopped_ticks<=stopped_ticks+1'b1;
            if(apply_wait && tick_i) begin
                apply_wait<=0;
                if(!run_o && stopped_i && stopped_ticks>=2) begin
                    version_o<=version_o+1'b1; applied_o<=1; response_code<=8'h03;
                end else response_code<=8'h83;
                response_id<=0; response_pending<=1;
            end
            if(command_valid_i) begin
                if(command_i==4) begin
                    run_o<=0;
                    if(serial_busy) deferred_stop<=1;
                    else begin response_code<=8'h04; response_id<=0; response_pending<=1; all_mode<=0; end
                end else if(serial_busy) deferred_busy<=1;
                else begin
                    all_mode<=0; trace_dump_mode<=0;trace_read_wait<=0;response_pending<=1; response_id<=id_i;
                    case(command_i)
                        0:begin response_code<=0; response_id<=40; end
                        1:begin response_code<=1; response_id<=0; all_mode<=1; end
                        2:if(EXT_POSITION && id_i==40) begin
                              if(EXT_POSITION>=3 && EXT_POSITION<32 && value_i!=0) response_code<=8'h89;
                              else if((EXT_POSITION>=33 && value_i==32'sh80000000) ||
                                  (EXT_POSITION<33 && (value_i < -32'sd102944 || value_i > 32'sd102944)))
                                  response_code<=8'h82;
                              else if(!run_o || !ready_i || phase_i!=5 || !position_ready_i)
                                  response_code<=8'h88;
                              else begin position_value_o<=value_i[TARGET_BITS-1:0]; position_set_o<=1;
                                  response_code<=8'h09; end
                          end else if((EXT_POSITION>=5 && (id_i==41 || id_i==42)) || (EXT_POSITION>=6 && id_i>=43 && id_i<=45) || (EXT_POSITION>=7 && id_i>=46 && id_i<=53) || (EXT_POSITION>=8 && id_i>=54 && id_i<=56) || (EXT_POSITION>=9 && id_i>=57 && id_i<=63) || (EXT_POSITION>=10 && id_i>=64 && id_i<=69) || (EXT_POSITION>=13 && id_i>=70 && id_i<=72) || (EXT_POSITION>=14 && id_i==73) || (EXT_POSITION>=15 && id_i>=74 && id_i<=76) || (EXT_POSITION>=16 && id_i==77) || (EXT_POSITION>=17 && id_i==78) || (EXT_POSITION>=18 && id_i==79) || (EXT_POSITION>=21 && id_i>=80 && id_i<=83) || (EXT_POSITION>=22 && id_i>=84 && id_i<=85) || (EXT_POSITION>=23 && id_i>=86 && id_i<=89) || (EXT_POSITION>=28 && id_i==90) || (EXT_POSITION>=32 && id_i>=91 && id_i<=94) || (EXT_POSITION>=33 && id_i>=95 && id_i<=101) || (EXT_POSITION>=37 && id_i>=102 && id_i<=109)) begin
                              if(!value_in_range) response_code<=8'h82;else response_code<=2;
                          end else if(id_i>=40 && id_i<48) response_code<=8'h87;
                          else if(id_i>=40) response_code<=8'h81;
                          else if(!value_in_range) response_code<=8'h82;
                          else response_code<=2;
                        3:if(run_o || !stopped_i || stopped_ticks<2) response_code<=8'h83;
                          else if(!combination_ok) response_code<=8'h84;
                          else begin response_pending<=0; apply_wait<=1; end
                        5:if(run_o || !ready_i || !stopped_i || stopped_ticks<2) response_code<=8'h85;
                          else begin run_o<=1; rearm_o<=1; response_code<=5; end
                        6:begin
                            response_code<=6;
                        end
                        7:if(run_o || !stopped_i) response_code<=8'h83;
                          else begin rearm_o<=1; response_code<=7; end
                        8:begin response_code<=EXT_POSITION ? 8'h0A : 8'h87; response_id<=40; end
                        9:begin response_code<=EXT_POSITION>=29 ? 8'h0B:8'h80;response_id<=8'hFE;end
                        10:if(EXT_POSITION<29)response_code<=8'h80;
                           else if(run_o || !stopped_i || stopped_ticks<2)response_code<=8'h83;
                           else if(!trace_status_i[17] || trace_status_i[7:0]==0)response_code<=8'h8A;
                           else begin response_code<=8'h0C;response_id<=8'hFE;trace_dump_mode<=1;trace_part<=0;trace_xor<=0;end
                        11:begin response_code<=EXT_POSITION>=29 ? 8'h0C:8'h80;response_id<=8'hFE;end
                        12:begin response_code<=EXT_POSITION>=29 ? 8'h0F:8'h80;response_id<=8'hFE;end
                        13:begin response_code<=EXT_POSITION>=30 ? 8'h0B:8'h80;response_id<=8'hFE;end
                        default:response_code<=8'h80;
                    endcase
                end
            end
            case(state)
                0:if(response_pending && !tx_busy_i) begin
                    if(trace_data_frame && !trace_read_wait)trace_read_wait<=1;
                    else begin
                    if(trace_data_frame)trace_xor<=trace_xor^trace_a^trace_p;
                    frame_code<=response_code; frame_id<=response_id;
                    frame_active<=trace_reply ? trace_a:(response_code==0) ? ((EXT_POSITION>=3) ? EXT_POSITION+32'd1 : (EXT_POSITION ? 32'd2 : 32'd1)) :
                        ((response_code==9 || response_code==10) ? position_actual_i :
                        ((response_id<40 || (EXT_POSITION>=5 && (response_id==41 || response_id==42)) || (EXT_POSITION>=6 && response_id>=43 && response_id<=45) || (EXT_POSITION>=7 && response_id>=46 && response_id<=53) || (EXT_POSITION>=8 && response_id>=54 && response_id<=56) || (EXT_POSITION>=9 && response_id>=57 && response_id<=63) || (EXT_POSITION>=10 && response_id>=64 && response_id<=69) || (EXT_POSITION>=13 && response_id>=70 && response_id<=72) || (EXT_POSITION>=14 && response_id==73) || (EXT_POSITION>=15 && response_id>=74 && response_id<=76) || (EXT_POSITION>=16 && response_id==77) || (EXT_POSITION>=17 && response_id==78) || (EXT_POSITION>=18 && response_id==79) || (EXT_POSITION>=21 && response_id>=80 && response_id<=83) || (EXT_POSITION>=22 && response_id>=84 && response_id<=85) || (EXT_POSITION>=23 && response_id>=86 && response_id<=89) || (EXT_POSITION>=28 && response_id==90) || (EXT_POSITION>=32 && response_id>=91 && response_id<=94) || (EXT_POSITION>=33 && response_id>=95 && response_id<=101) || (EXT_POSITION>=37 && response_id>=102 && response_id<=109)) ? active[response_id] : 32'd0));
                    frame_pending<=trace_reply ? trace_p:(response_code==0) ? 32'd34 :
                        ((response_code==9) ? {{(32-TARGET_BITS){position_value_o[TARGET_BITS-1]}},position_value_o} :
                        ((response_code==10) ? {{(32-TARGET_BITS){position_target_i[TARGET_BITS-1]}},position_target_i} :
                        ((response_id<40 || (EXT_POSITION>=5 && (response_id==41 || response_id==42)) || (EXT_POSITION>=6 && response_id>=43 && response_id<=45) || (EXT_POSITION>=7 && response_id>=46 && response_id<=53) || (EXT_POSITION>=8 && response_id>=54 && response_id<=56) || (EXT_POSITION>=9 && response_id>=57 && response_id<=63) || (EXT_POSITION>=10 && response_id>=64 && response_id<=69) || (EXT_POSITION>=13 && response_id>=70 && response_id<=72) || (EXT_POSITION>=14 && response_id==73) || (EXT_POSITION>=15 && response_id>=74 && response_id<=76) || (EXT_POSITION>=16 && response_id==77) || (EXT_POSITION>=17 && response_id==78) || (EXT_POSITION>=18 && response_id==79) || (EXT_POSITION>=21 && response_id>=80 && response_id<=83) || (EXT_POSITION>=22 && response_id>=84 && response_id<=85) || (EXT_POSITION>=23 && response_id>=86 && response_id<=89) || (EXT_POSITION>=28 && response_id==90) || (EXT_POSITION>=32 && response_id>=91 && response_id<=94) || (EXT_POSITION>=33 && response_id>=95 && response_id<=101) || (EXT_POSITION>=37 && response_id>=102 && response_id<=109)) ? pending[response_id] : 32'd0)));
                    frame_version<=version_o; frame_run<=run_o;
                    frame_phase<=phase_i; frame_reason<=reason_i;
                    byte_index<=0; state<=1;
                    end
                end
                1:if(!tx_busy_i) begin tx_data_o<=frame_char(byte_index); state<=2; end
                2:begin tx_start_o<=1; state<=3; end
                3:if(tx_busy_i) state<=4;
                4:if(!tx_busy_i) begin
                    if(byte_index==64) begin
                        state<=0;
                        trace_read_wait<=0;
                        if(trace_dump_mode && (deferred_stop || (command_valid_i && command_i==4)))begin
                            trace_dump_mode<=0;deferred_stop<=0;response_code<=8'h04;response_id<=0;
                        end else if(trace_dump_mode)begin
                            if(response_code==8'h0C && response_id==8'hFE)response_id<=8'hFD;
                            else if(response_code==8'h0C && response_id==8'hFD)response_id<=8'hFC;
                            else if(response_code==8'h0C)begin response_code<=8'h10;response_id<=0;trace_part<=0;end
                            else if(trace_part<TRACE_LAST_PART)begin trace_part<=trace_part+1'b1;response_code<=response_code+1'b1;end
                            else if({1'b0,response_id}+1<trace_status_i[7:0])begin
                                response_id<=response_id+1'b1;trace_part<=0;response_code<=8'h10;
                            end else begin trace_dump_mode<=0;response_code<=8'h0E;response_id<=8'hFF;end
                        end else if(all_mode && response_id<39) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=5 && response_id==39) response_id<=41;
                        else if(all_mode && EXT_POSITION>=5 && response_id==41) response_id<=42;
                        else if(all_mode && EXT_POSITION>=6 && response_id>=42 && response_id<45) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=37 && response_id>=101 && response_id<109) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=33 && response_id>=94 && response_id<101) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=32 && response_id>=90 && response_id<94) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=28 && response_id==89) response_id<=90;
                        else if(all_mode && EXT_POSITION>=23 && response_id>=85 && response_id<89) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=22 && response_id>=83 && response_id<85) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=21 && response_id>=79 && response_id<83) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=18 && response_id==78) response_id<=79;
                        else if(all_mode && EXT_POSITION>=17 && response_id==77) response_id<=78;
                        else if(all_mode && EXT_POSITION>=16 && response_id==76) response_id<=77;
                        else if(all_mode && EXT_POSITION>=15 && response_id>=73 && response_id<76) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=14 && response_id==72) response_id<=73;
                        else if(all_mode && EXT_POSITION>=13 && response_id>=69 && response_id<72) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=10 && response_id>=63 && response_id<69) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=9 && response_id>=56 && response_id<63) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=8 && response_id>=53 && response_id<56) response_id<=response_id+1'b1;
                        else if(all_mode && EXT_POSITION>=7 && response_id>=45 && response_id<53) response_id<=response_id+1'b1;
                        else if(all_mode) begin all_mode<=0; response_code<=8'h08; response_id<=8'hFF; end
                        else if(deferred_stop || (command_valid_i && command_i==4)) begin
                            deferred_stop<=0; response_code<=8'h04; response_id<=0;
                        end else if(deferred_busy || (command_valid_i && command_i!=4)) begin
                            deferred_busy<=0; response_code<=8'h86; response_id<=0;
                        end else response_pending<=0;
                    end else begin byte_index<=byte_index+1'b1; state<=1; end
                end
                default:state<=0;
            endcase
        end
    end
endmodule
`default_nettype wire
