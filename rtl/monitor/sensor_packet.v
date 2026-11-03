`timescale 1ns/1ps
`default_nettype none
// Reduced read-only telemetry: retain control/status/position/motion data.
// Omit legacy tuning probes PA/PD/PT/PV/PI, ADC internals, obsolete settle
// snapshots, duplicate raw speed and low-level arm-output probes.
// Same D34 key/value protocol, baud rate, UART handshake and frame arbitration.
module sensor_packet_lite(
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
    reg [31:0] snapshot[0:24];
    reg busy;
    reg [4:0] field_index;
    reg [2:0] section;
    reg [3:0] char_index;
    reg [1:0] tx_state;
    reg [31:0] hex_shift;
    reg [23:0] key;
    reg [1:0] key_length;
    reg signed_field;
    wire [31:0] selected_word=snapshot[field_index];
    wire negative=signed_field && selected_word[31];
    reg [7:0] next_byte;
    assign frame_busy_o=busy;
    always @* begin
      key=0;key_length=1;signed_field=0;
      case(field_index)
        5'd0:begin key=24'h540000;key_length=1;signed_field=0;end
        5'd1:begin key=24'h4e0000;key_length=1;signed_field=0;end
        5'd2:begin key=24'h544800;key_length=2;signed_field=1;end
        5'd3:begin key=24'h544400;key_length=2;signed_field=1;end
        5'd4:begin key=24'h414c00;key_length=2;signed_field=1;end
        5'd5:begin key=24'h414400;key_length=2;signed_field=1;end
        5'd6:begin key=24'h550000;key_length=1;signed_field=1;end
        5'd7:begin key=24'h580000;key_length=1;signed_field=0;end
        5'd8:begin key=24'h460000;key_length=1;signed_field=0;end
        5'd9:begin key=24'h510000;key_length=1;signed_field=0;end
        5'd10:begin key=24'h52554e;key_length=3;signed_field=0;end
        5'd11:begin key=24'h430000;key_length=1;signed_field=0;end
        5'd12:begin key=24'h484300;key_length=2;signed_field=0;end
        5'd13:begin key=24'h565200;key_length=2;signed_field=1;end
        5'd14:begin key=24'h415200;key_length=2;signed_field=1;end
        5'd15:begin key=24'h435400;key_length=2;signed_field=1;end
        5'd16:begin key=24'h4e5600;key_length=2;signed_field=1;end
        5'd17:begin key=24'h585000;key_length=2;signed_field=1;end
        5'd18:begin key=24'h534400;key_length=2;signed_field=0;end
        5'd19:begin key=24'h544500;key_length=2;signed_field=1;end
        5'd20:begin key=24'h534d00;key_length=2;signed_field=0;end
        5'd21:begin key=24'h535600;key_length=2;signed_field=0;end
        5'd22:begin key=24'h525000;key_length=2;signed_field=0;end
        5'd23:begin key=24'h525600;key_length=2;signed_field=0;end
        5'd24:begin key=24'h414900;key_length=2;signed_field=0;end
        default:begin end
      endcase
      next_byte=8'd32;
      case(section)
        0:case(char_index)0:next_byte="D";1:next_byte="3";2:next_byte="4";default:next_byte=" ";endcase
        1:case(char_index)0:next_byte=key[23:16];1:next_byte=key[15:8];default:next_byte=key[7:0];endcase
        2:next_byte="=";
        3:next_byte=negative ? "-":"+";
        4:next_byte=hex_shift[31:28]<10 ? 8'd48+{4'd0,hex_shift[31:28]}:8'd55+{4'd0,hex_shift[31:28]};
        5:next_byte=" ";
        6:next_byte=8'd13;
        7:next_byte=8'd10;
        default:next_byte=8'd32;
      endcase
    end
    // Snapshot registers need no reset: the complete frame is captured
    // before busy is asserted, and UART emits nothing while busy is clear.
    always @(posedge clk)begin
      if(rst_n && !busy && send_i)begin
        snapshot[0]<=ms_i;
        snapshot[1]<={16'd0,seq_i};
        snapshot[2]<=theta;
        snapshot[3]<=theta_dot;
        snapshot[4]<=alpha;
        snapshot[5]<=alpha_dot;
        snapshot[6]<={{16{control_u[15]}},control_u};
        snapshot[7]<=exec_i;
        snapshot[8]<=flags_i;
        snapshot[9]<=version_i;
        snapshot[10]<={31'd0,run_i};
        snapshot[11]<={31'd0,cal_i};
        snapshot[12]<=hold_control_i;
        snapshot[13]<=velocity_ref_i;
        snapshot[14]<=angle_ref_i;
        snapshot[15]<=capture_travel_i;
        snapshot[16]<=loop_reference_i;
        snapshot[17]<=hold_position_i;
        snapshot[18]<=speed_status_i[31:0];
        snapshot[19]<=est_speed_i;
        snapshot[20]<={31'd0,speed_source_i};
        snapshot[21]<={31'd0,est_valid_i};
        snapshot[22]<=motion_position_i;
        snapshot[23]<=motion_velocity_i;
        snapshot[24]<=outer_i_i;
      end
    end
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin
        busy<=0;field_index<=0;section<=0;char_index<=0;
        tx_state<=0;uart_data_o<=0;uart_start_o<=0;hex_shift<=0;
      end else begin
        uart_start_o<=0;
        if(!busy)begin
          if(send_i)begin busy<=1;field_index<=0;section<=0;char_index<=0;tx_state<=0;end
        end else case(tx_state)
          0:if(!uart_busy_i)begin uart_data_o<=next_byte;tx_state<=1;end
          1:begin uart_start_o<=1;tx_state<=2;end
          2:if(uart_busy_i)tx_state<=3;
          3:if(!uart_busy_i)begin
            tx_state<=0;
            case(section)
              0:if(char_index==3)begin section<=1;char_index<=0;end else char_index<=char_index+1'b1;
              1:if(char_index+1>=key_length)begin section<=2;char_index<=0;end else char_index<=char_index+1'b1;
              2:section<=3;
              3:begin section<=4;char_index<=0;hex_shift<=negative ? -selected_word:selected_word;end
              4:begin
                hex_shift<={hex_shift[27:0],4'd0};
                if(char_index==7)begin section<=field_index==24 ? 6:5;char_index<=0;end
                else char_index<=char_index+1'b1;
              end
              5:begin field_index<=field_index+1'b1;section<=1;char_index<=0;end
              6:section<=7;
              7:busy<=0;
              default:busy<=0;
            endcase
          end
        endcase
      end
    end
endmodule
`default_nettype wire
