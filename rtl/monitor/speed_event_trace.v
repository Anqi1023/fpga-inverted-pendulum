`timescale 1ns/1ps
`default_nettype none
// R15G passive recorder: 13 words/record, 64 history + 64 post-trigger records.
// Sensor/reference and controller snapshots have separate records. Two events
// on the same clock use the two RAM write ports and retain the same timestamp.
module speed_event_trace #(
    parameter integer POST_CYCLES=1350000,
    parameter integer JUMP_U=200
)(
    input wire clk,rst_n,arm_i,reference_mode_i,freeze_i,stop_i,active_i,
    input wire [31:0] time_ms_i,config_version_i,
    input wire signed [31:0] count_i,est_speed_i,raw_speed_i,
    input wire [3:0] ab_i,
    input wire illegal_i,speed_valid_i,reverse_i,
    input wire [36:0] speed_status_i,
    input wire [16:0] period_i,
    input wire [4:0] estimator_flags_i,
    input wire signed [31:0] position_i,alpha_i,alpha_dot_i,
    input wire signed [13:0] reference_i,target_i,
    input wire [3:0] reference_reason_i,
    input wire [12:0] coordinate_flags_i,
    input wire [14:0] hold_band_i,
    input wire control_i,
    input wire [15:0] control_seq_i,
    input wire signed [31:0] sampled_est_i,sampled_raw_i,velocity_u_q16_i,p_u_i,d_u_i,
    input wire sampled_valid_i,
    input wire [63:0] sampled_status_i,
    input wire signed [31:0] sampled_position_i,sampled_alpha_i,sampled_alpha_dot_i,
    input wire signed [13:0] sampled_reference_i,sampled_target_i,
    input wire [3:0] sampled_reference_reason_i,
    input wire [12:0] sampled_coordinate_flags_i,
    input wire signed [15:0] control_u_i,
    input wire [6:0] read_index_i,
    output wire [415:0] read_data_o,
    output wire [31:0] status_o,reference_cycle_o,
    output reg [31:0] capture_version_o,arm_ms_o,reference_ms_o
);
    reg [31:0] cycles,reference_cycle;
    reg armed,triggered,frozen,full,early_stop,manual_freeze,post_timeout,config_changed,reference_mode;
    reg [6:0] write_pointer,base_pointer,pre_count,post_count;
    reg [7:0] total_count;
    reg signed [31:0] last_count,last_est,last_raw,last_alpha,last_alpha_dot;
    reg signed [13:0] last_reference,last_target;
    reg [3:0] last_reference_reason;
    reg [12:0] last_coordinate_flags;
    reg last_valid;
    reg [2:0] last_reason;
    reg previous_control_valid;
    reg signed [31:0] previous_velocity;
    reg [15:0] previous_sequence;
    wire edge_event=count_i!=last_count;
    wire speed_event=est_speed_i!=last_est || raw_speed_i!=last_raw ||
        speed_valid_i!=last_valid || speed_status_i[19:17]!=last_reason;
    wire reference_event=reference_i!=last_reference || target_i!=last_target ||
        reference_reason_i!=last_reference_reason || coordinate_flags_i!=last_coordinate_flags;
    wire angle_event=alpha_i!=last_alpha || alpha_dot_i!=last_alpha_dot;
    wire [31:0] position_magnitude=position_i[31] ? -position_i:position_i;
    wire reference_withdraw=last_reference!=0 && reference_i==0 &&
        coordinate_flags_i[1] && coordinate_flags_i[7] && coordinate_flags_i[0] &&
        position_magnitude>{17'd0,hold_band_i};
    wire sensor_event=edge_event || speed_event || reference_event || angle_event || illegal_i || reverse_i;
    wire sampled_mode_active=sampled_coordinate_flags_i[1] && sampled_coordinate_flags_i[7];
    wire signed [32:0] velocity_delta=$signed({velocity_u_q16_i[31],velocity_u_q16_i})-
        $signed({previous_velocity[31],previous_velocity});
    wire [32:0] velocity_delta_abs=velocity_delta[32] ? -velocity_delta:velocity_delta;
    wire velocity_jump=control_i && sampled_mode_active && previous_control_valid &&
        control_seq_i==previous_sequence+16'd1 && velocity_delta_abs>=(JUMP_U*33'd65536);
    wire trigger_control=!triggered && !reference_mode && velocity_jump;
    wire trigger_sensor=!triggered && reference_mode && reference_withdraw;
    wire trigger_now=trigger_control || trigger_sensor;
    wire config_mismatch=config_version_i!=capture_version_o;
    wire capture_enable=armed && active_i && !arm_i && !freeze_i && !stop_i &&
        !config_mismatch && !(triggered && cycles-reference_cycle>=POST_CYCLES);
    wire two_requested=sensor_event && control_i;
    wire write_first=capture_enable && (sensor_event || control_i) && (!triggered || post_count<64);
    wire write_second=write_first && two_requested && (!triggered || post_count<63);
    wire [1:0] writes=write_second ? 2'd2:write_first ? 2'd1:2'd0;
    wire [7:0] pre_with_sensor={1'b0,pre_count}+{7'd0,sensor_event};
    wire [6:0] velocity_pre_count=pre_with_sensor>64 ? 7'd64:pre_with_sensor[6:0];
    wire [7:0] pre_next={1'b0,pre_count}+{6'd0,writes};
    wire [7:0] post_next={1'b0,post_count}+{6'd0,writes};
    wire [6:0] read_address=base_pointer+read_index_i;
    wire [6:0] port_a_address=write_first ? write_pointer:read_address;
    wire [6:0] second_address=write_pointer+7'd1;
    wire [15:0] sensor_detail={estimator_flags_i[4],trigger_sensor,1'b0,speed_event,
        edge_event,illegal_i,estimator_flags_i[3],estimator_flags_i[1],
        speed_valid_i,speed_status_i[19:17],ab_i};
    wire [15:0] control_detail={estimator_flags_i[4],trigger_control,1'b1,3'd0,
        estimator_flags_i[3],estimator_flags_i[1],sampled_valid_i,sampled_status_i[19:17],ab_i};
    wire [15:0] sensor_coordinate={reference_event,reference_withdraw,reverse_i,coordinate_flags_i};
    wire [15:0] control_coordinate={3'd0,sampled_coordinate_flags_i};
    wire [15:0] next_sequence=control_seq_i+16'd1;
    wire [415:0] sensor_record={
        reference_reason_i,target_i,reference_i,
        alpha_dot_i,alpha_i,position_i,{sensor_coordinate,control_seq_i},
        {15'd0,speed_status_i[36:20]},{15'd0,speed_status_i[16:0]},
        {10'd0,estimator_flags_i,period_i},raw_speed_i,est_speed_i,
        control_u_i,sensor_detail,count_i,cycles};
    wire [415:0] control_record={
        sampled_reference_reason_i,sampled_target_i,sampled_reference_i,
        sampled_alpha_dot_i,sampled_alpha_i,sampled_position_i,{control_coordinate,next_sequence},
        d_u_i,p_u_i,velocity_u_q16_i,sampled_raw_i,sampled_est_i,
        control_u_i,control_detail,count_i,cycles};
    wire [415:0] first_record=sensor_event ? sensor_record:control_record;
    genvar bank,lane;
    generate for(bank=0;bank<13;bank=bank+1)begin: storage
        // GW2A-18C DPB supports 16-bit dual ports, with four low address
        // bits outside the word address. Low two bits enable both byte lanes
        // on devices that implement byte enables. Two DPBs form one bank.
        // Explicit normal modes avoid PA2122 and any inference of mode 10.
        for(lane=0;lane<2;lane=lane+1)begin: lanes
            DPB #(
                .READ_MODE0(1'b0),.READ_MODE1(1'b0),
                .WRITE_MODE0(2'b00),.WRITE_MODE1(2'b00),
                .BIT_WIDTH_0(16),.BIT_WIDTH_1(16),
                .BLK_SEL_0(3'b000),.BLK_SEL_1(3'b000),.RESET_MODE("SYNC")
            ) ram (
                .DOA(read_data_o[bank*32+lane*16 +: 16]),.DOB(),
                .CLKA(clk),.CEA(1'b1),.OCEA(1'b1),.RESETA(1'b0),.WREA(write_first),
                .ADA({3'b000,port_a_address,4'b0011}),.BLKSELA(3'b000),
                .DIA(first_record[bank*32+lane*16 +: 16]),
                .CLKB(clk),.CEB(write_second),.OCEB(1'b1),.RESETB(1'b0),.WREB(write_second),
                .ADB({3'b000,second_address,4'b0011}),.BLKSELB(3'b000),
                .DIB(control_record[bank*32+lane*16 +: 16])
            );
        end
    end endgenerate
    assign reference_cycle_o=reference_cycle;
    // mode[28], schema2[27:24], flags[23:15], pre_count[14:8], count[7:0].
    assign status_o={3'd0,reference_mode,4'd2,config_changed,post_timeout,manual_freeze,
        early_stop,1'b0,full,frozen,triggered,armed,pre_count,total_count};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            cycles<=0;reference_cycle<=0;armed<=0;triggered<=0;frozen<=0;full<=0;
            early_stop<=0;manual_freeze<=0;post_timeout<=0;config_changed<=0;reference_mode<=0;
            write_pointer<=0;base_pointer<=0;pre_count<=0;post_count<=0;total_count<=0;
            capture_version_o<=0;arm_ms_o<=0;reference_ms_o<=0;
            last_count<=0;last_est<=0;last_raw<=0;last_valid<=0;last_reason<=0;
            last_reference<=0;last_target<=0;last_reference_reason<=0;last_coordinate_flags<=0;
            last_alpha<=0;last_alpha_dot<=0;previous_control_valid<=0;previous_velocity<=0;previous_sequence<=0;
        end else begin
            cycles<=cycles+1'b1;
            last_count<=count_i;last_est<=est_speed_i;last_raw<=raw_speed_i;
            last_valid<=speed_valid_i;last_reason<=speed_status_i[19:17];
            last_reference<=reference_i;last_target<=target_i;
            last_reference_reason<=reference_reason_i;last_coordinate_flags<=coordinate_flags_i;
            last_alpha<=alpha_i;last_alpha_dot<=alpha_dot_i;
            if(!armed || !active_i)previous_control_valid<=0;
            else if(control_i)begin
                previous_control_valid<=sampled_mode_active;
                previous_velocity<=velocity_u_q16_i;previous_sequence<=control_seq_i;
            end
            if(arm_i)begin
                armed<=1;triggered<=0;frozen<=0;full<=0;reference_mode<=reference_mode_i;
                early_stop<=0;manual_freeze<=0;post_timeout<=0;config_changed<=0;
                write_pointer<=0;base_pointer<=0;pre_count<=0;post_count<=0;total_count<=0;
                reference_cycle<=cycles;reference_ms_o<=time_ms_i;arm_ms_o<=time_ms_i;
                capture_version_o<=config_version_i;previous_control_valid<=0;
            end else if(armed)begin
                if(freeze_i || stop_i || config_mismatch)begin
                    armed<=0;frozen<=1;manual_freeze<=freeze_i;early_stop<=stop_i;config_changed<=config_mismatch;
                    if(!triggered)begin
                        base_pointer<=write_pointer-pre_count;total_count<={1'b0,pre_count};
                        // A long untriggered wait may span cycle-counter wraps.
                        // Re-anchor the retained recent history at freeze, not ARM.
                        reference_cycle<=cycles;reference_ms_o<=time_ms_i;
                    end
                end else if(triggered && cycles-reference_cycle>=POST_CYCLES)begin
                    armed<=0;frozen<=1;post_timeout<=1;
                end else if(write_first)begin
                    write_pointer<=write_pointer+{5'd0,writes};
                    if(!triggered)begin
                        if(trigger_now)begin
                            triggered<=1;reference_cycle<=cycles;reference_ms_o<=time_ms_i;
                            if(trigger_control)begin
                                pre_count<=velocity_pre_count;
                                base_pointer<=write_pointer+{6'd0,sensor_event}-velocity_pre_count;
                                total_count<={1'b0,velocity_pre_count}+8'd1;post_count<=1;
                            end else begin
                                base_pointer<=write_pointer-pre_count;
                                total_count<={1'b0,pre_count}+{6'd0,writes};post_count<={5'd0,writes};
                            end
                        end else begin
                            pre_count<=pre_next>64 ? 7'd64:pre_next[6:0];
                            total_count<=pre_next>64 ? 8'd64:pre_next;
                        end
                    end else begin
                        total_count<=total_count+{6'd0,writes};post_count<=post_next[6:0];
                        if(post_next>=64)begin armed<=0;frozen<=1;full<=1;end
                    end
                end
            end
        end
    end
    // synthesis translate_off
    initial if(JUMP_U<1 || JUMP_U>1000)$fatal(1,"Invalid diagnostic jump threshold");
    always @(posedge clk)if(rst_n && write_second && write_pointer==second_address)
        $fatal(1,"Diagnostic dual-port write collision");
    // synthesis translate_on
endmodule
`default_nettype wire
