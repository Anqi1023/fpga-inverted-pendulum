`timescale 1ns/1ps
module tb_packet;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,send=0,busy=0;integer count=0;
 wire [7:0] data;wire start;
 sensor_packet #(.LIVE_BRAKE_CONFIG(1),.BRAKE_DIAGNOSTICS(0),.LOOP_DIAGNOSTICS(1),.COMPACT_DIAGNOSTICS(1),
 .ARM_DIAGNOSTICS(1),.AGE_DIAGNOSTICS(1),.SPEED_DIAGNOSTICS(1),.SETTLE_DIAGNOSTICS(1),.PID_DIAGNOSTICS(1),.HOLD_DIAGNOSTICS(1)) dut(
 .brake_gain_i(10'd0),.brake_accel_i(19'd0),.approach_speed_i(17'd0),.release_rate_i(10'd0),
 .brake_distance_i(32'd0),.velocity_error_i(33'sd0),.brake_flags_i(4'd0),.brake_status_i(8'd0),
 .velocity_ref_i(32'sd2048),.angle_ref_i(32'sd1024),.hold_control_i(32'h7e0003d7),
 .pid_integral_i(32'sd0),.settle_status_i(32'd0),.boost_settings_i(32'd0),
 .capture_travel_i(32'sd7000),.lock_origin_i(32'sd0),.hold_position_i(32'sd7000),
 .raw_speed_i(32'sd0),.est_speed_i(32'sd0),.speed_source_i(1'b1),.est_valid_i(1'b0),
 .speed_status_i(64'd655360),.original_arm_i(32'sd2355200),.arm_request_i(32'sd2355200),
 .loop_reference_i(32'sd0),.loop_state_i(4'hb),.clk(clk),.rst_n(rst),.send_i(send),
 .version_i(32'd1),.run_i(1'b1),.ms_i(32'd1000),.seq_i(16'd1),
 .theta(32'sd7000),.theta_dot(32'sd0),.alpha(32'sd512),.alpha_dot(32'sd0),
 .control_u(16'sd18),.safe_u(16'sd18),.pa_i(32'sd17),.pd_i(32'sd0),.pt_i(32'sd0),.pv_i(32'sd0),
 .exec_i((32'd17<<15)|(32'd2<<12)|(32'd1<<10)|32'h183),.flags_i(32'h00280000),.raw_i(16'sd0),.adc_i(16'sd0),.reseeds_i(8'd0),
 .zero_i(32'sd0),.cal_i(1'b1),.go_i(1'b1),.uart_busy_i(busy),.uart_data_o(data),.uart_start_o(start));
 always @(posedge clk)begin
  if(start)begin $write("%c",data);busy<=1;count<=2;end
  else if(count>0)begin count<=count-1;if(count==1)busy<=0;end
 end
 initial begin #12;@(negedge clk);rst=1;repeat(5)@(negedge clk);send=1;@(negedge clk);send=0;
  repeat(6000)@(negedge clk);$finish;end
endmodule
