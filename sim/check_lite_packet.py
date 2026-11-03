"""Exercise the actual reduced packet module used by top, real UART handshake.
Check both modes, signed extrema, frozen snapshots, parser/CSV compatibility.
"""
import tempfile,sys,json
from pathlib import Path
import sim_common as c
sys.path.insert(0,str(c.ROOT/'pc'))
import protocol
vectors=[dict(hc=0xa000003b,sd=0,nv=0,xp=-98765,rp=0,rv=0,ai=0),dict(hc=0x7700888f,sd=0xf0800000,nv=823550,xp=-12345,rp=411775,rv=-23456,ai=-123)]

def literal(v):return "32'h%08x"%(v&0xffffffff)
lines=[]
with tempfile.TemporaryDirectory() as folder:
 tmp=Path(folder)
 for mode,v in enumerate(vectors):
  vals=dict(ms_i=12345,seq_i=123,theta=-205887,theta_dot=-32768,alpha=-500,alpha_dot=-2147483648,control_u=-123,exec_i=3<<12,flags_i=5<<19,version_i=42,run_i=1,cal_i=1,hold_control_i=v['hc'],velocity_ref_i=-23400,angle_ref_i=-65536 if mode==0 else 131,capture_travel_i=-411775,loop_reference_i=v['nv'],hold_position_i=v['xp'],speed_status_i=v['sd'],est_speed_i=-12345,speed_source_i=1,est_valid_i=1,motion_position_i=v['rp'],motion_velocity_i=v['rv'],outer_i_i=v['ai'])
  ports=','.join('.%s(%s)'%(k,literal(x)) for k,x in vals.items() if k!='theta')
  tb='''`timescale 1ns/1ps
module tb_lite_packet;
reg clk=0,rst_n=0,send=0,ubusy=0;
reg signed [31:0] th=-205887;
wire [7:0] data;wire start,frame;
integer delay=0,bytes=0;
always #5 clk=~clk;
sensor_packet_lite dut(.clk(clk),.rst_n(rst_n),.send_i(send),.theta(th),.uart_busy_i(ubusy),.uart_data_o(data),.uart_start_o(start),.frame_busy_o(frame),'''+ports+''');
always @(posedge clk)begin
 if(start)begin
   if(ubusy)$fatal(1,"UART collision");
   $write("%c",data);bytes=bytes+1;ubusy<=1;delay<=4;
 end else if(delay>0)begin delay<=delay-1;if(delay==1)ubusy<=0;end
end
initial begin
 repeat(3)@(negedge clk);rst_n=1;
 @(negedge clk);send=1;
 @(negedge clk);send=0;th=2147483647;
 wait(frame);wait(!frame);repeat(3)@(negedge clk);
 if(bytes<250)$fatal(1,"incomplete frame");
 $finish;
end
initial begin #200000;$fatal(1,"timeout");end
endmodule
'''
  (tmp/'tb.sv').write_text(tb)
  args=[c.IVERILOG]+(['-B',c.LIB] if c.LIB else [])+['-g2012','-s','tb_lite_packet','-o',str(tmp/'out'),'rtl/monitor/sensor_packet.v',str(tmp/'tb.sv')]
  c.call(args);out=c.call([c.VVP,str(tmp/'out')]);line=next(x for x in out.splitlines() if x.startswith('D34 '));d=protocol.parse_line(line)
  assert d and d['kind']=='sample',line
  assert abs(d['theta_deg']+205887/65536*180/3.141592653589793)<1e-9
  assert d['ad']==-32768 and d['seq']==123 and d['version']==42 and d['u']==-12.3
  assert d['cruise_firmware']==(mode==0)
  assert d['pendulum_p_u'] is None and d['pendulum_d_u'] is None and d['integral_u'] is None
  assert d['original_arm_u'] is None and d['arm_request_u'] is None
  if mode==0:assert d['cruise_reference_speed']==-23400/65536
  else:
   assert d['motion_firmware'] and d['point_control_firmware']
   assert abs(d['motion_reference_speed_rad_s']+23456/65536)<1e-9
  json.dumps(d,allow_nan=False);lines.append(line)
(c.ROOT/'sim/lite_packet_samples.txt').write_text('\n'.join(lines)+'\n')
print('PASS actual lite UART packets: both modes, signed extrema, frozen snapshot, parser, missing probes -> None, motion/target fields')
