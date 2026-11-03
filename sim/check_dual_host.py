from pathlib import Path
import csv,io,json,math,tempfile
ROOT=Path(__file__).resolve().parents[1]
def run(simulate,call):
 import protocol,web_panel,check_motion_ui
 check_motion_ui.run(simulate,call)
 with tempfile.TemporaryDirectory() as folder:
  tmp=Path(folder)
  src=(ROOT/'sim/tb_packet.sv').read_text().replace('sensor_packet #(', 'sensor_packet #(.MOTION_REPORT(1),')
  src=src.replace("32'h7e0003d7","32'ha000003b")
  src=src.replace(".velocity_ref_i(32'sd2048)",".velocity_ref_i(-32'sd23400)")
  src=src.replace(".angle_ref_i(32'sd1024)",".angle_ref_i(-32'sd32768)")
  src=src.replace(".speed_status_i(64'd655360)",".motion_position_i(32'd0),.motion_velocity_i(32'd0),.outer_i_i(32'd0),.speed_status_i(64'h80000000)")
  f=tmp/'packet.sv';f.write_text(src)
  raw=next(line for line in simulate('tb_packet',['sim/sensor_packet_full_reference.v',str(f)],tmp).splitlines() if line.startswith('D34 '))
 d=protocol.parse_line(raw)
 assert d and d['control_mode']==0 and d['cruise_state']==3 and d['cruise_correction']
 assert d['cruise_reference_speed']==-23400/65536 and d['cruise_target_speed']==-.5
 assert not d['motion_firmware'] and d['angle_ref_deg'] is None and d['position_integral_u'] is None
 e=web_panel.Engine();e.shutdown.set();e.thread.join(2)
 e.active={p['id']:p['default'] for p in protocol.PARAMS};e.active[80]=0
 e.line(raw)
 row=list(csv.DictReader(io.StringIO(e.export('csv').decode('utf-8-sig'))))[-1]
 assert row['control_mode']=='0' and float(row['cruise_target_speed'])==-.5
 assert row['k_alpha']=='2400' and row['k_alpha_dot']=='100'
 fixtures=[json.loads(p.read_text()) for p in (ROOT/'configs').glob('*.json')]
 fixtures += json.loads((ROOT/'sim/source_configs.json').read_text())
 expected=[protocol.config_values(f) for f in fixtures]
 js=r'''
const fs=require('fs'),vm=require('vm'),assert=require('assert');const source=fs.readFileSync('pc/web/app.js','utf8');
const params=JSON.parse(fs.readFileSync('pc/params.json','utf8')).params;
const elements={unitMode:{value:'human'},loadConfig:{},configFile:{click(){}},traceArm:{},traceMode:{value:'velocity_jump'},traceRefresh:{},traceRead:{},traceExport:{}};
const c={params,drafts:{},state:{},elements,notice(text){if(text.startsWith('加载失败'))throw Error(text)},renderParameters(){}};
c.$=id=>elements[id];vm.createContext(c);
vm.runInContext(source.slice(source.indexOf('function toScale('),source.indexOf('function allValues(')),c);
vm.runInContext(source.slice(source.indexOf("$('loadConfig').onclick="),source.indexOf("$('guideButton').onclick")),c);
(async()=>{for(const file of JSON.parse(process.argv[1])){await elements.configFile.onchange({target:{files:[{text:async()=>JSON.stringify(file)}],value:'x'}});console.log(JSON.stringify(c.drafts));}})().catch(e=>{console.error(e);process.exit(1)});
'''
 actual=[{int(k):v for k,v in json.loads(line).items()} for line in call(['node','-e',js,json.dumps(fixtures)]).splitlines()]
 assert actual==expected,[(a,b) for a,b in zip(actual,expected) if a!=b]
 print('PASS actual negative cruise RTL packet -> parser/CSV; native and source configs -> Python/browser parity')
