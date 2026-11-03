"""Current motion packet, atomic snapshots, multi-turn units and PI import/API."""
from pathlib import Path
import csv,io,json,math,re,tempfile
ROOT=Path(__file__).resolve().parents[1]

def run(simulate,call):
    import protocol,web_panel
    assert protocol.FIRMWARE_PROFILE==38 and len(protocol.PARAMS)==109
    with tempfile.TemporaryDirectory() as folder:
        tmp=Path(folder)
        source=(ROOT/'sim/tb_packet.sv').read_text()
        source=source.replace('sensor_packet #(', 'sensor_packet #(.MOTION_REPORT(1),')
        source=source.replace("32'h7e0003d7","32'h7f1e8583")
        source=source.replace('wire [7:0] data;',"reg signed [31:0] rp=-821000,rv=-23400,iv=-286;wire [7:0] data;")
        source=source.replace('.speed_status_i(64\'d655360)',
            ".motion_position_i(rp),.motion_velocity_i(rv),.outer_i_i(iv),.speed_status_i(64'h00000000e90c0000)")
        source=source.replace('.loop_reference_i(32\'sd0)',".loop_reference_i(-32'sd823550)")
        source=source.replace('.capture_travel_i(32\'sd7000)',".capture_travel_i(-32'sd820000)")
        source=source.replace('repeat(6000)@(negedge clk);',
            "repeat(30)@(negedge clk);rp=2147483647;rv=131072;iv=1144;repeat(6000)@(negedge clk);")
        f=tmp/'tb_motion_packet.sv';f.write_text(source)
        raw=next(line for line in simulate('tb_packet',['sim/sensor_packet_full_reference.v',str(f)],tmp).splitlines() if line.startswith('D34 '))
    assert len(raw.encode())+2==403,(len(raw),raw)
    d=protocol.parse_line(raw)
    assert d and d['motion_firmware'] and d['motion_enabled'] and d['motion_active']
    assert d['outer_integral_enabled'] and d['outer_integral_frozen'] and not d['outer_integral_updating']
    assert abs(d['motion_reference_deg']-(-821000/65536*180/math.pi))<1e-9
    assert d['motion_reference_speed_rad_s']==-23400/65536
    assert abs(d['outer_integral_angle_deg']-(-286/65536*180/math.pi))<1e-9
    assert abs(d['position_target_deg']-(-823550/65536*180/math.pi))<1e-9
    assert abs(d['position_error_deg']-(3550/65536*180/math.pi))<1e-9
    e=web_panel.Engine();e.shutdown.set();e.thread.join(timeout=2)
    e.active={p['id']:p['default'] for p in protocol.PARAMS};e.active[81]=983;e.pending=dict(e.active)
    e.line(raw)
    rows=list(csv.DictReader(io.StringIO(e.export('csv').decode('utf-8-sig'))))
    assert rows[-1]['motion_firmware']=='True' and rows[-1]['outer_integral_enabled']=='True'
    assert float(rows[-1]['motion_reference_speed_rad_s'])==-23400/65536
    old=dict(firmware='D34',protocol=1,control_profile='R15I_target_approach_v1',
        active={p['name']:e.active[p['id']] for p in protocol.PARAMS if p['id']<=94})
    imported=protocol.config_values(old)
    assert imported[81]==983 and imported[95]==1 and imported[98]==0 and imported[99]==10
    print('PASS motion403-byte packet, two-complement signed fields, frozen snapshots, CSV and old config migration')
    js=r'''
const fs=require('fs'),vm=require('vm'),assert=require('assert');const s=fs.readFileSync('pc/web/app.js','utf8');
const el=Object.fromEntries(['positionStatus','pointApproachStatus','positionSend','positionZero','positionTarget','positionUnit','positionPlusTurn','positionMinusTurn'].map(k=>[k,{disabled:false,value:'',textContent:''}]));
const actions=[],errors=[];const c={state:{verified:true,connected:true,demo:false,busy:false,run:true,phase:5,position_supported:true,age_ms:0,sample_age_ms:0,active:{80:1,81:983}},$:k=>el[k],act:(...a)=>actions.push(a),notice:t=>errors.push(t)};
vm.createContext(c);vm.runInContext(s.slice(s.indexOf('function positionAllowed()'),s.indexOf('function eventsHaveError()')),c);
vm.runInContext(s.slice(s.indexOf("if($('positionSend'))"),s.indexOf("$('guideButton').onclick=")),c);
el.positionUnit.value='turn';el.positionTarget.value='2';el.positionSend.onclick();assert.deepStrictEqual(actions.pop(),['position',720]);
el.positionTarget.value='-1.5';el.positionSend.onclick();assert.deepStrictEqual(actions.pop(),['position',-540]);
el.positionUnit.value='deg';el.positionTarget.value='360';el.positionSend.onclick();assert.deepStrictEqual(actions.pop(),['position',360]);
for(const v of ['','NaN','2000000','-2000000']){el.positionTarget.value=v;el.positionSend.onclick();}assert(actions.length===0&&errors.length===4);
c.state.last_sample={capture_travel_deg:30};el.positionPlusTurn.onclick();assert.deepStrictEqual(actions.pop(),['position',390]);el.positionMinusTurn.onclick();assert.deepStrictEqual(actions.pop(),['position',-330]);
console.log('PASS cumulative target UI: degrees/turns, positive/negative multi-turn, blank and overflow guards');
'''
    print(call(['node','-e',js]).strip())
