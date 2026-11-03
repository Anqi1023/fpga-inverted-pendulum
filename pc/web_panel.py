"""D34 local browser UI. Only binds 127.0.0.1; no public hosting is needed."""
import argparse, collections, csv, io, json, math, secrets, threading, time, webbrowser
import trace_protocol
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs
from protocol import PARAMS, BY_ID, CODES, PHASES, REASONS, parse_line, validate, PROFILE, FIRMWARE_PROFILE, theta_aliases
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial=None

class Cancelled(Exception):pass
class Engine:
    def __init__(self):
        self.lock=threading.RLock();self.jobs=collections.deque();self.stop_event=threading.Event();self.shutdown=threading.Event()
        self.port=None;self.connected=False;self.verified=False;self.demo=False;self.busy=False;self.run=False;self.phase=0;self.reason=0;self.version=0
        self.active={};self.pending={};self.buffer=bytearray();self.events=collections.deque(maxlen=180);self.samples=collections.deque(maxlen=1800);self.raw=collections.deque(maxlen=20000)
        self.position_supported=False;self.position=None;self.position_rx=0;self.position_poll=0
        self.position_query=None;self.position_query_rx=0;self.position_command_raw=None;self.position_ack_time=0;self.position_command_pending=False;self.position_mismatch_latched=False
        self.eid=0;self.sid=0;self.last_rx=0;self.last_sample_rx=0;self.demo_epoch=time.monotonic();self.port_name='';self.recording=False;self.record_samples=[];self.record_raw=[];self.disconnect_requested=False
        self.trace_status={};self.trace_meta={};self.trace_parts={};self.trace_words=[]
        self.trace_complete=False;self.trace_downloading=False;self.trace_config=None;self.trace_error=''
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
    def log(self,text,kind='info'):
        with self.lock:self.eid+=1;self.events.append(dict(id=self.eid,time=time.strftime('%H:%M:%S'),text=text,kind=kind))
    def ports(self):return [dict(name=p.device,label=f'{p.device} · {p.description}') for p in list_ports.comports()] if serial else []
    def submit(self,action,values=None):
        if action in ('stop','disconnect'):
            with self.lock:
                self.jobs.clear();self.disconnect_requested=self.disconnect_requested or action=='disconnect'
            self.stop_event.set();return
        with self.lock:
            if self.busy or self.jobs:raise ValueError('请等待当前操作完成')
            if action not in ('connect','demo','disconnect','read','apply','defaults','start','rearm','record','position','trace_arm','trace_status','trace_read'):raise ValueError('未知操作')
            if action not in ('connect','demo') and not self.connected:raise ValueError('请先连接串口或打开离线演示')
            if action in ('apply','defaults','start','rearm','position','trace_arm','trace_status','trace_read') and not self.verified:raise ValueError('尚未验证D34固件')
            self.jobs.append((action,values))
    def line(self,text):
        with self.lock:
            self.raw.append(text)
            if self.recording:self.record_raw.append(text)
        d=parse_line(text)
        if not d:return None
        with self.lock:
            self.last_rx=time.monotonic();self.run=d['run'];self.phase=d['phase'];self.reason=d['reason'];self.version=d['version']
            if d['kind']=='reply' and d['code'] in (11,12,14,15,16,17,18,19,20,21,22):
                self.trace_reply(d)
                return d
            if d['kind']=='sample':
                self.last_sample_rx=time.monotonic()
                self.sid+=1;d['id']=self.sid
                for key,idx,scale in [('k_alpha',0,1),('k_alpha_dot',1,1),('alpha_offset_q16',4,1),
                    ('outer_position_gain',81,1/65536),('balance_speed_kv',82,1),
                    ('outer_reverse',83,1),('outer_angle_limit_deg',84,180/math.pi/65536),
                    ('outer_angle_step_deg',85,180/math.pi/65536),('outer_hold_band_deg',86,180/math.pi/65536),
                    ('coordinate_alpha_gate_deg',87,180/math.pi/65536),('coordinate_ad_gate',88,1/65536),
                    ('balance_speed_u_limit',89,1),('velocity_bypass_setting',90,1)]:
                    d[key]=self.active[idx]*scale if idx in self.active else None
                if d.get('cruise_firmware'):
                    d['k_alpha']=self.active.get(108);d['k_alpha_dot']=self.active.get(109)
                    d['position_target_deg']=None
                    self.position_command_raw=None;self.position_query=None;self.position=None
                if d.get('point_control_firmware'):
                    near_range=self.active[92]/65536*180/math.pi if 92 in self.active else None
                    boundary_speed=self.active[93]/65536 if 93 in self.active else None
                    d['point_approach_range_deg']=near_range
                    d['point_approach_speed_rad_s']=boundary_speed
                    d['point_near_kv_setting']=self.active.get(94)
                    d['point_allowed_speed_rad_s']=(boundary_speed*min(1,abs(d['position_error_deg'])/near_range) if near_range and boundary_speed is not None and d['point_approach_enabled'] and d['point_near'] and d['coordinate_speed_known'] and not d['coordinate_recovery'] else None)
                if d.get('boost_enabled') is None and 43 in self.active:
                    d['boost_enabled']=bool(self.active[43])
                    d['boost_duty']=self.active[44]/10
                    d['boost_time_ms']=self.active[45]*5
                d['outer_equivalent_u']=(d['outer_angle_ref_deg']*math.pi/180*self.active[0]
                    if d.get('coordinate_enabled') and 0 in self.active else None)
                if not d.get('point_control_firmware'):d['position_target_deg']=self.position['target_deg'] if self.position else None
                d['position_relative_deg']=self.position['actual_deg'] if self.position else None
                d['position_status_age_ms']=round((time.monotonic()-self.position_rx)*1000) if self.position_rx else None
                if not d['run'] or d['phase']!=5:
                    self.position_command_raw=None;self.position_mismatch_latched=False
                expected=self.position_command_raw
                observed=d.get('position_target_deg')
                queried=self.position_query
                d['position_query_target_deg']=queried/65536*180/math.pi if queried is not None else None
                d['position_query_age_ms']=round((time.monotonic()-self.position_query_rx)*1000) if self.position_query_rx else None
                mismatch=bool(expected is not None and observed is not None and not self.position_command_pending and
                    time.monotonic()-self.position_ack_time>.2 and (abs(round(observed*math.pi/180*65536)-expected)>1 or
                    (self.position_query_rx>self.position_ack_time and queried is not None and queried!=expected)))
                d['position_command_target_deg']=expected/65536*180/math.pi if expected is not None else None
                d['position_target_mismatch']=mismatch
                if mismatch and not self.position_mismatch_latched:
                    self.log(f'目标异常：已确认 {d["position_command_target_deg"]:.6f}°，实板报文 {observed:.6f}°，查询目标={d['position_query_target_deg']}°；未自动改写目标，请停止检查','error')
                self.position_mismatch_latched=mismatch
                if self.samples and d['time_ms']<self.samples[-1]['time_ms']:self.samples.clear()
                self.samples.append(d)
                if self.recording:self.record_samples.append(dict(d))
            elif d['code'] in (9,10):
                self.position=dict(actual_deg=d['active']/65536*180/math.pi,target_deg=d['pending']/65536*180/math.pi);self.position_rx=time.monotonic()
                if d['code']==10:self.position_query=d['pending'];self.position_query_rx=time.monotonic()
            elif d['code'] in (1,2) and d['id'] in BY_ID:
                self.active[d['id']]=d['active'];self.pending[d['id']]=d['pending']
                if d['id']==77:
                    self.active=theta_aliases(self.active);self.pending=theta_aliases(self.pending)
        return d
    def trace_reply(self,d):
        code,idx=d['code'],d['id'];a=d['active']&0xffffffff;p=d['pending']&0xffffffff
        if code in (11,12,15) and idx==254:
            self.trace_status=trace_protocol.status(a,p)
            if self.trace_downloading:self.trace_meta=dict(self.trace_status)
        elif self.trace_downloading:
            if code==12 and idx==253:self.trace_meta.update(capture_version=a,clock_hz=p)
            elif code==12 and idx==252:self.trace_meta.update(arm_ms=a,reference_ms=p)
            elif 16<=code<=22:
                part=code-16
                if (idx,part) in self.trace_parts:self.trace_error='捕获回包重复'
                self.trace_parts[(idx,part)]=(a,p)
                if part==trace_protocol.part_count(self.trace_meta)-1 and p:self.trace_error='捕获尾部填充值异常'
            elif code==14:
                count=self.trace_meta.get('count',0)
                if p!=count or not 1<=count<=128 or len(self.trace_parts)!=count*trace_protocol.part_count(self.trace_meta):
                    self.trace_error='捕获回包缺失或数量不一致'
                try:
                    words=[];checksum=0
                    for row in range(count):
                        w=[]
                        for part in range(trace_protocol.part_count(self.trace_meta)):
                            va,vp=self.trace_parts[(row,part)];checksum^=va^vp
                            w.extend([va] if part==trace_protocol.part_count(self.trace_meta)-1 else [va,vp])
                        words.append(w)
                    if checksum!=a:self.trace_error='捕获校验和不一致'
                    if self.trace_error:raise ValueError(self.trace_error)
                    trace_protocol.decode_records(words,self.trace_meta)
                    self.trace_words=words;self.trace_complete=True
                except (KeyError,ValueError) as error:
                    self.trace_error=str(error);self.trace_complete=False

    def read_lines(self):
        if self.demo:return []
        data=self.port.read(min(self.port.in_waiting,65536))
        self.buffer.extend(data)
        if len(self.buffer)>131072:self.buffer.clear();raise ValueError('串口数据异常，请停止并检查')
        out=[]
        while b'\n' in self.buffer:
            raw,_,tail=self.buffer.partition(b'\n');self.buffer=bytearray(tail)
            d=self.line(raw.decode('ascii',errors='replace').strip('\r'))
            if d:out.append(d)
        return out
    def command(self,text,code,idx=None,allow_stop=True,quiet=False):
        if not quiet:self.log('→ '+text)
        with self.lock:
            audit='#TX '+text
            self.raw.append(audit)
            if self.recording:self.record_raw.append(audit)
        if self.demo:
            if allow_stop and self.stop_event.is_set():raise Cancelled()
            d=self.demo_command(text,code,idx)
            return d
        self.port.write((text+'\r\n').encode('ascii'));deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            if allow_stop and self.stop_event.is_set():raise Cancelled()
            for d in self.read_lines():
                if d['kind']!='reply':continue
                if d['code']>=128:raise ValueError(CODES.get(d['code'],'FPGA拒绝命令'))
                if d['code']==code and (idx is None or d['id']==idx):
                    if code!=1 and not quiet:self.log('← '+CODES.get(code,'完成'))
                    return d
            time.sleep(.004)
        raise ValueError('命令超时，未确认执行；检查COM、RX和固件。需要时现场停机。')
    def demo_command(self,text,code,idx):
        with self.lock:
            words=text.split();idx=idx or 0
            if words[0]=='SET':
                ident=int(words[1]);value=int(words[2]);self.pending[ident]=value
                if ident==2:self.pending[77]=value*10
                if ident in (2,77):self.pending=theta_aliases(self.pending)
            elif text=='APPLY':
                if self.run:raise ValueError('请先停止，再应用参数')
                validate(self.pending);self.active=dict(self.pending);self.version+=1
            elif text=='DEFAULT':self.pending={p['id']:p['default'] for p in PARAMS}
            elif text=='START':self.run=True;self.phase=7 if self.active.get(79)==1 else 4;self.demo_epoch=time.monotonic()
            elif text=='STOP':self.run=False;self.phase=0
            elif text=='GET ALL':pass
            elif text=='GET INFO':return dict(id=40,active=FIRMWARE_PROFILE,pending=34)
            elif words[:2]==['SET','40'] or text=='GET POS':raise ValueError('离线演示不执行位置运动；请连接本包R15M实板固件')
            self.log('← '+CODES.get(code,'完成')+' · 演示')
            return dict(id=idx,active=self.active.get(idx,0),pending=self.pending.get(idx,0))
    def stop(self,disconnect=False):
        with self.lock:
            disconnect=disconnect or self.disconnect_requested;self.disconnect_requested=False
        self.stop_event.clear()
        if self.connected:
            self.command('STOP',4,allow_stop=False)
            self.log('已确认停止；输出许可已撤销','ok')
        if disconnect:self.close()
    def close(self):
        if self.port:self.port.close()
        with self.lock:
            self.port=None;self.connected=False;self.verified=False;self.run=False;self.position_supported=False;self.position=None;self.port_name='';self.recording=False
    def do(self,action,values):
        if action in ('connect','demo'):
            if self.connected:raise ValueError('请先停止并断开当前连接')
            with self.lock:
                self.position_query=None;self.position_query_rx=0;self.position_command_raw=None;self.position_ack_time=0;self.position_mismatch_latched=False;self.trace_status={};self.trace_complete=False;self.trace_config=None;self.demo=action=='demo';self.position_supported=False;self.position=None;self.position_rx=0;self.position_poll=0;self.active={};self.pending={};self.samples.clear();self.last_rx=0;self.last_sample_rx=0;self.buffer.clear();self.recording=False
            if self.demo:
                self.active={p['id']:p['default'] for p in PARAMS};self.pending=dict(self.active);self.run=False;self.version=0;self.phase=0;self.reason=0;self.port_name='离线演示'
            else:
                if serial is None:raise ValueError('请先运行install_windows.bat安装pyserial')
                self.port=serial.Serial(str(values),460800,timeout=0,write_timeout=.5,rtscts=False,dsrdtr=False);self.port.reset_input_buffer();self.port_name=str(values)
            self.connected=True
            try:
                info=self.command('GET INFO',0)
                if (info['id'],info['active'],info['pending'])!=(40,FIRMWARE_PROFILE,34):raise ValueError('需要 双模式固件（profile38）；请烧录本包工程并使用本包网页')
                self.position_supported=info['active']==FIRMWARE_PROFILE and not self.demo
                self.verified=True;self.command('GET ALL',8)
                if not self.demo:self.command('TRACE STAT',12,254,quiet=True)
                if len(self.active)!=len(PARAMS):raise ValueError('参数回读不完整')
                self.log('已连接，101项参数读取完成；K_THETA显示实际增益，ID77内部为十倍整数'+(' · 模拟数据' if self.demo else ''),'ok')
                if self.demo:self.command('START',5)
            except Exception:self.close();raise
        elif action=='disconnect':self.stop(disconnect=True)
        elif action=='read':self.command('GET ALL',8)
        elif action=='apply':
            if self.run:raise ValueError('正在运行：请先点击停止')
            desired=dict(self.pending)
            for k,v in (values or {}).items():
                idx=int(k)
                if idx not in BY_ID or type(v) is not int:raise ValueError('参数格式错误')
                desired[idx]=v
            # A legacy API payload with ID2 only still means an integer gain.
            updates={int(k):v for k,v in (values or {}).items()}
            if 2 in updates and 77 not in updates:
                if not -100<=updates[2]<=100:raise ValueError('ID2仅支持-100至100整数；小数使用ID77')
                desired[77]=updates[2]*10
            if 77 in updates and 2 in updates and desired[2]!=theta_aliases(desired)[2]:
                raise ValueError('ID2与ID77冲突，请重新读取参数')
            desired=theta_aliases(desired)
            validate(desired)
            for idx,v in desired.items():
                if idx==2:continue # ID77 is authoritative; never send the readback sentinel.
                if self.pending.get(idx)!=v:self.command(f'SET {idx} {v}',2,idx)
            self.command('APPLY',3);self.command('GET ALL',8)
            if self.active!=desired:raise ValueError('回读不一致，请停止并检查')
            self.log('整组回读一致 · 参数版本 '+str(self.version),'ok')
        elif action=='defaults':
            if self.run:raise ValueError('先停止，再恢复默认参数')
            self.command('DEFAULT',6);self.command('GET ALL',8);self.log('R15默认值已暂存，请点击应用参数')
        elif action=='start':self.command('START',5)
        elif action=='rearm':self.command('REARM',7)
        elif action=='position':
            if self.demo or not self.position_supported:raise ValueError('目标偏移需要本包R15M实板固件')
            if not self.run or self.phase!=5:raise ValueError('请先启动所选模式并进入稳定平衡')
            if type(values) not in (int,float) or not math.isfinite(values):raise ValueError('累计目标必须为有限数值')
            if not self.last_sample_rx or time.monotonic()-self.last_sample_rx>1:raise ValueError('遥测已过期，请等待新的控制采样再发送目标')
            if self.active.get(80)!=1 or self.active.get(81,0)==0:raise ValueError('先启用模式1定位并确认ID81非零')
            target=round(values*math.pi/180*65536)
            if not -2147483647<=target<=2147483647:raise ValueError('累计目标超出32位Q16角度范围，约±5215圈')
            self.position_command_pending=True
            try:
                reply=self.command(f'SET 40 {target}',9,40)
                if reply['pending']!=target:raise ValueError('目标回读不一致，请停止检查')
                self.position_command_raw=target;self.position_ack_time=time.monotonic();self.position_mismatch_latched=False
                self.log(f'目标确认：原始值={target}，回读={reply["pending"]}，累计角度={target/65536*180/math.pi:.6f}°','ok')
            finally:self.position_command_pending=False
            self.log(f'目标 {values:+g}° 已接收；以本次进入平衡的位置为 0°，尚未确认到位','ok')
        elif action in ('trace_arm','trace_status','trace_read'):
            raise ValueError('本精简版已停用同步事件捕获，请使用普通CSV记录')
            if self.demo:raise ValueError('事件捕获仅支持R15M实板，离线演示不生成诊断证据')
            if action=='trace_arm':
                mode=values or 'velocity_jump'
                if mode not in ('velocity_jump','reference_withdrawal'):raise ValueError('捕获模式无效')
                self.command('GET ALL',8,quiet=True)
                self.command('GET POS',10,40,quiet=True)
                self.trace_parts={};self.trace_words=[];self.trace_complete=False;self.trace_meta={};self.trace_error=''
                self.trace_config=dict(firmware='D34',protocol=1,control_profile=PROFILE,version=self.version,
                    active={p['name']:self.active[p['id']] for p in PARAMS},
                    runtime_position_target_deg=self.position['target_deg'] if self.position else None)
                self.record_samples=[];self.record_raw=[];self.recording=True
                self.command('TRACE REF' if mode=='reference_withdrawal' else 'TRACE ARM',11,254)
                if self.trace_status.get('schema')!=2 or self.trace_status.get('trigger_mode')!=mode:
                    raise ValueError('捕获模式回读不一致，请确认使用本包R15L固件')
                self.log(('捕获已准备：大速度贡献跳变≥20个百分点/5ms' if mode=='velocity_jump' else '捕获已准备：目标范围外定位参考撤回')+'；普通日志同步记录，准备捕获不会启动电机。','ok')
            elif action=='trace_status':self.command('TRACE STAT',12,254)
            else:
                self.stop()
                self.trace_parts={};self.trace_words=[];self.trace_meta={};self.trace_complete=False;self.trace_error=''
                self.trace_downloading=True
                try:self.command('TRACE GET',14)
                finally:self.trace_downloading=False
                if not self.trace_complete:raise ValueError(self.trace_error or '捕获回读未完成，可再次停止并读取')
                self.recording=False
                self.log('捕获回读与校验完成，可导出诊断包','ok')
        elif action=='record':
            with self.lock:
                self.recording=not self.recording
                if self.recording:self.record_samples=[];self.record_raw=[]
            self.log('开始记录' if self.recording else '记录已暂停，可导出')
    def sample_demo(self):
        with self.lock:
            t=time.monotonic()-self.demo_epoch;r=self.run and self.phase!=7;self.sid+=1
            d=dict(kind='sample',id=self.sid,time_ms=int(time.monotonic()*1000),seq=self.sid,version=self.version,run=self.run,phase=self.phase,reason=0,cal=True,alpha_deg=145*math.cos(t*4) if r else 0,theta_deg=35*math.sin(t*3) if r else 0,td=4*math.sin(t*3) if r else 0,ad=0,u=48*math.sin(t*3) if r else 0,duty=abs(48*math.sin(t*3)) if r else 0)
            self.samples.append(d);self.last_rx=time.monotonic();self.last_sample_rx=self.last_rx
            if self.recording:self.record_samples.append(dict(d))
    def loop(self):
        while not self.shutdown.is_set():
            try:
                if self.stop_event.is_set():self.busy=True;self.stop();self.busy=False
                with self.lock:job=self.jobs.popleft() if self.jobs else None
                if job:
                    self.busy=True
                    try:self.do(*job)
                    except Cancelled:self.log('当前操作已中断，正在停止');self.stop()
                    except Exception as e:self.log(str(e),'error')
                    finally:self.busy=False
                elif self.connected:
                    if self.demo:self.sample_demo()
                    else:
                        self.read_lines()
                        if self.verified and self.position_supported and time.monotonic()-self.position_poll>=.5:
                            self.position_poll=time.monotonic();self.busy=True
                            try:
                                self.command('GET POS',10,40,quiet=True)
                                if self.trace_status.get('armed'):self.command('TRACE STAT',12,254,quiet=True)
                            except Cancelled:self.stop()
                            finally:self.busy=False
            except Exception as e:
                self.log('通信异常：'+str(e)+'；未确认电机状态。','error');self.close();self.busy=False
            time.sleep(.02)
    def state(self,cursor=0,sample_cursor=0):
        with self.lock:
            return dict(sample_age_ms=round((time.monotonic()-self.last_sample_rx)*1000) if self.last_sample_rx else None,trace_status=dict(self.trace_status),trace_complete=self.trace_complete,trace_error=self.trace_error,position_supported=self.position_supported,position=self.position,position_age_ms=round((time.monotonic()-self.position_rx)*1000) if self.position_rx else None,connected=self.connected,verified=self.verified,demo=self.demo,busy=self.busy or bool(self.jobs),run=self.run,phase=self.phase,phase_name=PHASES.get(self.phase,'未知'),reason=REASONS.get(self.reason,'未知'),version=self.version,port=self.port_name,active=dict(self.active),pending=dict(self.pending),recording=self.recording,record_count=len(self.record_samples),age_ms=round((time.monotonic()-self.last_rx)*1000) if self.last_rx else None,events=[x for x in self.events if x['id']>cursor],samples=[x for x in self.samples if x['id']>sample_cursor][-300:],last_sample=self.samples[-1] if self.samples else None)
    def export(self,kind,rows_override=None):
        if kind=='trace':
            with self.lock:
                if not self.trace_complete:raise ValueError('先停止并读取完整捕获，再导出诊断包')
                current=dict(firmware='D34',protocol=1,control_profile=PROFILE,version=self.version,
                    active={p['name']:self.active.get(p['id']) for p in PARAMS})
                config=self.trace_config if self.trace_config and self.trace_config['version']==self.trace_meta.get('capture_version') else None
                return trace_protocol.export_bundle(self.trace_words,self.trace_meta,config,current,
                    self.export('csv',rows_override=list(self.record_samples)),self.export('log'))
        with self.lock:
            if kind=='config':return json.dumps(dict(firmware='D34',protocol=1,control_profile=PROFILE,version=self.version,active={p['name']:self.active[p['id']] for p in PARAMS}),ensure_ascii=False,indent=2).encode()
            if kind=='log':
                if self.demo:return ('# 离线演示，不含真实串口数据\n'+'\n'.join(e['time']+' '+e['text'] for e in self.events)).encode()
                return '\n'.join(self.record_raw or self.raw).encode()
            rows=list(self.record_samples or self.samples) if rows_override is None else list(rows_override)
        b=io.StringIO();w=csv.writer(b);keys=['control_mode','cruise_firmware','cruise_state','cruise_correction','cruise_recovering','cruise_speed_known','cruise_reference_speed','cruise_target_speed','cruise_reference_deg','position_command_target_deg','position_query_target_deg','position_query_age_ms','position_target_mismatch','motion_firmware','motion_enabled','motion_active','motion_paused','motion_calculating','motion_reference_deg','motion_reference_speed_rad_s','motion_tracking_error_deg','outer_integral_enabled','outer_integral_updating','outer_integral_limited','outer_integral_frozen','outer_integral_angle_deg','speed_handoff_firmware','speed_provisional','speed_confirmed','speed_handoff_rejoining','point_control_firmware','position_error_deg','point_state','point_state_name','point_approach_enabled','point_near','point_toward','point_braking','point_window','point_approach_range_deg','point_approach_speed_rad_s','point_allowed_speed_rad_s','point_near_kv_setting','serial_velocity_bypass_firmware','velocity_bypass_setting','velocity_bypass_firmware','balance_velocity_bypass','velocity_transition_firmware','balance_velocity_releasing','balance_velocity_catch_release','balance_active','balance_priority_firmware','position_reference_enabled','balance_speed_kv','balance_speed_u_limit','balance_velocity_request_u','balance_velocity_applied_u','balance_velocity_limited','motor_state','motor_state_name','motor_command','motor_direction','motor_enabled','motor_permit','motor_protection_locked','motor_timeout','motor_command_error','motor_fault','motor_estop','safety_permit','boost_enabled','boost_duty','boost_time_ms','coordinated_firmware','coordinate_enabled','coordinate_state','coordinate_state_name','coordinate_speed_known','coordinate_confirmed_zero','coordinate_recovery','coordinate_hold','coordinate_limited','outer_angle_target_deg','outer_angle_ref_deg','inner_angle_error_deg','k_alpha','k_alpha_dot','alpha_offset_q16','outer_position_gain','outer_speed_gain','outer_reverse','outer_angle_limit_deg','outer_angle_step_deg','outer_hold_band_deg','coordinate_alpha_gate_deg','coordinate_ad_gate','outer_position_limit_deg','outer_equivalent_u','brake_enable','brake_accel_limit','approach_speed_limit','brake_release_rate','brake_distance_valid','planner_ready','pendulum_worsening','brake_distance','brake_distance_gain','brake_assumed_deceleration','brake_state','actual_arm_speed','brake_request_u','brake_active','brake_risk','r14m_enabled','arm_control_mode','arm_velocity_reference','velocity_error','planner_state','time_ms', 'seq', 'version', 'run', 'phase', 'reason', 'alpha_deg', 'ad', 'capture_travel_deg', 'position_target_deg', 'td', 'td_raw', 'td_est', 'speed_source', 'est_valid', 'speed_valid_age_us', 'speed_edge_age_us', 'speed_estimator_reason', 'speed_selection_reason', 'position_u', 'arm_feedback_u', 'original_arm_u', 'arm_request_u', 'pendulum_p_u', 'pendulum_d_u', 'integral_u', 'position_integral_u', 'u', 'duty', 'effective_kv', 'return_position_limit_u', 'fine_drive', 'settle_stage'];w.writerow(keys)
        for row in rows:w.writerow([row.get(k) for k in keys])
        return b.getvalue().encode('utf-8-sig')

WEB=Path(__file__).with_name('web')
def make_handler(engine,token):
 class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def valid_host(self):return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')
    def reply(self,status,data,mime='application/json',filename=None):
        if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False).encode()
        self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('X-Frame-Options','DENY')
        if filename:self.send_header('Content-Disposition',f'attachment; filename="{filename}"')
        self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def authorized(self):return self.valid_host() and secrets.compare_digest(self.headers.get('X-Hengxin-Token',''),token)
    def do_GET(self):
        if not self.valid_host():self.reply(403,dict(error='本地访问限定'));return
        u=urlparse(self.path);q=parse_qs(u.query)
        try:
            if u.path.startswith('/api/'):
                if not self.authorized():self.reply(403,dict(error='访问校验失败'));return
                if u.path=='/api/meta':self.reply(200,dict(params=PARAMS,ports=engine.ports()));return
                if u.path=='/api/state':self.reply(200,engine.state(int(q.get('cursor',[0])[0]),int(q.get('samples',[0])[0])));return
                if u.path=='/api/export':
                    kind=q.get('kind',['config'])[0]
                    if kind not in ('config','csv','log','trace'):raise ValueError('无效导出类型')
                    suffix={'config':'json','csv':'csv','log':'log','trace':'zip'}[kind];self.reply(200,engine.export(kind),'application/octet-stream',f'hengxin-D34-R15M-{time.strftime("%Y%m%d-%H%M%S")}.{suffix}');return
                self.reply(404,dict(error='接口不存在'));return
            file={'/':'index.html','/index.html':'index.html','/app.css':'app.css','/app.js':'app.js','/params.json':'params.json'}.get(u.path)
            if not file:self.reply(404,dict(error='文件不存在'));return
            data=(WEB/file).read_bytes()
            if file=='index.html':data=data.replace(b'/*HX_BOOT*/',('window.HX_BOOT='+json.dumps(dict(token=token,preview=False))+';').encode())
            mime={'html':'text/html; charset=utf-8','css':'text/css; charset=utf-8','js':'text/javascript; charset=utf-8','json':'application/json'}[file.split('.')[-1]];self.reply(200,data,mime)
        except (ValueError,KeyError) as e:self.reply(400,dict(error=str(e)))
    def do_POST(self):
        if not self.authorized():self.reply(403,dict(error='访问校验失败'));return
        origin=self.headers.get('Origin')
        if origin and origin not in (f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'):
            self.reply(403,dict(error='来源不匹配'));return
        try:
            size=int(self.headers.get('Content-Length',0))
            if not 0<size<16000:raise ValueError('请求长度无效')
            body=json.loads(self.rfile.read(size))
            if self.path!='/api/action':self.reply(404,dict(error='接口不存在'));return
            engine.submit(body['action'],body.get('values'));self.reply(202,dict(ok=True))
        except (ValueError,KeyError,TypeError) as e:self.reply(400,dict(error=str(e)))
 return Handler

def main():
    p=argparse.ArgumentParser();p.add_argument('--demo',action='store_true');p.add_argument('--no-browser',action='store_true');p.add_argument('--port',type=int,default=8765);args=p.parse_args()
    engine=Engine();token=secrets.token_urlsafe(32)
    try:server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(engine,token))
    except OSError:
        print('端口被占用。请关闭旧窗口，或运行 python web_panel.py --port 8766');engine.shutdown.set();return
    if args.demo:engine.submit('demo')
    print(f'衡芯 D34 网页调参台：http://127.0.0.1:{server.server_port}\n保留此窗口；在网页点击“停止并断开”后再退出。')
    if not args.no_browser:webbrowser.open(f'http://127.0.0.1:{server.server_port}')
    try:server.serve_forever()
    except KeyboardInterrupt:
        if engine.connected:engine.stop_event.set();time.sleep(.3)
    finally:server.server_close();engine.shutdown.set()
if __name__=='__main__':main()
