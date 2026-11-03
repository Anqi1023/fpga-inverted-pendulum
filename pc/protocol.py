"""D34 protocol helpers. SET values decimal; FPGA replies and telemetry hex."""
import json, re, math
from pathlib import Path
MAP = json.loads(Path(__file__).with_name('params.json').read_text(encoding='utf-8'))
PROFILE = 'D34_dual_mode_v1'
FIRMWARE_PROFILE = 38
PARAMS = MAP['params']
BY_ID = {p['id']: p for p in PARAMS}
CODES = {0x8A:'捕获尚未冻结或没有记录，请先准备捕获并运行',11:'捕获已准备',12:'捕获状态',13:'保留',14:'捕获回读结束',15:'捕获已冻结',0x89:'R11本轮固定目标0°，暂不接受非零位置目标',9:'位置目标已接收（尚未到位）',10:'位置状态',0x88:'请先进入稳定平衡，再发送位置目标',0:'固件信息',1:'参数回读',2:'已写入待应用',3:'整组参数已应用',4:'停止命令已执行',5:'已启动所选模式',6:'默认值已暂存，需应用',7:'已重新准备',8:'参数回读结束',0x80:'命令格式错误',0x81:'参数ID不存在',0x82:'数值超出范围',0x83:'请先停止并等待输出撤销',0x84:'参数组合不合法',0x85:'尚未就绪：检查标定、故障、停机和应用后100ms等待',0x86:'FPGA忙，请逐条等待回复',0x87:'拓展功能尚未启用'}
PHASES={0:'等待系统',1:'等待下垂静止',2:'起摆倒计时',3:'初始推动',4:'补能起摆',5:'稳摆',6:'停止锁定',7:'等待手动扶直'}
REASONS={0:'无',1:'系统/传感器未就绪',2:'速度超限',3:'行程超限',4:'起摆超时',5:'离开稳摆窗口',6:'状态异常',7:'仅启动测试结束'}
REPLY=re.compile(r'^@D34 C=([0-9A-F]{2}) I=([0-9A-F]{2}) A=([0-9A-F]{8}) P=([0-9A-F]{8}) V=([0-9A-F]{8}) RUN=([01]) PH=([0-7]) RS=([0-9A-F])$')
def signed32(value):return value-(1<<32) if value & (1<<31) else value
def parse_line(line):
    m=REPLY.fullmatch(line)
    if m:
        c,i,a,p,v,run,ph,rs=[int(x,16) for x in m.groups()]
        return dict(kind='reply',code=c,id=i,active=signed32(a),pending=signed32(p),version=v,run=bool(run),phase=ph,reason=rs)
    if not line.startswith('D34 '):return None
    parts=dict(re.findall(r'\b([A-Z]+)=([+-]?[0-9A-F]+)',line))
    required={'T','N','TH','TD','AL','AD','U','X','F','Q','RUN','C'}
    if not required <= parts.keys():return None
    try:
        d={k:(-1 if x.startswith('-') else 1)*int(x.lstrip('+-'),16) for k,x in parts.items()}
    except ValueError:return None
    sd=d.get('SD'); speed_reasons=['等待同向边沿','周期法','计数法','无边沿衰减','超时归零中','已归零','反向重新确认','计数不连续']
    ss=d.get('SS',0);kv=(ss>>11)&65535;kv=kv-65536 if kv&32768 else kv
    hc=d.get('HC',0)
    r14=bool(hc & (1<<31))
    extra=dict(fine_drive=bool(hc&1) if 'HC' in d else None,arm_hold=bool(hc&2) if 'HC' in d else None,arm_hold_ready=bool(hc&4) if 'HC' in d else None,reference_limited=bool(hc&8) if 'HC' in d else None,reference_blend=((hc>>8)&511)/256 if 'HC' in d else None,velocity_ref=d['VR']/65536 if 'VR' in d else None,angle_ref_deg=d['AR']/65536*180/math.pi if 'AR' in d else None,pendulum_p_u=d['PA']/10 if 'PA' in d else None,pendulum_d_u=d['PD']/10 if 'PD' in d else None,integral_u=d['PI']/10 if 'PI' in d else None,settle_mode=bool((ss>>27)&1) if 'SS' in d else None,settle_stage=ss&3 if 'SS' in d else None,
        position_weight=((ss>>2)&511)/256 if 'SS' in d else None,effective_kv=kv if 'SS' in d else None,
        capture_travel_deg=d['CT']/65536*180/math.pi if 'CT' in d else None,
        lock_origin_deg=d['LO']/65536*180/math.pi if 'LO' in d else None,
        hold_position_deg=d['XP']/65536*180/math.pi if 'XP' in d and (ss&3)>=2 and ((ss>>27)&1) else None,
        boost_enabled=bool(d['BC']&1) if 'BC' in d else None,
        boost_duty=((d['BC']>>1)&255)/10 if 'BC' in d else None,
        boost_time_ms=((d['BC']>>9)&15)*5 if 'BC' in d else None)
    extra.update(arm_control_mode=(d['NS']>>3) if 'NS' in d else None,planner_state=(d['NS']&7) if 'NS' in d else None,arm_velocity_reference=d['NV']/65536 if 'NV' in d else None,velocity_error=d['VE']/65536 if 'VE' in d else None)
    # R14M derived stopping-distance estimate, rad, NOT measured braking ability.
    brake_gain=d['BG']/256 if 'BG' in d else None
    brake_accel=d['BA']/65536 if 'BA' in d else None
    actual_speed=d['TE']/65536 if 'TE' in d else None
    bs=d.get('BS')
    extra.update(brake_distance=d['BD']/65536 if 'BD' in d and d.get('BF',0)&4 else None,
        brake_distance_gain=brake_gain,brake_assumed_deceleration=brake_accel,brake_accel_limit=brake_accel,
        approach_speed_limit=d['AS']/65536 if 'AS' in d else None,brake_release_rate=d.get('BR'),
        brake_enable=bool(bs&128) if bs is not None else None,
        brake_distance_valid=bool(d['BF']&4) if 'BF' in d else None,pendulum_worsening=bool(d['BF']&8) if 'BF' in d else None,
        planner_ready=bool(d['BF']&2) if 'BF' in d else None,
        brake_state=(bs&7) if bs is not None else None,actual_arm_speed=actual_speed,
        brake_active=bool(bs&8) if bs is not None else None,
        brake_request_u=(d.get('OA',0)/65536 if bs&64 else 0) if bs is not None else None,
        brake_risk=bool(bs&16) if bs is not None else None,
        r14m_enabled=bool(bs&32) if bs is not None else False)
    extra.update(original_arm_u=d['OA']/65536 if 'OA' in d else None,arm_request_u=d['AQ']/65536 if 'AQ' in d else None)
    extra.update(speed_valid_age_us=d.get('SA'),speed_edge_age_us=(sd&131071) if sd is not None else None,
        speed_estimator_reason=((sd>>17)&7) if sd is not None else None,
        speed_estimator_reason_name=speed_reasons[(sd>>17)&7] if sd is not None else None,
        speed_selection_reason=((sd>>20)&3) if sd is not None else None)
    extra.update(position_integral_u=d.get('VR',0)/65536/10 if r14 else None,
        position_i_enabled=bool(hc&2) if r14 else None,
        position_i_integrating=bool(hc&4) if r14 else None,
        position_i_limited=bool(hc&8) if r14 else None,
        position_i_antiwindup=bool(hc&16) if r14 else None,
        position_i_gated=bool(hc&32) if r14 else None,
        position_i_waiting=bool(hc&64) if r14 else None,
        position_i_improved=bool(hc&128) if r14 else None,
        speed_feedback_mode=((hc>>10)&3) if r14 else None,
        speed_before_filter=(d.get('TE',d['TD'])/65536 if d.get('SV',0) or 'SD' not in d else 0) if d.get('SM',0) else d.get('TR',d['TD'])/65536,
        return_coord_enabled=bool(hc&256) if r14 else None,
        return_position_limit_u=d.get('AR',0)/65536 if r14 and hc&256 else None,
        return_speed_ref_est=(-d['PT']/kv)*(-1 if hc&512 else 1) if r14 and 'PT' in d and kv!=0 else None)
    if r14:extra.update(arm_hold=False,arm_hold_ready=False,reference_limited=None,
        reference_blend=None,velocity_ref=None,angle_ref_deg=None)
    # R15K retains packet sizes/profile33; HC15 is the capability marker.
    # SV means usable estimate, while reason0/6+SV is explicitly provisional.
    handoff=bool(hc&(1<<15)) and bool(hc&(1<<30))
    provisional=handoff and bool(d.get('SV',0)) and sd is not None and ((sd>>17)&7) in (0,6)
    extra.update(speed_handoff_firmware=handoff,speed_provisional=provisional,
        speed_confirmed=bool(d.get('SV',0)) and not provisional,
        speed_handoff_rejoining=handoff and sd is not None and bool(sd&(1<<22)))
    if provisional:
        extra['speed_estimator_reason_name']='新方向临时测速（限幅，等待周期确认）'
    # R15 reuses AR/VR/HC with an explicit bit30 firmware marker.
    cruise=(hc & 0xe0000000)==0xa0000000
    r15=bool(hc & (1<<30))
    if r15:
        enabled=bool(hc&2)
        states=['未启用/等待捕获','位置回程','速度收敛请求','范围内保持','接摆优先，查看定位参考','等待测速确认','保留','保留']
        target=d.get('VR',0)/65536*180/math.pi
        reference=d.get('AR',0)/65536*180/math.pi
        extra.update(coordinated_firmware=True,coordinate_enabled=enabled,
            coordinate_state=(hc>>8)&7,coordinate_state_name=states[(hc>>8)&7],
            coordinate_speed_known=bool(hc&4),coordinate_confirmed_zero=bool(hc&16),
            coordinate_recovery=bool(hc&32),coordinate_hold=bool(hc&64),
            coordinate_limited=bool(hc&8),outer_angle_target_deg=target,
            outer_angle_ref_deg=reference,inner_angle_error_deg=reference-d['AL']/65536*180/math.pi,
            angle_ref_deg=reference,velocity_ref=None,reference_blend=None,
            arm_hold=False,arm_hold_ready=False,position_i_enabled=False,
            position_integral_u=0.0,planner_state=None,r14m_enabled=False,
            effective_kv=None if enabled else kv)
    # R15B bit29: position-only reference + independent U-domain velocity damping.
    if r15 and hc&(1<<29):
        position_enabled=bool(hc&(1<<11))
        extra.update(balance_priority_firmware=True,position_reference_enabled=position_enabled,
            balance_active=enabled and bool(hc&128) and ((d['F']>>19)&7)==5,
            balance_velocity_limited=bool(hc&(1<<12)),effective_kv=kv,
            balance_velocity_request_u=extra.get('original_arm_u'),
            balance_velocity_applied_u=extra.get('arm_request_u'),
            coordinate_state_name=('等待进入平衡/输出已撤销' if not (enabled and bool(hc&128) and ((d['F']>>19)&7)==5) else '平衡优先，定位关闭' if not position_enabled else extra['coordinate_state_name']))
    if r15 and hc&(1<<28):
        extra.update(velocity_transition_firmware=True,
            balance_velocity_releasing=bool(hc&(1<<13)),
            balance_velocity_catch_release=bool(hc&(1<<14)))
    if r15 and hc&(1<<26):
        extra.update(velocity_bypass_firmware=True,balance_velocity_bypass=bool(hc&(1<<27)))
    if r15 and hc&(1<<25):
        extra.update(serial_velocity_bypass_firmware=True)
    # R15I HC bit24: CT and NV are actual arm position and target from the
    # same control sample. Compact packets omit VE; compute error from CT-NV.
    if r15 and hc&(1<<24):
        if not {'CT','NV'}<=d.keys():return None
        ps=(hc>>16)&255
        names=['协调关闭/未激活','位置回程','接近目标','超速，减弱位置回拉','近点向外运动，纠偏','进入低速目标窗口','测速待确认','接摆优先']
        goal=d.get('NV',0)/65536*180/math.pi
        err=max(-2147483648,min(2147483647,d['CT']-d['NV']))/65536*180/math.pi
        extra.update(point_control_firmware=True,position_target_deg=goal,
            position_error_deg=err,point_state=ps&7,point_state_name=names[ps&7],
            point_approach_enabled=bool(ps&8),point_near=bool(ps&16),
            point_toward=bool(ps&32),point_braking=bool(ps&64),point_window=bool(ps&128),
            arm_velocity_reference=None,velocity_error=None)
    motion=not cruise and sd is not None and bool(sd&(1<<31))
    extra.update(motion_firmware=motion)
    if motion:
        if not {'RP','RV','AI','CT','NV'}<=d.keys():return None
        ref=signed32(d['RP']);rv=signed32(d['RV']);ival=signed32(d['AI'])
        extra.update(motion_enabled=bool(sd&(1<<30)),motion_active=bool(sd&(1<<29)),
            motion_paused=bool(sd&(1<<28)),outer_integral_enabled=bool(sd&(1<<27)),
            outer_integral_updating=bool(sd&(1<<26)),outer_integral_limited=bool(sd&(1<<25)),
            outer_integral_frozen=bool(sd&(1<<24)),motion_calculating=bool(sd&(1<<23)),
            motion_reference_deg=ref/65536*180/math.pi,motion_reference_speed_rad_s=rv/65536,
            outer_integral_angle_deg=ival/65536*180/math.pi,
            motion_tracking_error_deg=(d['CT']-ref)/65536*180/math.pi,
            position_error_deg=(d['CT']-d['NV'])/65536*180/math.pi,
            position_integral_u=None)
    if cruise:
        extra.update(cruise_firmware=True,control_mode=0,cruise_state=(hc>>4)&7,
            cruise_correction=bool(hc&2),cruise_recovering=bool(hc&4),cruise_speed_known=bool(hc&8),
            cruise_reference_speed=d.get('VR',0)/65536,cruise_target_speed=d.get('AR',0)/65536,
            cruise_reference_deg=d.get('XP',0)/65536*180/math.pi,
            angle_ref_deg=None,velocity_ref=d.get('VR',0)/65536,position_integral_u=None,
            position_i_enabled=False,position_i_integrating=False,position_i_limited=False,
            position_i_antiwindup=False,position_i_gated=False,position_i_waiting=False,
            coordinated_firmware=False,balance_priority_firmware=False,motion_enabled=False,
            position_target_deg=None,position_error_deg=None)
    else:extra.update(cruise_firmware=False,control_mode=1 if r15 and hc&2 else None)
    execution=d['X']
    motor_state=(execution>>12)&7
    extra.update(motor_state=motor_state,motor_state_name={0:'OFF',1:'READY',2:'FORWARD',3:'REVERSE',4:'WAIT_STOP'}.get(motor_state,'未知'),
        motor_command=(execution>>10)&3,motor_direction=(execution>>9)&1,motor_enabled=bool(execution&(1<<8)),
        motor_permit=bool(execution&(1<<7)),motor_protection_locked=bool(execution&(1<<6)),
        motor_timeout=bool(execution&(1<<5)),motor_command_error=bool(execution&(1<<4)),
        motor_fault=bool(execution&(1<<3)),motor_estop=bool(execution&(1<<2)),safety_permit=bool(execution&(1<<1)))
    return dict(**extra,kind='sample' ,td_raw=d['TR']/65536 if 'TR' in d else None,td_est=d['TE']/65536 if 'TE' in d else None,speed_source=d.get('SM',0),est_valid=bool(d.get('SV',0)),position_u=(d['PT']/10 if ((d['F']>>19)&7)==5 else 0) if 'PT' in d else None,arm_feedback_u=(d['PV']/10 if ((d['F']>>19)&7)==5 else 0) if 'PV' in d else None,time_ms=d['T'],seq=d['N'],theta_deg=d['TH']/65536*180/math.pi,alpha_deg=d['AL']/65536*180/math.pi,td=d['TD']/65536,ad=d['AD']/65536,u=d['U']/10,duty=((d['X']>>15)&1023)/10,phase=(d['F']>>19)&7,reason=(d['F']>>22)&15,version=d['Q'],run=bool(d['RUN']),cal=bool(d['C']),raw=line)
LEGACY_FRACTIONAL = -2147483648

def legacy_theta_alias(x10):
    return x10 // 10 if x10 % 10 == 0 else LEGACY_FRACTIONAL

def theta_aliases(values):
    result=dict(values)
    result[2]=legacy_theta_alias(result[77])
    return result

def validate(values):
    if values.get(80)==1 and values.get(98) and values.get(81,0)==0:
        raise ValueError('外环积分需要模式1且ID81非零')
    if values.get(80)==1:
        if values.get(41)!=1 or values.get(46)!=0 or values.get(58)!=0:raise ValueError("倾角协调需要ID41=1、ID46=0、ID58=0")
        if values[0]<=0 or values[87]>=values[34]:raise ValueError("平衡参数要求摆杆P>0、定位恢复偏角<保护窗口")
    if values.get(80)!=1 and values.get(74) and (values[70]!=1 or values[77]<=0 or values[76]*10>values[77]):raise ValueError("分区参数组合无效")
    for p in PARAMS:
        v=values[p['id']]
        if p['id']==2:
            if type(v) is not int or v!=legacy_theta_alias(values[77]):raise ValueError('ID2与ID77不一致；小数增益使用ID77')
            continue
        if type(v) is not int or not p['min']<=v<=p['max']:raise ValueError(f"{p['name']}: 允许 {p['min']} 至 {p['max']}")
    for ok,text in [(not(values[46] and values[58]),'R11锁零模式与R13收臂模式不能同时开启'),(values[21]>values[16],'顶部重装角度必须大于补推角度'),(values[24]>=values[23],'顶部最高速度必须不小于最低速度'),(values[37]<values[36],'刹车退出速度必须小于触发速度'),(values[38]<=values[5],'制动力度不能超过起摆输出上限'),(values[32]<=values[33],'位置项限幅不能超过总限幅'),(values[18]<=values[34],'捕获角度不能超过稳摆窗口'),(values[27]>=values[26],'快速恢复步长不能小于普通步长')]:
        if not ok:raise ValueError(text)
def human(p,value):
    if p['id']==2 and value==LEGACY_FRACTIONAL:return '小数增益：请读取ID77（除以10）'
    if p['id']==77:return f'{value/10:.1f}'
    u=p['unit']
    if u=='Q24 rad/update':return f'{value/16777216*200*180/math.pi:.3f}°/s'
    if u=='Q24 rad/update²':return f'{value/16777216*40000*180/math.pi:.3f}°/s²'
    if u=='Q16 ratio':return f'{value/65536:.5f} rad/rad'
    if u=='Q16 s':return f'{value/65536:.5f} s'
    if u=='Q16 rad/update':return f'{value/65536*180/math.pi:.4f}°/5ms'
    if u=='Q16 rad':return f'{value/65536*180/math.pi:.3f}°'
    if u=='Q16 rad/s²':return f'{value/65536:.3f} rad/s²'
    if u=='Q16 rad/s':return f'{value/65536:.3f} rad/s'
    if u=='‰':return f'{value/10:.1f}%'
    if u=='×5 ms':return f'{value*5} ms'
    return str(value)


def config_values(data):
    if data.get('control_profile')=='R14E_cruise_recovery_v1':
        if data.get('firmware')!='D34' or data.get('protocol')!=1:raise ValueError('配置格式不匹配')
        old=data.get('active',{});values={p['id']:(p['default'] if p['min']==p['max'] else old.get(p['name'],p['default'])) for p in PARAMS}
        values.update({0:BY_ID[0]['default'],1:BY_ID[1]['default'],80:0,108:old.get('K_ALPHA',2400),109:old.get('K_ALPHA_DOT',100)})
        values=theta_aliases(values);validate(values);return values
    old_profiles={'R15_angle_coordinate_v1','R14M_brake_control_v2','R14M_brake_uart_v1','R14L_speed_loop_v1','R14I_arm_slew_v1','R14H_speed_continuity_v1','R14G_start_modes_v1','R14F_return_cap_v1','R14E_ktheta_tenths_v1','R14E_near_return_v1','R14D_speed_feedback_v1','R14B_return_coord_v1','R14A_integral_gates_v1','R14_position_integral_v1','R13_fine_hold_v1','R12_pid_v1','R11_settle_lock_v1','R10_start_boost_v1','R9_speed_ab_v1','R8_single_speed_v1','R7_balance_restore_v1'}
    profile=data.get('control_profile')
    same_units={PROFILE,'R15N_continuous_motion_v1','R15M_resource_lite_v2','R15M_motion_lean_audit_v1','R15L_multiturn_motion_pi_v1','R15I_target_approach_v1','R15H_position_continuity_v1','R15G_sync_trace_v2','R15F_speed_trace_v1','R15E_serial_bypass_v1','R15D_velocity_bypass_v1','R15C_velocity_transition_v1','R15B_balance_priority_v1'}
    if data.get('firmware')!='D34' or data.get('protocol')!=1 or profile not in old_profiles|same_units:
        raise ValueError('需要R15或兼容历史配置')
    active=data.get('active',{});values={}
    for p in PARAMS:
        k=p['id']
        if profile==PROFILE:
            values[k]=active.get(p['name'],p['default'])
        elif k==90:
            # New switch absent in older files: D used direct by default;
            # B/C and earlier profiles used the limited velocity path.
            values[k]=active.get(p['name'],1) if profile in {PROFILE,'R15N_continuous_motion_v1','R15M_resource_lite_v2','R15M_motion_lean_audit_v1','R15L_multiturn_motion_pi_v1','R15I_target_approach_v1','R15H_position_continuity_v1','R15G_sync_trace_v2','R15F_speed_trace_v1','R15E_serial_bypass_v1','R15D_velocity_bypass_v1'} else 0
        elif profile not in same_units and k>=81:values[k]=p['default']
        elif 46<=k<=56 or 58<=k<=69 or k==80 or k==2 or 74<=k<=78:values[k]=p['default']
        elif k==77 and p['name'] not in active:values[k]=active.get('K_THETA',8)*10
        else:values[k]=active.get(p['name'],p['default'])
    values=theta_aliases(values)
    validate(values)
    return values
