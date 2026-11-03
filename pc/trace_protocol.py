"""R15G synchronized trace v2, with offline decoding of the old R15F v1."""
import csv,io,json,zipfile,math
from protocol import signed32
REASONS=['等待同向边沿','周期法','计数法','无边沿衰减','超时归零中','已归零','反向重新确认','计数不连续']
REFERENCE_REASONS=['位置跟踪','控制未就绪','协调配置无效','测速未知','位置乘积待确认',
    '目标附近保持','接摆协调撤至零','参考换向先归零','位置增益为零','目标参考为零',
    '接摆协调减弱','参考限幅或步进','模式0','定位尚未激活','定位参考关闭','接近目标超速，位置回拉减弱']

def part_count(meta):
    return 7 if meta.get('schema')==2 else 5

def signed_bits(value,bits):
    value&=(1<<bits)-1
    return value-(1<<bits) if value&(1<<(bits-1)) else value

def status(word,reference_cycle=0):
    schema=(word>>24)&15
    return dict(schema=schema,count=word&255,pre_count=(word>>8)&127,
        armed=bool(word&(1<<15)),triggered=bool(word&(1<<16)),frozen=bool(word&(1<<17)),
        full=bool(word&(1<<18)),overflow=bool(word&(1<<19)),early_stop=bool(word&(1<<20)),
        manual_freeze=bool(word&(1<<21)),post_timeout=bool(word&(1<<22)),
        config_changed=bool(word&(1<<23)),reference_cycle=reference_cycle&0xffffffff,
        trigger_mode=('reference_withdrawal' if word&(1<<28) else 'velocity_jump') if schema==2 else 'first_reversal',
        velocity_jump_threshold_u=200 if schema==2 else None,words_per_record=13 if schema==2 else 9)

def decode_records(words,meta):
    count=meta.get('count',0);clock=meta.get('clock_hz')
    schema=meta.get('schema');nwords=13 if schema==2 else 9
    if schema not in (1,2) or not 1<=count<=128 or clock!=27000000:
        raise ValueError('捕获格式、数量或时钟不匹配')
    if len(words)!=count or any(len(w)!=nwords for w in words):raise ValueError('捕获数据不完整')
    if meta.get('pre_count',0)>64 or meta.get('overflow'):raise ValueError('捕获窗口或溢出标志异常')
    result=[];last_cycle=None;last_count=None
    first=words[0][0];ref=meta['reference_cycle'];trigger_indices=[]
    for i,w in enumerate(words):
        cycle,cnt,flags,est,raw,a,b,c,seq=w[:9]
        if last_cycle is not None and ((cycle-last_cycle)&0xffffffff)>=0x80000000:
            raise ValueError('捕获时间顺序异常')
        delta=None if last_count is None else signed32((cnt-last_count)&0xffffffff)
        last_cycle=cycle;last_count=cnt
        control=bool(flags&(1<<13));trigger=bool(flags&(1<<14))
        if trigger:trigger_indices.append(i)
        reason=(flags>>4)&7;u=(flags>>16)&65535;u=u-65536 if u&32768 else u
        dt=signed32((cycle-ref)&0xffffffff)/clock
        row=dict(index=i,cycle=cycle,time_from_first_us=((cycle-first)&0xffffffff)/clock*1e6,
            time_from_trigger_us=dt*1e6 if meta.get('triggered') else None,
            approx_time_ms=meta.get('reference_ms',0)+dt*1000,
            control_seq=seq&65535,record_type='control' if control else 'sensor',
            count=signed32(cnt),delta_count=delta,ab_decoded=format(flags&3,'02b'),
            ab_sync=format((flags>>2)&3,'02b'),speed_snapshot_source='controller' if control else 'live_estimator',
            speed_valid=bool(flags&(1<<7)),speed_reason=reason,speed_reason_name=REASONS[reason],
            estimator_negative_live=bool(flags&(1<<8)),period_valid_live=bool(flags&(1<<9)),
            illegal_event=bool(flags&(1<<10)),count_event=bool(flags&(1<<11)),
            speed_event=bool(flags&(1<<12)),control_event=control,trigger_event=trigger,
            fast_mode_live=bool(flags&(1<<15)),est_speed_rad_s=signed32(est)/65536,
            raw_speed_rad_s=signed32(raw)/65536,held_control_u=u,
            control_u=u if control else None,control_percent=u/10 if control else None,
            velocity_contribution_u=signed32(a)/65536 if control else None,
            velocity_contribution_percent=signed32(a)/65536/10 if control else None,
            pendulum_p_u=signed32(b) if control else None,pendulum_p_percent=signed32(b)/10 if control else None,
            pendulum_d_u=signed32(c) if control else None,pendulum_d_percent=signed32(c)/10 if control else None,
            edge_period_us=None if control else a&131071,edge_age_us=None if control else b&131071,
            valid_age_us=None if control else c&131071,
            have_edge_live=None if control else bool(a&(1<<19)),
            divider_busy_live=None if control else bool(a&(1<<17)),
            same_cycle_as_previous=bool(result and cycle==result[-1]['cycle']))
        # Old firmware never publishes usable reason0/6. This combination
        # identifies bounded provisional evidence in R15K, including control snapshots.
        row['speed_provisional']=row['speed_valid'] and reason in (0,6)
        row['speed_confirmed']=row['speed_valid'] and not row['speed_provisional']
        if row['speed_provisional']:row['speed_reason_name']='新方向临时测速（限幅，等待周期确认）'
        if schema==2:
            pos,alpha,ad,reference=w[9:13];coord=(seq>>16)&65535
            rr=(reference>>28)&15
            row.update(reference_snapshot_source='controller' if control else 'live_coordinator',
                reference_event=bool(coord&(1<<15)),reference_withdraw_event=bool(coord&(1<<14)),
                reverse_event=bool(coord&(1<<13)),velocity_bypassed=bool(coord&(1<<12)),
                velocity_limited=bool(coord&(1<<11)),coordinate_state=(coord>>8)&7,
                position_active=bool(coord&(1<<7)),coordinate_hold=bool(coord&(1<<6)),
                coordinate_recovery=bool(coord&(1<<5)),confirmed_zero=bool(coord&(1<<4)),
                reference_limited=bool(coord&(1<<3)),coordinate_known=bool(coord&(1<<2)),
                mode1=bool(coord&(1<<1)),position_reference_enabled=bool(coord&1),
                position_error_q16=signed32(pos),position_error_deg=signed32(pos)/65536*180/math.pi,
                alpha_q16=signed32(alpha),alpha_deg=signed32(alpha)/65536*180/math.pi,
                alpha_dot_rad_s=signed32(ad)/65536,
                reference_q16=signed_bits(reference,14),reference_deg=signed_bits(reference,14)/65536*180/math.pi,
                target_reference_q16=signed_bits(reference>>14,14),
                target_reference_deg=signed_bits(reference>>14,14)/65536*180/math.pi,
                reference_reason=rr,reference_reason_name=REFERENCE_REASONS[rr],
                control_calculation_cycle=((cycle-1)&0xffffffff) if control else None,
                control_calculation_from_trigger_us=(dt*1e6-1e6/clock) if control and meta.get('triggered') else None)
        result.append(row)
    if meta.get('triggered') and trigger_indices!=[meta.get('pre_count')]:
        raise ValueError('触发位置与窗口元数据不一致')
    if not meta.get('triggered') and trigger_indices:raise ValueError('未触发窗口含错误触发标记')
    return result

def export_bundle(words,meta,config_at_arm,current_config,normal_csv,raw_log):
    rows=decode_records(words,meta)
    for row in rows:row['controller_position_error_deg']=row.get('position_error_deg')
    csv_text=io.StringIO();writer=csv.DictWriter(csv_text,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    metadata=dict(position_error_semantics=('R15L: controller position error is relative to the moving trajectory reference while enabled; final-goal error and full reference are in normal.csv' if current_config.get('control_profile')=='R15L_multiturn_motion_pi_v1' else 'controller error relative to fixed target'),format='hengxin_sync_trace_v2' if meta['schema']==2 else 'hengxin_speed_trace_v1',**meta,
        normal_sample_count=sum(1 for _ in csv.DictReader(io.StringIO(normal_csv.decode('utf-8-sig')))),
        config_at_arm_available=config_at_arm is not None,
        note='Sensor rows contain live sensor/reference state; control rows contain the coherent inputs/reference/P/D/velocity/U used by the controller. Coincident events have separate records with equal timestamps and no inferred internal order. Control calculations occur one 27MHz clock before their published records. Windows are event-count bounded and may be shorter than 50ms. Absolute ms is approximate to 1ms. No mechanical root-cause or stability claim.')
    out=io.BytesIO()
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('trace.csv',csv_text.getvalue().encode('utf-8-sig'))
        z.writestr('trace_metadata.json',json.dumps(metadata,ensure_ascii=False,indent=2))
        z.writestr('trace_words.json',json.dumps(words,separators=(',',':')))
        if meta['schema']==2:
            changes=[];previous=None
            ka=config_at_arm.get('active',{}).get('K_ALPHA') if config_at_arm else None
            for row in rows:
                if not row['control_event']:continue
                if previous is not None:
                    delta=lambda key:row[key]-previous[key]
                    changes.append(dict(cycle=row['cycle'],time_from_trigger_us=row['time_from_trigger_us'],
                        control_seq=row['control_seq'],interval_us=row['time_from_first_us']-previous['time_from_first_us'],
                        sequence_contiguous=(row['control_seq']-previous['control_seq'])%65536==1,
                        trigger_event=row['trigger_event'],u_percent=row['control_percent'],
                        delta_u_percent=delta('control_percent'),delta_p_percent=delta('pendulum_p_percent'),
                        delta_d_percent=delta('pendulum_d_percent'),delta_velocity_percent=delta('velocity_contribution_percent'),
                        delta_reference_only_percent=ka*delta('reference_q16')/65536/10 if ka is not None else None,
                        output_near_limit=abs(row['control_percent'])>=99.9,
                        note='Reference contribution is already included in P; do not add it again.'))
                previous=row
            f=io.StringIO();keys=list(changes[0]) if changes else ['cycle','time_from_trigger_us','control_seq']
            wr=csv.DictWriter(f,fieldnames=keys);wr.writeheader();wr.writerows(changes)
            z.writestr('control_changes.csv',f.getvalue().encode('utf-8-sig'))
        z.writestr('current_config.json',json.dumps(current_config,ensure_ascii=False,indent=2))
        if config_at_arm is not None:z.writestr('config_at_arm.json',json.dumps(config_at_arm,ensure_ascii=False,indent=2))
        z.writestr('normal.csv',normal_csv)
        z.writestr('raw.log',raw_log)
        z.writestr('README.txt','将整个诊断包发回分析。trace.csv含实时传感器/参考事件和同拍控制快照；normal.csv是普通运行记录。\ncontrol_changes.csv分解相邻控制拍的P/D/速度项变化；定位参考贡献已包含在P内，不能重复相加。\n同周期的两条记录不表示先后因果。若配置变化、窗口满或提前停止，元数据会保留标记。\n不把reason6直接判为编码器故障；请同时说明是否扶摆或施加扰动。\n')
    return out.getvalue()
