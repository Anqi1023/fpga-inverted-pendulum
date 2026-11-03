"""Reproducible dual-mode digital regression. Python, Icarus Verilog, Node required.
Does not certify Gowin placement/routing or physical pendulum performance.
"""
import sys,tempfile,json,subprocess
from pathlib import Path
import sim_common as c
sys.path.insert(0,str(c.ROOT/'pc'))
import protocol
log=[]
def record(s):
 print(s,flush=True);log.append(s)
with tempfile.TemporaryDirectory() as folder:
 tmp=Path(folder)
 rtl=[str(p.relative_to(c.ROOT)) for p in (c.ROOT/'rtl').rglob('*.v')]
 c.call([c.IVERILOG]+(['-B',c.LIB] if c.LIB else [])+['-g2012','-s','top','-o',str(tmp/'top')]+rtl+['sim/gowin_dpb_model.v'])
 record('PASS complete top elaboration (dual-mode project)')
 ctrl=[str(p.relative_to(c.ROOT)) for p in (c.ROOT/'rtl/control').glob('*.v')]
 tests=[('tb_compact_position',ctrl+['sim/position_original.v']),('tb_compact_integral',ctrl+['sim/cruise_original.v']),('tb_dual_mode',ctrl+['rtl/config/serial_parameter_bank.v','rtl/config/uart_command_parser.v','sim/position_original.v','sim/cruise_original.v']),
 ('tb_cruise_supervisor',['rtl/control/cruise_recovery.v']),
 ('tb_dual_bank',['rtl/config/serial_parameter_bank.v']),
 ('tb_motor_chain',[str(p.relative_to(c.ROOT)) for p in (c.ROOT/'rtl/motor').glob('*.v')]),
 ('tb_target_uart_hold',ctrl+['rtl/config/serial_parameter_bank.v','rtl/config/uart_command_parser.v']),
 ('tb_motion_reference',ctrl),('tb_motion_budget',ctrl),('tb_motion_rebase',ctrl),
 ('tb_motion_lean',['rtl/control/arm_balance_coordinator_r15.v']),
 ('tb_motion_recovery',['rtl/control/arm_balance_coordinator_r15.v']),
 ('tb_outer_integral',['rtl/control/arm_balance_coordinator_r15.v'])]
 for name,files in tests:
  output=c.simulate(name,files+['sim/'+name+'.sv'],tmp)
  assert 'PASS' in output and 'FAIL' not in output,output
  if name=='tb_dual_bank':
   fields=[list(map(int,x.split()[1:])) for x in output.splitlines() if x.startswith('PARAM ')]
   assert len(fields)==len(protocol.PARAMS)
   for idx,v,mi,ma in fields:
    p=protocol.BY_ID[idx];assert (v,mi,ma)==(p['default'],p['min'],p['max']),(idx,v,mi,ma,p)
   replies=[protocol.parse_line(x) for x in output.splitlines() if x.startswith('@D34')]
   defaults=[x for x in replies if x and x['code']==1][:len(protocol.PARAMS)]
   assert len(defaults)==len(protocol.PARAMS) and {x['id']:x['active'] for x in defaults}=={p['id']:p['default'] for p in protocol.PARAMS}
   assert any(x and x['code']==0 and x['active']==38 for x in replies)
   record('PASS all RTL defaults/ranges and actual serial bytes match Python/UI metadata, profile38')
  record('\n'.join(x for x in output.splitlines() if x.startswith(('PASS','REACHED'))))
import check_dual_host
check_dual_host.run(c.simulate,c.call);record('PASS dual-mode packet, CSV, Python/browser config import and chart units')
for mode in [0,1]:
 values={p['id']:p['default'] for p in protocol.PARAMS};values[80]=mode
 protocol.validate(values)
 data=dict(firmware='D34',protocol=1,control_profile=protocol.PROFILE,active={p['name']:values[p['id']] for p in protocol.PARAMS})
 assert protocol.config_values(data)==values
record('PASS both mode configs validate and round-trip with exact parameter identities')
assert json.loads((c.ROOT/'pc/params.json').read_text())==json.loads((c.ROOT/'pc/web/params.json').read_text())
c.call(['node','--check','pc/web/app.js'])
record('PASS metadata synchronized and JavaScript syntax')
(c.ROOT/'sim/dual_verification.txt').write_text('\n'.join(log)+'\nGowin synthesis/placement/routing/timing and physical stability NOT VERIFIED.\n')

import check_lite_packet
record("PASS reduced packet module used by top: real UART handshake, frozen samples, both modes and parser")
