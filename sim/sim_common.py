"""Digital logic/serial regression. Requires Icarus Verilog and Node.js.
No mechanical model is fitted; these tests do not prove +/-10 degree stability.
"""
from pathlib import Path
import importlib.util,json,math,os,re,shlex,subprocess,sys,tempfile

ROOT=Path(__file__).resolve().parents[1]
IVERILOG=os.environ.get('IVERILOG','iverilog')
VVP=os.environ.get('VVP','vvp')
LIB=os.environ.get('IVERILOG_LIB')
def call(args):
    p=subprocess.run(args,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode:raise RuntimeError(p.stdout)
    return p.stdout
def simulate(top,files,tmp):
    target=tmp/(top+'.vvp')
    args=[IVERILOG]+(['-B',LIB] if LIB else [])+['-g2012','-s',top,'-o',str(target)]+files
    args.append('sim/gowin_dpb_model.v')
    call(args);return call([VVP,str(target)])
