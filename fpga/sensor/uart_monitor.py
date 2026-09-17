"""Decode A-team 115200-baud sensor packets and print Q16.16 states."""
import argparse, struct
import serial

parser=argparse.ArgumentParser()
parser.add_argument("port",help="Windows COM port, for example COM6")
args=parser.parse_args()
ser=serial.Serial(args.port,115200,timeout=1)
buf=bytearray()
while True:
    buf+=ser.read(64)
    start=buf.find(b"\xA5\x5A")
    if start<0:
        buf=buf[-1:]
        continue
    if start:
        del buf[:start]
    if len(buf)<20:
        continue
    packet=bytes(buf[:20]);del buf[:20]
    check=0xFF
    for value in packet[:19]:check^=value
    if check!=packet[19]:
        del buf[:1]
        continue
    theta,theta_dot,alpha,alpha_dot=struct.unpack_from("<iiii",packet,2)
    print(f"theta={theta/65536:.6f} rad  theta_dot={theta_dot/65536:.6f} rad/s  "
          f"alpha={alpha/65536:.6f} rad  alpha_dot={alpha_dot/65536:.6f} rad/s  valid={packet[18]&1}")
