"""Decode normal A5 5A / raw-debug A5 5B frames. Serial is needed only for live use."""
import argparse
import struct

class Decoder:
    def __init__(self): self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data)
        result = []
        while len(self.buf) >= 2:
            if self.buf[0] != 0xA5 or self.buf[1] not in (0x5A, 0x5B):
                del self.buf[0]; continue
            if len(self.buf) < 20: break
            packet = bytes(self.buf[:20]); check = 0xFF
            for value in packet[:19]: check ^= value
            if check != packet[19]:
                del self.buf[0]; continue  # Rescan; do not discard the next good header.
            del self.buf[:20]
            if packet[1] == 0x5B:
                count, adc, average = struct.unpack_from("<ihh", packet, 2)
                result.append(dict(mode="raw", encoder=count, adc=adc, average=average, valid=packet[18]&1))
            else:
                values = struct.unpack_from("<iiii", packet, 2)
                result.append(dict(mode="state", values=tuple(v/65536 for v in values), valid=packet[18]&1))
        return result

def main():
    import serial
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="COM port, e.g. COM6")
    args = parser.parse_args()
    decoder = Decoder()
    with serial.Serial(args.port, 115200, timeout=1) as ser:
        while True:
            for item in decoder.feed(ser.read(64)):
                if item["mode"] == "raw":
                    print(f"encoder={item['encoder']} count  adc={item['adc']} raw  average={item['average']} raw  valid={item['valid']}")
                else:
                    t,td,a,ad = item["values"]
                    print(f"theta={t:.6f} rad theta_dot={td:.6f} rad/s alpha={a:.6f} rad alpha_dot={ad:.6f} rad/s valid={item['valid']}")
if __name__ == "__main__": main()
