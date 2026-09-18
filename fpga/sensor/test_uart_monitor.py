import struct
import unittest
from uart_monitor import Decoder

def frame(header, payload):
    data = bytes([0xA5, header]) + payload + bytes([1])
    check = 0xFF
    for b in data: check ^= b
    return data + bytes([check])

class PacketTests(unittest.TestCase):
    def test_signed_raw_fragmented(self):
        packet = frame(0x5B, struct.pack("<ihh", -12345, -1234, 19985) + bytes(8))
        decoder = Decoder(); result=[]
        for b in packet: result += decoder.feed(bytes([b]))
        self.assertEqual(result,[dict(mode="raw",encoder=-12345,adc=-1234,average=19985,valid=1)])
    def test_legacy_and_resync(self):
        packet = frame(0x5A, struct.pack("<iiii",65536,-65536,0,131072))
        broken = bytearray(packet); broken[4] ^= 1
        result = Decoder().feed(b"noise" + broken + packet + packet)
        self.assertEqual(len(result),2)
        self.assertEqual(result[0]["values"],(1,-1,0,2))
if __name__ == "__main__": unittest.main()
