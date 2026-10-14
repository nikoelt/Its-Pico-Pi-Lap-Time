"""Run from the project root in a host MicroPython VM built with float32.

Uses real parser, geometry, framebuf and upstream assembler. GPIO/UART/SPI are
stubs: this is compatibility coverage, not an actual Pico timing measurement.
"""

import sys
import gc
sys.path.insert(0, '.')
from lap_logic import TRACKS, FinishLine, LapDetector, degrees_e7, parse_sentence, rmc_fix

assert 152.6546275 == 152.6546285, 'Use a float32 MicroPython build for this test'
assert degrees_e7('152.6546275') == 1526546275

def sentence(payload):
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    return ('$%s*%02X\r\n' % (payload, checksum)).encode()

fields = parse_sentence(sentence('GNRMC,120000.100,A,2741.437320,S,15239.277650,E,97.192,0.0,141026,,,A,V'))
position = rmc_fix(fields)
assert position['valid'] and position['lon_e7'] == 1526546275
assert position['lat_e7'] == -276906220
next_fields = fields[:]
next_fields[1] = '120000.200'
assert rmc_fix(next_fields)['utc_ms'] - position['utc_ms'] == 100
next_fields[6] = ''
assert rmc_fix(next_fields) is None
assert parse_sentence(b'$O* F\r\n') is None

for track in TRACKS:
    line = FinishLine(track, 10)
    # Construct integer fixes near the midpoint using only small local floats.
    x, y = line.dx / 2, line.dy / 2
    def make_fix(side, stamp):
        east = x - line.dy / line.length * side
        north = y + line.dx / line.length * side
        return {'valid': True, 'utc_ms': stamp, 'speed_mps': 50,
                'lat_e7': line.lat_e7 + int(round(north / 0.011132)),
                'lon_e7': line.lon_e7 + int(round(east / (line.lon_scale / 10000000)))}
    detector = LapDetector([track], lambda a, b: a - b, end_margin_m=10)
    assert detector.update(make_fix(-5, 0), 0) == []
    event = detector.update(make_fix(5, 100), 100)
    assert len(event) == 1 and 48 <= event[0][1] <= 52

class Clock:
    now = 0
    @staticmethod
    def ticks_ms(): return Clock.now
    @staticmethod
    def ticks_add(value, delta): return (value + delta) % (1 << 30)
    @staticmethod
    def ticks_diff(a, b): return ((a - b + (1 << 29)) % (1 << 30)) - (1 << 29)
    @staticmethod
    def sleep_ms(ms): Clock.now += ms

class Pin:
    OUT, IN, PULL_UP = 1, 0, 2
    def __init__(self, number, mode=None, pull=None, value=1):
        self.number, self.level, self.edges = number, value, [(Clock.now, value)]
    def __call__(self, value=None):
        if value is not None:
            self.level = value
            self.edges.append((Clock.now, value))
        return self.level
    value = __call__
    def init(self, mode, value=0): self(value)

class SPI:
    def __init__(self, *args, **kw):
        self.config = kw
        self.writes = []
        self.fail = False
    def write(self, data):
        if self.fail: raise OSError('SPI failure')
        self.writes.append(bytes(data))

class UART:
    def __init__(self, *args, **kw): pass
    def any(self): return 0

class PWM:
    def __init__(self, *args): pass
    def freq(self, *args): pass
    def duty_u16(self, *args): pass

class Machine:
    pass
Machine.Pin, Machine.SPI, Machine.UART, Machine.PWM = Pin, SPI, UART, PWM
Machine.WDT = lambda **kw: None
sys.modules['machine'] = Machine
sys.modules['utime'] = Clock

import LCD
display = LCD.LCD_1inch14()
assert len(display.buffer) == 64800
assert display.rst.edges == [(0, 1), (0, 1), (5, 0), (25, 1)]
assert Clock.now == 265
assert display.spi.config['baudrate'] == 10000000
for colour, data in ((display.red, b'\xf8\x00'), (display.green, b'\x07\xe0'),
                     (display.blue, b'\x00\x1f'), (display.white, b'\xff\xff')):
    display.fill(colour)
    assert bytes(display.buffer[:2]) == data
display.spi.writes = []
display.show()
assert display.spi.writes[:5] == [b'\x2a', b'\x00\x28\x01\x17', b'\x2b', b'\x00\x35\x00\xbb', b'\x2c']
assert len(display.spi.writes[5]) == 64800
display.spi.fail = True
for operation in (lambda: display.write_cmd(0x29), lambda: display.write_data(b'x')):
    try:
        operation()
        assert False, 'SPI failure must propagate'
    except OSError:
        assert display.cs() == 1
del display
gc.collect()

class PIO:
    OUT_LOW = 2
class RP2Hardware:
    pass
RP2Hardware.PIO = PIO
sys.modules['_rp2'] = RP2Hardware
pio = {'__name__': 'rp2'}
# Unix has no native _rp2 module. The assembler itself is unmodified; omit its
# hardware import rather than using import-star on a class masquerading as one.
exec(open('docs/vendor/rp2-assembler-v1.24.0.py').read().replace('from _rp2 import *', ''), pio)
class StateMachine:
    def __init__(self, *args): pass
    def active(self, *args): pass
    def init(self, *args, **kw): pass
    def put(self, words): self.words = tuple(words)
    def rx_fifo(self): return 0
class RP2:
    pass
RP2.PIO, RP2.StateMachine, RP2.asm_pio = PIO, StateMachine, pio['asm_pio']
sys.modules['rp2'] = RP2
firmware = {'__name__': 'firmware_smoke'}
exec(open('main.py').read(), firmware)
program = firmware['_dash_pulse_program']
assert list(program[0]) == [0x80a0, 0xa047, 0x0082, 0x80a0, 0xa027, 0xe001, 0x0046, 0xe000, 0x8000]
device = firmware['GPSLapTrigger']()
assert not device.log.enabled and device.log.pending == []
assert device.send_pulse()
assert device.pulse_sm.words == (0, 249998)
device.stop_pulse()
assert device.send_pulse(150)
assert device.pulse_sm.words == (149995, 249998)
assert device.pulse_started_ms == Clock.ticks_add(Clock.now, 150)
print('PASS: float32 MicroPython parser, three gates, real LCD framebuffer/SPI contract, RP2 assembler, firmware startup/pulse requests')
