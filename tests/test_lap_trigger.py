"""Desktop regression tests: real detector/firmware with simulated Pico I/O."""

import datetime
import importlib.util
import random
import struct
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lap_logic import (TRACKS, METRES_PER_DEGREE, LapDetector, FinishLine,
                       NMEAFramer, DebouncedButton, parse_sentence, rmc_fix, DEGREE_SCALE)

TICKS_PERIOD = 1 << 30


def ticks_diff(now, before):
    return ((now - before + TICKS_PERIOD // 2) % TICKS_PERIOD) - TICKS_PERIOD // 2


def sentence(payload):
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    return ('$%s*%02X\r\n' % (payload, checksum)).encode('ascii')


def rmc(milliseconds=0, lat=-27.690622, lon=152.6546275, speed=50,
        status='A', talker='GN'):
    timestamp = datetime.datetime(2026, 10, 14, 12) + datetime.timedelta(milliseconds=milliseconds)
    def coordinate(value, width):
        absolute = abs(value)
        degrees = int(absolute)
        return ('%0*d' % (width, degrees)) + ('%09.6f' % ((absolute - degrees) * 60))
    payload = '%sRMC,%s.%03d,%s,%s,%s,%s,%s,%.3f,0.0,%s,,,A' % (
        talker, timestamp.strftime('%H%M%S'), timestamp.microsecond // 1000, status,
        coordinate(lat, 2), 'S' if lat < 0 else 'N',
        coordinate(lon, 3), 'W' if lon < 0 else 'E', speed / 0.5144444444,
        timestamp.strftime('%d%m%y'))
    return sentence(payload)


def point(track, side, along=0.5):
    line = FinishLine(track)
    x = line.dx * along - line.dy / line.length * side
    y = line.dy * along + line.dx / line.length * side
    return line.lat_e7 / DEGREE_SCALE + y / METRES_PER_DEGREE, line.lon_e7 / DEGREE_SCALE + x / line.lon_scale


def project_point(line, position):
    return line.project(*(round(value * DEGREE_SCALE) for value in position))


def fix(track, side, timestamp, speed=50, along=0.5):
    lat, lon = point(track, side, along)
    return {'valid': True, 'lat_e7': round(lat * DEGREE_SCALE), 'lon_e7': round(lon * DEGREE_SCALE),
            'utc_ms': timestamp, 'speed_mps': speed}


class ParsingTests(unittest.TestCase):
    def parse(self, line):
        return rmc_fix(parse_sentence(line))

    def test_all_supported_talkers_and_hemispheres(self):
        for talker in ('GP', 'GN', 'GL', 'GA', 'GB', 'BD'):
            for lat, lon in ((-27.690622, 152.6546275), (27.690622, -152.6546275)):
                with self.subTest(talker=talker, lat=lat):
                    result = self.parse(rmc(lat=lat, lon=lon, talker=talker))
                    self.assertTrue(result['valid'])
                    self.assertAlmostEqual(result['lat_e7'] / DEGREE_SCALE, lat, places=7)
                    self.assertAlmostEqual(result['lon_e7'] / DEGREE_SCALE, lon, places=7)
                    self.assertAlmostEqual(result['speed_mps'], 50, places=3)

    def test_bad_checksums_and_non_ascii_are_rejected(self):
        data = rmc()
        self.assertIsNone(parse_sentence(data.replace(b'2741', b'2742')))
        for line in (data.split(b'*')[0], b'$GNRMC,*ZZ', b'\xff\n', b'$GNRMC,*000', b'garbage'):
            self.assertIsNone(parse_sentence(line))

    def test_void_fix_and_invalid_mode_break_continuity(self):
        self.assertFalse(self.parse(sentence('GPRMC,120000.0,V,,,,,,,141026,,,N'))['valid'])
        self.assertTrue(self.parse(rmc())['valid'])
        payload = rmc().decode().split('*')[0][1:].replace(',,,A', ',,,N')
        self.assertFalse(self.parse(sentence(payload))['valid'])

    def test_simulator_estimated_and_unknown_modes_are_not_positions(self):
        fields = parse_sentence(rmc())
        for mode in ('E', 'S', 'N', 'F', 'R', 'X'):
            fields[12] = mode
            self.assertFalse(self.parse(sentence(','.join(fields)))['valid'])
        for mode in ('A', 'D', ''):
            fields[12] = mode
            self.assertTrue(self.parse(sentence(','.join(fields)))['valid'])
        fields[12] = 'A'
        self.assertTrue(self.parse(sentence(','.join(fields + ['V'])))['valid'])

    def test_checksum_must_have_two_hex_digits(self):
        self.assertIsNone(parse_sentence(b'$O* F\r\n'))  # int(' F',16) accepts it.
        self.assertIsNotNone(parse_sentence(b'$O*4F\r\n'))

    def test_malformed_coordinates_speed_date_and_clock(self):
        fields = parse_sentence(rmc())
        for index, bad in ((3, '2760.000'), (3, '9100.000'), (3, 'nan'),
                           (4, ''), (4, 'E'), (5, '18100.000'), (6, 'N'),
                           (7, 'nan'), (7, '-1'), (7, ''), (1, '246000.0'),
                           (1, '120000.x'), (9, '310226'), (9, '290225')):
            mutated = fields[:]
            mutated[index] = bad
            with self.subTest(index=index, bad=bad):
                self.assertIsNone(self.parse(sentence(','.join(mutated))))

    def test_utc_minute_and_midnight_rollovers(self):
        for start in (59900, 43199900):
            before = self.parse(rmc(start))
            after = self.parse(rmc(start + 100))
            self.assertEqual(after['utc_ms'] - before['utc_ms'], 100)
        fields = parse_sentence(rmc())
        fields[9] = '290224'
        self.assertIsNotNone(self.parse(sentence(','.join(fields))))

    def test_partial_multiple_and_oversized_serial_lines(self):
        framer = NMEAFramer()
        first, second = rmc(), rmc(100)
        self.assertEqual(framer.feed(first[:13]), [])
        self.assertEqual(framer.feed(first[13:] + second), [first.rstrip(b'\n'), second.rstrip(b'\n')])
        self.assertEqual(framer.feed(b'$' + b'x' * 10000), [])
        self.assertLessEqual(len(framer.buffer), 128)
        self.assertEqual(framer.feed(b'\n' + first), [first.rstrip(b'\n')])
        self.assertEqual(framer.feed(b'garbage' + second), [second.rstrip(b'\n')])

    def test_small_movements_survive_pico_float32_precision(self):
        before = self.parse(rmc(lon=152.6546275))
        after = self.parse(rmc(lon=152.6546285))
        f32 = lambda value: struct.unpack('f', struct.pack('f', value))[0]
        self.assertEqual(f32(152.6546275), f32(152.6546285))
        self.assertEqual(after['lon_e7'] - before['lon_e7'], 10)
        line = FinishLine(TRACKS[0])
        self.assertGreater(f32((after['lon_e7'] - before['lon_e7']) * f32(line.lon_scale / DEGREE_SCALE)), 0.09)


class CrossingTests(unittest.TestCase):
    def detector(self, tracks=None, **options):
        return LapDetector(tracks or [TRACKS[0]], ticks_diff, **options)

    def test_armed_second_lap_survives_glitch_near_finish(self):
        track = TRACKS[0]
        for fault in ('void', 'gap', 'jump'):
            with self.subTest(fault=fault):
                detector = self.detector()
                detector.update(fix(track, -5, 0), 0)
                self.assertEqual(len(detector.update(fix(track, 5, 100), 100)), 1)
                detector.update(fix(track, -25, 11000), 11000)
                self.assertTrue(detector.lines[0].armed)
                detector.update(fix(track, -15, 11100), 11100)
                if fault == 'void':
                    detector.update({'valid': False, 'utc_ms': 11200}, 11200)
                elif fault == 'jump':
                    detector.update(fix(track, 500, 11200), 11200)
                else:
                    detector.reset()
                self.assertEqual(detector.update(fix(track, -5, 11300), 11300), [])
                self.assertEqual(len(detector.update(fix(track, 1, 11400), 11400)), 1)

    def test_all_original_lines_at_realistic_speeds(self):
        for index, track in enumerate(TRACKS):
            for speed in (10, 50, 90):
                with self.subTest(line=index + 1, speed=speed):
                    detector = self.detector([track])
                    events = []
                    # Steady 10 Hz movement from 10 m before to 10 m after.
                    for step in range(int(20 / (speed * .1)) + 2):
                        side = -10 + speed * .1 * step
                        events.extend(detector.update(fix(track, side, step * 100, speed), step * 100))
                    self.assertEqual(len(events), 1)

    def test_original_five_metre_crossing_and_actual_sample_interval(self):
        for track in TRACKS:
            detector = self.detector([track])
            self.assertEqual(detector.update(fix(track, -2.5, 1000), 100), [])
            events = detector.update(fix(track, 2.5, 1200), 300)
            self.assertEqual(len(events), 1)
            self.assertAlmostEqual(events[0][1], 1100, delta=1)

    def test_endpoints_collinear_travel_and_outside_the_segment(self):
        track = TRACKS[0]
        for along in (0, 1):
            detector = self.detector()
            detector.update(fix(track, -2.5, 0, along=along), 0)
            self.assertEqual(len(detector.update(fix(track, 2.5, 100, along=along), 100)), 1)
        for along in (-.1, 1.1):
            detector = self.detector()
            detector.update(fix(track, -2.5, 0, along=along), 0)
            self.assertEqual(detector.update(fix(track, 2.5, 100, along=along), 100), [])
        line = FinishLine(track)
        self.assertIsNone(line.crossing_fraction(project_point(line, point(track, 0, 0)),
                                                  project_point(line, point(track, 0, 1))))

    def test_arrival_on_line_counts_once_and_startup_on_line_does_not(self):
        track = TRACKS[0]
        detector = self.detector()
        detector.update(fix(track, -5, 0), 0)
        self.assertEqual(len(detector.update(fix(track, 0, 100), 100)), 1)
        self.assertEqual(detector.update(fix(track, 5, 200), 200), [])
        detector = self.detector()
        self.assertEqual(detector.update(fix(track, 0, 0), 0), [])
        self.assertEqual(detector.update(fix(track, 5, 100), 100), [])

    def test_extended_gate_tolerates_lateral_error_in_both_directions(self):
        track = TRACKS[0]
        for along, expected in ((-0.5, 1), (1.5, 1), (-1.0, 0), (2.0, 0)):
            for before, after in ((-2.5, 2.5), (2.5, -2.5)):
                with self.subTest(along=along, before=before):
                    detector = self.detector(end_margin_m=10)
                    detector.update(fix(track, before, 0, along=along), 0)
                    events = detector.update(fix(track, after, 100, along=along), 100)
                    self.assertEqual(len(events), expected)

    def test_per_track_margin_override(self):
        track = dict(TRACKS[0], margin_m=0)
        detector = self.detector([track], end_margin_m=10)
        detector.update(fix(track, -2.5, 0, along=1.5), 0)
        self.assertEqual(detector.update(fix(track, 2.5, 100, along=1.5), 100), [])

    def test_cooldown_and_rearm_prevent_jitter_but_allow_next_lap(self):
        track = TRACKS[0]
        detector = self.detector()
        detector.update(fix(track, -2.5, 0), 0)
        self.assertEqual(len(detector.update(fix(track, 2.5, 100), 100)), 1)
        for now in range(200, 12000, 100):
            self.assertEqual(detector.update(fix(track, -2.5 if now % 200 else 2.5, now), now), [])
        for now, side in ((12000, 12), (12100, 25), (12200, 15), (12300, 5)):
            self.assertEqual(detector.update(fix(track, side, now), now), [])
        self.assertEqual(len(detector.update(fix(track, -5, 12400), 12400)), 1)

    def test_stationary_jitter_and_excessive_reported_speed(self):
        track = TRACKS[0]
        detector = self.detector()
        detector.update(fix(track, -2.5, 0, 0), 0)
        self.assertEqual(detector.update(fix(track, 2.5, 100, 0), 100), [])
        self.assertEqual(detector.update(fix(track, -2.5, 200, 111), 200), [])
        self.assertIsNone(detector.previous)

    def test_missing_void_or_implausible_positions_do_not_bridge_a_crossing(self):
        track = TRACKS[0]
        for fault in ('utc_gap', 'wall_gap', 'void', 'jump'):
            with self.subTest(fault=fault):
                detector = self.detector()
                detector.update(fix(track, -5, 0), 0)
                if fault == 'utc_gap':
                    result = detector.update(fix(track, 5, 1000), 100)
                elif fault == 'wall_gap':
                    result = detector.update(fix(track, 5, 100), 1000)
                elif fault == 'void':
                    detector.update({'valid': False}, 100)
                    result = detector.update(fix(track, 5, 200), 200)
                else:
                    self.assertEqual(detector.update(fix(track, 500, 100), 100), [])
                    result = detector.update(fix(track, 5, 200), 200)
                self.assertEqual(result, [])

    def test_duplicates_and_out_of_order_fixes_do_not_refresh_health(self):
        track = TRACKS[0]
        detector = self.detector()
        first = fix(track, -5, 1000)
        detector.update(first, 100)
        for stamp in (1000, 900):
            self.assertEqual(detector.update(fix(track, 5, stamp), 200), [])
            self.assertEqual(detector.last_fix_ms, 100)
            self.assertIs(detector.previous, first)
        detector.reset()
        self.assertEqual(detector.update(first, 2000), [])
        self.assertIsNone(detector.last_fix_ms)

    def test_direction_and_tick_counter_wrap(self):
        track = dict(TRACKS[0], direction=1)
        detector = self.detector([track])
        detector.update(fix(track, 2.5, 0), 0)
        self.assertEqual(detector.update(fix(track, -2.5, 100), 100), [])
        detector = self.detector([track])
        detector.update(fix(track, -2.5, 0), TICKS_PERIOD - 50)
        self.assertEqual(len(detector.update(fix(track, 2.5, 100), 50)), 1)
        detector.reset()
        detector.update(fix(track, -25, 200), 150)
        self.assertFalse(detector.lines[0].armed)


class FakeClock:
    def __init__(self):
        self.total = 0
        self.timers = []
        self.sleep_count = 0
        self.stop_after_sleeps = None

    def ticks_ms(self):
        return self.total % TICKS_PERIOD

    def ticks_add(self, value, delta):
        return (value + delta) % TICKS_PERIOD

    ticks_diff = staticmethod(ticks_diff)

    def advance(self, milliseconds):
        final = self.total + milliseconds
        while True:
            due = [timer for timer in self.timers if timer.due is not None and timer.due <= final]
            if not due:
                break
            timer = min(due, key=lambda item: item.due)
            self.total = timer.due
            timer.due = None
            timer.callback(timer)
        self.total = final

    def sleep_ms(self, milliseconds):
        self.sleep_count += 1
        if self.stop_after_sleeps is not None and self.sleep_count >= self.stop_after_sleeps:
            raise KeyboardInterrupt
        self.advance(milliseconds)


class FirmwareTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        clock = self.clock

        class Pin:
            OUT, IN, PULL_UP = 1, 0, 2
            instances = {}

            def __new__(cls, number, *args, **kwargs):
                if number not in cls.instances:
                    cls.instances[number] = super().__new__(cls)
                return cls.instances[number]

            def __init__(self, number, mode=None, pull=None, value=None):
                self.number = number
                if not hasattr(self, 'level'):
                    self.level = 1
                    self.transitions = []
                if value is not None:
                    self.value(value)

            def value(self, level=None):
                if level is not None:
                    self.level = level
                    self.transitions.append((clock.total, level))
                return self.level

            def init(self, mode, value=0):
                self.value(value)

            def on(self):
                self.value(1)

            def off(self):
                self.value(0)

        class StateMachine:
            def __init__(self, number):
                self.due = None
                self.fail_put = False
                self.pin = None
                self.word = None
                self.completed = False
                clock.timers.append(self)

            def init(self, program, freq, set_base):
                self.due = None
                self.completed = False
                self.pin = set_base
                self.frequency = freq
                self.pin.value(0)

            def active(self, state):
                if not state:
                    self.due = None

            def rx_fifo(self):
                return int(self.completed)

            def put(self, word):
                if self.fail_put:
                    raise OSError('PIO failed')
                self.words = tuple(word)
                delay_word, self.word = self.words
                def finish(unused):
                    self.pin.value(0)
                    self.completed = True
                def start(unused=None):
                    self.pin.value(1)
                    self.due = clock.total + (self.word + 2) * 1000 / self.frequency
                    self.callback = finish
                if delay_word:
                    self.due = clock.total + (delay_word + 5) * 1000 / self.frequency
                    self.callback = start
                else:
                    start()

        class UART:
            def __init__(self, number, **kwargs):
                self.baud = kwargs['baudrate']
                self.rx = bytearray()
                self.writes = []
                self.fail_write = False

            def any(self):
                return len(self.rx)

            def read(self, length):
                data = bytes(self.rx[:length])
                del self.rx[:length]
                return data

            def write(self, command):
                self.writes.append(command)
                return 0 if self.fail_write else len(command)

            def deinit(self):
                pass

        class PWM:
            def __init__(self, pin): pass
            def freq(self, value): pass
            def duty_u16(self, value): pass

        class WDT:
            instances = []
            def __init__(self, timeout):
                self.feeds = 0
                self.instances.append(self)
            def feed(self): self.feeds += 1

        class LCD:
            red, green, blue = 0x00f8, 0xe007, 0x1f00
            def __init__(self):
                self.shows = 0
                self.fail_show = False
            def fill(self, colour): pass
            def text(self, *args): pass
            def show(self):
                if self.fail_show:
                    raise OSError('LCD failed')
                self.shows += 1

        machine = types.SimpleNamespace(Pin=Pin, UART=UART, PWM=PWM, WDT=WDT)
        rp2 = types.SimpleNamespace(PIO=types.SimpleNamespace(OUT_LOW=2),
                                    StateMachine=StateMachine, asm_pio=lambda **kwargs: lambda fn: fn)
        self.modules = mock.patch.dict(sys.modules, {'machine': machine, 'utime': clock,
                                                    'rp2': rp2,
                                                    'LCD': types.SimpleNamespace(LCD_1inch14=LCD)})
        self.modules.start()
        self.addCleanup(self.modules.stop)
        spec = importlib.util.spec_from_file_location('firmware_under_test', ROOT / 'main.py')
        self.firmware = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.firmware)
        # Keep every test's firmware log writes out of the real project folder.
        self.log_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.log_directory.cleanup)
        original_stat = self.firmware.os.stat
        self.firmware.os = types.SimpleNamespace(
            stat=lambda path: original_stat(Path(self.log_directory.name) / path))
        self.firmware.open = lambda path, mode: open(Path(self.log_directory.name) / path, mode)
        self.device = self.firmware.GPSLapTrigger()
        self.machine = machine

    def feed(self, data, advance=0):
        self.clock.advance(advance)
        self.device.link.uart.rx.extend(data)
        self.device.step()

    def ready(self):
        self.feed(rmc(0))
        self.feed(sentence('PMTK001,220,3'))
        for timestamp in range(100, 1200, 100):
            self.feed(rmc(timestamp), 100)
            if timestamp == 100:
                self.feed(sentence('PMTK001,314,3'))
        self.assertTrue(self.device.link.ready)

    def wait_for_automatic_pulse(self):
        self.assertTrue(self.device.pulse_active)
        self.clock.advance(max(0, ticks_diff(self.device.pulse_started_ms,
                                             self.clock.ticks_ms())))
        self.assertEqual(self.device.pulse_pin.value(), 1)

    def test_default_logging_does_not_touch_filesystem_even_on_errors(self):
        self.assertFalse(self.device.log.enabled)
        with mock.patch.object(self.firmware, 'open', side_effect=AssertionError('file write')), \
                mock.patch.object(self.firmware.os, 'stat', side_effect=AssertionError('file stat')):
            self.device.log_message('error', True)
            self.device.send_pulse()
            self.clock.advance(1000)
            self.device.step()
            self.assertEqual(self.device.log.pending, [])
        self.assertEqual(list(Path(self.log_directory.name).iterdir()), [])

    def test_debug_logging_is_explicit_and_can_be_disabled(self):
        self.device.toggle_debug_mode()
        self.assertTrue(self.device.log.enabled)
        self.clock.advance(1000)
        self.device.log.flush(self.clock.ticks_ms())
        self.assertEqual(len(list(Path(self.log_directory.name).iterdir())), 1)
        self.device.toggle_debug_mode()
        self.assertFalse(self.device.log.enabled)
        self.device.log_message('should not queue', True)
        self.assertEqual(self.device.log.pending, [])

    def test_configuration_requires_both_positive_command_acknowledgements(self):
        for missing in ('220', '314'):
            with self.subTest(missing=missing):
                self.device = self.firmware.GPSLapTrigger()
                self.feed(rmc())
                if missing != '220':
                    self.feed(sentence('PMTK001,220,3'))
                for timestamp in range(100, 1200, 100):
                    self.feed(rmc(timestamp), 100)
                    if timestamp == 100 and missing != '314':
                        self.feed(sentence('PMTK001,314,3'))
                self.assertFalse(self.device.link.ready)
                self.feed(sentence('PMTK001,%s,3' % missing))
                self.feed(rmc(1200), 100)
                self.assertTrue(self.device.link.ready)

    def test_late_rejected_command_and_changed_rate_restart_setup(self):
        self.ready()
        self.feed(sentence('PMTK001,220,2'))
        self.assertFalse(self.device.link.ready)
        self.device.step()
        self.assertEqual(self.device.link.state, 'output_command')
        self.ready()
        for timestamp in (2100, 3100, 4100):
            self.feed(rmc(timestamp), 1000)
        self.assertFalse(self.device.link.ready)

    def test_failed_reconfiguration_write_recovers_without_button_or_reboot(self):
        self.ready()
        self.device.link.uart.fail_write = True
        with self.assertRaises(OSError):
            self.device.link._start_commands(self.clock.ticks_ms())
        self.assertFalse(self.device.link.ready)
        self.assertEqual(self.device.link.state, 'retry_wait')
        self.clock.advance(1000)
        self.device.step()
        self.assertEqual(self.device.link.state, 'probe')
        self.ready()

    def test_all_documented_uart_baudrates_are_probed_without_blocking_manual(self):
        seen = [self.device.link.baud]
        for unused in range(6):
            self.clock.advance(1500)
            self.device.step()
            seen.append(self.device.link.baud)
        self.assertEqual(set(seen), {4800, 9600, 14400, 19200, 38400, 57600, 115200})
        self.assertTrue(self.device.send_pulse())

    def test_silent_dropout_retries_current_baud_and_recovers_in_one_verify_window(self):
        self.ready()
        uart = self.device.link.uart
        self.clock.advance(3101)
        self.device.step()
        self.assertFalse(self.device.link.ready)
        self.assertEqual(self.device.link.baud, 38400)
        self.assertIs(self.device.link.uart, uart)  # Keep returning data queued.
        recovery_start = self.clock.total
        self.feed(rmc(4200))
        self.feed(sentence('PMTK001,220,3'))
        for timestamp in range(4300, 5400, 100):
            self.feed(rmc(timestamp), 100)
            if timestamp == 4300:
                self.feed(sentence('PMTK001,314,3'))
        self.assertTrue(self.device.link.ready)
        self.assertLessEqual(self.clock.total - recovery_start, 1200)
        self.feed(rmc(5400, *point(TRACKS[0], -5)), 100)
        self.feed(rmc(5500, *point(TRACKS[0], 5)), 100)
        self.wait_for_automatic_pulse()

    def test_known_baud_is_revisited_between_each_fallback_after_long_silence(self):
        self.ready()
        self.clock.advance(3101)
        self.device.step()
        seen = [self.device.link.baud]
        for unused in range(5):
            self.clock.advance(1500)
            self.device.step()
            seen.append(self.device.link.baud)
        self.assertEqual(seen, [38400, 9600, 38400, 115200, 38400, 57600])

    def test_returning_rmc_at_probe_deadline_is_read_before_switching_uart(self):
        self.ready()
        uart = self.device.link.uart
        self.clock.advance(3101)
        self.device.step()
        self.clock.advance(1500)
        uart.rx.extend(rmc(5700))
        self.device.step()
        self.assertIs(self.device.link.uart, uart)
        self.assertEqual(self.device.link.state, 'output_command')
        self.assertIn(b'$PMTK220,100*2F\r\n', uart.writes)

    def test_partial_returning_packet_gets_one_bounded_probe_grace(self):
        self.ready()
        uart = self.device.link.uart
        self.clock.advance(3101)
        self.device.step()
        self.clock.advance(1500)
        data = rmc(5700)
        uart.rx.extend(data[:10])
        self.device.step()
        self.assertIs(self.device.link.uart, uart)
        self.assertEqual(self.device.link.state, 'probe')
        self.clock.advance(30)
        uart.rx.extend(data[10:])
        self.device.step()
        self.assertIs(self.device.link.uart, uart)
        self.assertEqual(self.device.link.state, 'output_command')

    def test_incomplete_noise_cannot_extend_probe_forever(self):
        self.device.link.uart.rx.extend(b'$GN')
        self.device.step()
        uart = self.device.link.uart
        self.clock.advance(1500)
        self.device.step()
        self.assertIs(self.device.link.uart, uart)
        self.clock.advance(300)
        self.device.step()
        self.assertEqual(self.device.link.baud, 9600)

    def test_probe_grace_needs_only_indexing_not_bytearray_string_methods(self):
        class LegacyByteSequence:
            def __len__(self): return 3
            def __getitem__(self, index): return (36, 71, 78)[index]
        self.device.framer.buffer = LegacyByteSequence()
        self.clock.advance(1500)
        self.device.link.step(self.clock.ticks_ms())
        self.assertTrue(self.device.link.probe_extended)
        self.assertEqual(self.device.link.baud, 38400)

    def test_void_positions_with_advancing_time_do_not_start_baud_search(self):
        self.ready()
        uart = self.device.link.uart
        for timestamp in range(1200, 5000, 100):
            self.feed(rmc(timestamp, status='V'), 100)
        self.assertTrue(self.device.link.ready)
        self.assertIs(self.device.link.uart, uart)
        self.assertEqual(self.device.state, 'signal_lost')

    def test_void_rmc_without_date_also_breaks_position_continuity(self):
        self.ready()
        before, after = point(TRACKS[0], -5), point(TRACKS[0], 5)
        self.feed(rmc(1200, *before), 100)
        self.feed(sentence('GNRMC,,V,,,,,,,,,,N') + rmc(1300, *after), 100)
        self.assertFalse(self.device.pulse_active)

    def test_sample_phase_is_compensated_by_hardware_delay(self):
        # Sweep where the crossing falls inside the 100 ms update interval.
        for fraction in (0.05, 0.25, 0.5, 0.75, 0.95):
            with self.subTest(fraction=fraction):
                self.device = self.firmware.GPSLapTrigger()
                self.ready()
                before = point(TRACKS[0], -5 * fraction)
                after = point(TRACKS[0], 5 * (1 - fraction))
                self.feed(rmc(1100, *point(TRACKS[0], -10)), 100)
                self.feed(rmc(1200, *before), 100)
                sample_wall_time = self.clock.total
                self.feed(rmc(1300, *after), 100)
                self.assertEqual(self.device.pulse_pin.value(), 0)
                self.assertTrue(self.device.pulse_active)
                expected = sample_wall_time + fraction * 100 + 200
                self.assertAlmostEqual(self.clock.total + ticks_diff(
                    self.device.pulse_started_ms, self.clock.ticks_ms()), expected, delta=2)
                self.clock.advance(1000)  # No Python servicing during delay/high.
                high_edges = [stamp for stamp, level in self.device.pulse_pin.transitions if level == 1]
                self.assertAlmostEqual(high_edges[-1], expected, delta=2)
                self.assertIn((high_edges[-1] + 250, 0), self.device.pulse_pin.transitions)

    def test_manual_pulse_preempts_waiting_automatic_but_never_shortens_high(self):
        self.device.send_pulse(150)
        self.clock.advance(50)
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.assertTrue(self.device.send_pulse())
        high_start = self.clock.total
        self.clock.advance(100)
        self.assertFalse(self.device.send_pulse())
        self.clock.advance(150)
        self.assertIn((high_start + 250, 0), self.device.pulse_pin.transitions)

    def test_pio_setup_time_is_subtracted_from_automatic_wait(self):
        original = self.device.pulse_sm.init
        def slow_init(*args, **kwargs):
            self.clock.advance(60)
            return original(*args, **kwargs)
        with mock.patch.object(self.device.pulse_sm, 'init', side_effect=slow_init):
            self.device.send_pulse(150)
        self.assertEqual(self.device.pulse_started_ms, 150)
        self.clock.advance(90)
        self.assertEqual(self.device.pulse_pin.value(), 1)
        self.clock.advance(250)
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_pause_before_pio_handoff_cannot_make_backup_shorten_high(self):
        original = self.device.pulse_sm.put
        def paused_put(words):
            self.clock.advance(25)
            original(words)
        with mock.patch.object(self.device.pulse_sm, 'put', side_effect=paused_put):
            self.device.send_pulse()
        self.clock.advance(230)  # 255 ms from request, only 230 ms of high.
        self.device._service_pulse(self.clock.ticks_ms())
        self.assertEqual(self.device.pulse_pin.value(), 1)
        self.clock.advance(20)
        self.assertIn((275, 0), self.device.pulse_pin.transitions)

    def test_pause_after_handoff_does_not_lose_manual_press_after_pulse_ends(self):
        original = self.device.pulse_sm.put
        def paused_after_put(words):
            original(words)
            self.clock.advance(100)
        with mock.patch.object(self.device.pulse_sm, 'put', side_effect=paused_after_put):
            self.device.send_pulse()
        self.clock.advance(160)  # Hardware ended at 250; old software expects 350.
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.device.button_a.value(0)
        self.device.step()
        self.clock.advance(50)
        self.device.step()
        self.assertEqual(self.device.pulse_pin.value(), 1)
        self.clock.advance(250)
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_delayed_pulse_completion_survives_pause_covering_entire_pulse(self):
        original = self.device.pulse_sm.put
        def paused_after_put(words):
            original(words)
            self.clock.advance(600)
        with mock.patch.object(self.device.pulse_sm, 'put', side_effect=paused_after_put):
            self.device.send_pulse(150)
        self.device._service_pulse(self.clock.ticks_ms())
        self.assertFalse(self.device.pulse_active)
        self.assertTrue(self.device.send_pulse())

    def test_old_completion_is_cleared_before_a_new_delayed_pulse(self):
        self.device.send_pulse()
        self.clock.advance(300)
        self.assertEqual(self.device.pulse_sm.rx_fifo(), 1)
        self.assertTrue(self.device.send_pulse(150))
        self.clock.advance(100)
        self.device._service_pulse(self.clock.ticks_ms())
        self.assertTrue(self.device.pulse_active)
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.clock.advance(50)
        self.assertEqual(self.device.pulse_pin.value(), 1)
        self.clock.advance(250)
        self.device._service_pulse(self.clock.ticks_ms())
        self.assertFalse(self.device.pulse_active)

    def test_delay_deadline_survives_tick_wrap_and_late_sample_is_immediate(self):
        self.clock.total = TICKS_PERIOD - 50
        self.device.send_pulse(150)
        self.clock.advance(149)
        self.device._service_pulse(self.clock.ticks_ms())
        self.assertTrue(self.device.pulse_active)
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.clock.advance(1)
        self.assertEqual(self.device.pulse_pin.value(), 1)
        self.clock.advance(250)
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.device = self.firmware.GPSLapTrigger()
        self.ready()
        before, after = point(TRACKS[0], -2.5), point(TRACKS[0], 17.5)
        self.feed(rmc(1200, *before), 100)
        self.feed(rmc(1600, *after), 400)  # Crossing 350 ms old, target 200 ms.
        self.assertEqual(self.device.pulse_pin.value(), 1)

    def test_trickling_uart_byte_does_not_destroy_crossing_history(self):
        self.ready()
        before, after = point(TRACKS[0], -5), point(TRACKS[0], 5)
        self.feed(rmc(1200, *before), 100)
        uart = self.device.link.uart
        original_any = uart.any
        original_read = uart.read
        with mock.patch.object(uart, 'any', side_effect=lambda: original_any() or 1), \
                mock.patch.object(uart, 'read', side_effect=lambda n: original_read(n) or None):
            self.feed(rmc(1300, *after), 100)
        self.assertTrue(self.device.pulse_active)

    def test_boot_is_inactive_and_manual_trigger_works_without_gps(self):
        device = self.device
        self.assertEqual(device.pulse_pin.value(), 0)
        device.button_a.value(0)
        device.step()
        self.clock.advance(50)
        device.step()
        self.assertEqual(device.pulse_pin.value(), 1)
        start = self.clock.total
        self.clock.advance(249)
        device.step()
        self.assertEqual(device.pulse_pin.value(), 1)
        self.clock.advance(1)
        self.assertEqual(device.pulse_pin.value(), 0)
        self.assertIn((start + 250, 0), device.pulse_pin.transitions)
        self.clock.advance(2000)
        device.step()
        self.assertEqual(sum(level == 1 for _, level in device.pulse_pin.transitions), 1)
        device.button_a.value(1)
        device.step()
        self.clock.advance(50)
        device.step()
        device.button_a.value(0)
        device.step()
        self.clock.advance(50)
        device.step()
        self.assertEqual(device.pulse_pin.value(), 1)

    def test_nonblocking_pulse_keeps_gps_processing_and_does_not_extend(self):
        self.ready()
        device = self.device
        self.assertTrue(device.send_pulse())
        deadline = device.pulse_sm.due
        self.assertFalse(device.send_pulse())
        self.assertEqual(device.pulse_sm.due, deadline)
        self.feed(rmc(1200), 100)
        self.assertIsNotNone(device.detector.previous)
        self.assertEqual(device.detector.previous['utc_ms'], rmc_fix(parse_sentence(rmc(1200)))['utc_ms'])
        self.assertEqual(device.pulse_pin.value(), 1)
        self.clock.advance(150)
        self.assertEqual(device.pulse_pin.value(), 0)

    def test_gps_configuration_at_both_existing_baud_rates(self):
        self.ready()
        for command in self.device.link.uart.writes:
            self.assertIsNotNone(parse_sentence(command))
        self.device = self.firmware.GPSLapTrigger()
        self.clock.advance(1500)
        self.device.step()
        self.assertEqual(self.device.link.baud, 9600)
        old_uart = self.device.link.uart
        self.feed(rmc())
        self.assertIn(b'$PMTK251,38400*27\r\n', old_uart.writes)
        self.clock.advance(200)
        self.device.step()
        self.assertEqual(self.device.link.baud, 38400)
        self.feed(sentence('PMTK001,220,3'))
        for timestamp in range(100, 1200, 100):
            self.feed(rmc(timestamp), 100)
            if timestamp == 100:
                self.feed(sentence('PMTK001,314,3'))
        self.assertTrue(self.device.link.ready)

    def test_one_hz_duplicates_and_buffered_bursts_do_not_pass_configuration(self):
        for kind in ('slow', 'duplicates', 'burst'):
            with self.subTest(kind=kind):
                self.device = self.firmware.GPSLapTrigger()
                self.feed(rmc())
                self.clock.advance(100)
                self.device.step()
                for index in range(12):
                    stamp = index * (1000 if kind == 'slow' else 100)
                    self.feed(rmc(0 if kind == 'duplicates' else stamp),
                              1000 if kind == 'slow' else 100 if kind == 'duplicates' else 0)
                self.assertFalse(self.device.link.ready)

    def test_full_uart_to_dash_crossing_on_every_original_line(self):
        self.ready()
        for index, track in enumerate(TRACKS):
            with self.subTest(line=index + 1):
                self.device.detector.reset()
                self.device.stop_pulse()
                before, after = point(track, -2.5), point(track, 2.5)
                stamp = 2000 + index * 1000
                self.feed(rmc(stamp, *before), 100)
                self.feed(rmc(stamp + 100, *after), 100)
                self.wait_for_automatic_pulse()
                self.assertEqual(self.device.pulse_pin.value(), 1)
                self.clock.advance(250)
                self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_extended_gate_can_trigger_off_the_original_track_width(self):
        self.ready()
        before, after = point(TRACKS[0], -2.5, along=1.5), point(TRACKS[0], 2.5, along=1.5)
        self.feed(rmc(1200, *before), 100)
        self.feed(rmc(1300, *after), 100)
        self.wait_for_automatic_pulse()
        self.assertEqual(self.device.pulse_pin.value(), 1)
        self.clock.advance(250)
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_signal_loss_clears_previous_and_recovery_does_not_trigger(self):
        self.ready()
        before, after = point(TRACKS[0], -5), point(TRACKS[0], 5)
        self.feed(rmc(1200, *before), 100)
        self.clock.advance(1600)
        self.device.step()
        self.assertEqual(self.device.state, 'signal_lost')
        self.assertIsNone(self.device.detector.previous)
        self.feed(rmc(2900, *after), 100)
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.assertEqual(self.device.state, 'ready')

    def test_frozen_stream_stays_lost_and_restarts_configuration(self):
        self.ready()
        self.feed(rmc(1200), 100)
        for unused in range(35):
            self.feed(rmc(1200), 100)
        self.assertEqual(self.device.state, 'signal_lost')
        self.assertFalse(self.device.link.ready)
        self.assertIsNone(self.device.detector.last_fix_ms)

    def test_reverified_stream_recovers_after_receiver_clock_correction(self):
        self.ready()
        self.feed(rmc(1200), 100)
        self.device.link.ready = False
        self.device.link.state = 'verify'
        self.device.link._reset_rate()
        self.device.link.deadline = self.clock.ticks_add(self.clock.ticks_ms(), 3000)
        for timestamp in range(-5000, -3900, 100):
            self.feed(rmc(timestamp), 100)
        self.assertTrue(self.device.link.ready)
        self.feed(rmc(-3900), 100)
        self.assertEqual(self.device.state, 'ready')
        self.assertIsNotNone(self.device.detector.previous)

    def test_void_fix_in_same_batch_breaks_continuity(self):
        self.ready()
        before, after = point(TRACKS[0], -5), point(TRACKS[0], 5)
        self.feed(rmc(1200, *before), 100)
        self.feed(rmc(1300, status='V') + rmc(1400, *after), 200)
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_overlarge_backlog_does_not_emit_stale_pulses(self):
        self.ready()
        before, after = point(TRACKS[0], -5), point(TRACKS[0], 5)
        self.feed(rmc(1200, *before), 100)
        self.feed(rmc(1300, *after) + b'x\n' * 1500, 100)
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.assertIsNone(self.device.detector.previous)
        self.device.step()
        self.assertIsNone(self.device.detector.previous)
        self.feed(rmc(1400, *after), 100)
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_out_of_order_batch_uses_newest_fix_and_rejected_speed_loses_health(self):
        self.ready()
        before, after = point(TRACKS[0], -5), point(TRACKS[0], 5)
        self.feed(rmc(1200, *before), 100)
        self.feed(rmc(1400, *after) + rmc(1300, *before), 200)
        self.wait_for_automatic_pulse()
        self.assertEqual(self.device.pulse_pin.value(), 1)
        self.clock.advance(250)
        self.feed(rmc(1500, *after, speed=111), 100)
        self.assertEqual(self.device.state, 'signal_lost')

    def test_debug_hold_and_button_bounce_are_nonblocking(self):
        device = self.device
        device.button_b.value(0)
        device.step()
        self.clock.advance(50)
        device.step()
        self.clock.advance(3050)
        device.step()
        self.assertFalse(device.debug_mode)
        device.button_b.value(1)
        device.step()
        self.clock.advance(50)
        device.step()
        self.assertTrue(device.debug_mode)
        for level in (0, 1, 0, 1):
            device.button_a.value(level)
            device.step()
            self.clock.advance(10)
        self.assertEqual(device.pulse_pin.value(), 0)

    def test_pio_failure_fallback_and_tick_wrap(self):
        self.device.pulse_sm.fail_put = True
        with self.assertRaises(OSError):
            self.device.send_pulse()
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.device.pulse_sm.fail_put = False
        self.clock.total = TICKS_PERIOD - 100
        self.device.send_pulse()
        self.device.pulse_sm.due = None  # Simulate faulty hardware holding high.
        self.clock.advance(255)
        self.device._service_pulse(self.clock.ticks_ms())
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_hardware_pulse_finishes_without_servicing_python(self):
        self.device.send_pulse()
        self.clock.advance(2000)  # No firmware step/callback servicing.
        self.assertEqual(self.device.pulse_pin.value(), 0)
        self.assertIn((250, 0), self.device.pulse_pin.transitions)
        self.assertEqual(self.device.pulse_sm.word, 249998)

    def test_lcd_failure_does_not_prevent_manual_pulse(self):
        self.device.display.fail_show = True
        self.device.step()
        self.assertIsNone(self.device.display)
        self.assertTrue(self.device.send_pulse())
        self.clock.advance(250)
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_logging_rotation_survives_reboot_and_full_disk(self):
        with tempfile.TemporaryDirectory() as directory:
            original_open = open
            original_stat = self.firmware.os.stat
            def redirected_open(path, mode):
                return original_open(Path(directory) / path, mode)
            def redirected_stat(path):
                return original_stat(Path(directory) / path)
            with mock.patch.object(self.firmware, 'open', redirected_open, create=True), \
                    mock.patch.object(self.firmware.os, 'stat', redirected_stat):
                for reboot in range(3):
                    log = self.firmware.DebugLog(enabled=True)
                    for batch in range(80):
                        for entry in range(16):
                            log.queue('x' * 192)
                        self.clock.advance(1000)
                        log.flush(self.clock.ticks_ms())
                    files = list(Path(directory).glob('debug_log_*.txt'))
                    self.assertLessEqual(len(files), 5)
                    self.assertTrue(all(path.stat().st_size <= 16384 for path in files))
            log = self.firmware.DebugLog(enabled=True)
            log.queue('disk full')
            self.clock.advance(1000)
            with mock.patch.object(self.firmware, 'open', side_effect=OSError('Full'), create=True):
                log.flush(self.clock.ticks_ms())
            self.assertFalse(log.disk_enabled)
            for entry in range(100):
                log.queue('x')
            self.assertLessEqual(len(log.pending), 16)
            self.assertTrue(self.device.send_pulse())

    def test_watchdog_fed_only_after_success_and_exit_forces_output_low(self):
        self.clock.stop_after_sleeps = 4
        with self.assertRaises(KeyboardInterrupt):
            self.device.run()
        self.assertEqual(self.machine.WDT.instances[-1].feeds, 4)
        self.assertEqual(self.device.pulse_pin.value(), 0)
        before_fault = self.machine.WDT.instances[-1].feeds
        self.clock.stop_after_sleeps = 8
        with mock.patch.object(self.device, 'step', side_effect=OSError('Fault')):
            with self.assertRaises(KeyboardInterrupt):
                self.device.run()
        self.assertEqual(self.machine.WDT.instances[-1].feeds, before_fault)
        self.assertEqual(self.device.pulse_pin.value(), 0)

    def test_gps_errors_do_not_shorten_manual_pulse(self):
        self.device.send_pulse()
        self.clock.stop_after_sleeps = 52
        with mock.patch.object(self.device, 'step', side_effect=OSError('GPS failed')):
            with self.assertRaises(KeyboardInterrupt):
                self.device.run()
        self.assertIn((250, 0), self.device.pulse_pin.transitions)
        self.assertFalse(any(level == 0 and 0 < timestamp < 250
                             for timestamp, level in self.device.pulse_pin.transitions))

    def test_mixed_fault_replay_preserves_pulse_width_and_disabled_logging(self):
        # Deterministic long replay: wrap ticks, batch/split/corrupt GPS packets,
        # miss complete pulse intervals while Python is paused, and press A
        # repeatedly. This checks output invariants across interacting features.
        rng = random.Random(9046)
        self.clock.total = TICKS_PERIOD - 2000
        self.device = self.firmware.GPSLapTrigger()
        self.ready()
        epoch = self.clock.total - 1100
        next_sample = 1200
        with mock.patch.object(self.firmware, 'open', side_effect=AssertionError('file write')), \
                mock.patch.object(self.firmware.os, 'stat', side_effect=AssertionError('file stat')):
            for index in range(500):
                self.clock.advance(rng.choice((5, 20, 55, 100, 350, 700)))
                uart = self.device.link.uart
                elapsed = self.clock.total - epoch
                while next_sample <= elapsed:
                    phase = next_sample % 1400
                    side = -30 + min(phase, 1400 - phase) * 60 / 700
                    data = rmc(next_sample, *point(TRACKS[0], side), speed=80,
                               status='V' if rng.randrange(12) == 0 else 'A')
                    if rng.randrange(12) == 0:
                        data = data[:-4] + b'ZZ\r\n'  # Checksum corruption.
                    uart.rx.extend(data)
                    next_sample += 100
                if not self.device.link.ready:
                    uart.rx.extend(sentence('PMTK001,220,3') + sentence('PMTK001,314,3'))
                if rng.randrange(15) == 0:
                    uart.rx.extend(b'\xff$broken\x80\n')
                self.device.button_a.value(0 if index % 12 < 4 else 1)
                # Split UART reads at arbitrary byte boundaries.
                original_read = uart.read
                with mock.patch.object(uart, 'read', side_effect=lambda n: original_read(min(n, 37))):
                    self.device.step()
                self.assertLessEqual(len(self.device.framer.buffer), 128)
                self.assertFalse(self.device.log.enabled)
                self.assertEqual(self.device.log.pending, [])
            self.clock.advance(1000)
            self.device.step()
        start = None
        count = 0
        for stamp, level in self.device.pulse_pin.transitions:
            if level == 1:
                self.assertIsNone(start, 'Overlapping/extended high')
                start = stamp
            elif start is not None:
                self.assertEqual(stamp - start, 250)
                count += 1
                start = None
        self.assertIsNone(start)
        self.assertGreater(count, 10)
        self.assertFalse(self.device.pulse_active)


if __name__ == '__main__':
    unittest.main()
