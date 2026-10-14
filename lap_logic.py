"""GPS parsing and crossing detection, independent of Pico hardware."""

import math

TRACKS = [
    {'right_lat': '-27.690622', 'right_lon': '152.654567', 'left_lat': '-27.690622', 'left_lon': '152.654688'},
    {'right_lat': '-27.228533', 'right_lon': '152.964891', 'left_lat': '-27.228441', 'left_lon': '152.964919'},
    {'right_lat': '-28.262069', 'right_lon': '152.036330', 'left_lat': '-28.262086', 'left_lon': '152.036433'},
]

DAY_MS = 86400000
METRES_PER_DEGREE = 111320.0
DEGREE_SCALE = 10000000
METRES_PER_E7 = METRES_PER_DEGREE / DEGREE_SCALE


def degrees_e7(text):
    """Exact decimal degrees -> integer 1e-7 degrees, avoiding Pico float32."""
    negative = text.startswith('-')
    unsigned = text[1:] if negative else text
    whole, dot, fraction = unsigned.partition('.')
    if not whole.isdigit() or (dot and not fraction.isdigit()):
        raise ValueError('Invalid decimal coordinate')
    result = int(whole) * DEGREE_SCALE + int((fraction + '0000000')[:7])
    return -result if negative else result


def parse_sentence(line):
    """Return fields only for a complete ASCII NMEA sentence with a valid XOR."""
    try:
        text = line.decode('ascii').strip()
        if not text.startswith('$') or text.count('*') != 1:
            return None
        payload, checksum = text[1:].split('*')
        if len(checksum) != 2 or any(char not in '0123456789abcdefABCDEF' for char in checksum):
            return None
        actual = 0
        for char in payload:
            actual ^= ord(char)
        if actual != int(checksum, 16):
            return None
        return payload.split(',')
    except (ValueError, UnicodeError):
        return None


def is_rmc(fields):
    return (fields is not None and len(fields[0]) == 5
            and fields[0].endswith('RMC') and fields[0][:2].isalpha())


def _number(text):
    # Reject NaN, infinities, signs, exponent notation and empty fields.
    if not text or text.count('.') > 1 or not text.replace('.', '').isdigit():
        raise ValueError('Invalid NMEA number')
    return float(text)


def _coordinate(text, hemisphere, latitude):
    allowed = 'NS' if latitude else 'EW'
    whole = text.split('.')[0]
    if hemisphere not in allowed or len(hemisphere) != 1:
        raise ValueError('Invalid hemisphere')
    if len(whole) != (4 if latitude else 5):
        raise ValueError('Invalid coordinate width')
    if text.count('.') > 1 or not text.replace('.', '').isdigit():
        raise ValueError('Invalid NMEA coordinate')
    degrees = int(whole[:-2])
    minutes = int(whole[-2:])
    fraction = text.partition('.')[2]
    minute_e7 = minutes * DEGREE_SCALE + int((fraction + '0000000')[:7])
    limit = 90 if latitude else 180
    if minutes >= 60 or degrees > limit or (degrees == limit and minute_e7 != 0):
        raise ValueError('Coordinate out of range')
    value = degrees * DEGREE_SCALE + (minute_e7 + 30) // 60
    return -value if hemisphere in 'SW' else value


def _utc_ms(clock, date):
    whole, dot, fraction = clock.partition('.')
    if (len(whole) != 6 or not whole.isdigit() or
            (dot and not fraction.isdigit()) or len(date) != 6 or not date.isdigit()):
        raise ValueError('Invalid UTC time/date')
    hour, minute, second = int(whole[:2]), int(whole[2:4]), int(whole[4:])
    if hour > 23 or minute > 59 or second > 59:
        raise ValueError('UTC time out of range')
    day, month, year = int(date[:2]), int(date[2:4]), int(date[4:])
    year += 1900 if year >= 80 else 2000
    leap = year % 4 == 0 and (year % 100 != 0 or year % 400 == 0)
    months = (31, 29 if leap else 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31)
    if not 1 <= month <= 12 or not 1 <= day <= months[month - 1]:
        raise ValueError('UTC date out of range')
    prior_year = year - 1
    days = (365 * prior_year + prior_year // 4 - prior_year // 100
            + prior_year // 400 + sum(months[:month - 1]) + day - 1)
    milliseconds = int((fraction + '000')[:3]) if dot else 0
    return days * DAY_MS + ((hour * 60 + minute) * 60 + second) * 1000 + milliseconds


def rmc_fix(fields):
    """Parse checksum-validated RMC fields; void fixes retain their UTC time."""
    if not is_rmc(fields) or len(fields) < 10 or fields[2] not in ('A', 'V'):
        return None
    try:
        timestamp = _utc_ms(fields[1], fields[9])
        # The L76 documents autonomous/differential fixes. Never trigger from
        # estimated/simulator/unknown modes. Legacy RMC can omit the mode field.
        mode = fields[12] if len(fields) > 12 else ''
        valid = fields[2] == 'A' and mode in ('', 'A', 'D')
        if not valid:
            return {'utc_ms': timestamp, 'valid': False}
        return {
            'utc_ms': timestamp,
            'valid': True,
            'lat_e7': _coordinate(fields[3], fields[4], True),
            'lon_e7': _coordinate(fields[5], fields[6], False),
            'speed_mps': _number(fields[7]) * 0.5144444444,
        }
    except (ValueError, OverflowError):
        return None


class NMEAFramer:
    """Bound partial lines and discard oversized sentences through their newline."""

    def __init__(self, max_line=128):
        self.max_line = max_line
        self.reset()

    def reset(self):
        self.buffer = bytearray()
        self.dropping = False

    def feed(self, data):
        lines = []
        for byte in data:
            if byte == 10:
                if self.buffer and not self.dropping:
                    lines.append(bytes(self.buffer))
                self.reset()
            elif byte == 36:  # Resynchronise at the next '$'.
                self.buffer = bytearray(b'$')
                self.dropping = False
            elif not self.dropping:
                if len(self.buffer) >= self.max_line:
                    self.buffer = bytearray()
                    self.dropping = True
                else:
                    self.buffer.append(byte)
        return lines


class DebouncedButton:
    def __init__(self, ticks_diff, debounce_ms=50):
        self.ticks_diff = ticks_diff
        self.debounce_ms = debounce_ms
        self.stable = 1
        self.candidate = 1
        self.changed_at = 0

    def update(self, level, now):
        if level != self.candidate:
            self.candidate = level
            self.changed_at = now
        if (self.candidate != self.stable and
                self.ticks_diff(now, self.changed_at) >= self.debounce_ms):
            self.stable = self.candidate
            return 'pressed' if self.stable == 0 else 'released'
        return None


class FinishLine:
    def __init__(self, coordinates, margin_m=0.0):
        self.lat_e7 = degrees_e7(coordinates['left_lat'])
        self.lon_e7 = degrees_e7(coordinates['left_lon'])
        self.lon_scale = METRES_PER_DEGREE * math.cos(math.radians(self.lat_e7 / DEGREE_SCALE))
        self.dx, self.dy = self.project(degrees_e7(coordinates['right_lat']),
                                       degrees_e7(coordinates['right_lon']))
        self.length = math.sqrt(self.dx * self.dx + self.dy * self.dy)
        if self.length < 0.1:
            raise ValueError('Finish line must have two different endpoints')
        self.margin_m = coordinates.get('margin_m', margin_m)
        if not 0 <= self.margin_m <= 100:
            raise ValueError('Finish-line margin must be between 0 and 100 metres')
        # Optional +1/-1 means the required change in signed side. Unset preserves
        # the original bidirectional behaviour; do not guess track direction.
        self.direction = coordinates.get('direction', 0)
        self.last_trigger_ms = None
        self.armed = False

    def project(self, lat_e7, lon_e7):
        # Subtract integers BEFORE converting to local metre-scale floats.
        return ((lon_e7 - self.lon_e7) * (self.lon_scale / DEGREE_SCALE),
                (lat_e7 - self.lat_e7) * METRES_PER_E7)

    def side(self, point):
        return (self.dx * point[1] - self.dy * point[0]) / self.length

    def crossing_fraction(self, previous, current):
        before, after = self.side(previous), self.side(current)
        if abs(after) < 0.03:  # Allow centimetre-scale integer quantisation.
            after = 0.0
        # Count arrival on the line once, but never collinear travel or departure
        # from an exact on-line fix. Work in metres to avoid degree-scale epsilon.
        if abs(before) < 0.03 or before * after > 0:
            return None
        delta = after - before
        if abs(delta) < 1e-6 or (self.direction and delta * self.direction <= 0):
            return None
        fraction = -before / delta
        if not 0 <= fraction <= 1:
            return None
        x = previous[0] + fraction * (current[0] - previous[0])
        y = previous[1] + fraction * (current[1] - previous[1])
        along = (x * self.dx + y * self.dy) / self.length
        if -self.margin_m - 0.03 <= along <= self.length + self.margin_m + 0.03:
            return fraction
        return None


class LapDetector:
    def __init__(self, tracks, ticks_diff, min_lap_ms=10000, rearm_m=20.0,
                 max_gap_ms=500, min_speed_mps=2.0, max_speed_mps=110.0,
                 end_margin_m=0.0):
        self.lines = [FinishLine(track, end_margin_m) for track in tracks]
        self.ticks_diff = ticks_diff
        self.min_lap_ms = min_lap_ms
        self.rearm_m = rearm_m
        self.max_gap_ms = max_gap_ms
        self.min_speed_mps = min_speed_mps
        self.max_speed_mps = max_speed_mps
        self.latest_utc_ms = None
        self.reset()

    def reset(self, forget_utc=False):
        if forget_utc:
            self.latest_utc_ms = None
        self.previous = None
        self.last_fix_ms = None
        # Break the position segment, but retain an already armed gate. A brief
        # glitch near the finish must not impose another 20 m approach. A fired
        # gate remains disarmed until normal departure/cooldown rearming.

    def _arm(self, fix, now):
        for line in self.lines:
            distance = abs(line.side(line.project(fix['lat_e7'], fix['lon_e7'])))
            required = 2.0 if line.last_trigger_ms is None else self.rearm_m
            cooled = (line.last_trigger_ms is None or
                      self.ticks_diff(now, line.last_trigger_ms) >= self.min_lap_ms)
            if distance >= required and cooled:
                line.armed = True

    def update(self, fix, now):
        if not fix['valid'] or not 0 <= fix['speed_mps'] <= self.max_speed_mps:
            timestamp = fix.get('utc_ms')
            if timestamp is not None and (self.latest_utc_ms is None or timestamp > self.latest_utc_ms):
                self.latest_utc_ms = timestamp
            self.reset()
            return []
        if self.latest_utc_ms is not None and fix['utc_ms'] <= self.latest_utc_ms:
            return []
        # Remember the newest timestamp even across health/continuity resets, so
        # a receiver repeating one frozen fix cannot become "ready" again.
        self.latest_utc_ms = fix['utc_ms']
        previous = self.previous
        if previous is not None:
            elapsed = fix['utc_ms'] - previous['utc_ms']
            if elapsed <= 0:  # Repeated/stale/out-of-order fixes never refresh health.
                return []
            if (elapsed > self.max_gap_ms or
                    self.ticks_diff(now, self.last_fix_ms) > self.max_gap_ms):
                self.reset()
                previous = None
            else:
                north = (fix['lat_e7'] - previous['lat_e7']) * METRES_PER_E7
                east = ((fix['lon_e7'] - previous['lon_e7']) * METRES_PER_E7
                        * math.cos(math.radians(previous['lat_e7'] / DEGREE_SCALE)))
                maximum = self.max_speed_mps * elapsed / 1000 + 10.0
                if north * north + east * east > maximum * maximum:
                    # Do not seed the next segment from a suspected teleport.
                    self.reset()
                    return []
        events = []
        if previous is not None and fix['speed_mps'] >= self.min_speed_mps:
            for index, line in enumerate(self.lines):
                if not line.armed:
                    continue
                if (line.last_trigger_ms is not None and
                        self.ticks_diff(now, line.last_trigger_ms) < self.min_lap_ms):
                    continue
                fraction = line.crossing_fraction(
                    line.project(previous['lat_e7'], previous['lon_e7']),
                    line.project(fix['lat_e7'], fix['lon_e7']))
                if fraction is not None:
                    line.last_trigger_ms = now
                    line.armed = False
                    events.append((index, previous['utc_ms'] + int(fraction * elapsed)))
        self.previous = fix
        self.last_fix_ms = now
        self._arm(fix, now)
        return events
