"""MicroPython firmware for the Pico -> S1000RR lap trigger."""

from machine import Pin, UART, PWM, WDT
import rp2
import utime
import os
from array import array

from lap_logic import TRACKS, NMEAFramer, DebouncedButton, LapDetector
from lap_logic import parse_sentence, is_rmc, rmc_fix

# Existing wiring and dash pulse: deliberately unchanged.
GNSS_TX_PIN = 0
GNSS_RX_PIN = 1
PULSE_PIN = 2
BUTTON_A_PIN = 15
BUTTON_B_PIN = 17
LCD_BL_PIN = 13
PULSE_DURATION_MS = 250
AUTOMATIC_PULSE_LATENCY_MS = 200  # Crossing interpolation + constant target delay.
UART_BACKLOG_THRESHOLD = 512  # One arriving byte is normal, not a stale burst.

INITIAL_BAUDRATE = 9600
TARGET_BAUDRATE = 38400
PROBE_BAUDRATES = (38400, 9600, 115200, 57600, 19200, 14400, 4800)
GNSS_TIMEOUT_MS = 1500
MIN_LAP_INTERVAL_MS = 10000  # Per finish line; manual tests bypass this.
REARM_DISTANCE_M = 20.0
FINISH_LINE_MARGIN_M = 10.0  # Extend both ends; pit-lane crossings are wanted.
MAX_FIX_GAP_MS = 500
MIN_TRIGGER_SPEED_MPS = 2.0
MAX_SPEED_MPS = 110.0
ENABLE_WATCHDOG = True  # Set False while debugging at the MicroPython REPL.
ENABLE_LOGGING = False  # No file writes by default; B can explicitly enable debug.
WATCHDOG_TIMEOUT_MS = 8000
PULSE_SM_ID = 0
PULSE_SM_FREQ = 1000000


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def _dash_pulse_program():
    pull(block)
    mov(y, osr)
    label('delay')
    jmp(y_dec, 'delay')
    pull(block)
    mov(x, osr)
    set(pins, 1)
    label('high')
    jmp(x_dec, 'high')
    set(pins, 0)
    push(noblock)  # Completion marker only after the pin is low; never stall.


class DebugLog:
    """Small event queue, bounded disk rotation, and non-fatal write failures."""

    def __init__(self, enabled=ENABLE_LOGGING):
        self.enabled = enabled
        self.pending = []
        self.file_number = 0
        self.max_files = 5
        self.max_bytes = 16384
        self.disk_enabled = True
        self.last_flush = utime.ticks_ms()

    def queue(self, message):
        if not self.enabled:
            return
        if len(self.pending) >= 16:
            self.pending.pop(0)
        self.pending.append('[%d] %s\n' % (utime.ticks_ms(), str(message)[:192]))

    def flush(self, now):
        if not self.enabled or not self.pending or utime.ticks_diff(now, self.last_flush) < 1000:
            return
        self.last_flush = now
        payload = ''.join(self.pending)
        self.pending = []
        if not self.disk_enabled:
            return
        try:
            filename = 'debug_log_%d.txt' % self.file_number
            try:
                size = os.stat(filename)[6]
            except OSError:
                size = 0
            mode = 'a'
            if size + len(payload.encode('utf-8')) > self.max_bytes:
                self.file_number = (self.file_number + 1) % self.max_files
                filename = 'debug_log_%d.txt' % self.file_number
                mode = 'w'  # Truncate the reused slot, including after reboot.
            with open(filename, mode) as handle:
                handle.write(payload)
        except Exception:
            # A full/read-only filesystem must not stop GPS processing or pulses.
            self.disk_enabled = False


class GNSSLink:
    """Nonblocking baud discovery, PMTK setup, and measured 10 Hz verification."""

    def __init__(self, owner):
        self.owner = owner
        self.ready = False
        self.state = 'probe'
        self.baud = TARGET_BAUDRATE
        self.preferred_baud = None  # Established by an actually verified stream.
        self.probe_index = 0
        self.deadline = utime.ticks_add(utime.ticks_ms(), 1500)
        self.last_sentence_ms = None
        self.last_progress_ms = None
        self.attempt = 0
        self.uart = None
        self._reset_rate()
        self._open(self.baud)

    def _reset_rate(self):
        self.rate_last_utc = None
        self.rate_first_ms = None
        self.rate_first_utc = None
        self.rate_intervals = 0
        self.bad_rate_intervals = 0

    def _open(self, baud):
        if self.uart is not None:
            self.uart.deinit()
        self.baud = baud
        self.uart = UART(0, baudrate=baud, tx=Pin(GNSS_TX_PIN), rx=Pin(GNSS_RX_PIN),
                         rxbuf=4096, timeout=0, timeout_char=0)
        self.owner.framer.reset()
        self.owner.detector.reset()
        self.config_acks = {'220': False, '314': False}
        self.probe_extended = False

    def _write(self, command):
        written = self.uart.write(command)
        if written != len(command):
            raise OSError('Incomplete GNSS command write')

    def _start_commands(self, now):
        self.ready = False
        self._reset_rate()
        self.owner.detector.reset()
        self.config_acks = {'220': False, '314': False}
        # If UART.write fails while leaving 'ready', keep a recoverable state
        # rather than stranding automatic triggering until the next reboot.
        self.state = 'retry_wait'
        self.deadline = utime.ticks_add(now, 1000)
        self._write(b'$PMTK220,100*2F\r\n')
        self.state = 'output_command'
        self.deadline = utime.ticks_add(now, 100)

    def consume(self, fields, fix, now):
        if fields[0] == 'PMTK001':
            if len(fields) >= 3 and fields[1] in ('220', '314'):
                accepted = fields[2] == '3'
                self.config_acks[fields[1]] = accepted
                if not accepted:
                    self.ready = False
                    self.owner.detector.reset()
                    if self.state in ('verify', 'ready'):
                        self.state = 'verify'
                        self.deadline = now
                    self.owner.log_message('GNSS rejected PMTK command %s' % fields[1], True)
            return
        if not is_rmc(fields):
            return
        self.last_sentence_ms = now
        if self.state == 'probe':
            self.attempt = 0
            if self.baud != TARGET_BAUDRATE:
                self._write(b'$PMTK251,38400*27\r\n')
                self.state = 'baud_wait'
                self.deadline = utime.ticks_add(now, 200)
            else:
                self._start_commands(now)
            return
        if self.state == 'ready':
            if fix is not None and fix['utc_ms'] > self.rate_last_utc:
                interval = fix['utc_ms'] - self.rate_last_utc
                self.rate_last_utc = fix['utc_ms']
                self.last_progress_ms = now
                if 80 <= interval <= 120:
                    self.bad_rate_intervals = 0
                else:
                    self.bad_rate_intervals += 1
                if self.bad_rate_intervals >= 3:
                    self.owner.log_message('GNSS update rate changed; reconfiguring', True)
                    self._start_commands(now)
            return
        if self.state != 'verify' or fix is None:
            return
        timestamp = fix['utc_ms']
        if self.rate_last_utc is None:
            self.rate_first_ms = now
            self.rate_first_utc = timestamp
        else:
            interval = timestamp - self.rate_last_utc
            if interval == 0:
                return
            if 80 <= interval <= 120:
                self.rate_intervals += 1
            else:
                self.rate_first_ms = now
                self.rate_first_utc = timestamp
                self.rate_intervals = 0
        self.rate_last_utc = timestamp
        # Check both GNSS sample spacing and elapsed receive time. Buffered bursts,
        # repeated sentences, and a 1 Hz stream cannot masquerade as 10 Hz.
        wall_span = utime.ticks_diff(now, self.rate_first_ms)
        gnss_span = timestamp - self.rate_first_utc
        if (self.rate_intervals >= 10 and 800 <= wall_span <= 1500
                and abs(gnss_span - wall_span) <= 200
                and all(self.config_acks.values())):
            self.ready = True
            self.state = 'ready'
            self.preferred_baud = self.baud
            self.probe_index = PROBE_BAUDRATES.index(self.baud)
            self.last_progress_ms = now
            # A freshly verified advancing stream can establish a new UTC epoch
            # after a receiver restart/time correction. Keep lap cooldowns.
            self.owner.detector.reset(forget_utc=True)
            self.owner.log_message('GNSS verified at 10 Hz / 38400 baud', True)

    def step(self, now):
        if self.ready:
            if ((self.last_sentence_ms is not None and
                    utime.ticks_diff(now, self.last_sentence_ms) > 3000) or
                    (self.last_progress_ms is not None and
                     utime.ticks_diff(now, self.last_progress_ms) > 3000)):
                self.ready = False
                self.owner.detector.reset()
                self.state = 'probe'
                self.probe_extended = False
                # Keep the working UART and any returning bytes. Give this
                # baud a fresh window before trying alternatives.
                self.deadline = utime.ticks_add(now, 1500)
            else:
                return
        if utime.ticks_diff(now, self.deadline) < 0:
            return
        if self.state == 'probe':
            buffer = self.owner.framer.buffer
            if not self.probe_extended and buffer and buffer[0] == 36:
                # Finish one partial packet at the deadline before switching.
                # Even a 128-byte sentence at 4800 baud fits in 300 ms. This
                # grace is granted once, so garbage cannot pin the baud search.
                self.probe_extended = True
                self.deadline = utime.ticks_add(now, 300)
                return
            if self.preferred_baud is not None and self.baud != self.preferred_baud:
                baud = self.preferred_baud
            else:
                self.probe_index = (self.probe_index + 1) % len(PROBE_BAUDRATES)
                if PROBE_BAUDRATES[self.probe_index] == self.preferred_baud:
                    self.probe_index = (self.probe_index + 1) % len(PROBE_BAUDRATES)
                baud = PROBE_BAUDRATES[self.probe_index]
            # After a prolonged outage, revisit the known speed between each
            # alternative instead of abandoning it for a full seven-rate sweep.
            self._open(baud)
            self.deadline = utime.ticks_add(now, 1500)
        elif self.state == 'baud_wait':
            self._open(TARGET_BAUDRATE)
            self._start_commands(now)
        elif self.state == 'output_command':
            self._write(b'$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n')
            self.state = 'verify'
            self._reset_rate()
            self.deadline = utime.ticks_add(now, 3000)
        elif self.state == 'verify':
            self.attempt += 1
            if self.attempt < 3:
                self._start_commands(now)
            else:
                self.owner.log_message('GNSS setup failed; manual trigger remains available', True)
                self.state = 'retry_wait'
                self.deadline = utime.ticks_add(now, 5000)
        elif self.state == 'retry_wait':
            self.state = 'probe'
            self.probe_index = 0
            self._open(TARGET_BAUDRATE)
            self.deadline = utime.ticks_add(now, 1500)


class GPSLapTrigger:
    TRACKS = TRACKS

    def __init__(self):
        # Explicit inactive level at boot. GP2 remains the original output; do
        # not reconfigure it as a joystick input or change the existing wiring.
        self.pulse_pin = Pin(PULSE_PIN, Pin.OUT, value=0)
        self.button_a = Pin(BUTTON_A_PIN, Pin.IN, Pin.PULL_UP)
        self.button_b = Pin(BUTTON_B_PIN, Pin.IN, Pin.PULL_UP)
        self.pulse_sm = rp2.StateMachine(PULSE_SM_ID)
        self.pulse_words = array('I', (0, PULSE_DURATION_MS * 1000 - 2))
        self.pulse_active = False
        self.pulse_started_ms = None
        # Cover startup peripherals as well as the running loop. Normal boot
        # requires no physical button access under the tank.
        self.watchdog = WDT(timeout=WATCHDOG_TIMEOUT_MS) if ENABLE_WATCHDOG else None
        self.debug_mode = False
        self.log = DebugLog()
        self.framer = NMEAFramer()
        self.discarding_backlog = False
        self.detector = LapDetector(
            self.TRACKS, utime.ticks_diff, min_lap_ms=MIN_LAP_INTERVAL_MS,
            rearm_m=REARM_DISTANCE_M, max_gap_ms=MAX_FIX_GAP_MS,
            min_speed_mps=MIN_TRIGGER_SPEED_MPS, max_speed_mps=MAX_SPEED_MPS,
            end_margin_m=FINISH_LINE_MARGIN_M)
        self.debounce_a = DebouncedButton(utime.ticks_diff)
        self.debounce_b = DebouncedButton(utime.ticks_diff)
        self.debug_press_ms = None
        self.state = 'initializing'
        self.flash_start_ms = None
        self.display_signature = None
        self.display = None
        self.pwm = None
        self.link = GNSSLink(self)
        self._init_display()
        self.log_message('Lap trigger started', True)

    def _init_display(self):
        try:
            # Missing or failed LCD support must not prevent the dash trigger.
            from LCD import LCD_1inch14
            self.display = LCD_1inch14()
            self.pwm = PWM(Pin(LCD_BL_PIN))
            self.pwm.freq(1000)
            self.pwm.duty_u16(32768)
        except Exception as error:
            self.display = None
            self.log_message('LCD unavailable: %s' % error, True)

    def log_message(self, message, force_log=False):
        if self.log.enabled and (self.debug_mode or force_log):
            self.log.queue(message)

    def stop_pulse(self):
        self.pulse_sm.active(0)
        # Disabling a PIO machine freezes its output. Switch GP2 back to SIO
        # explicitly low instead of assuming Pin.off() overrides the PIO mux.
        self.pulse_pin.init(Pin.OUT, value=0)
        self.pulse_active = False

    def send_pulse(self, delay_ms=0):
        now = utime.ticks_ms()
        target_start = utime.ticks_add(now, delay_ms)
        self._service_pulse(now)
        if (self.pulse_active and delay_ms == 0 and
                utime.ticks_diff(now, self.pulse_started_ms) < 0 and
                not self.pulse_pin.value()):
            # A manual test takes priority over an automatic pulse still waiting
            # to start. Never truncate a high pulse already reaching the dash.
            self.stop_pulse()
        if self.pulse_active:
            return False  # Never extend or overlap a 250 ms pulse.
        try:
            self.pulse_sm.active(0)
            # init clears FIFO/state and sets GP2 low before every request.
            self.pulse_sm.init(_dash_pulse_program, freq=PULSE_SM_FREQ, set_base=self.pulse_pin)
            self.pulse_sm.active(1)
            queue_time = utime.ticks_ms()
            remaining_ms = max(0, utime.ticks_diff(target_start, queue_time))
            self.pulse_started_ms = utime.ticks_add(queue_time, remaining_ms)
            self.pulse_active = True
            # SET high + (X+1) JMP cycles = X+2 cycles high. At 1 MHz this is
            # nominally 250 ms, independent of Python, LCD and filesystem stalls.
            # Both FIFO words are queued in one C call. Hardware handles the
            # wait as well as the pulse even if the interpreter then stalls.
            # SET high occurs delay_word+5 cycles after the first PULL.
            self.pulse_words[0] = max(0, remaining_ms * 1000 - 5)
            self.pulse_sm.put(self.pulse_words)
            # Anchor the software backup after the handoff. A VM pause before
            # put() must not make its earlier clock estimate truncate the high.
            self.pulse_started_ms = utime.ticks_add(utime.ticks_ms(), remaining_ms)
        except Exception:
            self.stop_pulse()
            raise
        self.log_message('Pulse queued (%d ms delay, %d ms high)' %
                         (delay_ms, PULSE_DURATION_MS))
        return True

    def _service_pulse(self, now):
        if self.pulse_active:
            elapsed = utime.ticks_diff(now, self.pulse_started_ms)
            if self.pulse_sm.rx_fifo():
                # Hardware finished, even if Python missed the entire high
                # interval. init() clears this marker before the next request.
                self.stop_pulse()
            elif elapsed >= PULSE_DURATION_MS + 5:
                self.stop_pulse()  # Backup for a malfunctioning PIO output.

    def flash_screen(self):
        # Start the same two black/white flashes after the pulse, without blocking.
        self.flash_start_ms = utime.ticks_add(self.pulse_started_ms, PULSE_DURATION_MS)

    def toggle_debug_mode(self):
        self.debug_mode = not self.debug_mode
        self.log.enabled = ENABLE_LOGGING or self.debug_mode
        if not self.log.enabled:
            self.log.pending = []
        self.log_message('Debug mode %s' % ('enabled' if self.debug_mode else 'disabled'), True)

    def _service_buttons(self, now):
        if self.debounce_a.update(self.button_a.value(), now) == 'pressed':
            if self.send_pulse():
                self.flash_screen()
                self.log_message('Manual test trigger', True)
        event = self.debounce_b.update(self.button_b.value(), now)
        if event == 'pressed':
            self.debug_press_ms = now
        elif event == 'released' and self.debug_press_ms is not None:
            if utime.ticks_diff(now, self.debug_press_ms) >= 3000:
                self.toggle_debug_mode()
            self.debug_press_ms = None

    def _poll_gnss(self):
        latest = None
        # Bounded work per pass so buttons/timers keep being serviced. Use the
        # newest complete fix, rather than emitting pulses for a stale backlog.
        for unused in range(4):
            available = self.link.uart.any()
            if not available:
                break
            data = self.link.uart.read(min(available, 512))
            if not data:
                break
            for line in self.framer.feed(data):
                fields = parse_sentence(line)
                if fields is None:
                    continue
                fix = rmc_fix(fields)
                now = utime.ticks_ms()
                was_ready = self.link.ready
                self.link.consume(fields, fix, now)
                if was_ready and self.link.ready and is_rmc(fields):
                    if fix is None or not fix['valid']:
                        # A void fix breaks continuity even if a later fix in the
                        # same UART read is valid again.
                        if fix is None:
                            self.detector.reset()
                        else:
                            self.detector.update(fix, now)
                        latest = None
                        self.state = 'signal_lost'
                    elif latest is None or fix['utc_ms'] > latest['utc_ms']:
                        latest = fix
        if self.link.uart.any() > UART_BACKLOG_THRESHOLD:
            self.discarding_backlog = True
            self.detector.reset()
            latest = None
        elif self.discarding_backlog:
            # Finish draining the old burst, then wait for a newly received fix.
            self.discarding_backlog = False
            self.framer.reset()
            self.detector.reset()
            latest = None
        if latest is not None:
            now = utime.ticks_ms()
            for index, crossing_utc in self.detector.update(latest, now):
                # Timestamp differences stay integer-valued on float32 Pico.
                # A constant post-crossing offset cancels between lap pulses,
                # compensating sample phase while leaving receiver delay/noise.
                age_ms = latest['utc_ms'] - crossing_utc
                delay_ms = max(0, AUTOMATIC_PULSE_LATENCY_MS - age_ms)
                if age_ms > AUTOMATIC_PULSE_LATENCY_MS:
                    self.log_message('Late crossing: %d ms sample age; pulse immediately' % age_ms, True)
                if self.send_pulse(delay_ms):
                    self.flash_screen()
                    self.log_message('Finish line %d crossed; UTC ms %d' %
                                     (index + 1, crossing_utc), True)
            if self.detector.last_fix_ms == now:
                self.state = 'ready'
            elif self.detector.last_fix_ms is None:
                self.state = 'signal_lost'

    def _service_health(self, now):
        last_fix = self.detector.last_fix_ms
        if last_fix is not None and utime.ticks_diff(now, last_fix) > GNSS_TIMEOUT_MS:
            self.detector.reset()
            self.state = 'signal_lost'
            self.log_message('GNSS signal lost', True)
        if not self.link.ready and self.state == 'ready':
            self.state = 'signal_lost'

    def update_display(self, now):
        # Keep SPI transfers and flash writes out of the active pulse interval.
        if self.display is None or self.pulse_active:
            return
        phase = None
        if self.flash_start_ms is not None:
            elapsed = utime.ticks_diff(now, self.flash_start_ms)
            if elapsed >= 2000:
                self.flash_start_ms = None
            elif elapsed >= 0:
                phase = elapsed // 500
        signature = (self.state, self.link.ready, self.debug_mode, phase)
        if signature == self.display_signature:
            return
        try:
            if phase is not None:
                self.display.fill(0 if phase % 2 == 0 else 65535)
            else:
                self.display.fill(65535)
                if not self.link.ready:
                    self.display.text('Configuring GPS...', 10, 10, self.display.blue)
                elif self.state == 'ready':
                    self.display.text('GNSS Signal Acquired', 10, 10, self.display.green)
                else:
                    self.display.text('GNSS Signal Lost', 10, 10, self.display.red)
                if self.debug_mode:
                    self.display.text('DEBUG', 10, 50, self.display.red)
            self.display.show()
            self.display_signature = signature
        except Exception as error:
            self.display = None
            self.log_message('LCD disabled after error: %s' % error, True)

    def step(self):
        now = utime.ticks_ms()
        self._service_pulse(now)
        self._service_buttons(now)
        self._service_health(now)
        polled = False
        if self.link.state == 'probe' and self.link.uart.any():
            # Read returning packets before an expired probe closes their UART.
            self._poll_gnss()
            polled = True
            now = utime.ticks_ms()
        self.link.step(now)
        if not polled:
            self._poll_gnss()
        now = utime.ticks_ms()
        self._service_pulse(now)
        self._service_health(now)
        self.update_display(now)
        if not self.pulse_active and not self.link.uart.any():
            self.log.flush(now)

    def run(self):
        watchdog = self.watchdog
        try:
            while True:
                try:
                    self.step()
                    if watchdog is not None:
                        watchdog.feed()
                except Exception as error:
                    # An unrelated GPS error must not shorten an active dash
                    # pulse. PIO finishes independently; the deadline is backup.
                    self._service_pulse(utime.ticks_ms())
                    self.detector.reset()
                    self.framer.reset()
                    self.state = 'signal_lost'
                    self.log_message('Loop error: %s: %s' % (type(error).__name__, error), True)
                    if not self.pulse_active:
                        self.log.flush(utime.ticks_ms())
                    # Repeated failures deliberately stop feeding the watchdog.
                utime.sleep_ms(5)
        finally:
            self.stop_pulse()


if __name__ == '__main__':
    GPSLapTrigger().run()
