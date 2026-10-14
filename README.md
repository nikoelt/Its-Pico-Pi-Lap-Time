# It's Pico Pi Lap Time

GPS lap triggering for the BMW S1000RR's built-in dash timer, using a Raspberry
Pi Pico, Waveshare Pico-GPS-L76B and Pico-LCD-1.14. GP2 drives the existing
solid-state relay interface with a **250 ms high pulse**, then returns low.

The firmware starts automatically when powered. Logging is **off by default**,
and an eight-second watchdog recovers from software stalls without needing
access to the buttons under the tank.

## Install

Copy exactly these three files to the Pico's filesystem root, then restart it:

- `main.py`
- `lap_logic.py` — required by the updated firmware
- `LCD.py`

No additional Python packages are required on the Pico. Keep the desktop tests,
map, documentation and licence files on the computer. This update replaces the
old single-file firmware; copying only `main.py` is insufficient.

The updated code has desktop regression and host MicroPython compatibility
coverage. It has not yet been flashed to or measured on this project's actual
Pico or bike. Keep a copy of the firmware currently on your device for rollback.

## Hardware and controls

| Connection | Function |
| --- | --- |
| GP0 / GP1 | UART0 transmit / receive for the L76B |
| GP2 | Solid-state relay control; idle low, high for 250 ms |
| Button A / GP15 | Immediate manual test pulse, independent of GPS |
| Button B / GP17 | Hold three seconds, then release to toggle debug logging |
| GP8 / GP9 | LCD data/command / chip select |
| GP10 / GP11 | LCD SPI1 clock / MOSI |
| GP12 / GP13 | LCD reset / backlight |

The original bike interface uses a DC-controlled solid-state relay and the
existing 4.7 kΩ resistor connection to the K46 lap-trigger plug. GP2 controls
the relay input. This software update does not change that wiring.

The joystick UP connection on the LCD board shares GP2. Firmware keeps GP2 as
an output, matching the existing setup; the software manual trigger is Button
A. A held A sends one pulse per press. Another request during a high pulse is
ignored. A manual request can replace an automatic pulse still waiting to start.
The two screen flashes remain, while GPS processing continues.

The owner reports that the BMW dash ignores triggers less than ten seconds
apart and requires rear-wheel movement to accept them. The relay/output check
and actual dash acceptance are separate parts of hardware validation.

## Finish lines

The public configuration includes the existing endpoints for:

1. Queensland Raceway
2. Lakeside Raceway
3. Morgan Park, Warwick

All three gates are bidirectional. Each extends **10 metres past both endpoints**
to accommodate lateral GNSS error. Pit-lane activation is intentionally allowed.

Edit `TRACKS` in `lap_logic.py` to change a gate or add your own local test line.
Coordinate values must be **quoted decimal strings**. They are converted to
integer units of 1e-7 degrees before calculating local metre differences,
avoiding the Pico's 32-bit float rounding of full longitude/latitude values.
The existing public circuit coordinates have not been resurveyed.

A crossing requires two fresh valid positions and movement above 2 m/s. A gate
initially arms at least two metres from the line. After triggering, it requires
20 m departure and a ten-second cooldown. A glitch clears the position segment
while preserving an already armed gate, so recovery near the line does not
require another 20 m approach. No crossing is drawn from a stale pre-dropout
position to the first recovered fix.

For a known direction, add a numeric `direction` field of `1` or `-1` to the gate.
It controls increasing/decreasing signed side of the oriented left-to-right
line. Verify its sign before enabling it. A per-gate numeric `margin_m` field
overrides the global extension; zero keeps the original segment length.

`finish-lines-satellite.html` displays the three public gates. Open it in a
browser with internet access for Esri satellite imagery. Blue shows the saved
endpoints; green shows the extension. The margin slider changes the preview,
not the firmware. Morgan Park uses the available zoom-17 imagery when zoomed
further in. Satellite imagery alignment is not a survey of painted stripes.

## GPS setup and recovery

Setup searches all seven documented UART rates, trying 38400 and 9600 first,
then configures 38400 baud, 100 ms fixes and RMC-only output. Automatic pulses
remain disabled until both PMTK settings are acknowledged and an advancing
approximately 10 Hz stream is verified against elapsed receive time.

Checksum, position-validity status, mode, coordinates, UTC date/time and speed
are validated. Raw positions replace the old lagging filter. Void/malformed
fixes, implausible jumps and gaps break crossing continuity. Repeated or
out-of-order timestamps do not refresh position health.

A silent/frozen stream retries the known working baud first, revisiting it
between alternatives. Returning bytes are read before switching baud; a partial
packet gets one bounded 300 ms grace period. Ordinary satellite loss with
advancing void RMC messages waits for a valid fix without starting a baud search.
Retries run in the background, and the manual trigger remains available.

## Pulse timing

PIO state machine 0 generates the wait and pulse independently of Python.
The high interval is **250,000 cycles at 1 MHz**. After setting the pin low, PIO
publishes a nonblocking completion marker so a Python pause cannot leave the
software waiting on an already finished pulse. The loop deadline is a backup;
reinitialization clears the FIFO before the next request.

Automatic pulses target a constant **200 ms after the interpolated crossing**,
relative to receipt of the current sample. If the crossing was 70 ms before that
sample, hardware waits another 130 ms. That consistent offset cancels between
consecutive lap pulses and reduces sampling-phase error. Manual pulses start
immediately. A crossing already older than the target delay pulses immediately.

Interpolation estimates the crossing. GNSS position error and varying receiver
or UART delay remain; there is no measured on-bike accuracy claim. A pulse
already queued in PIO ends independently of display, filesystem and Python
stalls. LCD updates are deferred while a pulse is pending or high.

## Settings and fault handling

The main settings are near the top of `main.py`:

| Setting | Default |
| --- | --- |
| `PULSE_DURATION_MS` | 250 ms |
| `AUTOMATIC_PULSE_LATENCY_MS` | 200 ms |
| `FINISH_LINE_MARGIN_M` | 10 m per end |
| `MIN_LAP_INTERVAL_MS` | 10000 ms per gate |
| `REARM_DISTANCE_M` | 20 m |
| `MAX_FIX_GAP_MS` | 500 ms |
| `GNSS_TIMEOUT_MS` | 1500 ms |
| `MIN_TRIGGER_SPEED_MPS` | 2 m/s |
| `MAX_SPEED_MPS` | 110 m/s |
| `ENABLE_LOGGING` | `False` |
| `ENABLE_WATCHDOG` | `True` |

With logging disabled, there are no log entries, file writes or log-file size
checks, including for startup and errors. Explicitly enabling it with the
setting or Button B uses a maximum of five 16 KiB files and sixteen queued
messages. Full/read-only storage disables disk logging without stopping triggers.
Old oversized logs from earlier firmware can be removed when servicing the Pico.

A failed/missing LCD does not prevent triggers. The watchdog covers peripheral
startup and the main loop; repeated loop failures stop feeding it. A healthy
loop without GPS keeps running. **For REPL/Thonny servicing, set
`ENABLE_WATCHDOG = False` before the next restart**: an already running hardware
watchdog cannot be stopped. Normal startup requires no button hold.

The resource budget is bounded: one 64,800-byte LCD framebuffer, a 4,096-byte
UART receive buffer, 128-byte partial NMEA lines, and at most four 512-byte UART
reads per loop. Screen transfers happen only when status or flash phase changes.
The pulse-word array is allocated once at startup. There are no per-fix disk
writes or Python background threads.

## Validation

From the repository root on a computer:

```sh
python3 -m unittest discover -s tests -v
```

The 65 tests cover the three configured circuits, geometry/endpoints/margins,
UTC rollover, signal gaps, frozen/reordered data, jumps, checksums, baud discovery
and acknowledgements, recovery, button debounce, PIO faults and completion,
clock wrap, LCD failure, disabled logging, bounded enabled logs and watchdog
feeding. A deterministic 500-step replay combines corrupted/split GPS packets,
button presses and CPU pauses while checking pulse-width and logging invariants.

The actual MicroPython RP2 assembler encodes the pulse program in the desktop
suite. An instruction-level cycle check verifies the 250,000-cycle high and
completion strictly after its falling edge. The vendored assembler retains its
MIT licence under `docs/vendor/`; the project remains GPL-3.0.

`tests/micropython_smoke.py` additionally checks parser/geometry, the real RGB565
framebuffer, LCD reset/window/SPI cleanup, assembler and firmware startup in a
host MicroPython v1.24.0 build using `MICROPY_FLOAT_IMPL_FLOAT`. It requires that
float32 build; a default desktop float64 build will not satisfy the test.
Peripheral stubs are used, so this does not measure the physical Pico or dash.

## Vendor references

- [Waveshare Pico-GPS-L76B](https://www.waveshare.com/wiki/Pico-GPS-L76B)
- [Quectel L76 GNSS protocol](https://files.waveshare.com/upload/5/56/Quectel_L76_Series_GNSS_Protocol_Specification_V3.3.pdf)
- [Waveshare Pico-LCD-1.14](https://www.waveshare.com/wiki/Pico-LCD-1.14)
- [ST7789VW controller](https://files.waveshare.com/wiki/common/ST7789VW.pdf)
- [MicroPython PIO API](https://docs.micropython.org/en/v1.24.0/library/rp2.StateMachine.html)

## Hardware photos

![IMG_1812](https://github.com/user-attachments/assets/74d867d1-6cb6-4d9a-8565-2f2268c05a29)
![IMG_1814](https://github.com/user-attachments/assets/823b9c84-b33c-4fc5-8cbd-228a627e6ac4)
![IMG_1813](https://github.com/user-attachments/assets/8e38d66f-ded2-4444-a285-9e8a97157921)
![IMG_1054](https://github.com/user-attachments/assets/c53a4583-6757-4913-a776-ef678fb1fe3a)

