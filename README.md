# It's Pico Pi Lap Time

A GPS lap trigger for the BMW S1000RR's built-in dash timer. When the bike crosses a configured finish line, a Raspberry Pi Pico sends a 250 ms pulse to the dash, starting the next lap. The dash handles the timing; the Pico tells it when you've crossed the line.

The setup runs automatically when powered on and is designed to sit under the tank cover. Logging is off by default.

## Hardware

- Raspberry Pi Pico running MicroPython
- Waveshare Pico-GPS-L76B with an external GPS antenna
- Waveshare Pico-LCD-1.14
- DC-controlled solid-state relay for the bike's lap-trigger connection

| Pico pin | Connection |
| --- | --- |
| GP0 / GP1 | GPS UART transmit / receive |
| GP2 | Relay control: low at idle, high for 250 ms |
| GP15 | LCD Button A: manual test trigger |
| GP17 | LCD Button B: debug logging |
| GP8 / GP9 | LCD data/command / chip select |
| GP10 / GP11 | LCD SPI clock / data |
| GP12 / GP13 | LCD reset / backlight |

The LCD joystick's UP switch also shares GP2. The firmware uses **Button A** for manual testing.

## Installation

Use Thonny or another MicroPython file-transfer tool to copy these three files to the root of the Pico:

- `main.py` — runs the device
- `lap_logic.py` — GPS parsing and finish-line detection
- `LCD.py` — screen driver

Restart the Pico. All three files are required; no extra Python packages are needed. The map and tests stay on your computer.

## Using it

Power it on and let it acquire GPS. The screen shows `Configuring GPS...`, followed by `GNSS Signal Acquired` when it's ready, or `GNSS Signal Lost` when it doesn't have a usable position. The receiver is configured automatically for 10 position updates per second.

Crossing a finish line sends the dash pulse and flashes the screen twice. If GPS drops out, the device waits for fresh positions and resumes automatically. An eight-second watchdog restarts the Pico if the software stalls. No buttons are needed for startup or recovery.

**Button A** sends an immediate test pulse, even without GPS. Holding it sends one pulse; release it before pressing again. The BMW dash needs rear-wheel movement to accept a trigger and ignores triggers less than ten seconds apart.

**Button B** enables or disables debug logging when held for three seconds and released. Leave it off for normal use. Debug mode is optional.

## Finish lines

The included tracks are:

- Queensland Raceway
- Lakeside Raceway
- Morgan Park, Warwick

Each line works in both directions and extends 10 metres beyond each endpoint to allow for sideways GPS error. Pit-lane crossings are intentionally allowed.

To avoid repeated triggers, the bike must move at least 20 metres away after a crossing, and ten seconds must pass before that line can trigger again. Automatic triggers also require movement above 2 m/s, roughly 7 km/h.

Open `finish-lines-satellite.html` in a browser to check the lines against satellite imagery. It needs internet access. Blue marks the saved line; green marks its extensions. The map's margin slider changes the preview only.

To add or move a line, edit `TRACKS` in `lap_logic.py`. Keep latitude and longitude values in quotes, as they are in the existing entries. Change `FINISH_LINE_MARGIN_M` in `main.py` to adjust the extension used by the device.

## Settings

The settings are near the top of `main.py`. The main ones are:

| Setting | Default | What it controls |
| --- | --- | --- |
| `PULSE_DURATION_MS` | `250` | Length of the dash pulse |
| `FINISH_LINE_MARGIN_M` | `10.0` | Extra metres at each end of a line |
| `MIN_LAP_INTERVAL_MS` | `10000` | Minimum time between triggers on the same line |
| `REARM_DISTANCE_M` | `20.0` | Distance to leave the line before it can trigger again |
| `ENABLE_LOGGING` | `False` | Write debug logs |
| `ENABLE_WATCHDOG` | `True` | Restart automatically after a software stall |

For Thonny or REPL debugging, set `ENABLE_WATCHDOG = False` before restarting, so interrupting the program doesn't cause repeated reboots. Keep it enabled for normal use.

## How the software works

`lap_logic.py` checks GPS messages and detects crossings between consecutive valid positions. Bad data and signal gaps break that sequence so an old position isn't joined to a new one across the finish line.

Automatic triggers estimate where the crossing falls between GPS updates and target a consistent 200 ms delay from that estimate. GPS position and reception delays still affect timing. The Pico's PIO hardware generates the 250 ms pulse independently of Python and screen updates. Manual triggers start immediately.

## Development

Run the tests on your computer from the repository folder:

```sh
python3 -m unittest discover -s tests -v
```

The tests cover finish-line detection, GPS recovery, buttons, pulse timing, logging and display failures. `tests/micropython_smoke.py` adds compatibility checks for a MicroPython build using 32-bit floats.

Board documentation: [GPS module](https://www.waveshare.com/wiki/Pico-GPS-L76B) · [LCD module](https://www.waveshare.com/wiki/Pico-LCD-1.14).

The project is licensed under GPL-3.0. The MicroPython assembler used by the tests has its own MIT licence in `docs/vendor/`.

## Build photos

![Build photo 1](https://github.com/user-attachments/assets/74d867d1-6cb6-4d9a-8565-2f2268c05a29)
![Build photo 2](https://github.com/user-attachments/assets/823b9c84-b33c-4fc5-8cbd-228a627e6ac4)
![Build photo 3](https://github.com/user-attachments/assets/8e38d66f-ded2-4444-a285-9e8a97157921)
![Build photo 4](https://github.com/user-attachments/assets/c53a4583-6757-4913-a776-ef678fb1fe3a)
