from machine import Pin, SPI, UART, PWM, Timer
import utime
import math
from LCD import LCD_1inch14

# Pin configuration
GNSS_TX_PIN = 0
GNSS_RX_PIN = 1
PULSE_PIN = 2
BUTTON_A_PIN = 15

# LCD configuration
LCD_BL_PIN = 13

# GNSS signal loss timeout (in milliseconds)
GNSS_TIMEOUT = 180000  # 3 minutes

# UART configuration
INITIAL_BAUDRATE = 9600
TARGET_BAUDRATE = 115200

# Pulse duration (in milliseconds)
PULSE_DURATION_MS = 250  # Set the desired pulse duration here

class AdaptiveKalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = 1
        self.min_process_variance = 0.001
        self.max_process_variance = 0.1

    def update(self, measurement, velocity):
        self.process_variance = min(max(abs(velocity) * 0.01, self.min_process_variance), self.max_process_variance)
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        return self.estimate

class GPSLapTrigger:
    TRACKS = [
        {'right_lat': -27.690622, 'right_lon': 152.654567, 'left_lat': -27.690622, 'left_lon': 152.654688},
        {'right_lat': -27.228533, 'right_lon': 152.964891, 'left_lat': -27.228441, 'left_lon': 152.964919},
        {'right_lat': -28.262069, 'right_lon': 152.036330, 'left_lat': -28.262086, 'left_lon': 152.036433},
        {'right_lat': -27.435013, 'right_lon': 153.042565, 'left_lat': -27.435171, 'left_lon': 153.042642}
    ]

    def __init__(self):
        self.uart = UART(0, baudrate=INITIAL_BAUDRATE, tx=Pin(GNSS_TX_PIN), rx=Pin(GNSS_RX_PIN))
        self.pulse_pin = Pin(PULSE_PIN, Pin.OUT)
        self.button_a = Pin(BUTTON_A_PIN, Pin.IN, Pin.PULL_UP)
        
        self.display = LCD_1inch14()
        self.pwm = PWM(Pin(LCD_BL_PIN))
        self.pwm.freq(1000)
        self.pwm.duty_u16(32768)  # Set backlight to 50% brightness
        
        self.previous_lat = None
        self.previous_lon = None
        self.last_update_time = None
        self.last_valid_gnss_time = utime.ticks_ms()
        self.state = 'initializing'
        self.lat_filter = AdaptiveKalmanFilter(process_variance=0.01, measurement_variance=0.1)
        self.lon_filter = AdaptiveKalmanFilter(process_variance=0.01, measurement_variance=0.1)
        self.last_lat = None
        self.last_lon = None
        self.last_time = None
        self.debug_mode = False
        self.timer = Timer()

    def send_command(self, command):
        self.uart.write(command + '\r\n')
        utime.sleep_ms(1000)  # Give more time for the command to be processed

    def configure_gnss(self):
        # Set GPS module to output at 10Hz
        self.send_command('$PMTK220,100*2F')
        utime.sleep_ms(1000)  # Wait for 1 second
        
        # Enable GPRMC and GPGGA sentences only
        self.send_command('$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28')
        utime.sleep_ms(1000)  # Wait for 1 second

        self.log_message("GNSS configured for 10Hz updates")

    def parse_gnss_data(self, data: bytes):
        try:
            decoded_data = data.decode('ascii').strip()
            if self.debug_mode:
                self.log_message(f"Received GNSS data: {decoded_data}")
            if decoded_data.startswith('$GNRMC') or decoded_data.startswith('$GPRMC'):
                parts = decoded_data.split(',')
                if len(parts) > 9 and parts[2] == 'A':
                    lat = self._convert_to_degrees(parts[3], parts[4])
                    lon = self._convert_to_degrees(parts[5], parts[6])
                    
                    current_time = utime.ticks_ms()
                    if self.last_lat is not None and self.last_lon is not None and self.last_time is not None:
                        time_diff = utime.ticks_diff(current_time, self.last_time) / 1000  # in seconds
                        lat_velocity = (lat - self.last_lat) / time_diff
                        lon_velocity = (lon - self.last_lon) / time_diff
                        
                        filtered_lat = self.lat_filter.update(lat, lat_velocity)
                        filtered_lon = self.lon_filter.update(lon, lon_velocity)
                    else:
                        filtered_lat = self.lat_filter.update(lat, 0)
                        filtered_lon = self.lon_filter.update(lon, 0)
                    
                    self.last_lat = filtered_lat
                    self.last_lon = filtered_lon
                    self.last_time = current_time
                    self.last_valid_gnss_time = current_time
                    
                    if self.state != 'ready':
                        self.state = 'ready'
                        self.update_display()
                    
                    if self.debug_mode:
                        self.log_message(f"Filtered position: ({filtered_lat}, {filtered_lon})")
                    
                    return filtered_lat, filtered_lon
                else:
                    if self.state != 'signal_lost':
                        self.state = 'signal_lost'
                        self.update_display()
        except Exception as e:
            self.log_message(f"Error parsing GNSS data: {type(e).__name__}: {str(e)}")
        return None, None

    def is_crossing_finish_line(self, lat: float, lon: float) -> (bool, float):
        if self.previous_lat is None or self.previous_lon is None or self.last_update_time is None:
            return False, None
        for track in self.TRACKS:
            if self._line_segments_intersect(
                self.previous_lat, self.previous_lon, lat, lon,
                track['left_lat'], track['left_lon'], track['right_lat'], track['right_lon']
            ):
                crossing_time = self._interpolate_crossing_time(self.previous_lat, self.previous_lon, lat, lon, track)
                if self.debug_mode:
                    self.log_message(f"Finish line crossed! Previous: ({self.previous_lat}, {self.previous_lon}), Current: ({lat}, {lon})")
                return True, crossing_time
        return False, None

    def _line_segments_intersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
        def ccw(ax, ay, bx, by, cx, cy):
            return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)
        
        return (ccw(x1, y1, x3, y3, x4, y4) != ccw(x2, y2, x3, y3, x4, y4) and 
                ccw(x1, y1, x2, y2, x3, y3) != ccw(x1, y1, x2, y2, x4, y4))

    def _interpolate_crossing_time(self, lat1: float, lon1: float, lat2: float, lon2: float, finish_line: dict) -> int:
        dx, dy = lat2 - lat1, lon2 - lon1
        total_distance = math.sqrt(dx**2 + dy**2)
        
        intersection = self._line_intersection(lat1, lon1, lat2, lon2, 
                                              finish_line['left_lat'], finish_line['left_lon'], 
                                              finish_line['right_lat'], finish_line['right_lon'])
        if intersection:
            dx_finish, dy_finish = intersection[0] - lat1, intersection[1] - lon1
            distance_to_finish = math.sqrt(dx_finish**2 + dy_finish**2)
            time_fraction = distance_to_finish / total_distance
            crossing_time = self.last_update_time + int(time_fraction * 100)  # 100ms between updates
            return crossing_time
        return None

    def _line_intersection(self, x1, y1, x2, y2, x3, y3, x4, y4):
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if abs(denom) < 1e-8:
            return None
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        if ua < 0 or ua > 1:
            return None
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
        if ub < 0 or ub > 1:
            return None
        x = x1 + ua * (x2 - x1)
        y = y1 + ua * (y2 - y1)
        return (x, y)

    def _convert_to_degrees(self, raw_value: str, direction: str) -> float:
        if raw_value == '': return None
        value = float(raw_value)
        degrees = int(value / 100)
        minutes = value - (degrees * 100)
        result = degrees + (minutes / 60)
        return -result if direction in 'SW' else result

    def send_pulse(self):
        self.pulse_pin.on()
        utime.sleep_ms(PULSE_DURATION_MS)
        self.pulse_pin.off()
        self.log_message(f"Pulse sent (duration: {PULSE_DURATION_MS}ms)")

    def schedule_pulse(self, delay_ms):
        self.timer.init(mode=Timer.ONE_SHOT, period=delay_ms, callback=self.timed_pulse)

    def timed_pulse(self, t):
        self.pulse_pin.on()
        utime.sleep_ms(PULSE_DURATION_MS)
        self.pulse_pin.off()
        self.log_message(f"Pulse sent at scheduled time (duration: {PULSE_DURATION_MS}ms)")

    def is_button_pressed(self) -> bool:
        if not self.button_a.value():
            utime.sleep_ms(50)
            return not self.button_a.value()
        return False

    def update_display(self):
        try:
            self.display.fill(65535)  # White background
            if self.state == 'initializing':
                self.display.text('Initializing...', 10, 10, 31)  # Blue (RGB565: 0000000000011111)
            elif self.state == 'ready':
                self.display.text('GNSS Signal Acquired', 10, 10, 992)  # Green (RGB565: 0000011111000000)
            elif self.state == 'signal_lost':
                self.display.text('GNSS Signal Lost', 10, 10, 63488)  # Red (RGB565: 1111100000000000)
            elif self.state == 'crossing':
                self.display.text('Crossing Finish Line', 10, 10, 31)  # Blue
            elif self.state == 'button_pressed':
                self.display.text('Button Pressed', 10, 10, 992)  # Green
            self.display.show()
        except Exception as e:
            self.log_message(f"Display update error: {e}")

    def flash_screen(self):
        for _ in range(2):  # Flash twice
            self.display.fill(0)  # 0 represents black
            self.display.show()
            utime.sleep_ms(500)
            self.display.fill(65535)  # 65535 represents white
            self.display.show()
            utime.sleep_ms(500)

    def log_message(self, message: str):
        print(f"[{utime.ticks_ms()}] {message}")
        with open("debug_log.txt", "a") as f:
            f.write(f"[{utime.ticks_ms()}] {message}\n")

    def run(self):
        self.update_display()
        self.configure_gnss()
        buffer = bytearray()

        while True:
            try:
                if self.uart.any():
                    data = self.uart.read()
                    if data:
                        buffer.extend(data)
                        if b'\n' in buffer:
                            lines = buffer.split(b'\n')
                            for line in lines[:-1]:
                                lat, lon = self.parse_gnss_data(line)
                                if lat is not None and lon is not None:
                                    current_time = utime.ticks_ms()

                                    crossed, crossing_time = self.is_crossing_finish_line(lat, lon)
                                    if crossed:
                                        self.state = 'crossing'
                                        self.update_display()
                                        delay = crossing_time - current_time
                                        if delay > 0:
                                            self.schedule_pulse(delay)
                                            self.log_message(f"Pulse scheduled for {delay}ms from now")
                                        else:
                                            self.send_pulse()  # Send immediately if we're already past the crossing time
                                        self.flash_screen()
                                        self.log_message(f"Finish line crossed at interpolated time: {crossing_time}")
                                        self.state = 'ready'
                                        self.update_display()

                                    self.previous_lat, self.previous_lon = lat, lon
                                    self.last_update_time = current_time
                            buffer = lines[-1]
                elif utime.ticks_diff(utime.ticks_ms(), self.last_valid_gnss_time) > GNSS_TIMEOUT:
                    if self.state != 'signal_lost':
                        self.state = 'signal_lost'
                        self.update_display()
                        self.log_message("GNSS signal lost")

                if self.is_button_pressed():
                    self.state = 'button_pressed'
                    self.update_display()
                    self.send_pulse()
                    self.flash_screen()
                    self.log_message("Button pressed, pulse sent")
                    utime.sleep_ms(500)  # Debounce
                    self.state = 'ready'
                    self.update_display()

            except Exception as e:
                self.log_message(f"Error in main loop: {type(e).__name__}: {str(e)}")
                utime.sleep_ms(1000)  # Wait a bit before continuing

if __name__ == "__main__":
    gps_trigger = GPSLapTrigger()
    gps_trigger.run()
