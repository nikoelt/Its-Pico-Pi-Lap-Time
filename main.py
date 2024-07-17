from machine import Pin, SPI, UART, PWM, Timer
import utime
import math
import os
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
TARGET_BAUDRATE = 38400

# Pulse duration (in milliseconds)
PULSE_DURATION_MS = 250

# Maximum buffer size (in bytes)
MAX_BUFFER_SIZE = 1024

# LCD update interval (in milliseconds)
LCD_UPDATE_INTERVAL = 1000

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
            {'right_lat': -27.435013, 'right_lon': 153.042565, 'left_lat': -27.435171, 'left_lon': 153.042642},
            {'right_lat': -27.453118, 'right_lon': 153.043510, 'left_lat': -27.453697, 'left_lon': 153.043340},
            {'right_lat': -27.453907, 'right_lon': 153.046570, 'left_lat': -27.454613, 'left_lon': 153.046314},
            {'right_lat': -27.452388, 'right_lon': 153.048588, 'left_lat': -27.452975, 'left_lon': 153.048018}
        ]



    def __init__(self):
            # Initialize GPIO first
            self.pulse_pin = Pin(PULSE_PIN, Pin.OUT)
            self.button_a = Pin(BUTTON_A_PIN, Pin.IN, Pin.PULL_UP)

            # LCD initialization
            self.reset_lcd()

            # UART initialization
            self.uart = UART(0, baudrate=INITIAL_BAUDRATE, tx=Pin(GNSS_TX_PIN), rx=Pin(GNSS_RX_PIN))

            # Other initializations
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
            self.last_lcd_update = utime.ticks_ms()

            # Logging-related attributes
            self.log_file_number = 0
            self.log_line_count = 0
            self.max_log_lines = 1000
            self.max_log_files = 5
            self.log_interval = 1000
            self.last_log_time = utime.ticks_ms()

            # Initial display update
            self.update_display()

    def reset_lcd(self):
        try:
            # Reset the SPI bus
            spi = SPI(1, baudrate=40000000, polarity=0, phase=0)
            spi.deinit()
            utime.sleep_ms(50)

            # Reinitialize the LCD
            self.display = LCD_1inch14()
            utime.sleep_ms(100)  # Allow LCD to initialize

            # Reset and reconfigure the backlight PWM
            if hasattr(self, 'pwm'):
                self.pwm.deinit()
            self.pwm = PWM(Pin(LCD_BL_PIN))
            self.pwm.freq(1000)
            self.pwm.duty_u16(32768)  # Set backlight to 50% brightness

            # Clear the display and show a reset message
            self.display.fill(0)  # Black background
            self.display.text('LCD Reset', 10, 10, 65535)  # White text
            self.display.show()
            utime.sleep_ms(1000)  # Show reset message for 1 second
        except Exception as e:
            print(f"LCD reset error: {e}")



    def log_message(self, message: str):
        current_time = utime.ticks_ms()
        if self.log_line_count >= self.max_log_lines:
            self.log_file_number = (self.log_file_number + 1) % self.max_log_files
            self.log_line_count = 0
        
        log_filename = f"debug_log_{self.log_file_number}.txt"
        
        with open(log_filename, "a") as f:
            f.write(f"[{current_time}] {message}\n")
        
        self.log_line_count += 1
        
        # If we've reached the maximum number of files, delete the oldest one
        if self.log_line_count == 0 and self.log_file_number == 0:
            try:
                os.remove(f"debug_log_{self.max_log_files-1}.txt")
            except:
                pass  # If the file doesn't exist, just continue

    def configure_gnss(self):
        initial_baud = 9600
        new_baud = 38400  # or your preferred higher baud rate
        commands = [
            b'$PMTK220,100*2F\r\n',  # Set update rate to 100ms (10Hz)
            b'$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n'  # Set NMEA output
        ]

        # Initialize UART at the default baud rate
        self.uart = UART(0, baudrate=initial_baud, tx=Pin(GNSS_TX_PIN), rx=Pin(GNSS_RX_PIN))
        utime.sleep(1)
        
        # Change to the new baud rate
        self.uart.write(b'$PMTK251,38400*27\r\n')  # Change baud rate to 38400
        utime.sleep(0.5)
        
        # End UART and restart at the new baud rate
        self.uart.deinit()
        utime.sleep(1)
        self.uart = UART(0, baudrate=new_baud, tx=Pin(GNSS_TX_PIN), rx=Pin(GNSS_RX_PIN))
        utime.sleep(1)
        
        retries = 5
        for attempt in range(retries):
            for command in commands:
                self.uart.write(command)
                utime.sleep_ms(100)
                self.log_message(f"Sent command: {command}")
            
            # Verify configuration by checking GNSS data update rate
            utime.sleep(1)  # Allow some time for GNSS to stabilize and start sending data
            initial_time = utime.ticks_ms()
            valid_data_count = 0
            while utime.ticks_diff(utime.ticks_ms(), initial_time) < 5000:  # Observe for 5 seconds
                if self.uart.any():
                    response = self.uart.read()
                    self.log_message(f"Received response: {response}")
                    if b'PMTK001' in response:
                        self.log_message("GNSS command acknowledged")
                    if b'$GNRMC' in response:
                        valid_data_count += 1
                        timestamp = response.split(b',')[1]
                        self.log_message(f"GNSS timestamp: {timestamp}")
            
            if valid_data_count >= 10:  # Check if we received enough data points for 10Hz
                self.log_message("GNSS configured successfully")
                return
            
            self.log_message(f"Retrying GNSS configuration (Attempt {attempt + 1})")
        
        self.log_message("Failed to configure GNSS after retries")


    def parse_gnss_data(self, data: bytes):
        try:
            decoded_data = data.decode('ascii').strip()
            if decoded_data.startswith('$GNRMC') or decoded_data.startswith('$GPRMC'):
                parts = decoded_data.split(',')
                if len(parts) > 9 and parts[2] == 'A':
                    lat = self._convert_to_degrees(parts[3], parts[4])
                    lon = self._convert_to_degrees(parts[5], parts[6])
                    
                    if lat is not None and lon is not None:
                        current_time = utime.ticks_ms()
                        if self.last_lat is not None and self.last_lon is not None and self.last_time is not None:
                            time_diff = utime.ticks_diff(current_time, self.last_time) / 1000  # in seconds
                            lat_velocity = (lat - self.last_lat) / time_diff if time_diff > 0 else 0
                            lon_velocity = (lon - self.last_lon) / time_diff if time_diff > 0 else 0
                        else:
                            lat_velocity = lon_velocity = 0

                        filtered_lat = self.lat_filter.update(lat, lat_velocity)
                        filtered_lon = self.lon_filter.update(lon, lon_velocity)
                        
                        self.last_lat = filtered_lat
                        self.last_lon = filtered_lon
                        self.last_time = current_time
                        self.last_valid_gnss_time = current_time
                        
                        if self.state != 'ready':
                            self.state = 'ready'
                            self.update_display()
                        
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
                self.display.text('Initializing...', 10, 10, 31)  # Blue
            elif self.state == 'ready':
                self.display.text('GNSS Signal Acquired', 10, 10, 992)  # Green
            elif self.state == 'signal_lost':
                self.display.text('GNSS Signal Lost', 10, 10, 63488)  # Red
            elif self.state == 'crossing':
                self.display.text('Crossing Finish Line', 10, 10, 31)  # Blue
            elif self.state == 'button_pressed':
                self.display.text('Button Pressed', 10, 10, 992)  # Green
            self.display.show()
        except Exception as e:
            print(f"Display update error: {e}")




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
        self.log_message("Starting run method")
        self.update_display()
        self.configure_gnss()
        self.log_message("GNSS configuration completed")
        buffer = bytearray()
        self.log_message("Entering main loop")

        gnss_message_count = 0
        last_performance_check = utime.ticks_ms()
        performance_check_interval = 10000  # Check every 10 seconds
        missed_messages = 0
        last_timestamp = None

        while True:
            try:
                current_time = utime.ticks_ms()
                if utime.ticks_diff(current_time, self.last_log_time) >= self.log_interval:
                    self.log_message("Loop iteration")
                    self.last_log_time = current_time

                if self.uart.any():
                    data = self.uart.read(self.uart.any())  # Read all available data
                    if data:
                        buffer.extend(data)
                        if len(buffer) > MAX_BUFFER_SIZE:
                            buffer = buffer[-MAX_BUFFER_SIZE:]
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            if line.startswith(b'$GNRMC') or line.startswith(b'$GPRMC'):
                                gnss_message_count += 1
                                parts = line.decode().split(',')
                                if len(parts) > 1:
                                    current_timestamp = parts[1]
                                    if last_timestamp:
                                        time_diff = float(current_timestamp) - float(last_timestamp)
                                        if time_diff > 0.11:  # More than 110ms between messages
                                            missed_messages += 1
                                    last_timestamp = current_timestamp
                                lat, lon = self.parse_gnss_data(line)
                                if lat is not None and lon is not None:
                                    self.log_message(f"Valid position: lat={lat}, lon={lon}")
                                    current_time = utime.ticks_ms()

                                    crossed, crossing_time = self.is_crossing_finish_line(lat, lon)
                                    if crossed:
                                        self.log_message("Finish line crossed")
                                        self.state = 'crossing'
                                        self.update_display()
                                        delay = crossing_time - current_time
                                        if delay > 0:
                                            self.schedule_pulse(delay)
                                            self.log_message(f"Pulse scheduled for {delay}ms from now")
                                        else:
                                            self.send_pulse()
                                        self.flash_screen()
                                        self.log_message(f"Finish line crossed at interpolated time: {crossing_time}")
                                        self.state = 'ready'
                                        self.update_display()

                                    self.previous_lat, self.previous_lon = lat, lon
                                    self.last_update_time = current_time

                # Performance logging
                if utime.ticks_diff(current_time, last_performance_check) >= performance_check_interval:
                    elapsed_time = utime.ticks_diff(current_time, last_performance_check) / 1000  # in seconds
                    gnss_rate = gnss_message_count / elapsed_time
                    self.log_message(f"GNSS update rate: {gnss_rate:.2f} Hz")
                    self.log_message(f"Missed messages estimate: {missed_messages}")
                    
                    gnss_message_count = 0
                    missed_messages = 0
                    last_performance_check = current_time

                if utime.ticks_diff(current_time, self.last_valid_gnss_time) > GNSS_TIMEOUT:
                    if self.state != 'signal_lost':
                        self.state = 'signal_lost'
                        self.update_display()
                        self.log_message("GNSS signal lost")

                if self.is_button_pressed():
                    self.log_message("Button pressed")
                    previous_state = self.state
                    self.state = 'button_pressed'
                    self.update_display()
                    self.send_pulse()
                    self.flash_screen()
                    self.log_message("Button pressed, pulse sent")
                    utime.sleep_ms(500)  # Debounce
                    self.state = previous_state  # Restore the previous state
                    self.update_display()

                utime.sleep_ms(10)  # Small delay to prevent tight looping

            except Exception as e:
                self.log_message(f"Error in main loop: {type(e).__name__}: {str(e)}")
                utime.sleep_ms(100)  # Short delay to prevent rapid error logging


if __name__ == "__main__":
    gps_trigger = GPSLapTrigger()
    gps_trigger.run()
