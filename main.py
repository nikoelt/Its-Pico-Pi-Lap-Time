from machine import Pin, SPI, UART
import utime
import Pico_LCD_1_14 as lcd
import math

# Pin configuration
GNSS_TX_PIN = 0
GNSS_RX_PIN = 1
PULSE_PIN = 2
BUTTON_A_PIN = 15

# LCD configuration
LCD_SCK_PIN = 10
LCD_MOSI_PIN = 11
LCD_CS_PIN = 9
LCD_DC_PIN = 8
LCD_RST_PIN = 12
LCD_BL_PIN = 13

# GNSS signal loss timeout (in milliseconds)
GNSS_TIMEOUT = 5000

class AdaptiveKalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = 1
        self.min_process_variance = 0.001
        self.max_process_variance = 0.1

    def update(self, measurement, velocity):
        # Adjust process variance based on velocity
        self.process_variance = min(max(abs(velocity) * 0.01, self.min_process_variance), self.max_process_variance)
        
        # Prediction
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance

        # Update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error

        return self.estimate

class GPSLapTrigger:
    # Class variable for tracks
    TRACKS = [
        {'right_lat': -27.690622, 'right_lon': 152.654567, 'left_lat': -27.690622, 'left_lon': 152.654688},
        {'right_lat': -27.228533, 'right_lon': 152.964891, 'left_lat': -27.228441, 'left_lon': 152.964919},
        {'right_lat': -28.262069, 'right_lon': 152.036330, 'left_lat': -28.262086, 'left_lon': 152.036433}
    ]

    def __init__(self):
        self.uart = UART(0, baudrate=115200, tx=Pin(GNSS_TX_PIN), rx=Pin(GNSS_RX_PIN))
        self.pulse_pin = Pin(PULSE_PIN, Pin.OUT)
        self.button_a = Pin(BUTTON_A_PIN, Pin.IN, Pin.PULL_UP)
        self.spi = SPI(1, baudrate=40000000, polarity=1, phase=1, sck=Pin(LCD_SCK_PIN), mosi=Pin(LCD_MOSI_PIN))
        self.display = lcd.LCD_1inch14(self.spi, cs=Pin(LCD_CS_PIN), dc=Pin(LCD_DC_PIN), rst=Pin(LCD_RST_PIN), bl=Pin(LCD_BL_PIN))
        self.previous_lat = None
        self.previous_lon = None
        self.last_update_time = None
        self.last_valid_gnss_time = utime.ticks_ms()
        self.state = 'initializing'
        self.display_state = None
        self.lat_filter = AdaptiveKalmanFilter(process_variance=0.01, measurement_variance=0.1)
        self.lon_filter = AdaptiveKalmanFilter(process_variance=0.01, measurement_variance=0.1)
        self.last_lat = None
        self.last_lon = None
        self.last_time = None

    def configure_gnss(self):
        commands = [
            bytearray(b'$PMTK220,100*2F\r\n'),
            bytearray(b'$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n')
        ]
        for command in commands:
            self.uart.write(command)
            utime.sleep_ms(100)
        self.log_message("GNSS configured")

    def parse_gnss_data(self, data: str) -> (float, float):
        if data.startswith('$GPRMC'):
            parts = data.split(',', 7)
            if len(parts) > 6 and parts[2] == 'A':
                try:
                    lat = self._convert_to_degrees(parts[3], parts[4])
                    lon = self._convert_to_degrees(parts[5], parts[6])
                    
                    current_time = utime.ticks_ms()
                    if self.last_lat is not None and self.last_lon is not None and self.last_time is not None:
                        time_diff = utime.ticks_diff(current_time, self.last_time) / 1000  # in seconds
                        lat_velocity = (lat - self.last_lat) / time_diff
                        lon_velocity = (lon - self.last_lon) / time_diff
                        
                        # Predict next position
                        predicted_lat = self.last_lat + lat_velocity * time_diff
                        predicted_lon = self.last_lon + lon_velocity * time_diff
                        
                        # Update Kalman filter with predicted and measured positions
                        filtered_lat = self.lat_filter.update(predicted_lat, lat_velocity)
                        filtered_lon = self.lon_filter.update(predicted_lon, lon_velocity)
                    else:
                        filtered_lat = self.lat_filter.update(lat, 0)
                        filtered_lon = self.lon_filter.update(lon, 0)
                    
                    self.last_lat = filtered_lat
                    self.last_lon = filtered_lon
                    self.last_time = current_time
                    
                    self.last_valid_gnss_time = current_time
                    return filtered_lat, filtered_lon
                except (ValueError, IndexError):
                    pass
        return None, None

    def _convert_to_degrees(self, raw_value: str, direction: str) -> float:
        if len(raw_value) < 3:
            return None
        degrees = float(raw_value[:2])
        minutes = float(raw_value[2:])
        decimal_degrees = degrees + minutes / 60
        return -decimal_degrees if direction in 'SW' else decimal_degrees

    def is_crossing_finish_line(self, lat: float, lon: float) -> (bool, float):
        if self.previous_lat is None or self.previous_lon is None or self.last_update_time is None:
            return False, None
        for track in self.TRACKS:
            if self._lines_intersect(track['left_lat'], track['left_lon'], track['right_lat'], track['right_lon'],
                                    self.previous_lat, self.previous_lon, lat, lon):
                crossing_time = self._interpolate_crossing_time(self.previous_lat, self.previous_lon, lat, lon, track)
                return True, crossing_time
        return False, None

    def _lines_intersect(self, lat1: float, lon1: float, lat2: float, lon2: float, lat3: float, lon3: float, lat4: float, lon4: float) -> bool:
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        A, B, C, D = (lat1, lon1), (lat2, lon2), (lat3, lon3), (lat4, lon4)
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    def _interpolate_crossing_time(self, lat1: float, lon1: float, lat2: float, lon2: float, finish_line: dict) -> float:
        dx, dy = lat2 - lat1, lon2 - lon1
        total_distance = math.sqrt(dx**2 + dy**2)
        
        intersection = self._line_intersection(lat1, lon1, lat2, lon2, 
                                              finish_line['left_lat'], finish_line['left_lon'], 
                                              finish_line['right_lat'], finish_line['right_lon'])
        if intersection:
            dx_finish, dy_finish = intersection[0] - lat1, intersection[1] - lon1
            distance_to_finish = math.sqrt(dx_finish**2 + dy_finish**2)
            time_fraction = distance_to_finish / total_distance
            crossing_time = self.last_update_time + (time_fraction * 0.1)
            return crossing_time
        return None

    def _line_intersection(self, lat1: float, lon1: float, lat2: float, lon2: float, 
                          lat3: float, lon3: float, lat4: float, lon4: float) -> (float, float):
        denom = (lon4 - lon3) * (lat2 - lat1) - (lat4 - lat3) * (lon2 - lon1)
        if abs(denom) < 1e-8:  # Check for near-zero to avoid division by zero
            return None
        ua = ((lon4 - lon3) * (lat1 - lat3) - (lat4 - lat3) * (lon1 - lon3)) / denom
        if ua < 0 or ua > 1:
            return None
        ub = ((lon2 - lon1) * (lat1 - lat3) - (lat2 - lat1) * (lon1 - lon3)) / denom
        if ub < 0 or ub > 1:
            return None
        x = lon1 + ua * (lon2 - lon1)
        y = lat1 + ua * (lat2 - lat1)
        return (x, y)

    def send_pulse(self):
        self.pulse_pin.on()
        utime.sleep_ms(100)
        self.pulse_pin.off()
        self.log_message("Pulse sent")

    def is_button_pressed(self) -> bool:
        if not self.button_a.value():
            utime.sleep_ms(50)
            return not self.button_a.value()
        return False

    def update_display(self, new_state: str):
        try:
            if new_state != self.display_state:
                self.display_state = new_state
                if new_state == 'initializing':
                    self.display.fill(self.display.yellow)
                    self.display.text('Initializing...', 10, 10, self.display.black)
                elif new_state == 'ready':
                    self.display.fill(self.display.green)
                    self.display.text('GNSS Signal Acquired', 10, 10, self.display.black)
                elif new_state == 'low_power':
                    self.display.fill(self.display.blue)
                    self.display.text('Low Power Mode', 10, 10, self.display.white)
                elif new_state == 'signal_lost':
                    self.display.fill(self.display.red)
                    self.display.text('GNSS Signal Lost', 10, 10, self.display.white)
                elif new_state == 'crossing':
                    self.display.fill(self.display.white)
                    self.display.text('Crossing Finish Line', 10, 10, self.display.black)
                elif new_state == 'button_pressed':
                    self.display.fill(self.display.white)
                    self.display.text('Button Pressed', 10, 10, self.display.black)
        except Exception as e:
            self.log_message(f"Display update error: {e}")

    def log_message(self, message: str):
        print(f"[{utime.ticks_ms()}] {message}")

    def run(self):
        self.update_display('initializing')
        self.configure_gnss()
        self.state = 'ready'

        while True:
            try:
                if self.uart.any():
                    data = self.uart.readline(100).decode('utf-8').strip()  # Read max 100 bytes
                    lat, lon = self.parse_gnss_data(data)

                    if lat is not None and lon is not None:
                        current_time = utime.ticks_ms()
                        if self.state != 'ready':
                            self.state = 'ready'
                            self.update_display('ready')

                        crossed, crossing_time = self.is_crossing_finish_line(lat, lon)
                        if crossed:
                            self.update_display('crossing')
                            self.send_pulse()
                            self.log_message(f"Finish line crossed at interpolated time: {crossing_time}")
                            utime.sleep_ms(1000)
                            self.update_display('ready')

                        self.previous_lat, self.previous_lon = lat, lon
                        self.last_update_time = current_time
                    elif utime.ticks_diff(utime.ticks_ms(), self.last_valid_gnss_time) > GNSS_TIMEOUT:
                        if self.state != 'signal_lost':
                            self.state = 'signal_lost'
                            self.update_display('signal_lost')
                            self.log_message("GNSS signal lost")

            except Exception as e:
                self.log_message(f"Error during UART communication: {e}")

            if self.is_button_pressed():
                self.update_display('button_pressed')
                self.send_pulse()
                self.log_message("Button pressed, pulse sent")
                utime.sleep_ms(1000)
                self.update_display('ready' if self.state == 'ready' else 'signal_lost')

            # Add power management here
            if self.state == 'signal_lost':
                # Implement low power mode
                pass

            utime.sleep_ms(10)  # Reduced sleep time for more responsive system

if __name__ == '__main__':
    gps_lap_trigger = GPSLapTrigger()
    gps_lap_trigger.run()
