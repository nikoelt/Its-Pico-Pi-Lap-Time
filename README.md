# Lap Timer for S1000RR using Raspberry Pi Pico

This project implements a lap timer for a BMW S1000RR motorcycle using a Raspberry Pi Pico. The system utilizes GPS data to detect when the motorcycle crosses the finish line and sends a pulse signal. Additionally, it includes a display for status updates and a button for manual pulse sending. 

## Code Overview

### Pin Configuration
The code starts with defining the pins used for various peripherals connected to the Raspberry Pi Pico:
- `GNSS_TX_PIN` and `GNSS_RX_PIN` for the GNSS (GPS) module.
- `PULSE_PIN` for sending the pulse signal.
- `BUTTON_A_PIN` for the user button.
- Pins for the LCD display (`LCD_SCK_PIN`, `LCD_MOSI_PIN`, `LCD_CS_PIN`, `LCD_DC_PIN`, `LCD_RST_PIN`, `LCD_BL_PIN`).

### LCD Configuration
Pins are set up for the LCD screen to display the system's state.

### GNSS Timeout
A constant `GNSS_TIMEOUT` is set to handle GNSS signal loss timeout in milliseconds.

### Adaptive Kalman Filter
An `AdaptiveKalmanFilter` class is implemented to filter the noisy GPS data:
- The filter adjusts its process variance based on the velocity of the motorcycle.
- It updates its estimate with the new measurements and adjusts the Kalman gain accordingly.

### GPSLapTrigger Class
The main functionality is encapsulated in the `GPSLapTrigger` class.

#### Initialization
In the `__init__` method:
- UART is configured for GNSS communication.
- SPI is configured for the LCD display.
- The pulse pin and button are set up.
- An instance of the `AdaptiveKalmanFilter` is created for latitude and longitude filtering.

#### GNSS Configuration
The `configure_gnss` method sends commands to configure the GNSS module for the desired output.

#### GNSS Data Parsing
The `parse_gnss_data` method processes the GNSS data:
- Converts raw latitude and longitude data to decimal degrees.
- Uses the Kalman filter to smooth the latitude and longitude values.
- Updates the last valid GNSS time to keep track of signal validity.

#### Finish Line Detection
The `is_crossing_finish_line` method checks if the motorcycle crosses the finish line by:
- Interpolating the crossing time if the line is crossed.
- Using helper methods `_lines_intersect` and `_interpolate_crossing_time` to determine intersection and crossing time accurately.

#### Pulse Sending
The `send_pulse` method sends a pulse signal by toggling the `PULSE_PIN`.

#### Button Press Detection
The `is_button_pressed` method checks if the button is pressed and debounces the input.

#### Display Updates
The `update_display` method updates the LCD display based on the system state.

#### Logging
The `log_message` method logs messages with timestamps for debugging purposes.

#### Main Loop
The `run` method contains the main loop:
- Configures GNSS and sets the initial display state.
- Reads GNSS data, parses it, and checks for finish line crossing.
- Sends a pulse when the finish line is crossed.
- Updates the display and handles button presses.
- Manages the system state based on GNSS signal availability.

### Running the Code
The script is executed by creating an instance of `GPSLapTrigger` and calling the `run` method.

```python
if __name__ == '__main__':
    gps_lap_trigger = GPSLapTrigger()
    gps_lap_trigger.run()
```

## Summary
This project integrates GPS data processing, Kalman filtering, pulse signaling, and an LCD display to create a lap timer system for a BMW S1000RR motorcycle. The code is structured to handle real-time GPS data, detect finish line crossings accurately, and provide user feedback through a display and manual button interaction.
