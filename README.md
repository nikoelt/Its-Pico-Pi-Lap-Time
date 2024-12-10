## Project Overview
This project implements a high-precision lap timer for a BMW S1000RR Motorbike using a Raspberry Pi Pico. The system utilizes GPS data to accurately detect when the bike crosses predefined finish lines and sends a pulse signal to start a new lap on the bike's dashboard. It connects to the pins of the three-prong plug on the right-hand side of the frame (K46). The system features an LCD display for real-time status updates, a manual button for user-triggered pulse sending, and a debug mode for on-the-fly diagnostics. However it's designed to be hidden under the tank cover

## Key Features
- GPS-based lap timing with multi-track finish line detection
- Adaptive Kalman filtering for enhanced GPS accuracy
- LCD display for real-time system status visualization
- Dual-button interface: manual pulse triggering and debug mode toggle
- Robust error handling and comprehensive logging system
- Power-efficient operation for extended battery life
- Configurable for infinate race tracks, anything can be a finish line

## Hardware Components
- Raspberry Pi Pico
- GNSS (GPS) module: I used Waveshare Pico-GPS-L76B (overclocked to 10Hz sampling rate)
- LCD display: I used Waveshare Pico-LCD-1.14
- Pulse output: Directly from Raspberry Pi Pico
- User input buttons: Utilizing LCD display controls
- Relay: SSR-10DD 25DD 40DD Single Phase DC Controlled DC SSR Solid State Relay
- Voltage step down: aka 4.7K Resistor output from the relay into the S1000RR Lap Connector - Three prong front right of the frame)(to step down 12V to 10V for S1000RR compatibility)

## Software Architecture

### AdaptiveKalmanFilter Class
- Implements a Kalman filter with dynamic process variance adjustment
- Enhances GPS data accuracy based on motorcycle velocity
- Crucial for precise finish line detection and timing

### GPSLapTrigger Class
The core of the system, managing all major functionalities:

#### Initialization and Configuration
- Sets up UART for GNSS communication (9600 baud initially, then 38400 baud - for whatever reason this was the highest BAUD rate that was more or less stable on my board, even though it advertises 100K+)
- Configures SPI for the LCD display
- Initializes GPIO for pulse output and dual-button input
- Creates separate Kalman filter instances for latitude and longitude

#### GNSS Data Processing
- Configures GNSS module for 10Hz update rate
- Parses NMEA sentences, focusing on GNRMC data
- Applies Kalman filtering to smooth GPS coordinates
- Monitors signal quality and handles potential timeouts

#### Finish Line Detection System
- Maintains an array of predefined track finish lines
- Utilizes line intersection algorithms for precise crossing detection
- Interpolates exact crossing times for high-accuracy lap timing

#### User Interface and Display Management
- Updates LCD to reflect current system state (e.g., initializing, ready, signal lost, debug mode)
- Handles dual-button input: one for manual pulse triggering, one for debug mode toggle
- Provides visual feedback for finish line crossings, button presses, and mode changes

#### Pulse Generation and Timing
- Sends accurately timed pulses upon finish line crossing or manual button press
- Implements pulse scheduling for microsecond timing accuracy
- Configurable pulse duration for compatibility with various timing systems

#### Debug Mode and Logging System
- Toggleable debug mode for enhanced on-site diagnostics
- Comprehensive logging system with rotating log files
- Captures system events, errors, and performance metrics

#### Main Operational Loop
- Continuously reads and processes GNSS data
- Checks for finish line crossings and button presses
- Updates system state, display, and logs
- Manages GNSS signal loss scenarios and recovery

### Power Efficiency Considerations
- Implements sleep modes during periods of inactivity
- Optimizes processing to reduce power consumption
- Ensures reliable operation throughout extended racing sessions

## Usage Instructions
1. Connect all hardware components to the Raspberry Pi Pico as per the pin configuration.
2. Upload the complete code to the Pico.
3. Power on the system at the race track.
4. The system will initialize, acquire GPS signal, and begin monitoring for finish line crossings.
5. Use Button A for manual pulse triggering - (mostly to double check everything is actually working correctly - word of note, for the S1000RR the rear wheel needs to be in motion for the laptimer to activate. I used a rear stand but you do you boo)
6. Hold Button B for 3 seconds to toggle debug mode for on-site diagnostics.

## Customization and Track Configuration
- Edit the `TRACKS` array in the `GPSLapTrigger` class to add or modify finish line coordinates.
- Adjust `GNSS_TIMEOUT`, `PULSE_DURATION_MS`, and other constants as needed for specific requirements.

## Troubleshooting
- If experiencing GPS signal issues, ensure clear sky visibility and check antenna connections. The only two good places for antenna are over the very front dash cover (under the windshield) or attached to the rear cowling
- For timing discrepancies, verify the accuracy of finish line coordinates in the `TRACKS` array. Recommend Google Earth
- Consult the debug logs (accessible in debug mode) for detailed system diagnostics.
- Put the bike on the rear stand, start in first gear and press B button to manually send a pulse into the system to see if you wired up the thing correctly. CAUTION triggers under 10 seconds are ignored by the BMW Lap Computer

- S1000RR GEN 3 Integration
- ![IMG_1812](https://github.com/user-attachments/assets/74d867d1-6cb6-4d9a-8565-2f2268c05a29)
![IMG_1814](https://github.com/user-attachments/assets/823b9c84-b33c-4fc5-8cbd-228a627e6ac4)
![IMG_1813](https://github.com/user-attachments/assets/8e38d66f-ded2-4444-a285-9e8a97157921)

