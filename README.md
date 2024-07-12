# Lap Timer for BMW S1000RR using Raspberry Pi Pico

## Project Overview
This project implements a sophisticated lap timer for a BMW S1000RR motorcycle using a Raspberry Pi Pico. The system utilizes GPS data to accurately detect when the motorcycle crosses the finish line and sends a pulse signal. It features an LCD display for real-time status updates and a manual button for user-triggered pulse sending.

## Key Features
- GPS-based lap timing with finish line detection
- Adaptive Kalman filtering for improved GPS accuracy
- LCD display for system status visualization
- Manual pulse triggering via button press
- Error handling and logging for robust operation
- Power management considerations

## Hardware Components
- Raspberry Pi Pico
- GNSS (GPS) module
- LCD display
- Pulse output pin (for timing system integration) - Uses Raspberry Pi Pico for this
- User input button - Uses LCD display for this

## Code Structure and Functionality

### Pin Configuration
The code begins by defining pins for various peripherals:
- GNSS module (TX and RX)
- Pulse output
- User button
- LCD display (SCK, MOSI, CS, DC, RST, BL)

### Adaptive Kalman Filter
A custom `AdaptiveKalmanFilter` class is implemented to enhance GPS data accuracy:
- Dynamically adjusts process variance based on motorcycle velocity
- Provides smoothed latitude and longitude estimates
- Improves finish line crossing detection accuracy

### GPSLapTrigger Class
This is the core class of the system, encapsulating all major functionality:

#### Initialization and Setup
- Configures UART for GNSS communication
- Sets up SPI for the LCD display
- Initializes GPIO for pulse output and button input
- Creates Kalman filter instances for latitude and longitude

#### GNSS Handling
- Configures GNSS module for optimal performance
- Parses raw GNSS data, converting to decimal degrees
- Applies Kalman filtering to smooth GPS coordinates
- Tracks signal validity and handles timeouts

#### Finish Line Detection
- Maintains a list of predefined track finish lines
- Uses line intersection algorithms to detect crossings
- Interpolates precise crossing times for accurate lap timing

#### User Interface
- Updates LCD display based on system state (e.g., initializing, ready, signal lost)
- Handles button press events for manual pulse triggering
- Provides visual feedback for finish line crossings and button presses

#### Pulse Generation
- Sends precisely timed pulses upon finish line crossing or button press
- Configurable pulse duration for compatibility with various timing systems

#### Error Handling and Logging
- Implements comprehensive error catching and handling
- Logs system events and errors with timestamps for debugging

#### Main Loop
The `run` method orchestrates the system operation:
- Continuously reads and processes GNSS data
- Checks for finish line crossings and button presses
- Updates system state and display
- Handles GNSS signal loss scenarios

### Power Management
The code includes considerations for power management, particularly during signal loss, to optimize battery life in real-world racing conditions.

## Usage
To run the lap timer system:
1. Connect all hardware components to the Raspberry Pi Pico.
2. Upload the code to the Pico.
3. Power on the system at the race track.
4. The system will initialize, acquire GPS signal, and begin monitoring for finish line crossings.

## Future Enhancements
- Integration with additional sensors (e.g., accelerometer) for improved accuracy
- Wireless data transmission for real-time lap time reporting
- Expanded track database with automatic track recognition
- User-configurable settings via the LCD interface

This lap timer system provides a robust, accurate, and user-friendly solution for timing laps on a BMW S1000RR motorcycle, leveraging the power of GPS technology and advanced filtering techniques.
