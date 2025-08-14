# BMC-OHMS-REVENGE-ARDUINO

## Project Overview
This project is designed for the STM32F103C8T6 (Blue Pill) board using PlatformIO and the Arduino framework. It features multiple test modes for motors and line following.

## Requirements
- PlatformIO (https://platformio.org/)
- STM32F103C8T6 board (Blue Pill)
- USB to Serial adapter 

## How to Build and Upload
1. Connect your STM32 board to your computer via USB.
2. Open a terminal in the project directory.
3. Build and upload the firmware:
   ```bash
   pio run -t upload
   ```

## Serial Monitor
To interact with the board via serial:
1. List available serial devices:
   ```bash
   pio device list
   ```
2. Start the serial monitor at 115200 baud:
   ```bash
   pio device monitor
   ```

## Test Modes
- **Motor Test**: Verifies motor driver functionality.
- **Line Follower**: Uses sensors to follow a line.

