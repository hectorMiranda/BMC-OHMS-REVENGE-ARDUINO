# BMC-OHMS-REVENGE-ARDUINO

## Project Overview
This project is for a line-following robot based on the STM32F103C8T6 (Blue Pill) using the Arduino framework. The robot uses 2 DC motors (L298N driver) and 6 TCRT5000 sensors for line detection. Multiple test and debug modes are available via serial commands.

## Requirements
- PlatformIO (https://platformio.org/)
- STM32F103C8T6 (Blue Pill) board
- L298N motor driver
- 6x TCRT5000 line sensors
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

## Modes and Serial Commands
Switch modes at runtime by sending a single character over serial:
- `l` : Line following mode
- `t` : Fake line (PID debug) mode
- `m` : Motor test mode
- `e` : LED blink test
- `s` : Sensor test (verbose debug)
- `?` : Print help

### Mode Details
- **Line Following (`l`)**: Robot follows a line using all sensors and a PID controller. If the line is lost, the robot wiggles clockwise to search, and stops after a timeout.
- **Fake Line (`t`)**: Simulate line error input for PID tuning. Use keys `a`/`d` to decrease/increase error, `s` to zero, `1`..`5` for presets.
- **Motor Test (`m`)**: Runs a scripted sequence to test both motors (forward, reverse, left, right).
- **LED Test (`e`)**: Blinks the onboard LED for hardware check.
- **Sensor Test (`s`)**: Prints detailed sensor readings, weighted sum, error, PID output, and what the robot would do. Useful for calibration and debugging.

## Code Structure and Explanation
- `main.cpp` contains all logic and mode switching.
- **Sensor Reading**: Reads all 6 sensors, prints their state, and computes a weighted error for line position.
- **PID Control**: Uses KP, KI, KD constants to calculate how much to turn left/right to stay on the line.
- **Lost Line Recovery**: If no sensors detect the line, the robot wiggles in a clockwise pattern to search. If the line is not found after a set time, the robot stops.
- **Test Modes**: Allow you to test motors, sensors, and debug the PID algorithm without running the full line follower.

## Example Serial Output (Sensor Test)
```
SENSORS: [ 0 1 1 0 0 0 ]  Active: 2  SUM: 2  WSUM: -4  -> ERROR: -2  PID: -31.00  LEFT: 231  RIGHT: 169  ACTION: TURN LEFT
```
- `SENSORS`: Raw sensor states (1 = on line, 0 = off line)
- `Active`: Number of sensors detecting the line
- `SUM`: Number of active sensors
- `WSUM`: Weighted sum for line position
- `ERROR`: Calculated line error
- `PID`: PID controller output
- `LEFT`/`RIGHT`: Motor speeds
- `ACTION`: What the robot would do (turn, forward, stop)

## Tuning and Calibration
- Adjust `KP`, `KI`, `KD`, and `BASE_SPEED` in `main.cpp` for best line following performance.
- Use sensor test mode to verify sensor alignment and response.

## Notes
- All features can be tested and debugged via serial commands.

---
For more details, see comments in `main.cpp`.

