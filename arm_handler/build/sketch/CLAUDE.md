#line 1 "C:\\Users\\kogm\\Desktop\\ROS项目\\Code\\arm_handler\\CLAUDE.md"
# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino project for robotic arm control, likely integrated with ROS (Robot Operating System) based on the directory structure. The project contains a single Arduino sketch that will handle arm movement and control logic.

## Development Environment

This is an Arduino project (.ino file) that should be developed using:
- Arduino IDE
- PlatformIO (recommended for advanced development)
- Or any Arduino-compatible development environment

## Build and Upload Commands

### Arduino IDE
- Open `arm_handler.ino` in Arduino IDE
- Select appropriate board and port from Tools menu
- Use Ctrl+R to verify/compile
- Use Ctrl+U to upload to connected Arduino

### PlatformIO (if configured)
```bash
# Compile the project
pio run

# Upload to connected device
pio run --target upload

# Monitor serial output
pio device monitor
```

## Code Architecture

The project follows standard Arduino sketch structure:

- `setup()`: Initialization code that runs once when the Arduino starts
- `loop()`: Main program logic that runs continuously

Currently the sketch is a template with empty functions, ready for implementation of:
- Servo motor control
- Sensor input processing  
- ROS communication (based on project path context)
- Arm movement algorithms

## Development Notes

- This appears to be part of a larger ROS robotics project
- The arm handler will likely interface with ROS nodes for receiving movement commands
- Consider implementing proper error handling for hardware failures
- Serial communication will likely be used for debugging and ROS integration