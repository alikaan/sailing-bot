# Sailing Bot

This repository contains the code for a sailing bot using an Arduino. The bot uses an MPU6050 sensor for orientation and motion detection, and controls servos for sail and rudder adjustments.

## Hardware Requirements

- Arduino Uno (or compatible)
- MPU6050 sensor
- Servos (for sail and rudder control)
- LED
- Button (for mode selection)

## Software Requirements

- Arduino IDE
- [I2Cdev library](https://github.com/jrowberg/i2cdevlib)
- [MPU6050 library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)

## Wiring

- MPU6050:
  - VCC to 5V
  - GND to GND
  - SDA to A4
  - SCL to A5
  - INT to pin 2

- Servos:
  - Sail servo signal to pin 10
  - Rudder servo signal to pin 9

- LED:
  - Anode to pin 13
  - Cathode to GND

- Button:
  - One side to pin 4
  - Other side to GND

## Installation

1. Clone this repository.
2. Open `sailing_bot.ino` in the Arduino IDE.
3. Install the required libraries (I2Cdev and MPU6050) via the Arduino Library Manager.
4. Connect your Arduino to your computer.
5. Upload the code to the Arduino.

## Usage

The bot operates in three modes: LINE, CIRCLE, and EIGHT. The mode can be changed by pressing the button connected to pin 4.

- **LINE Mode**: The bot sails in a straight line.
- **CIRCLE Mode**: The bot sails in a circular pattern.
- **EIGHT Mode**: The bot sails in a figure-eight pattern.

The current mode is indicated by the LED:
- OFF: LINE mode
- ON: CIRCLE mode
- Blinking: EIGHT mode

## Code Overview

- `setup()`: Initializes the MPU6050 sensor, servos, and other components.
- `loop()`: Main program loop that reads sensor data, determines the mode, and performs actions based on the mode.
- `setMPU6050()`: Configures the MPU6050 sensor.
- `getValMPU60()`: Reads values from the MPU6050 sensor.
- `getMode()`: Reads the mode button state and returns the current mode.
- `blinkLed()`: Blinks the LED at a set interval.
- `lineAction()`, `circleAction()`, `eightAction()`: Perform actions based on the current mode.

## License

This project is licensed under the MIT License.