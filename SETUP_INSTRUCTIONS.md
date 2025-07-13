# Required Arduino Libraries for ESP32 RC Car Project

## Installation Instructions:
1. Open Arduino IDE
2. Go to Tools -> Manage Libraries...
3. Search for and install these libraries:

## Required Libraries:
- Adafruit NeoPixel (for LED ring control)
  - Search: "Adafruit NeoPixel"
  - Install the latest version by Adafruit

## ESP32 Board Package:
Make sure you have the ESP32 board package installed:
1. Go to File -> Preferences
2. Add this URL to "Additional Board Manager URLs":
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
3. Go to Tools -> Board -> Boards Manager
4. Search for "ESP32" and install "esp32 by Espressif Systems"

## Hardware Setup:
### Joystick Transmitter:
- VRX -> GPIO A0 (analog pin)
- VRY -> GPIO A1 (analog pin) 
- SW -> GPIO 2 (digital pin with internal pullup)
- VCC -> 3.3V
- GND -> GND

### Receiver (Car):
- Motors connected to pins 16, 17, 18, 19 (direction)
- Motor enable pins: 22, 23 (PWM speed control)
- LED Ring -> GPIO 12
- Power: Make sure both ESP32s and motors have adequate power supply
