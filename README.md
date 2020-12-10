# Fadecandy Teensy 4 Device Firmware

- Use in combination with fork of Fadecandy Server, and "type": "teensy4" device
- Modified Fadecandy Server: https://github.com/pixelmatix/fadecandy
- SmartMatrix Library: https://github.com/pixelmatix/SmartMatrix/
- Compile with USB Type: Dual Serial (one Serial channel is for Fadecandy data, the other for debug)
    - If you see "error: 'SerialUSB1' was not declared in this scope", make sure you change the USB Type in Teensyduino
