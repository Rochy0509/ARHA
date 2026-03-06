#!/usr/bin/env python3
"""Loopback test: checks if TX and RX of the adapter are actually shorted."""
import serial

PORT = "/dev/ttyUSB0"

ser = serial.Serial(PORT, 1000000, timeout=0.1)
ser.reset_input_buffer()

# Send a known test pattern
test = bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
ser.write(test)
ser.flush()

# If TX and RX are shorted, we should see our own data back
echo = ser.read(20)
ser.close()

if len(echo) == 0:
    print("❌ Got NOTHING back. TX and RX are NOT connected together,")
    print("   or the adapter isn't working.")
elif echo == test:
    print(f"✅ Got clean echo: {echo.hex(' ')}")
    print("   Adapter works, TX/RX shorted correctly.")
    print("   Servo is not responding — try the second servo.")
else:
    print(f"⚠️  Got {len(echo)} bytes: {echo.hex(' ')}")
    print("   Partial echo or servo response mixed in.")
