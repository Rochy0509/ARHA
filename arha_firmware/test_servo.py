#!/usr/bin/env python3
"""STS3215 servo PING test via USB-to-TTL adapter."""
import serial
import sys

PORT = "/dev/ttyUSB0"
BAUDS = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]

def ping(ser, servo_id):
    """Send PING and check for response."""
    # Build PING packet: FF FF ID 02 01 CHECKSUM
    length = 0x02
    instruction = 0x01
    checksum = (~(servo_id + length + instruction)) & 0xFF
    packet = bytes([0xFF, 0xFF, servo_id, length, instruction, checksum])

    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    # Read response (6 bytes: FF FF ID 02 ERR CSUM)
    resp = ser.read(6)
    if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
        print(f"  ✅ FOUND! ID={resp[2]}, Error=0x{resp[3]:02X}, "
              f"Raw: {resp.hex(' ')}")
        return True
    elif len(resp) > 0:
        print(f"  ⚠️  Got {len(resp)} bytes: {resp.hex(' ')}")
    return False

print("STS3215 Servo Scanner")
print("=" * 40)

for baud in BAUDS:
    print(f"\nBaud {baud}:")
    try:
        ser = serial.Serial(PORT, baud, timeout=0.05)
    except Exception as e:
        print(f"  ❌ Can't open {PORT}: {e}")
        sys.exit(1)

    for sid in range(11):
        if ping(ser, sid):
            ser.close()
            sys.exit(0)

    ser.close()

print("\n❌ No servo found at any baud rate or ID (0-10)")
