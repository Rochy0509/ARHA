#!/usr/bin/env python3
"""Test script for ARHA Gripper TCP commands."""
import socket
import struct
import sys
import time

IP = "192.168.1.100"  # Update this if your STM32 uses a different IP
PORT = 5000

CMD_GRIPPER_PING       = 0x40
CMD_GRIPPER_OPEN       = 0x41
CMD_GRIPPER_CLOSE      = 0x42
CMD_GRIPPER_MOVE_TO    = 0x43
CMD_GRIPPER_GET_STATUS = 0x44

def send_command(cmd_id, payload=b""):
    """Packs and sends an ARHA protocol command."""
    length = len(payload)
    # Header: Start (AA), Cmd, Length (uint16)
    header = struct.pack("<B B H", 0xAA, cmd_id, length)
    packet = header + payload + bytes([0x55]) # End byte

    print(f"Sending CMD 0x{cmd_id:02X} (len {length})...")
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((IP, PORT))
        s.sendall(packet)
        
        # Read response
        resp_header = s.recv(4)
        if len(resp_header) == 4:
            start, r_cmd, r_len = struct.unpack("<B B H", resp_header)
            r_payload = s.recv(r_len) if r_len > 0 else b""
            end_byte = s.recv(1)
            
            print(f"  Response: CMD 0x{r_cmd:02X}, Len {r_len}")
            if r_cmd == cmd_id:
                print("  ✅ Success (ACK received)")
                if r_payload:
                    print(f"  Payload: {r_payload.hex(' ')}")
                s.close()
                return True, r_payload
            else:
                print("  ❌ NACK or Error response")
        else:
            print("  ❌ No valid response")
            
        s.close()
    except Exception as e:
        print(f"  ❌ Connection failed: {e}")
        
    return False, b""

if __name__ == "__main__":
    if len(sys.argv) > 1:
        IP = sys.argv[1]
        
    print(f"Testing Gripper API at {IP}:{PORT}")
    print("-" * 40)
    
    print("1. PING")
    send_command(CMD_GRIPPER_PING)
    time.sleep(1)
    
    print("\n2. OPEN")
    # Payload: speed (uint16)
    payload_open = struct.pack("<H", 0) # speed 0 = default
    send_command(CMD_GRIPPER_OPEN, payload_open)
    time.sleep(2)
    
    print("\n3. CLOSE")
    send_command(CMD_GRIPPER_CLOSE, payload_open)
    time.sleep(2)
    
    print("\n4. GET_STATUS")
    success, data = send_command(CMD_GRIPPER_GET_STATUS)
    if success and len(data) >= 15:
        # GripperStatus is 15 bytes: pos(2), speed(2), load(2), volt(1), temp(1), async(1), status(1), moving(1), pwm(2), current(2)
        pos, spd, load, volt, temp, async_flag, stat, mov, pwm, curr = struct.unpack("<h h h B B B B B h h", data[:15])
        print(f"\nStatus decoded:")
        print(f"  Position: {pos}")
        print(f"  Speed:    {spd}")
        print(f"  Load:     {load}")
        print(f"  Voltage:  {volt/10.0}V")
        print(f"  Temp:     {temp}°C")
        print(f"  Moving:   {bool(mov)}")
    elif success:
        print(f"Status payload too short: {len(data)} bytes")
