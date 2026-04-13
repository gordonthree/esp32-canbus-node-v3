#!/usr/bin/env python3
import can
import time

# CAN interface configuration
CAN_INTERFACE = "can0"
CAN_ID = 0x40C

# Host ID for Raspberry Pi (32-bit)
HOST_ID = 0xDEADBEEF

def main():
    # Get current epoch time (32-bit)
    epoch = int(time.time()) & 0xFFFFFFFF

    # Build payload: 4 bytes host ID + 4 bytes epoch (big endian)
    data = [
        (HOST_ID >> 24) & 0xFF,
        (HOST_ID >> 16) & 0xFF,
        (HOST_ID >> 8)  & 0xFF,
        (HOST_ID >> 0)  & 0xFF,
        (epoch >> 24) & 0xFF,
        (epoch >> 16) & 0xFF,
        (epoch >> 8)  & 0xFF,
        (epoch >> 0)  & 0xFF,
    ]

    msg = can.Message(
        arbitration_id=CAN_ID,
        data=data,
        is_extended_id=False
    )

    # New python-can API: use interface= and a context manager
    print(f"Sending epoch {epoch} with host ID 0x{HOST_ID:08X}")
    with can.Bus(interface="socketcan", channel=CAN_INTERFACE) as bus:
        bus.send(msg)
    print("Message sent.")

if __name__ == "__main__":
    main()
