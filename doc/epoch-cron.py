#!/usr/bin/env python3
import can
import time
import sys

CAN_INTERFACE = "can0"
CAN_ID = 0x40C
HOST_ID = 0xDEADBEEF

def main():
    try:
        epoch = int(time.time()) & 0xFFFFFFFF

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
        with can.Bus(interface="socketcan", channel=CAN_INTERFACE) as bus:
            bus.send(msg)

    except Exception as e:
        # Only print errors so cron can email/log them
        print(f"Epoch send error: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()
