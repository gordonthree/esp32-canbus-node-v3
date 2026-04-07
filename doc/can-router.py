#!/usr/bin/env python3
import struct
import can

# -----------------------------
# Protocol constants
# -----------------------------
CFG_ROUTE_BEGIN_ID = 0x300
CFG_ROUTE_DATA_ID  = 0x301
CFG_ROUTE_END_ID   = 0x302

ROUTE_CHUNK_SIZE = 3
ROUTE_ENTRY_SIZE = 27
ROUTE_CHUNKS_PER_ROUTE = (ROUTE_ENTRY_SIZE + ROUTE_CHUNK_SIZE - 1) // ROUTE_CHUNK_SIZE
# NODE_ID = [0x25, 0xA5, 0x6D, 0x84]   # big-endian


# -----------------------------
# Helper: ask user for an integer
# -----------------------------
def ask_int(prompt, bits=32):
    while True:
        try:
            val = int(input(prompt + ": ").strip(), 0)
            if bits == 8 and not (0 <= val <= 0xFF):
                print("Value must fit in uint8_t (0–255)")
                continue
            if bits == 16 and not (0 <= val <= 0xFFFF):
                print("Value must fit in uint16_t (0–65535)")
                continue
            if bits == 32 and not (0 <= val <= 0xFFFFFFFF):
                print("Value must fit in uint32_t")
                continue
            return val
        except ValueError:
            print("Invalid number, try again.")

# -----------------------------
# Helper: convert uint32 -> 4-byte array (Big Endian)
# -----------------------------
def u32_to_be_bytes(value):
    return [
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8)  & 0xFF,
        (value >> 0)  & 0xFF,
    ]


# -----------------------------
# Build a route_entry_t struct
# -----------------------------
def build_route_entry():
    
    print("\n=== Enter Route Entry Fields ===")

    source_node_id  = ask_int("source_node_id (e.g. 0x25A56D84)", 32)
    source_msg_id   = ask_int("source_msg_id (uint16_t)", 16)
    source_msg_dlc  = ask_int("source_msg_dlc (uint8_t)", 8)
    source_sub_idx  = ask_int("source_sub_idx (uint8_t)", 8)

    target_msg_id   = ask_int("target_msg_id (uint16_t)", 16)
    target_msg_dlc  = ask_int("target_msg_dlc (uint8_t)", 8)
    target_sub_idx  = ask_int("target_sub_idx (uint8_t)", 8)

    event_type      = ask_int("event_type (uint8_t)", 8)
    action_type     = ask_int("action_type (uint8_t)", 8)

    print("\nEnter 8 parameter bytes (hex or decimal).")
    params = []
    for i in range(8):
        params.append(ask_int(f"param[{i}] (uint8_t)", 8))

    param_len       = ask_int("param_len (uint8_t)", 8)
    wildcard        = ask_int("wildcard (0 or 1)", 8)
    enabled         = ask_int("enabled (0 or 1)", 8)

    reserved        = ask_int("reserved (uint16_t, usually 0)", 16)

    # Pack into the exact 27‑byte struct (packed, little‑endian)
    packed = struct.pack(
        "<I H B B H B B B B 8B B B B H",
        source_node_id,
        source_msg_id,
        source_msg_dlc,
        source_sub_idx,
        target_msg_id,
        target_msg_dlc,
        target_sub_idx,
        event_type,
        action_type,
        *params,
        param_len,
        wildcard,
        enabled,
        reserved
    )

    assert len(packed) == ROUTE_ENTRY_SIZE
    return packed


# -----------------------------
# Serialize into CAN frames
# -----------------------------
def serialize_route(route_idx, packed):
    frames = []

    # BEGIN frame
    begin = can.Message(
        arbitration_id=CFG_ROUTE_BEGIN_ID,
        data=NODE_ID + [route_idx, ROUTE_CHUNKS_PER_ROUTE],
        is_extended_id=False
    )
    frames.append(begin)

    # DATA frames
    for chunk in range(ROUTE_CHUNKS_PER_ROUTE):
        base = chunk * ROUTE_CHUNK_SIZE
        b0 = packed[base + 0] if base + 0 < len(packed) else 0
        b1 = packed[base + 1] if base + 1 < len(packed) else 0
        b2 = packed[base + 2] if base + 2 < len(packed) else 0

        msg = can.Message(
            arbitration_id=CFG_ROUTE_DATA_ID,
            data=NODE_ID + [chunk, b0, b1, b2],
            is_extended_id=False
        )

        frames.append(msg)

    # END frame
    end = can.Message(
        arbitration_id=CFG_ROUTE_END_ID,
        data=NODE_ID + [route_idx, 1],  # commit flag = 1
        is_extended_id=False
    )

    frames.append(end)

    return frames


# -----------------------------
# Main interactive flow
# -----------------------------
def main():
    print("=== CAN Route Programming Tool ===")

    target_node_id = ask_int("Enter target node ID (e.g. 0x25A56D84)", 32)
    global NODE_ID
    NODE_ID = u32_to_be_bytes(target_node_id)
    
    route_idx = ask_int("Enter route index (0–255)", 8)
    entry = build_route_entry()
    frames = serialize_route(route_idx, entry)

    print("\nGenerated frames:")
    for i, f in enumerate(frames):
        print(f"{i:02d}: ID=0x{f.arbitration_id:03X}  DATA={f.data}")

    send = input("\nSend frames on CAN bus? (y/n): ").strip().lower()
    if send == "y":
        with can.Bus(interface="socketcan", channel="can0") as bus:
            for f in frames:
                bus.send(f)
            print("Frames sent.")



if __name__ == "__main__":
    main()
