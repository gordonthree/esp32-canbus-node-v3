#!/usr/bin/env python3
import struct
import can

# ---------------------------------------------------------
# CAN protocol constants
# ---------------------------------------------------------
CFG_ROUTE_BEGIN_ID = 0x300
CFG_ROUTE_DATA_ID  = 0x301
CFG_ROUTE_END_ID   = 0x302

ROUTE_CHUNK_SIZE = 3
ROUTE_ENTRY_SIZE = 27
ROUTE_CHUNKS_PER_ROUTE = (ROUTE_ENTRY_SIZE + ROUTE_CHUNK_SIZE - 1) // ROUTE_CHUNK_SIZE  # 9


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
def ask_int(prompt, base=0, minv=None, maxv=None):
    while True:
        s = input(prompt).strip()
        try:
            v = int(s, base)
            if minv is not None and v < minv:
                print(f"Value must be >= {minv}")
                continue
            if maxv is not None and v > maxv:
                print(f"Value must be <= {maxv}")
                continue
            return v
        except ValueError:
            print("Invalid number, try again.")


def ask_byte(prompt):
    return ask_int(prompt, base=0, minv=0, maxv=255)


def print_event_table():
    events = [
        (0,  "EVENT_ALWAYS"),
        (1,  "EVENT_ON_BINARY_CHANGE"),
        (2,  "EVENT_ON_BINARY_RISING"),
        (3,  "EVENT_ON_BINARY_FALLING"),
        (4,  "EVENT_ON_BINARY_MATCH"),
        (5,  "EVENT_ON_FULL_MATCH"),
        (6,  "EVENT_MASKED_BINARY_MATCH"),
        (7,  "EVENT_MASKED_FULL_MATCH"),
        (8,  "EVENT_THRESHOLD_GT"),
        (9,  "EVENT_THRESHOLD_LT"),
        (10, "EVENT_THRESHOLD_IN_RANGE"),
        (11, "EVENT_THRESHOLD_OUT_RANGE"),
        (12, "EVENT_THRESHOLD_TOLERANCE"),
        (13, "EVENT_BIT_SET"),
        (14, "EVENT_BIT_CLEAR"),
        (15, "EVENT_BIT_TOGGLE"),
        (16, "EVENT_BIT_RISING"),
        (17, "EVENT_BIT_FALLING"),
        (18, "EVENT_MATCH_BYTE4"),
        (19, "EVENT_MATCH_BYTE5"),
        (20, "EVENT_MATCH_BYTE6"),
        (21, "EVENT_MATCH_BYTE7"),
        (22, "EVENT_IDENTIFIER_MATCH"),
        (23, "EVENT_IDENTIFIER_RANGE"),
        (31, "EVENT_NEVER"),
    ]

    print("\nEvent types (5-bit, 0–31):")
    col_width = 35
    # two-column layout
    for i in range(0, len(events), 2):
        left = events[i]
        line = f"{left[0]:2d}: {left[1]:<{col_width}}"
        if i + 1 < len(events):
            right = events[i + 1]
            line += f"{right[0]:2d}: {right[1]}"
        print(line)


def print_action_list():
    actions = [
        (0, "ACTION_NO_ACTION"),
        (1, "ACTION_FOLLOW_BINARY"),
        (2, "ACTION_SEND_DYNAMIC_PAYLOAD"),
        (3, "ACTION_SEND_STATIC_PAYLOAD"),
        (4, "ACTION_RESERVED_4"),
        (5, "ACTION_RESERVED_5"),
        (6, "ACTION_RESERVED_6"),
        (7, "ACTION_RESERVED_7"),
    ]
    print("\nAction types (3-bit, 0–7):")
    for val, name in actions:
        print(f" {val}: {name}")


# ---------------------------------------------------------
# Main
# ---------------------------------------------------------
def main():
    print("\n=== Route Entry Programmer ===\n")

    # -----------------------------------------------------
    # Node ID and route index
    # -----------------------------------------------------
    node_id = ask_int("Node ID (uint32, hex or dec, e.g. 0x25A56D84): ")
    route_index = ask_int("Route index (0–79): ", minv=0, maxv=79)

    # -----------------------------------------------------
    # Source fields
    # -----------------------------------------------------
    print("\n--- Source fields ---")
    source_node_id = ask_int("Source node ID (hex or dec): ")
    source_msg_id  = ask_int("Source CAN message ID (0–0x7FF): ", minv=0, maxv=0x7FF)
    source_msg_dlc = ask_byte("Source DLC (0–8): ")
    source_sub_idx = ask_byte("Source submodule index (0–255): ")

    # -----------------------------------------------------
    # Target fields
    # -----------------------------------------------------
    print("\n--- Target fields ---")
    target_msg_id  = ask_int("Target CAN message ID (0–0x7FF): ", minv=0, maxv=0x7FF)
    target_msg_dlc = ask_byte("Target DLC (0–8): ")
    target_sub_idx = ask_byte("Target submodule index (0–255): ")

    # -----------------------------------------------------
    # Event + Action (ea)
    # -----------------------------------------------------
    print_event_table()
    event_type = ask_int("Select event_type (0–31): ", minv=0, maxv=31)

    print_action_list()
    action_type = ask_int("Select action_type (0–7): ", minv=0, maxv=7)

    # 5-bit event_type, 3-bit action_type
    ea = ((event_type & 0x1F) << 3) | (action_type & 0x07)
    print(f"Packed ea byte = 0x{ea:02X}")

    # -----------------------------------------------------
    # Flags (raw byte)
    # -----------------------------------------------------
    print("\nFlags byte (route_flags_t.raw):")
    print(" bit0: enabled")
    print(" bit1: wildcard")
    print(" bit2: dynamic_params")
    print(" bit3: inUse")
    print(" bit4-7: reserved")
    flags = ask_byte("Flags (0–255, hex or dec): ")

    # -----------------------------------------------------
    # Comparison fields (match_value / match_mask)
    # -----------------------------------------------------
    print("\nComparison fields:")
    print("You will enter match_value and match_mask directly.")
    print("Interpretation depends on the event_type you selected.\n")

    match_value = ask_int("match_value (uint32, hex or dec): ")
    match_mask  = ask_int("match_mask  (uint32, hex or dec): ")

    # -----------------------------------------------------
    # Static parameters (3 bytes)
    # -----------------------------------------------------
    print("\nStatic parameters (3 bytes):")
    p0 = ask_byte("parameters[0]: ")
    p1 = ask_byte("parameters[1]: ")
    p2 = ask_byte("parameters[2]: ")

    params = bytes([p0, p1, p2])
    reserved = bytes([0, 0])

    # -----------------------------------------------------
    # Pack route_entry_t (27 bytes)
    # -----------------------------------------------------
    fmt = "<I H B B H B B B B I I 3s 2s"
    packed = struct.pack(
        fmt,
        source_node_id,
        source_msg_id,
        source_msg_dlc,
        source_sub_idx,
        target_msg_id,
        target_msg_dlc,
        target_sub_idx,
        ea,
        flags,
        match_value,
        match_mask,
        params,
        reserved
    )

    print(f"\nPacked route_entry_t ({len(packed)} bytes):")
    print(packed.hex(" "))

    # -----------------------------------------------------
    # Slice into 3-byte chunks
    # -----------------------------------------------------
    chunks = []
    for i in range(ROUTE_CHUNKS_PER_ROUTE):
        start = i * ROUTE_CHUNK_SIZE
        end   = start + ROUTE_CHUNK_SIZE
        chunk = packed[start:end]
        if len(chunk) < ROUTE_CHUNK_SIZE:
            chunk = chunk + bytes(ROUTE_CHUNK_SIZE - len(chunk))
        chunks.append(chunk)

    print(f"\nTotal chunks: {len(chunks)} (expected {ROUTE_CHUNKS_PER_ROUTE})")

    # -----------------------------------------------------
    # Build CAN frames
    # -----------------------------------------------------
    frames = []

    # Node ID as big-endian 4 bytes
    node_id_be = node_id.to_bytes(4, "big")

    # RouteBegin: data[0..3]=node_id, [4]=route_index, [5]=total_chunks
    begin_payload = node_id_be + bytes([route_index, ROUTE_CHUNKS_PER_ROUTE])
    frames.append(
        can.Message(
            arbitration_id=CFG_ROUTE_BEGIN_ID,
            data=begin_payload,
            is_extended_id=False
        )
    )

    # RouteData: data[0..3]=node_id, [4]=chunk_idx, [5..7]=3 bytes of payload
    for idx, chunk in enumerate(chunks):
        data_payload = node_id_be + bytes([idx]) + chunk
        frames.append(
            can.Message(
                arbitration_id=CFG_ROUTE_DATA_ID,
                data=data_payload,
                is_extended_id=False
            )
        )

    # RouteEnd: data[0..3]=node_id, [4]=route_index, [5]=commitFlag(1)
    end_payload = node_id_be + bytes([route_index, 1])
    frames.append(
        can.Message(
            arbitration_id=CFG_ROUTE_END_ID,
            data=end_payload,
            is_extended_id=False
        )
    )

    print(f"\nBuilt {len(frames)} CAN frames "
          f"({1} begin, {len(chunks)} data, 1 end).")

    # -----------------------------------------------------
    # Transmit via socketcan
    # -----------------------------------------------------
    print("\nSending frames on socketcan: can0 ...\n")
    with can.Bus(interface="socketcan", channel="can0") as bus:
        for f in frames:
            bus.send(f)
        print("Frames sent.\n")


if __name__ == "__main__":
    main()
