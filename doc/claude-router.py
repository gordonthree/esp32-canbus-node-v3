#!/usr/bin/env python3
"""
program_route.py — Interactive CAN route table entry programmer.
Targets socketcan interface 'can0'.
"""

import struct
import can

# ---------------------------------------------------------------------------
# CAN message IDs
# ---------------------------------------------------------------------------
CFG_ROUTE_BEGIN_ID = 0x300
CFG_ROUTE_DATA_ID  = 0x301
CFG_ROUTE_END_ID   = 0x302

ROUTE_CHUNK_SIZE       = 3
ROUTE_ENTRY_SIZE       = 27
ROUTE_CHUNKS_PER_ROUTE = (ROUTE_ENTRY_SIZE + ROUTE_CHUNK_SIZE - 1) // ROUTE_CHUNK_SIZE  # 9

# ---------------------------------------------------------------------------
# Enums (value → name for display)
# ---------------------------------------------------------------------------
EVENT_TYPES = [
    (0,  "EVENT_ALWAYS",              "Fire on every matching message"),
    (1,  "EVENT_ON_BINARY_CHANGE",    "Fire only when payload changes"),
    (2,  "EVENT_ON_BINARY_RISING",    "Fire when value goes 0→1"),
    (3,  "EVENT_ON_BINARY_FALLING",   "Fire when value goes 1→0"),
    (4,  "EVENT_ON_BINARY_MATCH",     "Fire when 3-byte payload is a full match"),
    (5,  "EVENT_ON_FULL_MATCH",       "Fire when 4-byte payload is a full match"),
    (6,  "EVENT_MASKED_BINARY_MATCH", "Fire when 3-byte payload is a partial match"),
    (7,  "EVENT_MASKED_FULL_MATCH",   "Fire when 4-byte payload is a partial match"),
    (8,  "EVENT_THRESHOLD_GT",        "Fire when value > threshold"),
    (9,  "EVENT_THRESHOLD_LT",        "Fire when value < threshold"),
    (10, "EVENT_THRESHOLD_IN_RANGE",  "Fire when value is within range"),
    (11, "EVENT_THRESHOLD_OUT_RANGE", "Fire when value is outside range"),
    (12, "EVENT_THRESHOLD_TOLERANCE", "Fire when value is within tolerance"),
    (13, "EVENT_BIT_SET",             "Fire when bit is set"),
    (14, "EVENT_BIT_CLEAR",           "Fire when bit is clear"),
    (15, "EVENT_BIT_TOGGLE",          "Fire when bit is toggled"),
    (16, "EVENT_BIT_RISING",          "Fire when bit goes 0→1"),
    (17, "EVENT_BIT_FALLING",         "Fire when bit goes 1→0"),
    (18, "EVENT_MATCH_BYTE4",         "Fire when byte 4 matches"),
    (19, "EVENT_MATCH_BYTE5",         "Fire when byte 5 matches"),
    (20, "EVENT_MATCH_BYTE6",         "Fire when byte 6 matches"),
    (21, "EVENT_MATCH_BYTE7",         "Fire when byte 7 matches"),
    (22, "EVENT_IDENTIFIER_MATCH",    "Fire when identifier matches"),
    (23, "EVENT_IDENTIFIER_RANGE",    "Fire when identifier is within range"),
    (31, "EVENT_NEVER",               "Never match — stop here"),
]

ACTION_TYPES = [
    (0, "ACTION_NO_ACTION",           "No action"),
    (1, "ACTION_FOLLOW_BINARY",       "Follow binary value"),
    (2, "ACTION_SEND_DYNAMIC_PAYLOAD","Send synthetic message with dynamic payload"),
    (3, "ACTION_SEND_STATIC_PAYLOAD", "Send synthetic message with static payload"),
]

# ---------------------------------------------------------------------------
# Input helpers
# ---------------------------------------------------------------------------

def prompt_hex(label, bits=32):
    mask = (1 << bits) - 1
    while True:
        raw = input(f"  {label} (hex): ").strip()
        try:
            val = int(raw, 16)
            if 0 <= val <= mask:
                return val
            print(f"  ! Value must fit in {bits} bits (0x0 – 0x{mask:X})")
        except ValueError:
            print("  ! Enter a valid hex value, e.g. 0x1A or 1A")


def prompt_int(label, lo, hi):
    while True:
        raw = input(f"  {label} ({lo}–{hi}): ").strip()
        try:
            val = int(raw, 0)
            if lo <= val <= hi:
                return val
            print(f"  ! Must be between {lo} and {hi}")
        except ValueError:
            print("  ! Enter a valid integer")


def prompt_bool(label):
    while True:
        raw = input(f"  {label} [y/n]: ").strip().lower()
        if raw in ("y", "yes", "1"):
            return True
        if raw in ("n", "no", "0"):
            return False
        print("  ! Enter y or n")


def print_event_table():
    print()
    print(f"  {'Val':<4}  {'Name':<30}  {'Val':<4}  {'Name'}")
    print(f"  {'-'*4}  {'-'*30}  {'-'*4}  {'-'*30}")
    left  = EVENT_TYPES[:len(EVENT_TYPES)//2 + len(EVENT_TYPES)%2]
    right = EVENT_TYPES[len(EVENT_TYPES)//2 + len(EVENT_TYPES)%2:]
    for i, (lv, ln, _) in enumerate(left):
        if i < len(right):
            rv, rn, _ = right[i]
            print(f"  {lv:<4}  {ln:<30}  {rv:<4}  {rn}")
        else:
            print(f"  {lv:<4}  {ln}")
    print()


def print_action_table():
    print()
    print(f"  {'Val':<4}  {'Name':<30}  Description")
    print(f"  {'-'*4}  {'-'*30}  {'-'*40}")
    for v, n, d in ACTION_TYPES:
        print(f"  {v:<4}  {n:<30}  {d}")
    print()

# ---------------------------------------------------------------------------
# Struct packing
# ---------------------------------------------------------------------------

def pack_route_entry(f):
    """
    Pack route_entry_t (27 bytes) little-endian.

    Layout (offsets):
      0  source_node_id   uint32
      4  source_msg_id    uint16
      6  source_msg_dlc   uint8
      7  source_sub_idx   uint8
      8  target_msg_id    uint16
     10  target_msg_dlc   uint8
     11  target_sub_idx   uint8
     12  ea               uint8
     13  flags            uint8
     14  union1           uint32  (match_value / threshold_low / etc.)
     18  union2           uint32  (match_mask / threshold_high / etc.)
     22  parameters[3]    3 × uint8
     25  reserved[2]      2 × uint8
    """
    return struct.pack(
        "<IHBBHBBBBII3B2B",
        f["source_node_id"],
        f["source_msg_id"],
        f["source_msg_dlc"],
        f["source_sub_idx"],
        f["target_msg_id"],
        f["target_msg_dlc"],
        f["target_sub_idx"],
        f["ea"],
        f["flags"],
        f["match_value"],
        f["match_mask"],
        f["parameters"][0],
        f["parameters"][1],
        f["parameters"][2],
        0, 0,  # reserved
    )

# ---------------------------------------------------------------------------
# Frame builders
# ---------------------------------------------------------------------------

def node_id_bytes(node_id):
    """4 bytes, big-endian."""
    return struct.pack(">I", node_id)


def build_begin_frame(node_id, route_idx):
    data = node_id_bytes(node_id) + bytes([route_idx, ROUTE_CHUNKS_PER_ROUTE])
    return can.Message(arbitration_id=CFG_ROUTE_BEGIN_ID, data=data, is_extended_id=False)


def build_data_frames(node_id, route_bytes):
    frames = []
    prefix = node_id_bytes(node_id)
    for chunk_idx in range(ROUTE_CHUNKS_PER_ROUTE):
        base = chunk_idx * ROUTE_CHUNK_SIZE
        chunk = route_bytes[base:base + ROUTE_CHUNK_SIZE]
        # Pad last chunk to 3 bytes if needed
        chunk = chunk.ljust(3, b'\x00')
        data = prefix + bytes([chunk_idx]) + chunk
        frames.append(can.Message(arbitration_id=CFG_ROUTE_DATA_ID, data=data, is_extended_id=False))
    return frames


def build_end_frame(node_id, route_idx):
    data = node_id_bytes(node_id) + bytes([route_idx, 0x01])  # commit=1
    return can.Message(arbitration_id=CFG_ROUTE_END_ID, data=data, is_extended_id=False)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=== CAN Route Table Programmer ===\n")

    # Node ID
    print("Node ID")
    node_id = prompt_hex("node_id (uint32)", bits=32)

    # Route index
    print("\nRoute Index")
    route_idx = prompt_int("route_index", 0, 79)

    # Source fields
    print("\n--- Source ---")
    source_node_id = prompt_hex("source_node_id (uint32)", bits=32)
    source_msg_id  = prompt_hex("source_msg_id  (uint16)", bits=16)
    source_msg_dlc = prompt_int("source_msg_dlc", 0, 8)
    source_sub_idx = prompt_int("source_sub_idx", 0, 255)

    # Target fields
    print("\n--- Target ---")
    target_msg_id  = prompt_hex("target_msg_id  (uint16)", bits=16)
    target_msg_dlc = prompt_int("target_msg_dlc", 0, 8)
    target_sub_idx = prompt_int("target_sub_idx", 0, 255)

    # Event type
    print("\n--- Event Type ---")
    print_event_table()
    valid_event_vals = [e[0] for e in EVENT_TYPES]
    while True:
        event_val = prompt_int("event_type value", 0, 31)
        if event_val in valid_event_vals:
            break
        print(f"  ! {event_val} is not a defined event type. Valid values: {valid_event_vals}")

    # Action type
    print("\n--- Action Type ---")
    print_action_table()
    action_val = prompt_int("action_type value", 0, 3)

    # Pack ea byte: action_type in bits [7:5], event_type in bits [4:0]
    ea = ((action_val & 0x07) << 5) | (event_val & 0x1F)

    # Flags
    print("\n--- Flags ---")
    flag_enabled       = prompt_bool("enabled        (route is active)")
    flag_wildcard      = prompt_bool("wildcard       (match any source node)")
    flag_dynamic_params= prompt_bool("dynamic_params (use dynamic payload params)")
    flag_in_use        = prompt_bool("inUse          (slot is in use)")
    flags = (
        (flag_enabled        & 1) << 0 |
        (flag_wildcard       & 1) << 1 |
        (flag_dynamic_params & 1) << 2 |
        (flag_in_use         & 1) << 3
    )

    # Comparison unions
    print("\n--- Comparison Fields ---")
    match_value = prompt_hex("match_value  (union1 / uint32)", bits=32)
    match_mask  = prompt_hex("match_mask   (union2 / uint32)", bits=32)

    # Parameters
    print("\n--- Static Payload Parameters ---")
    params = []
    for i in range(3):
        params.append(prompt_int(f"parameters[{i}]", 0, 255))

    # Build and display summary
    fields = {
        "source_node_id": source_node_id,
        "source_msg_id":  source_msg_id,
        "source_msg_dlc": source_msg_dlc,
        "source_sub_idx": source_sub_idx,
        "target_msg_id":  target_msg_id,
        "target_msg_dlc": target_msg_dlc,
        "target_sub_idx": target_sub_idx,
        "ea":             ea,
        "flags":          flags,
        "match_value":    match_value,
        "match_mask":     match_mask,
        "parameters":     params,
    }

    route_bytes = pack_route_entry(fields)
    assert len(route_bytes) == ROUTE_ENTRY_SIZE, f"Packed size {len(route_bytes)} != {ROUTE_ENTRY_SIZE}"

    print("\n=== Summary ===")
    print(f"  node_id       : 0x{node_id:08X}")
    print(f"  route_index   : {route_idx}")
    print(f"  source_node_id: 0x{source_node_id:08X}")
    print(f"  source_msg_id : 0x{source_msg_id:04X}  dlc={source_msg_dlc}  sub={source_sub_idx}")
    print(f"  target_msg_id : 0x{target_msg_id:04X}  dlc={target_msg_dlc}  sub={target_sub_idx}")
    print(f"  ea            : 0x{ea:02X}  (event={event_val}, action={action_val})")
    print(f"  flags         : 0x{flags:02X}  (enabled={int(flag_enabled)}, wildcard={int(flag_wildcard)}, dynamic={int(flag_dynamic_params)}, inUse={int(flag_in_use)})")
    print(f"  match_value   : 0x{match_value:08X}")
    print(f"  match_mask    : 0x{match_mask:08X}")
    print(f"  parameters    : {[hex(p) for p in params]}")
    print(f"  packed bytes  : {route_bytes.hex(' ').upper()}")
    print(f"  chunks        : {ROUTE_CHUNKS_PER_ROUTE}")

    confirm = prompt_bool("\nSend frames?")
    if not confirm:
        print("Aborted.")
        return

    frames  = [build_begin_frame(node_id, route_idx)]
    frames += build_data_frames(node_id, route_bytes)
    frames += [build_end_frame(node_id, route_idx)]

    with can.Bus(interface="socketcan", channel="can0") as bus:
        for f in frames:
            bus.send(f)
        print(f"Sent {len(frames)} frames ({1} BEGIN + {ROUTE_CHUNKS_PER_ROUTE} DATA + {1} END).")


if __name__ == "__main__":
    main()