import can
import struct

# Constants from C structures 
CFG_ROUTE_BEGIN_ID = 0x300
CFG_ROUTE_DATA_ID  = 0x301
CFG_ROUTE_END_ID   = 0x302

ROUTE_ENTRY_SIZE = 27
ROUTE_CHUNK_SIZE = 3
TOTAL_CHUNKS = 9 # ROUTE_ENTRY_SIZE // ROUTE_CHUNK_SIZE 

# Enum Definitions [cite: 25, 30]
EVENTS = {
    0: "EVENT_ALWAYS", 1: "EVENT_ON_BINARY_CHANGE", 2: "EVENT_ON_BINARY_RISING",
    3: "EVENT_ON_BINARY_FALLING", 4: "EVENT_ON_BINARY_MATCH", 5: "EVENT_ON_FULL_MATCH",
    6: "EVENT_MASKED_BINARY_MATCH", 7: "EVENT_MASKED_FULL_MATCH", 8: "EVENT_THRESHOLD_GT",
    9: "EVENT_THRESHOLD_LT", 10: "EVENT_THRESHOLD_IN_RANGE", 11: "EVENT_THRESHOLD_OUT_RANGE",
    12: "EVENT_THRESHOLD_TOLERANCE", 13: "EVENT_BIT_SET", 14: "EVENT_BIT_CLEAR",
    15: "EVENT_BIT_TOGGLE", 16: "EVENT_BIT_RISING", 17: "EVENT_BIT_FALLING",
    18: "EVENT_MATCH_BYTE4", 19: "EVENT_MATCH_BYTE5", 20: "EVENT_MATCH_BYTE6",
    21: "EVENT_MATCH_BYTE7", 22: "EVENT_IDENTIFIER_MATCH", 23: "EVENT_IDENTIFIER_RANGE",
    31: "EVENT_NEVER"
}

ACTIONS = {
    0: "ACTION_NO_ACTION", 1: "ACTION_FOLLOW_BINARY", 
    2: "ACTION_SEND_DYNAMIC_PAYLOAD", 3: "ACTION_SEND_STATIC_PAYLOAD"
}

def display_tables():
    """Displays enums in the requested table formats[cite: 38, 39]."""
    print("\n--- EVENT TYPES ---")
    ev_list = list(EVENTS.items())
    for i in range(0, len(ev_list), 2):
        left = f"{ev_list[i][0]}: {ev_list[i][1]}"
        right = f"{ev_list[i+1][0]}: {ev_list[i+1][1]}" if i+1 < len(ev_list) else ""
        print(f"{left:<30} {right}")

    print("\n--- ACTION TYPES ---")
    for k, v in ACTIONS.items():
        print(f"{k}: {v}")

def get_input(prompt, min_val, max_val, is_hex=False):
    while True:
        try:
            val_str = input(prompt)
            val = int(val_str, 16) if is_hex or val_str.startswith("0x") else int(val_str)
            if min_val <= val <= max_val:
                return val
            print(f"Error: Value must be between {min_val} and {max_val}")
        except ValueError:
            print("Invalid input. Please enter a number.")

def pack_ea(event, action):
    """Packs 5-bit event and 3-bit action into 1 byte[cite: 21, 22, 37]."""
    return (action << 5) | (event & 0x1F)

def pack_flags():
    """Prompts for flags and packs them into 1 byte[cite: 23, 24, 40]."""
    print("\n--- ROUTE FLAGS ---")
    enabled = get_input("Enabled? (1=Yes, 0=No): ", 0, 1)
    wildcard = get_input("Wildcard? (1=Yes, 0=No): ", 0, 1)
    dyn_params = get_input("Dynamic Params? (1=Yes, 0=No): ", 0, 1)
    in_use = get_input("In Use? (1=Yes, 0=No): ", 0, 1)
    return (in_use << 3) | (dyn_params << 2) | (wildcard << 1) | enabled

def main():
    # 1. Global Node ID and Index 
    node_id = get_input("Enter Node ID (e.g., 0x25A56D84): ", 0, 0xFFFFFFFF, is_hex=True)
    node_id_bytes = struct.pack(">I", node_id) # Big Endian prefix 
    
    route_idx = get_input("Enter Route Index (0-79): ", 0, 79)

    # 2. Source/Target Info
    src_node = get_input("Source Node ID (uint32): ", 0, 0xFFFFFFFF)
    src_msg = get_input("Source CAN Msg ID (uint16): ", 0, 0xFFFF)
    src_dlc = get_input("Source DLC (0-8): ", 0, 8)
    src_sub = get_input("Source Submodule Index: ", 0, 255)
    
    tgt_msg = get_input("Target CAN Msg ID (uint16): ", 0, 0xFFFF)
    tgt_dlc = get_input("Target DLC (0-8): ", 0, 8)
    tgt_sub = get_input("Target Submodule Index: ", 0, 255)

    # 3. EA and Flags 
    display_tables()
    event = get_input("Select Event Type ID: ", 0, 31)
    action = get_input("Select Action Type ID: ", 0, 7)
    ea_byte = pack_ea(event, action)
    flags_byte = pack_flags()

    # 4. Match Values (Unions) [cite: 42]
    match_val = get_input("Enter Match Value / Threshold Low (uint32): ", 0, 0xFFFFFFFF)
    match_mask = get_input("Enter Match Mask / Threshold High (uint32): ", 0, 0xFFFFFFFF)
    
    params = [get_input(f"Parameter byte {i} (0-255): ", 0, 255) for i in range(3)]

    # 5. Build the 27-byte struct [cite: 6, 20]
    # Format: < (Little Endian), I (4), H (2), B (1), B (1), H (2), B (1), B (1), B (1), B (1), I (4), I (4), 3B (3), 2B (2)
    struct_data = struct.pack("<IHB B HB B BB II 3B 2B", 
        src_node, src_msg, src_dlc, src_sub,
        tgt_msg, tgt_dlc, tgt_sub,
        ea_byte, flags_byte,
        match_val, match_mask,
        params[0], params[1], params[2],
        0, 0 # Reserved bytes [cite: 20]
    )

    # 6. Transmission via SocketCAN [cite: 32, 33, 43]
    try:
        with can.Bus(interface="socketcan", channel="can0") as bus:
            # A. Send Route Begin 
            # Data: [NodeID 0-3][Idx 4][Count 5][0 6][0 7]
            begin_data = node_id_bytes + struct.pack("BBxx", route_idx, TOTAL_CHUNKS)
            bus.send(can.Message(arbitration_id=CFG_ROUTE_BEGIN_ID, data=begin_data, is_extended_id=False))

            # B. Send Route Data Chunks [cite: 53, 56]
            # Data: [NodeID 0-3][ChunkIdx 4][Payload 5-7]
            for i in range(TOTAL_CHUNKS):
                chunk_payload = struct_data[i*3 : (i*3)+3]
                data_frame = node_id_bytes + struct.pack("B", i) + chunk_payload
                bus.send(can.Message(arbitration_id=CFG_ROUTE_DATA_ID, data=data_frame, is_extended_id=False))

            # C. Send Route End [cite: 62, 63]
            # Data: [NodeID 0-3][Idx 4][Commit 5][0 6][0 7]
            end_data = node_id_bytes + struct.pack("BBxx", route_idx, 1) # Commit = 1
            bus.send(can.Message(arbitration_id=CFG_ROUTE_END_ID, data=end_data, is_extended_id=False))
            
            print(f"\nSuccessfully transmitted route {route_idx} to node {hex(node_id)}.")
            
    except Exception as e:
        print(f"CAN Error: {e}")

if __name__ == "__main__":
    main()