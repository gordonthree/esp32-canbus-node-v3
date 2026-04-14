#!/usr/bin/env python3
"""
CAN Route Table Programmer TUI
Uses Rich for rendering and Blessed for raw keyboard input.

pip install python-can rich blessed
"""

import sys
import os
import csv
import time
import struct
import threading
import queue
from copy import deepcopy
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict

try:
    import can
except ImportError:
    print("ERROR: python-can not installed. Run: pip install python-can")
    sys.exit(1)

try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.text import Text
    from rich.prompt import Prompt, Confirm
    from rich.columns import Columns
    from rich import box
    from rich.style import Style
    from rich.live import Live
    from rich.layout import Layout
    from rich.align import Align
except ImportError:
    print("ERROR: rich not installed. Run: pip install rich")
    sys.exit(1)

try:
    from blessed import Terminal
except ImportError:
    print("ERROR: blessed not installed. Run: pip install blessed")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CAN_INTERFACE       = "can0"

CFG_ROUTE_BEGIN     = 0x300
CFG_ROUTE_DATA      = 0x301
CFG_ROUTE_END       = 0x302
DATA_ROUTE_ACK      = 0x303
REQ_ROUTE_LIST      = 0x308
ROUTE_LIST_BEGIN    = 0x309
ROUTE_LIST_END      = 0x30B

CFG_ROUTE_BEGIN_DLC = 6
CFG_ROUTE_DATA_DLC  = 8
CFG_ROUTE_END_DLC   = 6
DATA_ROUTE_ACK_DLC  = 7
REQ_ROUTE_LIST_DLC  = 4
ROUTE_LIST_BEGIN_DLC = 7
ROUTE_LIST_END_DLC   = 7

ROUTE_CHUNK_SIZE        = 3
ROUTE_ENTRY_SIZE        = 27
ROUTE_CHUNKS_PER_ROUTE  = (ROUTE_ENTRY_SIZE + ROUTE_CHUNK_SIZE - 1) // ROUTE_CHUNK_SIZE  # = 9
MAX_ROUTES              = 80

ROUTE_FLAG_ENABLED        = (1 << 0)
ROUTE_FLAG_WILDCARD       = (1 << 1)
ROUTE_FLAG_DYNAMIC_PARAMS = (1 << 2)
ROUTE_FLAG_INUSE          = (1 << 3)

HISTORY_FILE = "node_id_history.txt"
CSV_FILE     = "route_table.csv"

RX_TIMEOUT_SECS = 2.0

# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

EVENT_TYPE_MAP = {
    0:  "EVENT_ALWAYS",
    1:  "EVENT_ON_BINARY_CHANGE",
    2:  "EVENT_ON_BINARY_RISING",
    3:  "EVENT_ON_BINARY_FALLING",
    4:  "EVENT_ON_BINARY_MATCH",
    5:  "EVENT_ON_FULL_MATCH",
    6:  "EVENT_MASKED_BINARY_MATCH",
    7:  "EVENT_MASKED_FULL_MATCH",
    8:  "EVENT_THRESHOLD_GT",
    9:  "EVENT_THRESHOLD_LT",
    10: "EVENT_THRESHOLD_IN_RANGE",
    11: "EVENT_THRESHOLD_OUT_RANGE",
    12: "EVENT_THRESHOLD_TOLERANCE",
    13: "EVENT_BIT_SET",
    14: "EVENT_BIT_CLEAR",
    15: "EVENT_BIT_TOGGLE",
    16: "EVENT_BIT_RISING",
    17: "EVENT_BIT_FALLING",
    18: "EVENT_MATCH_BYTE4",
    19: "EVENT_MATCH_BYTE5",
    20: "EVENT_MATCH_BYTE6",
    21: "EVENT_MATCH_BYTE7",
    22: "EVENT_IDENTIFIER_MATCH",
    23: "EVENT_IDENTIFIER_RANGE",
    31: "EVENT_NEVER",
}
EVENT_NAME_TO_VAL = {v: k for k, v in EVENT_TYPE_MAP.items()}

ACTION_TYPE_MAP = {
    0: "ACTION_NO_ACTION",
    1: "ACTION_FOLLOW_BINARY",
    2: "ACTION_SEND_DYNAMIC_PAYLOAD",
    3: "ACTION_SEND_STATIC_PAYLOAD",
    4: "ACTION_USE_32BIT_PAYLOAD",
    5: "ACTION_USE_64BIT_PAYLOAD",
    6: "ACTION_RESERVED_6",
    7: "ACTION_RESERVED_7",
}
ACTION_NAME_TO_VAL = {v: k for k, v in ACTION_TYPE_MAP.items()}

# ---------------------------------------------------------------------------
# route_entry_t  (packed, 27 bytes + 1 reserved = 28 bytes in struct,
#                 but ROUTE_ENTRY_SIZE = 27 so reserved byte IS included)
#
# Layout (little-endian):
#   source_node_id  : uint32  (4)
#   source_msg_id   : uint16  (2)
#   source_msg_dlc  : uint8   (1)
#   source_sub_idx  : uint8   (1)
#   target_msg_id   : uint16  (2)
#   target_msg_dlc  : uint8   (1)
#   target_sub_idx  : uint8   (1)
#   event_type      : uint8   (1)
#   action_type     : uint8   (1)
#   route_flags     : uint8   (1)
#   match_value     : uint32  (4)
#   match_mask      : uint32  (4)
#   parameters[3]   : uint8*3 (3)
#   reserved        : uint8   (1)
#   Total = 27 bytes  (ROUTE_ENTRY_SIZE)
# ---------------------------------------------------------------------------

ROUTE_STRUCT_FMT = "<IHBBHBBBBBIIBBBx"
# Breakdown:
#  I  = source_node_id  (4)
#  H  = source_msg_id   (2)
#  B  = source_msg_dlc  (1)
#  B  = source_sub_idx  (1)
#  H  = target_msg_id   (2)
#  B  = target_msg_dlc  (1)
#  B  = target_sub_idx  (1)
#  B  = event_type      (1)
#  B  = action_type     (1)
#  B  = route_flags     (1)
#  I  = match_value     (4)
#  I  = match_mask      (4)
#  B  = parameters[0]   (1)
#  B  = parameters[1]   (1)
#  B  = parameters[2]   (1)
#  x  = reserved pad    (1)
# Total packed = 27 bytes (the 'x' pads to 28 but struct.pack gives 28;
# however ROUTE_ENTRY_SIZE=27 means we only send/receive 27 bytes.
# We'll slice to 27 on TX and zero-pad to 28 on RX.

assert struct.calcsize(ROUTE_STRUCT_FMT) == 27  # 26 data bytes + 1 reserved = 27


@dataclass
class RouteEntry:
    route_index:    int   = 0
    source_node_id: int   = 0
    source_msg_id:  int   = 0
    source_msg_dlc: int   = 0
    source_sub_idx: int   = 0
    target_msg_id:  int   = 0
    target_msg_dlc: int   = 0
    target_sub_idx: int   = 0
    event_type:     int   = 0
    action_type:    int   = 0
    route_flags:    int   = 0
    match_value:    int   = 0
    match_mask:     int   = 0
    param0:         int   = 0
    param1:         int   = 0
    param2:         int   = 0

    def serialize(self) -> bytes:
        """Pack to 27 bytes (ROUTE_ENTRY_SIZE), little-endian."""
        raw = struct.pack(
            ROUTE_STRUCT_FMT,
            self.source_node_id,
            self.source_msg_id,
            self.source_msg_dlc,
            self.source_sub_idx,
            self.target_msg_id,
            self.target_msg_dlc,
            self.target_sub_idx,
            self.event_type,
            self.action_type,
            self.route_flags,
            self.match_value,
            self.match_mask,
            self.param0,
            self.param1,
            self.param2,
        )
        return raw[:ROUTE_ENTRY_SIZE]  # trim reserved pad byte → 27 bytes

    @staticmethod
    def deserialize(idx: int, data: bytes) -> "RouteEntry":
        """Unpack from 27 bytes."""
        buf = data[:ROUTE_ENTRY_SIZE]
        (src_nid, src_mid, src_dlc, src_sub,
         tgt_mid, tgt_dlc, tgt_sub,
         evt, act, flags,
         mv, mm,
         p0, p1, p2) = struct.unpack(ROUTE_STRUCT_FMT, buf)
        return RouteEntry(
            route_index=idx,
            source_node_id=src_nid,
            source_msg_id=src_mid,
            source_msg_dlc=src_dlc,
            source_sub_idx=src_sub,
            target_msg_id=tgt_mid,
            target_msg_dlc=tgt_dlc,
            target_sub_idx=tgt_sub,
            event_type=evt,
            action_type=act,
            route_flags=flags,
            match_value=mv,
            match_mask=mm,
            param0=p0,
            param1=p1,
            param2=p2,
        )


# ---------------------------------------------------------------------------
# CRC16-CCITT  (direct port of C routine)
# ---------------------------------------------------------------------------

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Node ID history
# ---------------------------------------------------------------------------

def load_history() -> List[str]:
    if not os.path.exists(HISTORY_FILE):
        return []
    with open(HISTORY_FILE, "r") as f:
        lines = [l.strip() for l in f.readlines() if l.strip()]
    return lines


def save_history(history: List[str]):
    with open(HISTORY_FILE, "w") as f:
        for h in history:
            f.write(h + "\n")


def add_to_history(node_id_str: str, history: List[str]) -> List[str]:
    if node_id_str in history:
        history.remove(node_id_str)
    history.insert(0, node_id_str)
    return history[:10]  # keep last 10


# ---------------------------------------------------------------------------
# CSV persistence
# ---------------------------------------------------------------------------

CSV_FIELDS = [
    "route_index", "source_node_id", "source_msg_id", "source_msg_dlc",
    "source_sub_idx", "target_msg_id", "target_msg_dlc", "target_sub_idx",
    "event_type", "action_type", "route_flags",
    "match_value", "match_mask", "param0", "param1", "param2",
]


def load_csv() -> Dict[int, RouteEntry]:
    routes: Dict[int, RouteEntry] = {}
    if not os.path.exists(CSV_FILE):
        return routes
    with open(CSV_FILE, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                idx = int(row["route_index"])
                evt_raw = row["event_type"]
                act_raw = row["action_type"]
                evt = EVENT_NAME_TO_VAL.get(evt_raw, int(evt_raw, 0) if evt_raw.startswith("0x") else int(evt_raw))
                act = ACTION_NAME_TO_VAL.get(act_raw, int(act_raw, 0) if act_raw.startswith("0x") else int(act_raw))
                e = RouteEntry(
                    route_index    = idx,
                    source_node_id = int(row["source_node_id"], 0),
                    source_msg_id  = int(row["source_msg_id"], 0),
                    source_msg_dlc = int(row["source_msg_dlc"]),
                    source_sub_idx = int(row["source_sub_idx"]),
                    target_msg_id  = int(row["target_msg_id"], 0),
                    target_msg_dlc = int(row["target_msg_dlc"]),
                    target_sub_idx = int(row["target_sub_idx"]),
                    event_type     = evt,
                    action_type    = act,
                    route_flags    = int(row["route_flags"], 0),
                    match_value    = int(row["match_value"], 0),
                    match_mask     = int(row["match_mask"], 0),
                    param0         = int(row["param0"]),
                    param1         = int(row["param1"]),
                    param2         = int(row["param2"]),
                )
                routes[idx] = e
            except (KeyError, ValueError) as ex:
                print(f"[CSV] Warning: skipping malformed row: {ex}")
    return routes


def save_csv(routes: Dict[int, RouteEntry]):
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for idx in sorted(routes.keys()):
            e = routes[idx]
            writer.writerow({
                "route_index":    e.route_index,
                "source_node_id": hex(e.source_node_id),
                "source_msg_id":  hex(e.source_msg_id),
                "source_msg_dlc": e.source_msg_dlc,
                "source_sub_idx": e.source_sub_idx,
                "target_msg_id":  hex(e.target_msg_id),
                "target_msg_dlc": e.target_msg_dlc,
                "target_sub_idx": e.target_sub_idx,
                "event_type":     EVENT_TYPE_MAP.get(e.event_type, str(e.event_type)),
                "action_type":    ACTION_TYPE_MAP.get(e.action_type, str(e.action_type)),
                "route_flags":    hex(e.route_flags),
                "match_value":    hex(e.match_value),
                "match_mask":     hex(e.match_mask),
                "param0":         e.param0,
                "param1":         e.param1,
                "param2":         e.param2,
            })


# ---------------------------------------------------------------------------
# CAN helpers – frame construction
# ---------------------------------------------------------------------------

def node_id_bytes(node_id: int) -> bytes:
    return node_id.to_bytes(4, "big")


def make_req_route_list(node_id: int) -> can.Message:
    return can.Message(
        arbitration_id=REQ_ROUTE_LIST,
        data=node_id_bytes(node_id),
        dlc=REQ_ROUTE_LIST_DLC,
        is_extended_id=False,
    )


def make_route_frames(node_id: int, route_idx: int, entry: RouteEntry) -> List[can.Message]:
    frames = []
    raw = entry.serialize()  # 27 bytes
    nid = node_id_bytes(node_id)

    # BEGIN
    begin_data = bytearray(nid) + bytes([route_idx, ROUTE_CHUNKS_PER_ROUTE])
    frames.append(can.Message(
        arbitration_id=CFG_ROUTE_BEGIN,
        data=bytes(begin_data),
        dlc=CFG_ROUTE_BEGIN_DLC,
        is_extended_id=False,
    ))

    # DATA chunks
    for chunk_idx in range(ROUTE_CHUNKS_PER_ROUTE):
        base = chunk_idx * ROUTE_CHUNK_SIZE
        chunk = raw[base:base + ROUTE_CHUNK_SIZE]
        # pad if last chunk is short
        chunk = chunk.ljust(ROUTE_CHUNK_SIZE, b'\x00')
        data_payload = bytearray(nid) + bytes([chunk_idx]) + chunk
        frames.append(can.Message(
            arbitration_id=CFG_ROUTE_DATA,
            data=bytes(data_payload),
            dlc=CFG_ROUTE_DATA_DLC,
            is_extended_id=False,
        ))

    # END  (commit flag = 0x01)
    end_data = bytearray(nid) + bytes([route_idx, 0x01])
    frames.append(can.Message(
        arbitration_id=CFG_ROUTE_END,
        data=bytes(end_data),
        dlc=CFG_ROUTE_END_DLC,
        is_extended_id=False,
    ))

    return frames


# ---------------------------------------------------------------------------
# CAN RX state machine  (runs in background thread)
# ---------------------------------------------------------------------------

class RxState:
    IDLE          = "IDLE"
    WAIT_BEGIN    = "WAIT_BEGIN"
    RECEIVING     = "RECEIVING"
    DONE          = "DONE"


class RouteRxStateMachine:
    def __init__(self, node_id: int, result_queue: queue.Queue):
        self.node_id = node_id
        self.result_queue = result_queue
        self.state = RxState.WAIT_BEGIN

        self.active_count   = 0
        self.total_frames   = 0
        self.frames_seen    = 0

        self.current_slot   = None
        self.current_total  = 0
        self.current_recv   = 0
        self.rx_buf         = bytearray(ROUTE_ENTRY_SIZE)

        self.completed: Dict[int, RouteEntry] = {}
        self.start_time = time.monotonic()

    def _check_node_id(self, data: bytes) -> bool:
        rx_nid = int.from_bytes(data[0:4], "big")
        return rx_nid == self.node_id

    def feed(self, msg: can.Message) -> bool:
        """Returns True when transfer is complete."""
        arb = msg.arbitration_id
        d   = msg.data

        if self.state == RxState.DONE:
            return True

        if time.monotonic() - self.start_time > RX_TIMEOUT_SECS:
            self.result_queue.put(("timeout", None))
            self.state = RxState.DONE
            return True

        if arb == ROUTE_LIST_BEGIN:
            if not self._check_node_id(d):
                return False
            self.active_count = d[4]
            self.total_frames = (d[5] << 8) | d[6]
            self.state = RxState.RECEIVING
            return False

        if arb == CFG_ROUTE_BEGIN and self.state == RxState.RECEIVING:
            if not self._check_node_id(d):
                return False
            self.current_slot  = d[4]
            self.current_total = d[5]
            self.current_recv  = 0
            self.rx_buf        = bytearray(ROUTE_ENTRY_SIZE)
            return False

        if arb == CFG_ROUTE_DATA and self.state == RxState.RECEIVING:
            if not self._check_node_id(d):
                return False
            chunk_idx = d[4]
            base = chunk_idx * ROUTE_CHUNK_SIZE
            for i in range(ROUTE_CHUNK_SIZE):
                if base + i < ROUTE_ENTRY_SIZE:
                    self.rx_buf[base + i] = d[5 + i]
            self.current_recv += 1
            self.frames_seen  += 1
            return False

        if arb == CFG_ROUTE_END and self.state == RxState.RECEIVING:
            if not self._check_node_id(d):
                return False
            route_idx  = d[4]
            # commit flag (d[5]) ignored on rx per spec
            if route_idx == self.current_slot and self.current_recv == self.current_total:
                entry = RouteEntry.deserialize(route_idx, bytes(self.rx_buf))
                self.completed[route_idx] = entry
            self.current_slot = None
            return False

        if arb == ROUTE_LIST_END:
            if not self._check_node_id(d):
                return False
            self.result_queue.put(("ok", self.completed))
            self.state = RxState.DONE
            return True

        return False


def receive_route_table(node_id: int, bus: can.Bus, console: Console) -> Optional[Dict[int, RouteEntry]]:
    """
    Sends REQ_ROUTE_LIST and collects the full transfer.
    Returns dict of routes on success, None on timeout/error.
    """
    result_q: queue.Queue = queue.Queue()
    sm = RouteRxStateMachine(node_id, result_q)

    done_event = threading.Event()

    def rx_thread():
        while not done_event.is_set():
            msg = bus.recv(timeout=0.1)
            if msg is None:
                if time.monotonic() - sm.start_time > RX_TIMEOUT_SECS:
                    result_q.put(("timeout", None))
                    break
                continue
            if sm.feed(msg):
                break

    t = threading.Thread(target=rx_thread, daemon=True)
    t.start()

    # Send the request
    req = make_req_route_list(node_id)
    bus.send(req)

    t.join(timeout=RX_TIMEOUT_SECS + 0.5)
    done_event.set()

    if result_q.empty():
        console.print("[bold red]⏱  Timeout: no response from node within 2 seconds.[/bold red]")
        return None

    status, data = result_q.get_nowait()
    if status == "timeout":
        console.print("[bold red]⏱  Timeout: route table transfer did not complete within 2 seconds.[/bold red]")
        return None

    return data


def send_route_entry(node_id: int, route_idx: int, entry: RouteEntry,
                     bus: can.Bus, console: Console) -> bool:
    """
    Serializes and sends a route entry; waits for DATA_ROUTE_ACK.
    Returns True on success (CRC match), False otherwise.
    """
    frames = make_route_frames(node_id, route_idx, entry)

    ack_q: queue.Queue = queue.Queue()
    done_event = threading.Event()

    def ack_listener():
        deadline = time.monotonic() + RX_TIMEOUT_SECS
        while not done_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                ack_q.put(("timeout", None))
                return
            msg = bus.recv(timeout=min(0.1, remaining))
            if msg is None:
                continue
            if msg.arbitration_id == DATA_ROUTE_ACK and len(msg.data) >= 7:
                rx_nid = int.from_bytes(msg.data[0:4], "big")
                if rx_nid == node_id and msg.data[4] == route_idx:
                    remote_crc = (msg.data[5] << 8) | msg.data[6]
                    ack_q.put(("ack", remote_crc))
                    return

    t = threading.Thread(target=ack_listener, daemon=True)
    t.start()

    for f in frames:
        bus.send(f)

    t.join(timeout=RX_TIMEOUT_SECS + 0.5)
    done_event.set()

    if ack_q.empty():
        console.print("[bold red]⏱  Timeout: no ACK received from node.[/bold red]")
        return False

    status, remote_crc = ack_q.get_nowait()
    if status == "timeout":
        console.print("[bold red]⏱  Timeout: no ACK received from node.[/bold red]")
        return False

    local_crc = crc16_ccitt(entry.serialize())
    if local_crc != remote_crc:
        console.print(
            f"[bold yellow]⚠  CRC mismatch! Local: 0x{local_crc:04X}  Remote: 0x{remote_crc:04X}[/bold yellow]"
        )
        return False

    console.print(f"[bold green]✓  Route {route_idx} committed. CRC: 0x{local_crc:04X}[/bold green]")
    return True


# ---------------------------------------------------------------------------
# Rich rendering helpers
# ---------------------------------------------------------------------------

console = Console()


def flags_str(flags: int) -> str:
    parts = []
    if flags & ROUTE_FLAG_ENABLED:        parts.append("EN")
    if flags & ROUTE_FLAG_WILDCARD:       parts.append("WC")
    if flags & ROUTE_FLAG_DYNAMIC_PARAMS: parts.append("DYN")
    if flags & ROUTE_FLAG_INUSE:          parts.append("INUSE")
    return "|".join(parts) if parts else "0x00"


def render_route_table(routes: Dict[int, RouteEntry], selected: int, node_id: int) -> Table:
    table = Table(
        title=f"Route Table  —  Node: 0x{node_id:08X}",
        box=box.SIMPLE_HEAVY,
        highlight=True,
        show_lines=True,
    )
    table.add_column("#",           style="bold cyan",    width=4,  justify="right")
    table.add_column("Idx",         style="cyan",         width=4,  justify="right")
    table.add_column("Src Node",    style="white",        width=12)
    table.add_column("Src MID",     style="white",        width=8)
    table.add_column("Src DLC",     width=5, justify="center")
    table.add_column("Tgt MID",     style="white",        width=8)
    table.add_column("Tgt DLC",     width=5, justify="center")
    table.add_column("Event",       style="yellow",       width=26)
    table.add_column("Action",      style="magenta",      width=24)
    table.add_column("Flags",       style="green",        width=16)
    table.add_column("Match Val",   style="white",        width=10)
    table.add_column("Match Mask",  style="white",        width=10)

    sorted_keys = sorted(routes.keys())
    for row_num, idx in enumerate(sorted_keys):
        e = routes[idx]
        is_sel = (row_num == selected)
        style = Style(bgcolor="dark_blue", bold=True) if is_sel else Style()
        prefix = "▶" if is_sel else " "

        table.add_row(
            f"{prefix}{row_num}",
            str(e.route_index),
            f"0x{e.source_node_id:08X}",
            f"0x{e.source_msg_id:03X}",
            str(e.source_msg_dlc),
            f"0x{e.target_msg_id:03X}",
            str(e.target_msg_dlc),
            EVENT_TYPE_MAP.get(e.event_type, str(e.event_type)),
            ACTION_TYPE_MAP.get(e.action_type, str(e.action_type)),
            flags_str(e.route_flags),
            f"0x{e.match_value:08X}",
            f"0x{e.match_mask:08X}",
            style=style,
        )
    return table


def render_summary_table(entry: RouteEntry) -> Table:
    table = Table(title="Route Entry Summary", box=box.ROUNDED, show_header=True)
    table.add_column("Field", style="bold cyan", width=20)
    table.add_column("Value", style="white")

    def add(name, val):
        table.add_row(name, str(val))

    add("route_index",    entry.route_index)
    add("source_node_id", f"0x{entry.source_node_id:08X}")
    add("source_msg_id",  f"0x{entry.source_msg_id:03X}")
    add("source_msg_dlc", entry.source_msg_dlc)
    add("source_sub_idx", entry.source_sub_idx)
    add("target_msg_id",  f"0x{entry.target_msg_id:03X}")
    add("target_msg_dlc", entry.target_msg_dlc)
    add("target_sub_idx", entry.target_sub_idx)
    add("event_type",     EVENT_TYPE_MAP.get(entry.event_type, str(entry.event_type)))
    add("action_type",    ACTION_TYPE_MAP.get(entry.action_type, str(entry.action_type)))
    add("route_flags",    f"0x{entry.route_flags:02X}  ({flags_str(entry.route_flags)})")
    add("match_value",    f"0x{entry.match_value:08X}")
    add("match_mask",     f"0x{entry.match_mask:08X}")
    add("param0",         entry.param0)
    add("param1",         entry.param1)
    add("param2",         entry.param2)
    return table


# ---------------------------------------------------------------------------
# Edit / Add interview
# ---------------------------------------------------------------------------

def prompt_int(prompt_text: str, default: int, hex_fmt: bool = False) -> int:
    """Prompt for an integer; supports 0x prefix for hex."""
    default_str = hex(default) if hex_fmt else str(default)
    while True:
        raw = Prompt.ask(prompt_text, default=default_str, console=console)
        try:
            return int(raw, 0)
        except ValueError:
            console.print("[red]Invalid value. Enter decimal or 0x hex.[/red]")


def prompt_enum(prompt_text: str, mapping: Dict[int, str], default: int) -> int:
    """Display an enum picker and return the selected value."""
    console.print(f"\n[bold cyan]{prompt_text}[/bold cyan]")
    t = Table(box=box.SIMPLE, show_header=False)
    t.add_column("Val", style="cyan", width=4, justify="right")
    t.add_column("Name", style="yellow")
    for val in sorted(mapping.keys()):
        mark = " ◀" if val == default else ""
        t.add_row(str(val), mapping[val] + mark)
    console.print(t)
    default_str = str(default)
    while True:
        raw = Prompt.ask("Enter value", default=default_str, console=console)
        try:
            v = int(raw, 0)
            if v in mapping:
                return v
            console.print("[red]Not a valid enum value.[/red]")
        except ValueError:
            console.print("[red]Invalid value.[/red]")


def prompt_flags(default: int) -> int:
    console.print("\n[bold cyan]route_flags[/bold cyan]  (enter hex or decimal)")
    console.print("  bit 0 = ROUTE_FLAG_ENABLED")
    console.print("  bit 1 = ROUTE_FLAG_WILDCARD")
    console.print("  bit 2 = ROUTE_FLAG_DYNAMIC_PARAMS")
    console.print("  bit 3 = ROUTE_FLAG_INUSE")
    return prompt_int("route_flags", default, hex_fmt=True)


def interview(entry: RouteEntry, adding: bool = False) -> RouteEntry:
    """Walk the user through all route_entry_t fields."""
    e = deepcopy(entry)
    console.print()
    console.print(Panel(
        "[bold]Route Entry Interview[/bold]  (press Enter to keep default)",
        style="bold blue"
    ))

    e.source_node_id = prompt_int("source_node_id", e.source_node_id, hex_fmt=True)
    e.source_msg_id  = prompt_int("source_msg_id",  e.source_msg_id,  hex_fmt=True)
    e.source_msg_dlc = prompt_int("source_msg_dlc", e.source_msg_dlc)
    e.source_sub_idx = prompt_int("source_sub_idx", e.source_sub_idx)
    e.target_msg_id  = prompt_int("target_msg_id",  e.target_msg_id,  hex_fmt=True)
    e.target_msg_dlc = prompt_int("target_msg_dlc", e.target_msg_dlc)
    e.target_sub_idx = prompt_int("target_sub_idx", e.target_sub_idx)
    e.event_type     = prompt_enum("event_type",     EVENT_TYPE_MAP,   e.event_type)
    e.action_type    = prompt_enum("action_type",    ACTION_TYPE_MAP,  e.action_type)
    e.route_flags    = prompt_flags(e.route_flags)
    e.match_value    = prompt_int("match_value",     e.match_value,    hex_fmt=True)
    e.match_mask     = prompt_int("match_mask",      e.match_mask,     hex_fmt=True)
    e.param0         = prompt_int("param0",          e.param0)
    e.param1         = prompt_int("param1",          e.param1)
    e.param2         = prompt_int("param2",          e.param2)

    return e


# ---------------------------------------------------------------------------
# Startup: node ID prompt
# ---------------------------------------------------------------------------

def prompt_node_id() -> int:
    history = load_history()
    console.print(Panel("[bold cyan]CAN Route Table Programmer[/bold cyan]", style="cyan"))

    if history:
        console.print("\n[bold]Recently used node IDs:[/bold]")
        t = Table(box=box.SIMPLE, show_header=False)
        t.add_column("Idx", style="cyan", width=4, justify="right")
        t.add_column("Node ID", style="white")
        for i, nid in enumerate(history):
            t.add_row(str(i), nid)
        console.print(t)
        console.print("Enter an index to reuse, or type a new node ID (e.g. 0x25A56D84):")
    else:
        console.print("\nEnter remote node ID (e.g. 0x25A56D84):")

    while True:
        raw = Prompt.ask("Node ID", console=console)
        # Check if it's a history index
        if raw.isdigit() and history:
            idx = int(raw)
            if 0 <= idx < len(history):
                raw = history[idx]
        try:
            node_id = int(raw, 0)
            if 0 <= node_id <= 0xFFFFFFFF:
                history = add_to_history(raw if raw.startswith("0x") or raw.startswith("0X")
                                         else hex(node_id), history)
                save_history(history)
                console.print(f"[green]Using node ID: 0x{node_id:08X}[/green]")
                return node_id
            console.print("[red]Value out of 32-bit range.[/red]")
        except ValueError:
            console.print("[red]Invalid format. Use decimal or 0x hex.[/red]")


# ---------------------------------------------------------------------------
# Main TUI loop
# ---------------------------------------------------------------------------

def next_free_index(routes: Dict[int, RouteEntry]) -> Optional[int]:
    for i in range(MAX_ROUTES):
        if i not in routes:
            return i
    return None


def run_tui(node_id: int, routes: Dict[int, RouteEntry], bus: can.Bus):
    term = Terminal()
    selected = 0  # index into sorted list of routes

    def sorted_keys():
        return sorted(routes.keys())

    def clamp_selected():
        nonlocal selected
        n = len(routes)
        if n == 0:
            selected = 0
        else:
            selected = max(0, min(selected, n - 1))

    def print_table():
        console.clear()
        console.print(render_route_table(routes, selected, node_id))
        n = len(routes)
        console.print(
            f"[dim]↑/↓ navigate  │  [bold]e[/bold] edit  │  [bold]a[/bold] add  │  "
            f"[bold]d[/bold] delete/zero  │  [bold]r[/bold] reload from node  │  "
            f"[bold]q[/bold] quit & save[/dim]"
        )
        console.print(f"[dim]Entries: {n}  │  Node: 0x{node_id:08X}[/dim]")

    print_table()

    with term.cbreak(), term.hidden_cursor():
        while True:
            key = term.inkey(timeout=None)

            if key.name == "KEY_UP":
                selected = max(0, selected - 1)
                print_table()

            elif key.name == "KEY_DOWN":
                keys = sorted_keys()
                selected = min(len(keys) - 1, selected + 1)
                print_table()

            elif str(key) == "q":
                break

            elif str(key) == "r":
                # Reload from node
                console.print("\n[bold cyan]Requesting route table from node...[/bold cyan]")
                result = receive_route_table(node_id, bus, console)
                if result is not None:
                    routes.clear()
                    routes.update(result)
                    clamp_selected()
                    console.print(f"[green]Received {len(result)} routes.[/green]")
                    time.sleep(0.8)
                else:
                    time.sleep(1.5)
                print_table()

            elif str(key) == "e":
                keys = sorted_keys()
                if not keys:
                    console.print("[yellow]No entries to edit.[/yellow]")
                    time.sleep(0.8)
                    print_table()
                    continue
                entry = routes[keys[selected]]
                updated = interview(entry, adding=False)
                console.print()
                console.print(render_summary_table(updated))
                if Confirm.ask("\nSend this entry to the node?", console=console):
                    ok = send_route_entry(node_id, updated.route_index, updated, bus, console)
                    if ok:
                        routes[updated.route_index] = updated
                time.sleep(0.8)
                print_table()

            elif str(key) == "a":
                free_idx = next_free_index(routes)
                if free_idx is None:
                    console.print("[red]Route table is full (80 entries).[/red]")
                    time.sleep(1.0)
                    print_table()
                    continue
                new_entry = RouteEntry(route_index=free_idx)
                # ROUTE_FLAG_INUSE cleared (already 0 by default)
                updated = interview(new_entry, adding=True)
                console.print()
                console.print(render_summary_table(updated))
                if Confirm.ask("\nSend this entry to the node?", console=console):
                    ok = send_route_entry(node_id, updated.route_index, updated, bus, console)
                    if ok:
                        routes[updated.route_index] = updated
                time.sleep(0.8)
                clamp_selected()
                print_table()

            elif str(key) == "d":
                keys = sorted_keys()
                if not keys:
                    console.print("[yellow]No entries to delete.[/yellow]")
                    time.sleep(0.8)
                    print_table()
                    continue
                idx = keys[selected]
                entry = routes[idx]
                zeroed = RouteEntry(route_index=idx)  # all zeros
                console.print(f"\n[bold red]Zero out route index {idx}?[/bold red]")
                console.print(render_summary_table(zeroed))
                if Confirm.ask("Send zeroed entry to node?", console=console):
                    ok = send_route_entry(node_id, idx, zeroed, bus, console)
                    if ok:
                        routes[idx] = zeroed
                time.sleep(0.8)
                clamp_selected()
                print_table()

    # On exit: save CSV
    save_csv(routes)
    console.print(f"\n[green]Route table saved to {CSV_FILE}[/green]")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    node_id = prompt_node_id()

    # Load CSV
    routes: Dict[int, RouteEntry] = {}
    if os.path.exists(CSV_FILE):
        console.print(f"\n[dim]Found {CSV_FILE}.[/dim]")
        if Confirm.ask(f"Load route table from {CSV_FILE}?", console=console):
            routes = load_csv()
            console.print(f"[green]Loaded {len(routes)} entries from CSV.[/green]")

    # Open CAN bus
    try:
        bus = can.Bus(interface="socketcan", channel=CAN_INTERFACE)
    except Exception as ex:
        console.print(f"[bold red]Failed to open CAN interface '{CAN_INTERFACE}': {ex}[/bold red]")
        sys.exit(1)

    # Offer live query
    if Confirm.ask(f"\nQuery node 0x{node_id:08X} for live route table?", console=console):
        console.print("[cyan]Sending request...[/cyan]")
        result = receive_route_table(node_id, bus, console)
        if result is not None:
            routes.clear()
            routes.update(result)
            console.print(f"[green]Received {len(result)} active routes.[/green]")
        else:
            console.print("[yellow]Continuing with existing data.[/yellow]")
        time.sleep(0.5)

    try:
        run_tui(node_id, routes, bus)
    finally:
        bus.shutdown()
        console.print("[dim]CAN bus closed.[/dim]")


if __name__ == "__main__":
    main()