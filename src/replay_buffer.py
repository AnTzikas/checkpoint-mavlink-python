import struct
from dataclasses import dataclass
from typing import List, Union
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

import constants

@dataclass(frozen=True)
class ReplayEntry:
    """A single restored interaction event."""
    api_id: int
    timestamp_s: float
    payload: Union[object, str, None]  # Message object, args string, or None

class ReplayBuffer:
    """FIFO queue of pending interactions[cite: 118]."""

    def __init__(self, entries: List[ReplayEntry]):
        self._entries = entries
        self._index = 0

    @property
    def is_exhausted(self) -> bool:
        return self._index >= len(self._entries)

    def next_entry(self) -> ReplayEntry:
        if self.is_exhausted:
            raise StopIteration("Replay buffer is empty")
        entry = self._entries[self._index]
        self._index += 1
        return entry

# ---------------------------------------------------------------------------
# FACTORY METHODS
# ---------------------------------------------------------------------------

def parse_receive_log(log_data: bytes) -> ReplayBuffer:
    """Parses binary RECEIVE log. Reconstructs MAVLink objects."""
    entries: List[ReplayEntry] = []
    mv = memoryview(log_data)
    offset = 0
    limit = len(mv)
    mav_parser = mavlink2.MAVLink(None)
    mav_parser.robust_parsing = True

    while offset < limit:
        if offset + constants.RECV_HEADER_SZ > limit: break

        rec_type, api_id, p_len, ts_us = struct.unpack_from(
            constants.RECV_HEADER_FMT, mv, offset
        )
        
        record_end = offset + constants.RECV_HEADER_SZ + p_len
        if record_end > limit: break

        payload_bytes = bytes(mv[offset + constants.RECV_HEADER_SZ : record_end])
        
        msg_obj = None
        if rec_type == constants.REC_MSG:
            # Re-inject bytes into parser
            for b in payload_bytes:
                # FIX START: Capture result in temporary variable
                res = mav_parser.parse_char(bytes([b]))
                if res: 
                    msg_obj = res  # Only update if we got a valid object
                # FIX END
        
        entries.append(ReplayEntry(api_id, ts_us / 1e6, msg_obj))
        offset = record_end

    return ReplayBuffer(entries)

def parse_send_log(log_data: bytes) -> ReplayBuffer:
    """Parses binary SEND log. reconstructs UTF-8 summaries[cite: 141]."""
    entries: List[ReplayEntry] = []
    mv = memoryview(log_data)
    offset = 0
    limit = len(mv)

    while offset < limit:
        if offset + constants.SEND_HEADER_SZ > limit: break

        api_id, p_len, ts_us = struct.unpack_from(
            constants.SEND_HEADER_FMT, mv, offset
        )

        record_end = offset + constants.SEND_HEADER_SZ + p_len
        if record_end > limit: break

        summary_bytes = bytes(mv[offset + constants.SEND_HEADER_SZ : record_end])
        summary_str = summary_bytes.decode("utf-8", errors="replace")

        entries.append(ReplayEntry(api_id, ts_us / 1e6, summary_str))
        offset = record_end

    return ReplayBuffer(entries)