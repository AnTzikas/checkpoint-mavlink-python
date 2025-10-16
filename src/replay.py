from dataclasses import dataclass
from typing import Optional, List, Iterator, Tuple
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 
from typing import Iterator, Tuple, Optional
import struct
from .constants import REC_NONE, REC_MSG, HEADER_FMT, HDR_SZ



def iter_recvmatch_records_from_bytes(data: bytes) -> Iterator[Tuple[Optional[bytes], float]]:
    """
    Parse the in-memory buffer `data` and yield (frame_bytes_or_None, ts_wall).
    Record layout per entry: type(u8), ts(LE float64), len(LE u32), payload(len bytes if type!=REC_NONE)
    """
    mv = memoryview(data)
    off = 0
    HDR_SZ = 1 + 8 + 4  # B d I

    while off < len(mv):
        # Header
        if off + HDR_SZ > len(mv):
            raise EOFError("truncated record header")
        rtype, ts, n = struct.unpack_from("<BdI", mv, off)
        off += HDR_SZ

        if rtype == REC_NONE:
            if n != 0:
                raise ValueError("REC_NONE record must have length 0")
            yield (None, ts)
            continue

        # Payload
        if off + n > len(mv):
            raise EOFError("truncated frame payload")
        frame = bytes(mv[off:off+n])   # copy; keep as bytes for downstream
        off += n
        yield (frame, ts)

def rebuild_messages_from_bytes(data: bytes, live_mav) -> Iterator[Tuple[Optional[object], float]]:
    """
    Replay all frames from in-memory buffer `data` through a fresh MAVLink parser.
    Yields (msg, ts) where msg is a pymavlink MAVLink_*_message or None.
    """
    
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True

    # if hasattr(parser, "mavlink20"):
    #     parser.mavlink20(True)   # enable MAVLink2 so extensions are kept

    # parser = _clone_live_parser(live_mav)
    for frame_or_none, ts in iter_recvmatch_records_from_bytes(data):
        if frame_or_none is None:
            yield (None, ts)
            continue

        msg = None
        # Feed the *entire* frame; don't break on first out (handles MAVLink2 signatures).
        for b in frame_or_none:
            out = parser.parse_char(bytes([b]))
            if out is not None:
                msg = out

        if msg is None:
            raise ValueError("Failed to parse recorded frame into a message")
        yield (msg, ts)

@dataclass(frozen=True)
class ReplayEntry:
    msg: Optional[object]   # pymavlink MAVLink_*_message or None
    ts: float               # wall-clock timestamp recorded

class ReplayMode:
    """Load all records into memory once; iterate like a FIFO with indexing."""
    def __init__(self, data: bytes, live_mav):
        # materialize everything up front
       
        self._entries: List[ReplayEntry] = [
            ReplayEntry(msg, ts) for (msg, ts) in rebuild_messages_from_bytes(data, live_mav)
        ]
        self.index: int = 0  # next unread slot

    # --- Introspection ---
    @property
    def total(self) -> int:
        return len(self._entries)

    @property
    def remaining(self) -> int:
        return self.total - self.index

    def exhausted(self) -> bool:
        return self.index >= self.total

    # --- Random access (does not advance) ---
    def __getitem__(self, i: int) -> ReplayEntry:
        return self._entries[i]

    # --- Sequential read API (advances) ---
    def peek(self, k: int = 1) -> List[ReplayEntry]:
        """Look ahead at up to k upcoming entries without consuming."""
        if k <= 0:
            return []
        return self._entries[self.index:min(self.index + k, self.total)]

    def next(self) -> ReplayEntry:
        """Consume and return the next entry."""
        if self.exhausted():
            raise StopIteration("ReplayMode is exhausted")
        e = self._entries[self.index]
        self.index += 1
        return e

    def __iter__(self) -> Iterator[ReplayEntry]:
        """Iterate remaining entries (consumes them)."""
        while not self.exhausted():
            yield self.next()

    # --- Navigation ---
    def reset(self) -> None:
        """Rewind to the beginning."""
        self.index = 0

    def seek(self, idx: int) -> None:
        """Set next unread index (0..total)."""
        if not (0 <= idx <= self.total):
            raise IndexError(f"seek index {idx} out of range 0..{self.total}")
        self.index = idx

    def skip(self, n: int) -> None:
        """Skip forward n entries (clamped)."""
        if n <= 0:
            return
        self.index = min(self.total, self.index + n)
