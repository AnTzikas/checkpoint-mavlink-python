
import struct, time, os
# from pymavlink import mavutil
from .replay import ReplayMode
from typing import Iterator, Tuple, Optional
from .constants import REC_NONE, REC_MSG, HEADER_FMT, HDR_SZ


# Return the filesize
def _filesize_now(f) -> int:
    return os.fstat(f.fileno()).st_size

# Flush and write the file
def _sync_write(f, line: str, fsize):
    
    f.write(line)
    f.flush()
    os.fsync(f.fileno())  # block until the OS commits to disk
    return 


def _safe_end_offset(buf: memoryview, start: int) -> int:
    """
    Walk records from 'start' and return the maximum offset that ends on a full record.
    Stops before any truncated header/payload.
    """
    off = start
    n_total = len(buf)
    while True:
        if off + HDR_SZ > n_total:
            # Not enough for a header -> stop before it.
            break
        rtype, _, length = struct.unpack_from("<BdI", buf, off)
        off2 = off + HDR_SZ
        if rtype == REC_NONE:
            # length must be 0 for REC_NONE; if not, treat as incomplete/corrupt tail.
            if length != 0:
                break
            # full record is header only
            off = off2
            continue
        # Need full payload
        if off2 + length > n_total:
            # partial payload; don't consume it
            break
        # full record present
        off = off2 + length
    return off

def read_new_records(path: str, last_size: int, live_mav) -> Tuple[ReplayMode, int]:
    """
    Read only the new bytes appended since last_size,
    but consume *only* fully complete records.

    Returns (replay, new_safe_size)

    - replay: ReplayMode over the newly available complete records (can be empty)
    - new_safe_size: byte offset you should persist for the next call
    """
    with open(path, "rb") as f:
        data = f.read()
    if last_size < 0 or last_size > len(data):
        raise ValueError(f"last_size {last_size} out of range 0..{len(data)}")

    mv = memoryview(data)
    safe_end = _safe_end_offset(mv, last_size)

    # Nothing new (or only a partial tail)
    if safe_end <= last_size:
        return ReplayMode(b""), last_size

    # Build a ReplayMode for the new, fully complete slice
    delta = bytes(mv[last_size:safe_end])
    return ReplayMode(delta, live_mav), safe_end

class RecvMatchLog:

    def __init__(self, path: str):
        self._f = open(path, "ab", buffering=0)
        self._fsize = 0
        self.replay_buffer = None
        self._fpath = path

    def check_for_restore(self):
        fsize = _filesize_now(self._f)
        # if (fsize != self._fsize and fsize != 0):
        #     #print("Restore Detected")
        #     return True
        # return False
        return (fsize != self._fsize and fsize != 0)
    
    def add(self, msg):
        ts = time.time()  

        if msg is None:
            # type(1B)=NONE, ts(8B LE float), len(4B LE)=0
            rec = struct.pack("<BdI", REC_NONE, ts, 0)
            _sync_write(self._f, rec, self._fsize)
            
            self._fsize = _filesize_now(self._f)
            return

        frame = msg.get_msgbuf() # full wire bytes (includes header+payload+crc)
        
        # type(1B)=MSG, ts(8B LE float), len(4B LE), bytes
        rec = struct.pack("<BdI", REC_MSG, ts, len(frame)) + frame
        
        _sync_write(self._f, rec, self._fsize)
        self._fsize = _filesize_now(self._f)

    def read_data(self, live_mav):
        last_size = self._fsize
        self.replay_buffer, last_size = read_new_records(self._fpath, last_size, live_mav)
        self._fsize = _filesize_now(self._f)
        

    def close(self):
        try:
            self._f.flush()
            os.fsync(self._f.fileno())
        except Exception:
            pass
        self._f.close()
