
import struct, time, os
# from pymavlink import mavutil
from .replay_class import ReplayMode
from typing import Iterator, Tuple, Optional


HEADER_FMT = "<BQI"                  # type(u8), ts(usec, u64 LE), len(u32 LE)
HDR_SZ = struct.calcsize(HEADER_FMT)
REC_NONE = 0   # recv_match() returned None (timeout)
REC_MSG  = 1   # recv_match() returned a MAVLink messag
REC_FRAME = 2

class TLogWriter:
    def __init__(self, path: str):
        # unbuffered so size updates instantly
        self._f = open(path, "wb", buffering=0)

    def add(self, msg):
        # skip parser noise
        if msg and msg.get_type() != "BAD_DATA":
            t_usec = int(time.time() * 1_000_000)
            self._f.write(struct.pack(">Q", t_usec) + msg.get_msgbuf())


    def close(self):
        try:
            self._f.close()
        except Exception:
            pass

def info_for(msg):
    """Pick concise, meaningful fields per message type."""
    t = msg.get_type()
    d = msg.to_dict() if hasattr(msg, "to_dict") else {}

    if t == "GLOBAL_POSITION_INT":
        return {
            "lat": d.get("lat", 0)/1e7,
            "lon": d.get("lon", 0)/1e7,
            "alt_m": d.get("alt", 0)/1000.0,
            "rel_alt_m": d.get("relative_alt", 0)/1000.0,
            "vx": d.get("vx"), "vy": d.get("vy"), "vz": d.get("vz")
        }
    elif t == "ATTITUDE":
        return {
            "roll":  d.get("roll"),
            "pitch": d.get("pitch"),
            "yaw":   d.get("yaw"),
            "rollspeed":  d.get("rollspeed"),
            "pitchspeed": d.get("pitchspeed"),
            "yawspeed":   d.get("yawspeed"),
        }
    elif t == "VFR_HUD":
        return {
            "airspeed": d.get("airspeed"),
            "groundspeed": d.get("groundspeed"),
            "alt": d.get("alt"),
            "climb": d.get("climb"),
            "heading": d.get("heading")
        }
    elif t == "HEARTBEAT":
        return {
            "type": d.get("type"),
            "autopilot": d.get("autopilot"),
            "base_mode": d.get("base_mode"),
            "system_status": d.get("system_status"),
            "mavlink_version": d.get("mavlink_version")
        }
    else:
        # generic: show a few fields (skip big arrays)
        summary = {}
        for k, v in d.items():
            if k.startswith("_"): 
                continue
            if isinstance(v, (bytes, bytearray, memoryview, list, tuple)) and len(str(v)) > 40:
                continue
            summary[k] = v
            if len(summary) >= 6:
                break
        return summary


def _filesize_now(f) -> int:
    # f.flush()                 # push Python buffers
    # os.fsync(f.fileno())      # ensure OS writes hit disk (optional but safest)
    return os.fstat(f.fileno()).st_size

def _sync_write(f, line: str, fsize):
    if _filesize_now(f) != fsize:
        #print("File size is different Replay!")
        return False 
    f.write(line)
    f.flush()
    os.fsync(f.fileno())  # block until the OS commits to disk
    return True

class RecvMatchLogger:
    def __init__(self, path: str):
        # unbuffered for simplicity; you can add your own buffering
        self._f = open(path, "ab", buffering=0)
        self._fsize = 0
        self.replay_buffer = None
        self._fpath = path

    def check_for_restore(self):
        fsize = _filesize_now(self._f)
        if (fsize != self._fsize and fsize != 0):
            #print("Restore Detected")
            return True


        return False
    def add(self, msg):
        ts = time.time()  # wall-clock; change if you prefer monotonic
        if msg is None:
            # type(1B)=NONE, ts(8B LE float), len(4B LE)=0
            rec = struct.pack("<BdI", REC_NONE, ts, 0)
            if (not _sync_write(self._f, rec, self._fsize)):
                #print("TODO Replay mode")
                return
            
            self._fsize = _filesize_now(self._f)
            #self._f.write(rec)
            return

        # must be a pymavlink message object
        frame = msg.get_msgbuf()       # full wire bytes (includes header+payload+crc)
        # type(1B)=MSG, ts(8B LE float), len(4B LE), bytes
        rec = struct.pack("<BdI", REC_MSG, ts, len(frame)) + frame
        if (not _sync_write(self._f, rec, self._fsize)):
                #print("TODO Replay mode")
                return
        
        self._fsize = _filesize_now(self._f)
        # self._f.write(rec)

    def read_data(self):
        last_size = self._fsize
        self.replay_buffer, last_size = read_new_records(self._fpath, last_size)
        

    def close(self):
        try:
            self._f.flush()
            os.fsync(self._f.fileno())
        except Exception:
            pass
        self._f.close()

# assumes: REC_NONE, ReplayMode, rebuild_messages_from_bytes, etc.
HDR_SZ = 1 + 8 + 4  # B d I

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

def read_new_records(path: str, last_size: int) -> Tuple[ReplayMode, int]:
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
    return ReplayMode(delta), safe_end

    with open(path, "rb") as f:
        data = f.read()
    if not (0 <= last_size <= len(data)):
        raise ValueError(f"last_size {last_size} out of range 0..{len(data)}")

    mv = memoryview(data)
    safe_end = _safe_end_offset(mv, last_size)
    if safe_end <= last_size:
        return ReplayMode(b""), last_size

    delta = bytes(mv[last_size:safe_end])
    return ReplayMode(delta), safe_end