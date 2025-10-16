
#!/usr/bin/env python3
import argparse
import struct
from typing import Iterator, Tuple, Optional
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 
# from src.util import info_for
# Record type tags (match your writer)
REC_NONE = 0   # recv_match() returned None (timeout)
REC_MSG  = 1   # recv_match() returned a MAVLink message

def iter_recvmatch_records(path: str) -> Iterator[Tuple[Optional[bytes], float]]:
    """
    Yields (frame_bytes_or_None, ts_wall) for each record in the recv_match log.
    - None indicates recv_match returned None (e.g., timeout).
    - bytes is the raw MAVLink frame returned by recv_match (msg.get_msgbuf()).
    """
    with open(path, "rb") as f:
        while True:
            hdr = f.read(1 + 8 + 4)  # type (u8) + ts (LE float64) + len (LE u32)
            if not hdr:
                return
            if len(hdr) < 13:
                raise EOFError("truncated record header")
            rtype, ts, n = struct.unpack("<BdI", hdr)
            if rtype == REC_NONE:
                yield (None, ts)
                continue
            frame = f.read(n)
            if len(frame) != n:
                raise EOFError("truncated frame payload")
            yield (frame, ts)

def rebuild_messages(path: str) -> Iterator[Tuple[Optional[object], float]]:
    """
    For each record, yields (msg, ts) where:
      - msg is a pymavlink MAVLink_*_message object reconstructed from the raw frame
      - or None if the original recv_match returned None
    """
    # Fresh MAVLink parser (no I/O endpoint)
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    
    for frame_or_none, ts in iter_recvmatch_records(path):
        if frame_or_none is None:
            yield (None, ts)
            continue

        msg = None
        # Feed the exact recorded bytes back into the parser
        # parse_char expects bytes one at a time; it returns a message when complete.
        for b in frame_or_none:
            out = parser.parse_char(bytes([b]))
            if out is not None:
                msg = out
                break

        if msg is None:
            raise ValueError("Failed to parse recorded frame into a message")
        yield (msg, ts)

def _msg_to_dict(msg) -> dict:
    """Best-effort conversion of a MAVLink message to a printable dict of fields."""
    # pymavlink messages typically have .to_dict() and .get_fieldnames()
    if hasattr(msg, "to_dict"):
        d = dict(msg.to_dict())
    else:
        d = {}
        if hasattr(msg, "get_fieldnames"):
            for name in msg.get_fieldnames():
                d[name] = getattr(msg, name, None)
    # Add header-ish fields that are often useful
    try:
        d["_type"]   = msg.get_type()
    except Exception:
        pass
    for attr, key in (("seq", "_seq"), ("get_srcSystem", "sysid"), ("get_srcComponent", "compid")):
        try:
            if attr == "seq":
                d["_seq"] = getattr(msg, "seq")
            else:
                d[key] = getattr(msg, attr)()
        except Exception:
            pass
    return d

def main():
    ap = argparse.ArgumentParser(description="Rebuild and print messages from a recv_match log.")
    ap.add_argument("logfile", help="Path to recv_match log")
    ap.add_argument("--limit", type=int, default=0, help="Max records to print (0 = all)")
    ap.add_argument("--show-none", action="store_true", help="Print None/timeout records")
    ap.add_argument("--raw-hex", action="store_true", help="Also print raw frame bytes as hex")
    args = ap.parse_args()

    count = 0
    none_count = 0
    msg_count = 0

    for idx, (msg, ts) in enumerate(rebuild_messages(args.logfile), start=1):
        if msg is None:
            none_count += 1
            if args.show_none:
                print(f"[{idx}] ts={ts:.6f}  recv_match -> None")
            continue
        

        if msg is None:
                print("None continue\n")
                continue
        if msg.get_type() == 'BAD_DATA':
            print("Bad data continue\n")
            continue
        mtype = msg.get_type()
            # Pretty compact printout without relying on external helpers
        try:
            as_dict = msg.to_dict()
        except Exception:
            as_dict = {k: getattr(msg, k) for k in dir(msg) if not k.startswith('_')}
        print(f"{mtype}: {as_dict}")
        msg_count += 1
       
        count += 1
        if args.limit and count >= args.limit:
            break

    # print(f"\nDone. messages={msg_count}, none={none_count}, total={msg_count+none_count}")
    print(f"Total messages are: {msg_count}")

if __name__ == "__main__":
    main()
