from typing import Final


REC_NONE: Final = 0   # recv_match() returned None (timeout)
REC_MSG: Final  = 1   # recv_match() returned a MAVLink messag

HEADER_FMT: Final = "<BQI"                  # type(u8), ts(usec, u64 LE), len(u32 LE)
HDR_SZ: Final = 1 + 8 + 4  # B d I assumes: REC_NONE, ReplayMode, rebuild_messages_from_bytes, etc.
