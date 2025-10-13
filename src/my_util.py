
import struct, time

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
