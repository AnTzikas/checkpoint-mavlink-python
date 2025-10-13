from pymavlink import mavutil
from my_util import TLogWriter
from typing import Optional
from datetime import datetime, timezone
from pathlib import Path


"""
This Class handles the logging logic:
 --> It creates hook so every time a message is parsed by the parser, it gets logged
"""

class MyMavlinkConn:

    """Thin wrapper around a pymavlink mavfile instance."""
    # def __init__(self, inner):
    #     self._inner = inner# keep a ref so we can remove it on close
    def __init__(self, inner, tlog: Optional[TLogWriter] = None, hook=None):
        self._inner = inner
        self._tlog = tlog
        self._hook = hook  # keep a ref so we can remove it on close

    def recv_msg(self):
        #Call the core method
        msg = self._inner.recv_msg()
        
        # print("Custom message is called")
        return msg
    
    def recv_match(
        self,
        condition=None,
        type=None,
        blocking=False,
        timeout=None,
        **kwargs,  # forward-compat with pymavlink
    ):
        # call through
        msg = self._inner.recv_match(
            condition=condition,
            type=type,
            blocking=blocking,
            timeout=timeout,
            **kwargs,
        )
        return msg

    def close(self):
        # remove hook if we added one
        try:
            if self._hook and getattr(self._inner, "message_hooks", None) is not None:
                try:
                    self._inner.message_hooks.remove(self._hook)
                except ValueError:
                    pass
        finally:
            # close tlog first to flush any pending writes
            if self._tlog:
                self._tlog.close()
            try:
                self._inner.close()
            except Exception:
                pass
    def __getattr__(self, name):
        # forward attribute/method access (e.g. write, wait_heartbeat, target_system, etc.)
        return getattr(self._inner, name)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self._inner.close()
        except Exception:
            pass

"""
    --> Establish a default mavlink connection
    --> Init the logfile
    --> add a hook to recv_msg() in order to log each and every valid message parsed
"""
def my_mavlink_connection(*args, **kwargs) -> MyMavlinkConn:
    """
    Drop-in replacement for mavutil.mavlink_connection that returns your wrapper.
    Adds a message hook that writes a raw .tlog (timestamp + payload) if tlog_path is not None.
    """
    inner = mavutil.mavlink_connection(*args, **kwargs)

    # ensure message_hooks exists (some builds already have it)
    if not hasattr(inner, "message_hooks") or inner.message_hooks is None:
        inner.message_hooks = []
    
    log_dir = Path("logfiles")
    log_dir.mkdir(parents=True, exist_ok=True)

    # ISO-ish, filesystem-safe timestamp (UTC, no colons)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")

    tlog_path = log_dir / f"parsed_log_{ts}.tlog"
    tlog = TLogWriter(tlog_path) if tlog_path else None

    # Hook must tolerate different signatures:
    # - Some builds call hook(msg)
    # - Others call hook(m, msg) or hook(**{"msg": msg})
    def _hook(*h_args, **h_kw):
        msg = h_kw.get("msg")
        if msg is None:
            # last positional is typically the parsed MAVLinkMessage
            msg = h_args[-1] if h_args else None
        if tlog and msg:
            tlog.add(msg)

    inner.message_hooks.append(_hook)

    return MyMavlinkConn(inner, tlog=tlog, hook=_hook)