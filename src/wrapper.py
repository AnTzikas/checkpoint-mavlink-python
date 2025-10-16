from pymavlink import mavutil
from .util import TLogWriter, RecvMatchLogger, _filesize_now
from typing import Optional
from datetime import datetime, timezone
from pathlib import Path


"""
This Class handles the logging logic:
 --> It creates hook so every time a message is parsed by the parser, it gets logged
"""

class MyMavlinkConn:

    """Thin wrapper around a pymavlink mavfile instance."""
    def __init__(self, inner, tlog: Optional[TLogWriter] = None, hook=None):
        self._inner = inner
        self._tlog = tlog
        self._hook = hook  # keep a ref so we can remove it on close

        log_dir = Path("logfiles")
        log_dir.mkdir(parents=True, exist_ok=True)

        # ISO-ish, filesystem-safe timestamp (UTC, no colons)
        # ts = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
        ts = 0
        recv_log_path = log_dir / f"recv_match_{ts}.tlog"
        self._recv_log = RecvMatchLogger(recv_log_path) if recv_log_path else None
    

    def recv_msg(self):

        
        #Call the core method
        msg = self._inner.recv_msg()
        
        self._recv_log.add(msg)
        return msg
    
    def recv_match(
        self,
        condition=None,
        type=None,
        blocking=False,
        timeout=None,
        **kwargs,  # forward-compat with pymavlink
    ):
        #print(f"Fsize equals {self._recv_log._fsize}")
        #*Check if the replay log is not empty
        if (self._recv_log.replay_buffer is not None):
            
            if (self._recv_log.replay_buffer.total == 0):
                # print("Replay buffer is empty")
                self._recv_log.replay_buffer = None
                self._recv_log._fsize = _filesize_now(self._recv_log._f)
            else:
                # #print("Got the item return msg")
                entry = self._recv_log.replay_buffer.next()
                return entry.msg
            
        #* Check if its replay phase
        if (self._recv_log.check_for_restore()):
            # print("Replay detected")

            self._recv_log.read_data()
            totalsize = self._recv_log.replay_buffer.total
            # print(f"Created replay history with size: {totalsize}")
            
            return self.recv_match(
                condition=condition,
                type=type,
                blocking=blocking,
                timeout=timeout,
                **kwargs,
            )
            #Create buffer
            
            
        #* call through
        # print("New call")
        msg = self._inner.recv_match(
            condition=condition,
            type=type,
            blocking=blocking,
            timeout=timeout,
            **kwargs,
        )
        
        #todo log the response before forwarding
        self._recv_log.add(msg)
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
            if self._recv_log:
                self._recv_log.close()
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
    # if not hasattr(inner, "message_hooks") or inner.message_hooks is None:
    #     inner.message_hooks = []
    
    log_dir = Path("logfiles")
    log_dir.mkdir(parents=True, exist_ok=True)

    # ISO-ish, filesystem-safe timestamp (UTC, no colons)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")

    tlog_path = log_dir / f"parsed_log_{ts}.tlog"
    tlog = TLogWriter(tlog_path) if tlog_path else None

    # Hook must tolerate different signatures:
    # - Some builds call hook(msg)
    # - Others call hook(m, msg) or hook(**{"msg": msg})
    # def _hook(*h_args, **h_kw):
    #     msg = h_kw.get("msg")
    #     if msg is None:
    #         # last positional is typically the parsed MAVLinkMessage
    #         msg = h_args[-1] if h_args else None
    #     if tlog and msg:
    #         tlog.add(msg)

    # inner.message_hooks.append(_hook)

    return MyMavlinkConn(inner, tlog=tlog)