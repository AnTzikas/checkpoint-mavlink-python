from pymavlink import mavutil
from .logging import RecvMatchLog
from datetime import datetime, timezone
from pathlib import Path


"""
This Class handles the logging logic:
"""

class MavlinkConnection:

    """Thin wrapper around a pymavlink mavfile instance."""
    def __init__(self, inner, hook=None):
        self._inner = inner
        
        #* Check/Create folder for logs
        log_dir = Path("logfiles")
        log_dir.mkdir(parents=True, exist_ok=True)

        #Create logfile for recv_log
        recv_log_path = log_dir / f"recv_match_{0}.tlog"
        self._recv_log = RecvMatchLog(recv_log_path) if recv_log_path else None
    

    def recv_msg(self):

        #Call the core method
        msg = self._inner.recv_msg()
        
        #TODO handle logging
        return msg
    
    def recv_match(
        self,
        condition=None,
        type=None,
        blocking=False,
        timeout=None,
        **kwargs,  # forward-compat with pymavlink
    ):
        
        #* Replay phase: return the next available message from logs
        if (self._recv_log.replay_buffer is not None):
            
            if (self._recv_log.replay_buffer.total == 0):
                
                #* There replay mode finished
                self._recv_log.replay_buffer = None
                # self._recv_log._fsize = _filesize_now(self._recv_log._f)
            
            else:
                
                #*Return the next available message form log 
                entry = self._recv_log.replay_buffer.next()
                return entry.msg
            
        #* Check if its need to start the replay mode (restoration process)
        if (self._recv_log.check_for_restore()):

            #* Read data from the logfile and create the msg list
            self._recv_log.read_data(self._inner.mav)
            
            #* The logged msg list has been created so just call again the same method
            return self.recv_match(
                condition=condition,
                type=type,
                blocking=blocking,
                timeout=timeout,
                **kwargs,
            )
            #Create buffer
            
            
        #* call through the default method and add the response to the logs
        msg = self._inner.recv_match(
            condition=condition,
            type=type,
            blocking=blocking,
            timeout=timeout,
            **kwargs,
        )
        self._recv_log.add(msg)
        
        return msg

    def close(self):
        # remove hook if we added one
        # try:
        #     if self._hook and getattr(self._inner, "message_hooks", None) is not None:
        #         try:
        #             self._inner.message_hooks.remove(self._hook)
        #         except ValueError:
        #             pass
    # finally:
        # close tlog first to flush any pending writes
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

def my_mavlink_connection(*args, **kwargs) -> MavlinkConnection:
    """
    Drop-in replacement for mavutil.mavlink_connection that returns your wrapper.
    """
    inner = mavutil.mavlink_connection(*args, **kwargs)
    
    return MavlinkConnection(inner)


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