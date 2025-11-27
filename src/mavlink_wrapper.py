import time
import struct
from typing import Optional, Any, List
from pymavlink import mavutil
import os 

# Import infrastructure components
from src import constants
from src.interaction_journal import InteractionJournal
from src import replay_buffer
from src.replay_buffer import ReplayBuffer
from src.ipc_client import IPCClient
# Proxy class for intercepting outgoing commands (e.g., master.mav.command_long_send)
class _MavSenderProxy:
    """
    Internal Proxy for the 'mav' attribute of the connection.
    Intercepts outgoing commands to enable Side-Effect Suppression.
    """
    def __init__(self, real_mav, wrapper_ref):
        self._real_mav = real_mav
        self._wrapper = wrapper_ref

    def _execute_send_flow(self, api_id: int, summary: str, method_name: str, *args, **kwargs):
        
        # print("[Out] Try to acquire!")
        self._wrapper._checkpoint_lock.acquire()

        if self._wrapper.is_replay_mode:
            msg = self._wrapper._handle_replay_send(summary)
            
            if msg is not None:
                print("Replay Mode: return!")
                
                return 
            print("Exit replay mode go to Live.")

        #* Check if a restore happened
        if self._wrapper._poll_restore_status():
            print("Restore detected!")
            return self._execute_send_flow(api_id, summary, method_name, *args, **kwargs)

        real_method = getattr(self._real_mav, method_name)
        real_method(*args, **kwargs)
        
        # Log the interaction
        self._wrapper._log_send_interaction(api_id, summary)

        self._wrapper._checkpoint_lock.release()

    def command_long_send(self, target_system, target_component, command, confirmation, p1, p2, p3, p4, p5, p6, p7):
        args_summary = f"CMD_LONG(sys={target_system}, cmd={command}, p1={p1})"
        self._execute_send_flow(
            constants.COMMAND_LONG_SEND_ID, args_summary, "command_long_send",
            target_system, target_component, command, confirmation, p1, p2, p3, p4, p5, p6, p7
        )
    def command_int_send(self, target_system, target_component, frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z):
        args_summary = f"CMD_INT(sys={target_system}, cmd={command}, frame={frame})"
        self._execute_send_flow(
            constants.COMMAND_INT_SEND_ID, args_summary, "command_int_send",
            target_system, target_component, frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z
        )

    def set_mode_send(self, target_system, custom_mode, base_mode):
        args_summary = f"SET_MODE(sys={target_system}, base={base_mode}, custom={custom_mode})"
        self._execute_send_flow(
            constants.SET_MODE_SEND_ID, args_summary, "set_mode_send",
            target_system, custom_mode, base_mode
        )

    def param_set_send(self, target_system, target_component, param_id, param_value, param_type):
        p_id_str = param_id.decode('utf-8') if isinstance(param_id, bytes) else str(param_id)
        args_summary = f"PARAM_SET(sys={target_system}, id={p_id_str}, val={param_value})"
        self._execute_send_flow(
            constants.PARAM_SET_SEND_ID, args_summary, "param_set_send",
            target_system, target_component, param_id, param_value, param_type
        )
    def __getattr__(self, name):
        """Pass through all other attributes to the real .mav object."""
        return getattr(self._real_mav, name)


class MavlinkWrapper:
    """Transparent proxy for a MAVLink connection providing fault tolerance."""

    def __init__(self, connection: Any, recv_path: str, send_path: str):
        """
        Args:
            connection: The underlying pymavlink connection object.
            recv_path: Path to the receive log.
            send_path: Path to the send log.
        """
        self._conn = connection
        
        # 1. Initialize Persistent Storage (Dumb Layer)
        # These maintain the _known_size cursor to detect restore.
        self._recv_journal = InteractionJournal(recv_path)
        self._send_journal = InteractionJournal(send_path)

        # 2. Initialize Replay State (Smart Layer)
        self._recv_buffer: Optional[ReplayBuffer] = None
        self._send_buffer: Optional[ReplayBuffer] = None
        self._is_replay_mode: bool = False

        # 3. Setup the .mav Proxy
        self._mav_proxy = _MavSenderProxy(connection.mav, self)
        self._checkpoint_lock = IPCClient()

        # # 4. Initial State Check
        # #! Checks if logs exist on disk that are larger than our memory state (0).
        self._poll_restore_status()

    def _poll_restore_status(self) -> bool:
        """
        Checks if the physical logs are larger than our internal memory cursor.
        If yes, it means a crash/restore occurred, and we must ingest the 'delta'.
        """
        recv_needs_restore = self._recv_journal.check_restore_needed()
        send_needs_restore = self._send_journal.check_restore_needed()

        if not (recv_needs_restore or send_needs_restore):
            return False

        # # print(f"[Wrapper] RESTORE DETECTED (Disk > Memory). Ingesting history...")

        # 1. Read only the NEW bytes (Delta)
        new_recv_bytes = self._recv_journal.read_restore_chunk()
        new_send_bytes = self._send_journal.read_restore_chunk()

        # 2. Parse Delta into temporary buffers
        new_recv_buf = replay_buffer.parse_receive_log(new_recv_bytes)
        new_send_buf = replay_buffer.parse_send_log(new_send_bytes)

        # 3. Merge or Set Buffers
        # If we already have a buffer (rare nested case), we extend it. 
        # Otherwise, we just use the new one.
        self._extend_replay_buffer('recv', new_recv_buf)
        self._extend_replay_buffer('send', new_send_buf)
        
        self._is_replay_mode = True
        # # print(f"[Wrapper] Replay Mode ENABLED. Pending Recv: {self._recv_buffer.total if self._recv_buffer else 0}")

        return True

    def _extend_replay_buffer(self, kind: str, new_buf: ReplayBuffer):
        """Helper to merge new entries into the existing replay queue."""
        # Determine which buffer we are updating
        current_buf = self._recv_buffer if kind == 'recv' else self._send_buffer
        
        if current_buf is None or current_buf.is_exhausted:
            # Simple case: replace the buffer
            if kind == 'recv': self._recv_buffer = new_buf
            else: self._send_buffer = new_buf
        else:
            # Merge case: We have remaining items, append new ones
            # (Requires accessing internal list, assuming ReplayBuffer exposes it or we recreate)
            combined_entries = current_buf._entries[current_buf._index:] + new_buf._entries
            merged_buf = ReplayBuffer(combined_entries)
            
            if kind == 'recv': self._recv_buffer = merged_buf
            else: self._send_buffer = merged_buf

    @property
    def is_replay_mode(self) -> bool:
        return self._is_replay_mode

    @property
    def mav(self):
        """Returns the Proxy object instead of real .mav"""
        return self._mav_proxy

    def _execute_receive_flow(self, api_id: int, method_name: str, *args, **kwargs):
        
        #* Enter checkpoint-safe region
        # print("[Out] Try to acquire!")
        self._checkpoint_lock.acquire()

        #* Check if reply mode exists
        if self.is_replay_mode:
            
            msg = self._handle_replay_receive()

            if msg is not None:
                # print("Replay Mode: return log payload!")
                # print("Replay Receive")
                return msg

        #* Check if a restore happened
        if self._poll_restore_status():
            # print("Restore detected")
            return self._execute_receive_flow(api_id, method_name, *args, **kwargs)


        # * ---Live Mode---
        # * Dynamically fetsh method and execute
        real_method = getattr(self._conn, method_name)
        msg = real_method(*args, **kwargs)
        
        # * Log the interaction
        self._log_receive_interaction(msg, api_id)

        self._checkpoint_lock.release()
        
        return msg

    def wait_hertbeat(self):
        msg = self._execute_receive_flow(
            constants.WAIT_HEARTBEAT_ID,
            "wait_heartbeat"
        )

        # * If this is a replay mode answer
        if self._is_replay_mode:
            self._conn.wait_heartbeat()

        return msg
    
    def recv_match(
        self,
        condition=None,
        type=None,
        blocking=False,
        timeout=None,
        **kwargs, 
    ):
        return self._execute_receive_flow(
            constants.RECV_MATCH_ID,
            "recv_match",
            condition=condition,
            type=type,
            blocking=blocking,
            timeout=timeout,
            **kwargs
        )

    def recv_msg(self):
        """Intercepts recv_msg using the generic flow."""
        return self._execute_receive_flow(
            constants.RECV_MSG_ID,
            "recv_msg"
        )
    
    def _handle_replay_receive(self):
        """Fetches next message from buffer or transitions to Live Mode."""
        if self._recv_buffer is None or self._recv_buffer.is_exhausted:
            # print("[Wrapper] Replay buffer exhausted. Transitioning to LIVE MODE.")
            self._is_replay_mode = False
            return None # Return None to allow loop to retry in live mode
        
        entry = self._recv_buffer.next_entry()
        return entry.payload

    def _handle_replay_send(self, summary):
        """Consumes a send entry from buffer."""
        if self._send_buffer is None or self._send_buffer.is_exhausted:
            return None
        entry = self._send_buffer.next_entry()
        return entry
        # Verification logic could go here (entry.payload == summary?)

    def _log_receive_interaction(self, msg, api_id):
        """Formats and writes a receive entry to disk."""
        ts_us = int(time.time() * 1e6)
        
        if msg is None:
            # Header: Type=REC_NONE
            header = struct.pack(constants.RECV_HEADER_FMT, constants.REC_NONE, api_id, 0, ts_us)
            self._recv_journal.append_bytes(header)
        else:
            # Header: Type=REC_MSG
            buf = msg.get_msgbuf()
            header = struct.pack(constants.RECV_HEADER_FMT, constants.REC_MSG, api_id, len(buf), ts_us)
            self._recv_journal.append_bytes(header + buf)

    def _log_send_interaction(self, api_id, summary: str):
        """Formats and writes a send entry to disk."""
        ts_us = int(time.time() * 1e6)
        summary_bytes = summary.encode('utf-8')
        
        header = struct.pack(constants.SEND_HEADER_FMT, api_id, len(summary_bytes), ts_us)
        self._send_journal.append_bytes(header + summary_bytes)


    def close(self):
        self._recv_journal.close()
        self._send_journal.close()
        
        try:
            self._conn.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __getattr__(self, name):
        """Forward all non-intercepted calls to the real connection."""
        return getattr(self._conn, name)

# Factory funciton
def mavlink_connection_wrapper(*args, **kwargs) -> MavlinkWrapper:
    
    inner = mavutil.mavlink_connection(*args, **kwargs)
    
    LOG_DIR = os.environ.get("LOG_DIR", "logfiles/log")
    
    os.makedirs(LOG_DIR, exist_ok=True)
    
    recv_log = f"{LOG_DIR}/recv.bin" 
    send_log = f"{LOG_DIR}/send.bin"

    return MavlinkWrapper(inner, recv_log, send_log)