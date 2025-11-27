import socket
import threading
import time
import os
import subprocess
import pathlib
import signal
import errno
from src.ipc_client import IPCClient


CHECKPOINT_DIR = os.environ.get("CHECKPOINT_DIR", "/app/logfiles/checkpoint")
RECV_LOG_PATH = os.environ.get("RECV_LOG_PATH", "/app/logfiles/log/recv.bin")
SEND_LOG_PATH = os.environ.get("SEND_LOG_PATH", "/app/logfiles/log/send.bin")
COMMAND = os.environ.get("COMMAND", "python3 /app/surveillance_mission.py")
APP_NAME = os.environ.get("APP_NAME", "surveillance_mission.py")
CHECKPOINT_INTERVAL = int(os.environ.get("CHECKPOINT_INTERVAL", 30))
LOCK_SERVER_HOST = os.environ.get("IPC_IP", "0.0.0.0") 
LOCK_SERVER_PORT = int(os.environ.get("IPC_PORT", 12345))

def get_pid_by_name(process_name):
    try:
        output = subprocess.check_output(["pgrep", "-f", "-o", process_name]).decode('utf-8').strip()
        return int(output) if output else None
    except: return None
def rotate_log(file_path):
    """
    Supervisor function to rotate logs.
    We DELETE and RECREATE to force an Inode change.
    Do NOT use truncate() here, as it preserves Inode.
    """
    try:
        if os.path.exists(file_path):
            os.remove(file_path) # Deletes the directory entry
        
        # Create fresh file
        with open(file_path, 'wb') as f:
            pass # Create empty file
            
    except OSError as e:
        print(f"Rotation failed: {e}")

def run_checkpoint():
        
    pid = get_pid_by_name(APP_NAME)

    try:
        # os.kill(pid, signal.SIGSTOP)
        subprocess.run(["criu", "dump", "-t", str(pid), "-D", CHECKPOINT_DIR, 
                        "--shell-job", "--leave-running", 
                        "--tcp-close", "--ext-unix-sk", "--skip-in-flight"], check=True)
        #, "-v0", "-o", "dump_criu.log"
        print("[Supervisor] [Checkpointer]: Snapshot SUCCESS.")
        # os.kill(pid, signal.SIGCONT)

    except:
        return


# app_pid = get_pid_by_name(APP_NAME)
lock = IPCClient()
print("[Checkpoint] Wait to acquire lock")

lock.acquire()
print("[Checkpoint] Lock acquired, make checkpoint")
time.sleep(2)
run_checkpoint()
rotate_log(RECV_LOG_PATH)
rotate_log(SEND_LOG_PATH)
print("Checkpoint Finished, Release Lock!")

time.sleep(2)
lock.release()

