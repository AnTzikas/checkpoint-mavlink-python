import socket
import threading
import time
import os
import subprocess
import pathlib
import signal
import errno

# --- Configuration ---
CHECKPOINT_DIR = os.environ.get("CHECKPOINT_DIR", "/app/logfiles/checkpoint")
RECV_LOG_PATH = os.environ.get("RECV_LOG_PATH", "/app/logfiles/log/recv.bin")
SEND_LOG_PATH = os.environ.get("SEND_LOG_PATH", "/app/logfiles/log/send.bin")
COMMAND = os.environ.get("COMMAND", "python3 /app/surveillance_mission.py")
APP_NAME = os.environ.get("APP_NAME", "surveillance_mission.py")
CHECKPOINT_INTERVAL = int(os.environ.get("CHECKPOINT_INTERVAL", 30))
LOCK_SERVER_HOST = os.environ.get("IPC_IP", "0.0.0.0") 
LOCK_SERVER_PORT = int(os.environ.get("IPC_PORT", 12345))
# ---------------------

master_lock = threading.Lock()
server_sock = None
app2_pid = None

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

def handle_lock_client(conn, addr):

    print(f"[Supervisor] [IPC]: Client {addr} connected.")
    try:
        while True:
            # 1. Wait for command (Idle state)
            cmd = conn.recv(1024).decode()
            if not cmd: break # Client disconnected
            # print(f"Got  message {cmd}!")

            if cmd == "Acquire": # ACQUIRE REQUEST
                # Block here until Checkpoint is done
                # print(f"Inside acquire!")
                with master_lock:
                    # Signal Granted
                    msg = "Granted"
                    # print(f"Got lock, send info: {msg}!")
                    conn.sendall(msg.encode())
                    
                    # 2. Hold Lock State
                    # We stay inside this 'with' block until client says Release
                    # This prevents the Checkpointer from running
                    while True:
                        # print(f"Enter wait to release mode")
                        release_cmd = conn.recv(1024).decode()
                        if not release_cmd: 
                            return # Client died holding lock -> Lock auto-released by context manager
                        # print(f"Got release cmd: {release_cmd}!")
                        if release_cmd == "Release": # RELEASE REQUEST
                            conn.sendall("Done".encode()) # Done
                            break # Break inner loop to exit 'with master_lock'
            
            elif cmd == 'R':
                # Should not happen in Idle state, but handle gracefully
                conn.sendall("Done".encode())

    except (ConnectionResetError, BrokenPipeError):
        pass # Expected on disconnect
    except Exception as e:
        print(f"[Supervisor] [IPC]: Error handling client {addr}: {e}")
    finally:
        conn.close()
        print(f"[Supervisor] [IPC]: Client {addr} disconnected.")


def run_lock_server():
    global server_sock
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((LOCK_SERVER_HOST, LOCK_SERVER_PORT))
    server_sock.listen()
    print(f"[Supervisor]: Lock server listening on {LOCK_SERVER_HOST}:{LOCK_SERVER_PORT}")

    try:
        while True:
            conn, addr = server_sock.accept()
            client_handler = threading.Thread(target=handle_lock_client, args=(conn, addr))
            client_handler.daemon = True
            client_handler.start()
    except OSError as e: 
        print(f"IPC ServerError, {e}")
        pass

def run_checkpoint_loop():
    global app2_pid
    print(f"[Supervisor] [Checkpointer]: Loop started ({CHECKPOINT_INTERVAL}s).")
    while True:
        
        time.sleep(CHECKPOINT_INTERVAL)

        with master_lock:
            print("[Supervisor] [Checkpointer]: Snapshotting...")
            try:
                subprocess.run(["criu", "dump", "-t", str(app2_pid), "-D", CHECKPOINT_DIR, 
                                "--shell-job", "--leave-running", "--tcp-close", 
                                "--ext-unix-sk", "--skip-in-flight", "-v0", "-o", "dump_criu.log"], check=True)
                
                print("[Supervisor] [Checkpointer]: Snapshot SUCCESS.")
                rotate_log(RECV_LOG_PATH)
                rotate_log(SEND_LOG_PATH)
            except Exception as e:
                print(f"[Supervisor] [Checkpointer]: FAILED: {e}")
            

# --- Main script execution ---
try:
    
    #Start IPC SERVER
    server_thread = threading.Thread(target=run_lock_server)
    server_thread.daemon = True
    server_thread.start()
    
    print(f"[Supervisor]: Init Checkpoint Dir: '{CHECKPOINT_DIR}'")
    chkpt_path = pathlib.Path(CHECKPOINT_DIR)
    chkpt_path.mkdir(parents=True, exist_ok=True)
    
    # 2. Restore or Start logic
    if any(chkpt_path.iterdir()):
        # --- RESTORE ---
        print("[Supervisor]: Found existing checkpoint. Restoring...")
        
        restore_cmd = [
            "criu", "restore",
            "-D", CHECKPOINT_DIR,
            "--shell-job",
            "--tcp-close",
            "--ext-unix-sk",
            "--skip-in-flight"
        ]

        subprocess.Popen(restore_cmd)
        time.sleep(2) # Give it time to spawn
        
        # We find the PID by name because CRIU might restore it with the same PID
        # or we track the parent-child relationship.
        # Ideally, wait for it to appear.
        for _ in range(5):
            app2_pid = get_pid_by_name(APP_NAME)
            if app2_pid: break
            time.sleep(1)
            
        print(f"[Supervisor]: Restored PID: {app2_pid}")

    else:
        # --- FRESH START ---
        print(f"[Supervisor]: Starting fresh process: {COMMAND}")
        child_process = subprocess.Popen(COMMAND.split())
        app2_pid = child_process.pid
        print(f"[Supervisor]: Started PID: {app2_pid}")

    if not app2_pid:
        raise Exception("Failed to start/restore application.")

    # 3. Start Threads
    
    checkpoint_thread = threading.Thread(target=run_checkpoint_loop)
    checkpoint_thread.daemon = True
    checkpoint_thread.start()

    # 4. Watchdog
    while server_thread.is_alive() and checkpoint_thread.is_alive():
        time.sleep(1)
    # server_thread.join()

except KeyboardInterrupt:
    print("\n[Supervisor]: Shutting down...")
except Exception as e:
    print(f"\n[Supervisor]: Fatal Error: {e}")
finally:
    if app2_pid:
        try:
            os.kill(app2_pid, signal.SIGTERM)
        except OSError:
            pass
    if server_sock:
        server_sock.close()