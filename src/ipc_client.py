import socket
import time
import os

LOCK_SERVER_HOST = os.environ.get("IPC_IP", "127.0.0.1")
LOCK_SERVER_PORT = int(os.environ.get("IPC_PORT", 12345))

class IPCClient:

    def __init__(self):
        self._sock = None
        self._is_connected = False
        self._lock_acquired = False
        # We connect immediately on init

        self._connect_loop()
        print("Client connected")

    def _connect_loop(self):
        
        while not self._is_connected:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.connect((LOCK_SERVER_HOST, LOCK_SERVER_PORT))
                self._is_connected = True
            except Exception:
                time.sleep(3) 

    def acquire(self):

        while True:
            
            if not self._is_connected:
                self._lock_acquired = False #After reconnection you lost the lock, you have to retry.
                self._connect_loop()

            if self._lock_acquired:
                return
            
            try:
                # print("[Inside] Try to acquire!")
                self._sock.sendall("Acquire".encode())
                
                # 3. Wait for Grant
                resp = self._sock.recv(1024).decode()
                
                if resp == "Granted":
                    self._lock_acquired = True
                    return # Success
                print(f"Error got unexpected msg code {resp}")

            except (OSError, BrokenPipeError):
                # If failed, close and retry loop immediately
                self._close()
                self._lock_acquired
                

    def release(self):
        if not self._is_connected:
            print("[Error] Lost lock while on checkpoint safe, Exit.")
            exit(2)

        try:
            # Protocol: Request Release
            # print("Try to release lock")
            self._sock.sendall("Release".encode())
            # Wait for Confirmation (Sync)
            resp = self._sock.recv(1024).decode() 
            # print("Got Accepted for releasing lock!")
            self._lock_acquired = False

        except Exception:
            self._close()

    def _close(self):
        if self._sock:
            try: self._sock.close()
            except: pass
        self._sock = None
        self._is_connected = False
        self._lock_acquired - False
