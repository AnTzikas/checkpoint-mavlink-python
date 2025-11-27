import os, sys
from typing import BinaryIO

class InteractionJournal:

    def __init__(self, file_path: str):
        self._path = file_path
        # Open in Append Binary mode
        self._file: BinaryIO | None = None
        self._current_inode: int | None = None
        self._open_journal()
    
    def _open_journal(self):
        if self._file and not self._file.closed:
            self._file.close()
        
        self._file = open(self._path, "ab", buffering=0)
        self._file.seek(0)

        # Record the Inode. This is our "ID Card" for the file.
        self._current_inode = os.fstat(self._file.fileno()).st_ino


    def append_bytes(self, data: bytes) -> None:
        
        try:
            self._file.write(data)
            self._file.flush()
            os.fsync(self._file.fileno())
            
        except Exception as e:
            #Catch anything else to prevent crash
            error_line = sys.exc_info()[-1].tb_lineno
            print(f"[Logger] Unexpected error: {e} on line: {error_line}")


    def check_restore_needed(self) -> bool:
        
        try:
            stat_info = os.stat(self._path)
            disk_inode = stat_info.st_ino
            disk_size = stat_info.st_size

            if disk_inode != self._current_inode:
                # * Checkpoint happened reopen the file
                print(f"[Wrapper] Log Rotation Detected! (Old Inode: {self._current_inode} -> New: {disk_inode})")

                self._open_journal()

                return disk_size > 0
            
            return disk_size > self._file.tell()
            
        except Exception as e:
            error_line = sys.exc_info()[-1].tb_lineno
            print(f"[Logger] Unexpected error: {e} on line: {error_line}")
            exit(1)

       

    def read_restore_chunk(self) -> bytes:

        try:
            current_disk_size = os.fstat(self._file.fileno()).st_size
            read_len = current_disk_size - self._file.tell()
            
            if read_len <= 0:
                return b""

            # Open a read handle, seek to where we thought we were, read the rest
            with open(self._path, "rb") as reader:
                reader.seek(self._file.tell())
                delta_bytes = reader.read(read_len)
            
            # Fast-forward our internal state to match disk
            self._file.seek(0, 2)
        except Exception as e:
            error_line = sys.exc_info()[-1].tb_lineno
            print(f"[Logger] Unexpected error: {e} on line: {error_line}")
        
        return delta_bytes

    def truncate(self) -> None:
        """Resets log to zero (Supervisor Log Reset)."""
        self._file.truncate(0)
        self._file.seek(0)
        os.fsync(self._file.fileno())
        # self._known_size = 0

    def close(self) -> None:
        if not self._file.closed:
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
            except OSError:
                pass 
            self._file.close()