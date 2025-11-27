import os, sys
from typing import BinaryIO

class InteractionJournal:
    """A write-ahead log handler that enforces strict disk synchronization.
    
    Maintains an internal cursor (_known_size) to detect discrepancies 
    between process memory and physical disk state (Checkpoints).
    """

    def __init__(self, file_path: str):
        self._path = file_path
        # Open in Append Binary mode
        self._file: BinaryIO = open(file_path, "ab", buffering=0)
        self._file.seek(0)
        

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
            current_disk_size = os.fstat(self._file.fileno()).st_size
            value = current_disk_size - self._file.tell()
            # print(f"\nCurrent size={current_disk_size}\nDiff={value}\n")
            if value < 0:
                print("[Wrapper] [Interaction Journal] Checkpoint truncated the log go to pos 0.")
                self._file.seek(0)
            
        except Exception as e:
            error_line = sys.exc_info()[-1].tb_lineno
            print(f"[Logger] Unexpected error: {e} on line: {error_line}")

        return current_disk_size > self._file.tell()

    def read_restore_chunk(self) -> bytes:
        """
        Reads only the NEW bytes (Delta) that appeared since our last write.
        Updates _known_size to match the new reality.
        """

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