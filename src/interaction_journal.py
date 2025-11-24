import os
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
        
        # Internal tracker: "What size does THIS process think the file is?"
        # On a fresh start, this might be 0 or existing size.
        # We initialize it to the current file size so we append correctly.
        # BUT, if we are restored from a snapshot, this variable will 
        # 'time travel' back to the old value, triggering the detection logic.
        self._known_size = 0

    def append_bytes(self, data: bytes) -> None:
        """Writes raw bytes and forces immediate OS-level flush."""
        self._file.write(data)
        self._file.flush()
        os.fsync(self._file.fileno())
        
        # Update our internal view of the world
        self._known_size += len(data)

    def check_restore_needed(self) -> bool:
        """
        Checks if the physical file is larger than our internal tracker.
        
        Logic:
        - If disk_size > known_size: External write occurred (Crash/Restore).
        - If disk_size == known_size: We are in sync.
        - If disk_size < known_size: Impossible (unless file truncated externally).
        """
        # We use stat on the path (or fd) to get the REAL on-disk size
        current_disk_size = os.fstat(self._file.fileno()).st_size
        return current_disk_size > self._known_size

    def read_restore_chunk(self) -> bytes:
        """
        Reads only the NEW bytes (Delta) that appeared since our last write.
        Updates _known_size to match the new reality.
        """
        current_disk_size = os.fstat(self._file.fileno()).st_size
        read_len = current_disk_size - self._known_size
        
        if read_len <= 0:
            return b""

        # Open a read handle, seek to where we thought we were, read the rest
        with open(self._path, "rb") as reader:
            reader.seek(self._known_size)
            delta_bytes = reader.read(read_len)
        
        # Fast-forward our internal state to match disk
        self._known_size = current_disk_size
        
        return delta_bytes

    def truncate(self) -> None:
        """Resets log to zero (Supervisor Log Reset)."""
        self._file.truncate(0)
        self._file.seek(0)
        os.fsync(self._file.fileno())
        self._known_size = 0

    def close(self) -> None:
        if not self._file.closed:
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
            except OSError:
                pass 
            self._file.close()