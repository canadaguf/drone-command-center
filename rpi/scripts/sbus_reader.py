"""
Minimal SBUS reader over UART for RC channel overlays.
"""

import threading
import time
from typing import Dict, List, Optional

import serial


SBUS_FRAME_LEN = 25
SBUS_HEADER = 0x0F

# SBUS channel raw ranges (approx.)
SBUS_MIN = 172
SBUS_MAX = 1811


def _decode_frame(frame: bytes) -> Optional[List[int]]:
    """Decode 16 channel values from a 25-byte SBUS frame."""
    if len(frame) != SBUS_FRAME_LEN or frame[0] != SBUS_HEADER:
        return None

    # 16 channels, 11 bits each
    chans = []
    bits = 0
    bit_index = 0
    for i in range(1, 23):  # bytes 1..22 carry channel bits
        bits |= frame[i] << bit_index
        bit_index += 8
        while bit_index >= 11 and len(chans) < 16:
            chans.append(bits & 0x7FF)
            bits >>= 11
            bit_index -= 11

    if len(chans) < 16:
        chans.extend([SBUS_MIN] * (16 - len(chans)))

    return chans[:16]


def normalize(value: int, in_min: int = SBUS_MIN, in_max: int = SBUS_MAX) -> float:
    """Normalize SBUS value to 0..1 clamped."""
    if in_max == in_min:
        return 0.0
    return max(0.0, min(1.0, (value - in_min) / float(in_max - in_min)))


class SBUSReader:
    """Background SBUS reader using pyserial."""

    def __init__(
        self,
        port: str = "/dev/ttyS0",
        baudrate: int = 100000,
        inverted: bool = True,
        poll_hz: int = 50,
    ):
        # Note: Inversion typically handled by hardware inverter/adapter.
        self.port = port
        self.baudrate = baudrate
        self.inverted = inverted
        self.poll_hz = max(10, poll_hz)

        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[List[int]] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sbus-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)
        self._close()

    def latest_channels(self) -> Optional[List[int]]:
        with self._lock:
            return list(self._latest) if self._latest else None

    def latest_normalized(self) -> Optional[List[float]]:
        chans = self.latest_channels()
        if chans is None:
            return None
        return [normalize(v) for v in chans]

    def _run(self) -> None:
        interval = 1.0 / self.poll_hz
        try:
            self._ser = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO,
                timeout=0.02,
            )
        except Exception:
            self._close()
            return

        buffer = bytearray()
        while not self._stop.is_set():
            start = time.time()
            try:
                data = self._ser.read(self._ser.in_waiting or SBUS_FRAME_LEN)
                if data:
                    buffer.extend(data)
                    # search for header and extract frames
                    while len(buffer) >= SBUS_FRAME_LEN:
                        if buffer[0] != SBUS_HEADER:
                            # discard until header
                            buffer.pop(0)
                            continue
                        frame = bytes(buffer[:SBUS_FRAME_LEN])
                        buffer = buffer[SBUS_FRAME_LEN:]
                        decoded = _decode_frame(frame)
                        if decoded:
                            with self._lock:
                                self._latest = decoded
                        else:
                            continue
            except Exception:
                pass

            elapsed = time.time() - start
            sleep_for = max(0.0, interval - elapsed)
            time.sleep(sleep_for)

        self._close()

    def _close(self) -> None:
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass

    def map_named_channels(self, channel_map: Dict[str, int]) -> Dict[str, Optional[float]]:
        """Return normalized named channels according to map."""
        normalized = self.latest_normalized()
        if normalized is None:
            return {name: None for name in channel_map.keys()}
        out: Dict[str, Optional[float]] = {}
        for name, idx in channel_map.items():
            out[name] = normalized[idx] if 0 <= idx < len(normalized) else None
        return out

