"""
Threaded single VL53L1X ToF reader for quick overlay use.
"""

import threading
import time
from typing import Optional, Dict, Any

import board
import busio
from smbus2 import SMBus
import adafruit_vl53l1x


class ToFReader:
    """Background reader for a single VL53L1X sensor."""

    def __init__(self, bus: int = 1, address: int = 0x29, poll_hz: int = 20):
        self.bus_id = bus
        self.address = address
        self.poll_hz = max(1, poll_hz)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[float] = None

        self._smbus: Optional[SMBus] = None
        self._i2c = None
        self._sensor = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="tof-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)
        self._cleanup()

    def latest_distance_m(self) -> Optional[float]:
        with self._lock:
            return self._latest

    def _run(self) -> None:
        interval = 1.0 / self.poll_hz
        try:
            self._smbus = SMBus(self.bus_id)
            self._i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            self._sensor = adafruit_vl53l1x.VL53L1X(self._i2c, address=self.address)
            self._sensor.start_ranging()
        except Exception:
            self._cleanup()
            return

        while not self._stop.is_set():
            start = time.time()
            reading = self._read_once()
            if reading is not None:
                with self._lock:
                    self._latest = reading
            elapsed = time.time() - start
            to_sleep = max(0.0, interval - elapsed)
            time.sleep(to_sleep)

        self._cleanup()

    def _read_once(self) -> Optional[float]:
        if not self._sensor:
            return None
        try:
            if not self._sensor.data_ready:
                return None
            dist_mm = self._sensor.distance
            self._sensor.clear_interrupt()
            if dist_mm is None:
                return None
            meters = dist_mm / 1000.0
            if meters < 0 or meters > 4.0:
                return None
            return meters
        except Exception:
            return None

    def _cleanup(self) -> None:
        try:
            if self._sensor:
                self._sensor.stop_ranging()
        except Exception:
            pass
        try:
            if self._smbus:
                self._smbus.close()
        except Exception:
            pass

    def to_dict(self) -> Dict[str, Any]:
        """Return latest reading as a dict for logging."""
        return {"distance_m": self.latest_distance_m()}

