import math
import os
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class Telemetry:
    timestamp_ms: int
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_m: Optional[float] = None
    yaw_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    groundspeed_mps: Optional[float] = None

    def to_dict(self) -> dict:
        return {
            "timestamp_ms": int(self.timestamp_ms),
            "lat": self.lat,
            "lon": self.lon,
            "alt_m": self.alt_m,
            "yaw_deg": self.yaw_deg,
            "pitch_deg": self.pitch_deg,
            "roll_deg": self.roll_deg,
            "groundspeed_mps": self.groundspeed_mps,
        }


class MAVLinkBridge:
    """
    Reads MAVLink from SITL and emits normalized telemetry over Socket.IO.

    Default connection string is controlled by env MAVLINK_CONNECTION.
    """

    def __init__(self, socketio, connection: Optional[str] = None, emit_hz: float = 15.0):
        self.socketio = socketio
        self.connection = connection or os.environ.get("MAVLINK_CONNECTION", "udp:127.0.0.1:14550")
        self.emit_hz = emit_hz
        self._running = False
        self._telemetry = Telemetry(timestamp_ms=int(time.time() * 1000))

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self.socketio.start_background_task(self._run)

    def stop(self) -> None:
        self._running = False

    def _run(self) -> None:
        try:
            from pymavlink import mavutil
        except Exception as e:
            # If pymavlink isn't installed, just don't stream telemetry.
            self.socketio.emit("telemetry", {"timestamp_ms": int(time.time() * 1000), "error": str(e)})
            return

        m = None
        while self._running and m is None:
            try:
                m = mavutil.mavlink_connection(self.connection, autoreconnect=True)
            except Exception:
                m = None
            if m is None:
                self.socketio.sleep(1.0)

        if m is None:
            return

        try:
            # Don't block forever; we still want to emit if only partial data is available
            m.wait_heartbeat(timeout=5)
        except Exception:
            pass

        next_emit = time.time()
        emit_period = 1.0 / max(self.emit_hz, 1.0)

        while self._running:
            try:
                msg = m.recv_match(blocking=False)
            except Exception:
                msg = None

            if msg is not None:
                t_ms = int(time.time() * 1000)
                self._telemetry.timestamp_ms = t_ms

                msg_type = msg.get_type()
                if msg_type == "GLOBAL_POSITION_INT":
                    # lat/lon in 1E7, alt is mm above MSL, relative_alt is mm above ground
                    self._telemetry.lat = float(msg.lat) / 1e7
                    self._telemetry.lon = float(msg.lon) / 1e7
                    # Prefer relative_alt if present (AGL), else alt (MSL)
                    rel = getattr(msg, "relative_alt", None)
                    if rel is not None:
                        self._telemetry.alt_m = float(rel) / 1000.0
                    else:
                        self._telemetry.alt_m = float(msg.alt) / 1000.0
                    # vx/vy in cm/s
                    vx = getattr(msg, "vx", None)
                    vy = getattr(msg, "vy", None)
                    if vx is not None and vy is not None:
                        self._telemetry.groundspeed_mps = math.sqrt((vx / 100.0) ** 2 + (vy / 100.0) ** 2)
                elif msg_type == "ATTITUDE":
                    # radians
                    self._telemetry.roll_deg = float(msg.roll) * 180.0 / math.pi
                    self._telemetry.pitch_deg = float(msg.pitch) * 180.0 / math.pi
                    self._telemetry.yaw_deg = float(msg.yaw) * 180.0 / math.pi

            now = time.time()
            if now >= next_emit:
                self.socketio.emit("telemetry", self._telemetry.to_dict())
                next_emit = now + emit_period
            self.socketio.sleep(0.01)

