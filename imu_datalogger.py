#!/usr/bin/env python3
"""
imu_datalogger.py — Eventide module program for the SparkFun Qwiic 9DoF IMU
(ICM-20948: 3-axis accel + gyro + AK09916 magnetometer + temperature).

* Samples the IMU at a fixed rate and timestamps every sample with UTC wall-clock
  time (system clock — on an Eventide payload this is disciplined by the RTC/GPS/
  NTP), so the logs can be lined up with other sensors afterwards.
* Writes one JSON object per line (.jsonl) into --output-dir, which is the
  directory the dashboard's PLAYBACK tab lists. Line 1 of each file is a
  "header" record describing the sensor configuration and scale factors.
* Exposes a small HTTP API for the dashboard:
      GET  /api/recording   -> {"recording": bool, "recording_control": true, "current_file": str|null}
      PUT  /api/recording   <- {"recording": bool, "duration_seconds"?: number}
      GET  /api/telemetry   -> latest sample + status/statistics

The sensor is always read (so live telemetry works), but samples are only
written to disk while recording.

Stays in the foreground and logs to stdout, as supervisord expects.
"""

import argparse
import json
import logging
import math
import os
import sys
import threading
import time
from datetime import datetime, timezone

# ── Sensor scale factors (ICM-20948 datasheet) ────────────────────────────────
ACCEL_RANGES = {2: 0, 4: 1, 8: 2, 16: 3}               # g  -> register code
ACCEL_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}

GYRO_RANGES = {250: 0, 500: 1, 1000: 2, 2000: 3}       # dps -> register code
GYRO_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}

MAG_UT_PER_LSB = 0.15                                   # AK09916, fixed
TEMP_SENSITIVITY = 333.87                               # LSB / °C
TEMP_OFFSET_C = 21.0                                    # raw 0 == 21 °C

RAW_ORDER = ["ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "temp"]

RECONNECT_DELAY_S = 3.0
MAX_CONSECUTIVE_FAILURES = 5
FLUSH_INTERVAL_S = 1.0


def log(msg: str) -> None:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp} UTC] {msg}", flush=True)


def utc_iso(t_ns: int) -> str:
    """Epoch nanoseconds -> 'YYYY-MM-DDTHH:MM:SS.ffffffZ'."""
    dt = datetime.fromtimestamp(t_ns / 1e9, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def to_signed16(v: int) -> int:
    return v - 65536 if v > 32767 else v


# ── Sensor back-ends ──────────────────────────────────────────────────────────

class QwiicImu:
    """Real hardware, via the sparkfun-qwiic-icm20948 driver."""

    def __init__(self, address: int, accel_g: int, gyro_dps: int):
        self.address = address
        self.accel_g = accel_g
        self.gyro_dps = gyro_dps
        self.imu = None

    def open(self) -> bool:
        import qwiic_icm20948  # imported lazily so --simulate works anywhere

        imu = qwiic_icm20948.QwiicIcm20948(address=self.address)
        if not imu.connected:
            return False
        if not imu.begin():
            return False
        imu.setFullScaleRangeAccel(ACCEL_RANGES[self.accel_g])
        imu.setFullScaleRangeGyro(GYRO_RANGES[self.gyro_dps])
        self.imu = imu
        return True

    def read(self) -> dict:
        """Returns the raw integer readings; raises IOError on a failed read."""
        imu = self.imu
        if not imu.getAgmt():
            raise IOError("getAgmt() returned False")
        return {
            "ax": imu.axRaw, "ay": imu.ayRaw, "az": imu.azRaw,
            "gx": imu.gxRaw, "gy": imu.gyRaw, "gz": imu.gzRaw,
            "mx": imu.mxRaw, "my": imu.myRaw, "mz": imu.mzRaw,
            # the driver does not sign-convert the temperature register
            "temp": to_signed16(imu.tmpRaw),
        }


class SimulatedImu:
    """Fake sensor for bench testing without hardware (--simulate)."""

    def __init__(self, accel_g: int, gyro_dps: int):
        self.accel_g = accel_g
        self.gyro_dps = gyro_dps

    def open(self) -> bool:
        return True

    def read(self) -> dict:
        t = time.monotonic()
        a = ACCEL_LSB_PER_G[self.accel_g]
        g = GYRO_LSB_PER_DPS[self.gyro_dps]
        return {
            "ax": int(0.05 * a * math.sin(t)), "ay": int(0.05 * a * math.cos(t)), "az": int(a),
            "gx": int(10 * g * math.sin(0.5 * t)), "gy": int(5 * g * math.cos(0.5 * t)), "gz": 0,
            "mx": int(25 / MAG_UT_PER_LSB), "my": int(-10 / MAG_UT_PER_LSB), "mz": int(-40 / MAG_UT_PER_LSB),
            "temp": int((30.0 - TEMP_OFFSET_C) * TEMP_SENSITIVITY),
        }


# ── Logger ────────────────────────────────────────────────────────────────────

class ImuLogger:
    def __init__(self, sensor, args):
        self.sensor = sensor
        self.args = args
        self.output_dir = args.output_dir

        self._lock = threading.Lock()          # guards the recording state below
        self._file = None
        self._current_file = None
        self._last_flush = 0.0
        self._stop_at = None                   # monotonic deadline for timed recordings
        self._written_this_file = 0

        self.status = "starting"
        self.latest = None                     # most recent sample dict
        self.samples_written = 0               # across the current file
        self.actual_rate_hz = 0.0
        self._seq = 0

    # ---- recording control (called from the HTTP thread and the sensor loop) --

    def recording_state(self) -> dict:
        with self._lock:
            return {
                "recording": self._file is not None,
                "recording_control": True,
                "current_file": self._current_file,
            }

    def start_recording(self, duration_s: float | None = None) -> None:
        with self._lock:
            if self._file is not None:
                # Already recording; just (re)arm or clear the auto-stop timer.
                self._stop_at = time.monotonic() + duration_s if duration_s else None
                return
            os.makedirs(self.output_dir, exist_ok=True)
            start_ns = time.time_ns()
            stamp = datetime.fromtimestamp(start_ns / 1e9, tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            name = f"imu_{stamp}.jsonl"
            path = os.path.join(self.output_dir, name)
            n = 1
            while os.path.exists(path):        # two starts inside one second
                name = f"imu_{stamp}_{n}.jsonl"
                path = os.path.join(self.output_dir, name)
                n += 1
            self._file = open(path, "w", buffering=1 << 16, encoding="utf-8")
            self._current_file = name
            self._written_this_file = 0
            self.samples_written = 0
            self._last_flush = time.monotonic()
            self._stop_at = time.monotonic() + duration_s if duration_s else None
            self._file.write(json.dumps(self._header(start_ns), separators=(",", ":")) + "\n")
            self._file.flush()
        log(f"recording started -> {path}" + (f" (auto-stop in {duration_s:g}s)" if duration_s else ""))

    def stop_recording(self) -> None:
        with self._lock:
            if self._file is None:
                return
            name, count = self._current_file, self._written_this_file
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
            finally:
                self._file.close()
                self._file = None
                self._current_file = None
                self._stop_at = None
        log(f"recording stopped: {name} ({count} samples)")

    def _header(self, start_ns: int) -> dict:
        a = self.args
        return {
            "type": "header",
            "format": "eventide-imu-jsonl",
            "version": 1,
            "sensor": "ICM-20948",
            "start_utc": utc_iso(start_ns),
            "start_ns": start_ns,
            "timebase": "t_ns / t_utc = system UTC clock (CLOCK_REALTIME), taken at the midpoint of the I2C read",
            "rate_hz": a.rate,
            "accel_range_g": a.accel_range,
            "gyro_range_dps": a.gyro_range,
            "scale": {
                "accel_lsb_per_g": ACCEL_LSB_PER_G[a.accel_range],
                "gyro_lsb_per_dps": GYRO_LSB_PER_DPS[a.gyro_range],
                "mag_ut_per_lsb": MAG_UT_PER_LSB,
                "temp": "temp_c = raw/333.87 + 21",
            },
            "raw_order": RAW_ORDER,
            "axes_note": "Values are in the sensor frame. The magnetometer axes are not aligned with accel/gyro axes in the chip's native frame.",
            "simulated": bool(a.simulate),
        }

    # ---- sample handling ------------------------------------------------------

    def _make_sample(self, raw: dict, t_ns: int) -> dict:
        a_scale = ACCEL_LSB_PER_G[self.args.accel_range]
        g_scale = GYRO_LSB_PER_DPS[self.args.gyro_range]
        self._seq += 1
        r = lambda v: round(v, 5)
        return {
            "t_utc": utc_iso(t_ns),
            "t_ns": t_ns,
            "seq": self._seq,
            "accel_g": {k: r(raw["a" + k] / a_scale) for k in "xyz"},
            "gyro_dps": {k: r(raw["g" + k] / g_scale) for k in "xyz"},
            "mag_ut": {k: r(raw["m" + k] * MAG_UT_PER_LSB) for k in "xyz"},
            "temp_c": r(raw["temp"] / TEMP_SENSITIVITY + TEMP_OFFSET_C),
            "raw": [raw[k] for k in RAW_ORDER],
        }

    def _record_sample(self, sample: dict) -> None:
        with self._lock:
            if self._file is None:
                return
            self._file.write(json.dumps(sample, separators=(",", ":")) + "\n")
            self._written_this_file += 1
            self.samples_written = self._written_this_file
            now = time.monotonic()
            if now - self._last_flush >= FLUSH_INTERVAL_S:
                self._file.flush()
                self._last_flush = now

    # ---- main sampling loop ---------------------------------------------------

    def _connect(self) -> None:
        """Block until the sensor is open (retrying), updating status as we go."""
        while True:
            try:
                if self.sensor.open():
                    self.status = "ok"
                    log("IMU connected")
                    return
                self.status = "not connected"
                log("IMU not found / failed to initialise "
                    f"(I2C address 0x{self.args.i2c_address:02X}) — retrying")
            except Exception as exc:  # noqa: BLE001 - hardware/driver errors vary
                self.status = "error"
                log(f"IMU open error: {exc!r} — retrying")
            time.sleep(RECONNECT_DELAY_S)

    def run(self) -> None:
        period = 1.0 / self.args.rate
        self._connect()
        if self.args.autostart:
            self.start_recording()

        failures = 0
        next_t = time.monotonic()
        rate_window_start = time.monotonic()
        rate_window_count = 0

        while True:
            # Timed recording expired?
            with self._lock:
                expired = self._stop_at is not None and time.monotonic() >= self._stop_at
            if expired:
                self.stop_recording()

            try:
                t0 = time.time_ns()
                raw = self.sensor.read()
                t1 = time.time_ns()
                failures = 0
                if self.status != "ok":
                    self.status = "ok"
            except Exception as exc:  # noqa: BLE001
                failures += 1
                if failures == 1 or failures % 50 == 0:
                    log(f"IMU read error ({failures} in a row): {exc!r}")
                if failures >= MAX_CONSECUTIVE_FAILURES:
                    self.status = "reconnecting"
                    self._connect()
                    failures = 0
                    next_t = time.monotonic()
                time.sleep(period)
                continue

            sample = self._make_sample(raw, (t0 + t1) // 2)
            self.latest = sample
            self._record_sample(sample)

            # actual-rate estimate over ~2 s windows
            rate_window_count += 1
            now = time.monotonic()
            if now - rate_window_start >= 2.0:
                self.actual_rate_hz = rate_window_count / (now - rate_window_start)
                rate_window_start, rate_window_count = now, 0

            # fixed-cadence scheduling; if we fall badly behind, resync rather than burst
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -5 * period:
                next_t = time.monotonic()


# ── HTTP API ──────────────────────────────────────────────────────────────────

def build_app(logger: ImuLogger):
    from flask import Flask, jsonify, request

    app = Flask(__name__)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)   # dashboard polls constantly

    @app.get("/api/recording")
    def get_recording():
        return jsonify(logger.recording_state())

    @app.route("/api/recording", methods=["PUT", "POST"])
    def put_recording():
        body = request.get_json(silent=True) or {}
        want = bool(body.get("recording", True))
        if want:
            duration = body.get("duration_seconds")
            try:
                duration = float(duration) if duration else None
            except (TypeError, ValueError):
                duration = None
            if logger.status != "ok":
                # Still allowed — the file is created and fills once the sensor is back.
                log(f"recording requested while IMU status is '{logger.status}'")
            logger.start_recording(duration)
        else:
            logger.stop_recording()
        return jsonify(logger.recording_state())

    @app.get("/api/telemetry")
    def get_telemetry():
        return jsonify({
            "status": logger.status,
            "sample": logger.latest,
            "actual_rate_hz": logger.actual_rate_hz,
            "target_rate_hz": logger.args.rate,
            "samples_written": logger.samples_written,
            **logger.recording_state(),
        })

    return app


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_bind(value: str) -> tuple[str, int]:
    host, _, port = value.rpartition(":")
    return (host or "127.0.0.1"), int(port)


def main() -> None:
    p = argparse.ArgumentParser(description="Eventide IMU datalogger (SparkFun Qwiic ICM-20948)")
    p.add_argument("--output-dir", required=True, help="Directory for .jsonl recordings.")
    p.add_argument("--bind", default="127.0.0.1:8100", help="HTTP API bind address host:port.")
    p.add_argument("--rate", type=float, default=50.0, help="Sample rate in Hz (1-200).")
    p.add_argument("--accel-range", type=int, default=4, choices=sorted(ACCEL_RANGES), help="Accel full scale, g.")
    p.add_argument("--gyro-range", type=int, default=500, choices=sorted(GYRO_RANGES), help="Gyro full scale, deg/s.")
    p.add_argument("--i2c-address", type=lambda s: int(s, 0), default=0x69,
                   help="I2C address (decimal or 0x-prefixed hex). 0x69 default, 0x68 with jumper closed.")
    p.add_argument("--autostart", type=int, default=0, help="1 = begin recording at launch.")
    p.add_argument("--simulate", action="store_true", help="Use fake data (no hardware needed).")
    args = p.parse_args()

    if not 1.0 <= args.rate <= 200.0:
        p.error("--rate must be between 1 and 200 Hz")

    if args.simulate:
        sensor = SimulatedImu(args.accel_range, args.gyro_range)
        log("SIMULATION MODE — no hardware in use")
    else:
        sensor = QwiicImu(args.i2c_address, args.accel_range, args.gyro_range)

    logger = ImuLogger(sensor, args)

    host, port = parse_bind(args.bind)
    app = build_app(logger)
    threading.Thread(
        target=lambda: app.run(host=host, port=port, threaded=True, use_reloader=False),
        daemon=True,
    ).start()
    log(f"HTTP API on {host}:{port}; recordings -> {args.output_dir}; "
        f"{args.rate:g} Hz, ±{args.accel_range} g, ±{args.gyro_range} dps")

    try:
        logger.run()
    except KeyboardInterrupt:
        pass
    finally:
        logger.stop_recording()   # make sure the file is flushed/closed on SIGINT
        log("shutting down")


if __name__ == "__main__":
    # supervisord stops programs with SIGTERM: turn it into a clean exit so the
    # recording file gets flushed in the `finally` above.
    import signal
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    main()
