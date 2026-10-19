# imu-datalogger

Eventide module that logs a **SparkFun Qwiic 9DoF IMU (ICM-20948)** — 3-axis
accelerometer, gyroscope, magnetometer and temperature — to UTC-timestamped
JSONL files, so the data can be lined up with the other payload sensors later.

It uses the SparkFun `sparkfun-qwiic-icm20948` package for all sensor I/O.

## Hardware / Pi 5 setup

1. Plug the board into the Pi's I2C bus (Qwiic → Pi header: 3V3, GND, SDA = GPIO2, SCL = GPIO3).
2. Enable I2C once: `sudo raspi-config nonint do_i2c 0` (or *Interface Options → I2C*), then reboot.
3. Check the board is visible: `sudo i2cdetect -y 1` → expect `69` (or `68` if the address jumper is closed).

## Install

Dashboard → **MODULES** → paste the repo URL (or upload a zip of this folder) → **INSTALL**.
A venv is created and `sparkfun-qwiic-icm20948` + `flask` are installed into it.

## Using it

* Dashboard MAIN tab shows an **IMU** live readout and an **IMU RECORD** control. It is
  also picked up by **RECORD ALL / STOP ALL** and the scheduler, including timed recordings.
* Recordings land in `<recordings_dir>/imu/imu_<UTC start>.jsonl` and appear in the PLAYBACK tab.
* By default the program *reads* the sensor continuously (live telemetry) but only *writes* while
  recording. Set `--autostart 1` to record from launch.

## Log format (JSON Lines)

Line 1 is a header; every following line is one sample:

```json
{"type":"header","format":"eventide-imu-jsonl","version":1,"sensor":"ICM-20948","start_utc":"2026-10-19T23:32:18.995597Z","rate_hz":50.0,"accel_range_g":4,"gyro_range_dps":500,"scale":{...},"raw_order":["ax","ay","az","gx","gy","gz","mx","my","mz","temp"]}
{"t_utc":"2026-10-19T23:32:19.010880Z","t_ns":1792452739010880431,"seq":97,"accel_g":{"x":-0.048,"y":0.013,"z":1.0},"gyro_dps":{"x":6.1,"y":-3.95,"z":0.0},"mag_ut":{"x":24.9,"y":-9.9,"z":-39.9},"temp_c":30.0,"raw":[-396,103,8192,400,-259,0,166,-66,-266,3004]}
```

* `t_utc` / `t_ns` — system UTC clock (epoch nanoseconds), taken at the midpoint of the I2C read.
  Use `t_ns` to merge with other sensors; it is only as good as the payload clock (RTC/GPS/NTP).
* `accel_g` (g), `gyro_dps` (°/s), `mag_ut` (µT), `temp_c` (°C) — scaled from `raw` using the ranges in the header.
* `raw` — the unscaled 16-bit values in `raw_order`, so nothing is lost if you want to rescale.
* `seq` — sample counter since the program started (gaps = dropped reads).

Load into pandas:

```python
import pandas as pd, json
rows = [json.loads(l) for l in open("imu_20261019T233218Z.jsonl")][1:]
df = pd.json_normalize(rows); df["t"] = pd.to_datetime(df["t_ns"], unit="ns", utc=True)
```

## Arguments (editable from the MODULES tab)

| Flag | Default | Notes |
| --- | --- | --- |
| `--rate` | 50 | Hz, 1–200 |
| `--accel-range` | 4 | ±g: 2, 4, 8, 16 |
| `--gyro-range` | 500 | ±°/s: 250, 500, 1000, 2000 |
| `--i2c-address` | 105 (0x69) | decimal in the manifest; the CLI also accepts `0x68` |
| `--autostart` | 0 | 1 = record at launch |

## HTTP API (socket `http_api`, reachable via `/proxy/imu-datalogger/http_api/...`)

* `GET /api/recording` → `{"recording", "recording_control", "current_file"}`
* `PUT /api/recording` with `{"recording": true|false, "duration_seconds"?: n}`
* `GET /api/telemetry` → latest sample, sensor status, actual vs. target rate

## Local bench test (no hardware)

```bash
pip install flask
python3 imu_datalogger.py --output-dir ./rec --bind 127.0.0.1:8123 --simulate --autostart 1
```

## Notes

* The sensor driver is polled at a fixed cadence rather than interrupt-driven, so timestamps carry
  a little scheduling jitter (typically ≪ 1 ms on a Pi 5, but not hard real-time).
* The SparkFun driver's `begin()` disables the on-chip low-pass filters; if you log at a low rate
  you may want to low-pass or decimate in post.
* If the sensor drops off the bus the program retries automatically and keeps the file open.
