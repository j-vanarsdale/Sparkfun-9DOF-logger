#!/usr/bin/env python3
"""Basic IMU logger: raw ICM-20948 values + UTC timestamp, one JSON line per sample."""
import argparse, json, os, sys, time
from datetime import datetime, timezone

import qwiic_icm20948


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output-dir", required=True)
    args = p.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    IMU = qwiic_icm20948.QwiicIcm20948()
    while not IMU.connected:
        print("IMU not connected, retrying...", file=sys.stderr, flush=True)
        time.sleep(3)
    IMU.begin()

    name = datetime.now(timezone.utc).strftime("imu_%Y%m%dT%H%M%SZ.jsonl")
    path = os.path.join(args.output_dir, name)
    print(f"logging to {path}", flush=True)

    last_flush = time.monotonic()
    with open(path, "w") as f:
        while True:
            if IMU.dataReady():
                IMU.getAgmt()
                now = datetime.now(timezone.utc)
                f.write(json.dumps({
                    "t_utc": now.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z",
                    "ax": IMU.axRaw, "ay": IMU.ayRaw, "az": IMU.azRaw,
                    "gx": IMU.gxRaw, "gy": IMU.gyRaw, "gz": IMU.gzRaw,
                    "mx": IMU.mxRaw, "my": IMU.myRaw, "mz": IMU.mzRaw,
                }) + "\n")
                if time.monotonic() - last_flush > 1:
                    f.flush()
                    last_flush = time.monotonic()
                time.sleep(0.03)
            else:
                time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, SystemExit):
        sys.exit(0)
