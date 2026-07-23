#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time

import serial


def parse_nmea_lat_lon(lat_value, lat_dir, lon_value, lon_dir):
    if lat_value == "" or lon_value == "":
        return None

    lat_raw = float(lat_value)
    lat = int(lat_raw / 100) + (lat_raw % 100) / 60.0
    if lat_dir == "S":
        lat = -lat

    lon_raw = float(lon_value)
    lon = int(lon_raw / 100) + (lon_raw % 100) / 60.0
    if lon_dir == "W":
        lon = -lon

    return lat, lon


def parse_gga(line):
    if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
        return None

    parts = line.split(",")
    if len(parts) < 10:
        return None
    if parts[2] == "" or parts[4] == "" or parts[6] == "0":
        return None

    lat_lon = parse_nmea_lat_lon(parts[2], parts[3], parts[4], parts[5])
    if lat_lon is None:
        return None

    lat, lon = lat_lon
    return {
        "time": parts[1],
        "lat": lat,
        "lon": lon,
        "fix_quality": int(parts[6]) if parts[6].isdigit() else 0,
        "satellites": int(parts[7]) if parts[7].isdigit() else 0,
        "hdop": float(parts[8]) if parts[8] else math.nan,
        "altitude": float(parts[9]) if parts[9] else math.nan,
    }


def parse_uniheadinga(line):
    if not line.startswith("#UNIHEADINGA"):
        return None
    if ";" not in line:
        return None

    _, payload = line.split(";", 1)
    payload = payload.split("*", 1)[0]
    parts = payload.split(",")
    if len(parts) < 5:
        return None

    try:
        return {
            "solution_status": parts[0],
            "position_type": parts[1],
            "baseline_length_m": float(parts[2]),
            "heading_deg": float(parts[3]),
            "pitch_deg": float(parts[4]),
            "heading_std_deg": float(parts[6]) if len(parts) > 6 and parts[6] else math.nan,
            "pitch_std_deg": float(parts[7]) if len(parts) > 7 and parts[7] else math.nan,
        }
    except ValueError:
        return None


def ros_yaw_from_heading_deg(heading_deg):
    yaw = math.radians(90.0 - heading_deg)
    return math.atan2(math.sin(yaw), math.cos(yaw))


def main():
    parser = argparse.ArgumentParser(
        description="Read GPS GGA and UNIHEADINGA heading from one serial port."
    )
    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--print-raw", action="store_true")
    args = parser.parse_args()

    deadline = time.time() + args.timeout
    got_gps = False
    got_heading = False

    print(f"Opening {args.port} at {args.baud} baud")
    print("Waiting for both $GNGGA/$GPGGA and #UNIHEADINGA...")

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        while time.time() < deadline:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            if args.print_raw:
                print(raw)

            gps = parse_gga(raw)
            if gps is not None:
                got_gps = True
                print(
                    "GPS: lat={lat:.12f} lon={lon:.12f} alt={altitude:.3f}m "
                    "fix={fix_quality} sats={satellites} hdop={hdop}".format(**gps)
                )

            heading = parse_uniheadinga(raw)
            if heading is not None:
                got_heading = True
                yaw = ros_yaw_from_heading_deg(heading["heading_deg"])
                print(
                    "HEADING: heading={heading_deg:.4f}deg ros_yaw={yaw:.6f}rad "
                    "pitch={pitch_deg:.4f}deg baseline={baseline_length_m:.4f}m "
                    "heading_std={heading_std_deg:.4f}deg "
                    "pitch_std={pitch_std_deg:.4f}deg "
                    "status={solution_status} type={position_type}".format(
                        yaw=yaw,
                        **heading,
                    )
                )

            if got_gps and got_heading:
                print("OK: parsed both GPS position and dual-antenna heading.")
                return 0

    missing = []
    if not got_gps:
        missing.append("GPS GGA")
    if not got_heading:
        missing.append("UNIHEADINGA heading")
    print("FAILED: missing " + ", ".join(missing), file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
