#!/usr/bin/env python3
"""
test_sender.py — synthetic AircraftState UDP emitter for testing udp_plot.py

Sends 10 Hz JSON datagrams to 127.0.0.1:8000 (or --host / --port).
All values are driven by simple sinusoids so you get visible motion
on every channel immediately.

Usage:
    python3 test_sender.py
    python3 test_sender.py --host 127.0.0.1 --port 8000 --hz 10
"""

import argparse
import json
import math
import socket
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--hz",   type=float, default=10.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.hz
    print(f"Sending to {args.host}:{args.port} at {args.hz} Hz — Ctrl-C to stop")

    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0

        state = {
            "theta":              15.0  * math.sin(2 * math.pi * t / 8.0),
            "q":                   5.0  * math.cos(2 * math.pi * t / 4.0),
            "phi":                20.0  * math.sin(2 * math.pi * t / 10.0),
            "p":                   8.0  * math.cos(2 * math.pi * t / 3.0),
            "psi":               180.0  * math.sin(2 * math.pi * t / 30.0) + 180.0,
            "r":                   3.0  * math.sin(2 * math.pi * t / 6.0),
            "altitude_ft":      2000.0  + 500.0 * math.sin(2 * math.pi * t / 20.0),
            "vertical_speed_fpm": 300.0 * math.cos(2 * math.pi * t / 20.0),
            "airspeed_kts":      120.0  + 10.0  * math.sin(2 * math.pi * t / 15.0),
            "slip_deg":            3.0  * math.sin(2 * math.pi * t / 5.0),
            "throttle":            0.5  + 0.4   * math.sin(2 * math.pi * t / 12.0),
        }

        payload = json.dumps(state).encode()
        sock.sendto(payload, (args.host, args.port))

        time.sleep(period)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
