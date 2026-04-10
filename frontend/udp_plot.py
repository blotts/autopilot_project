#!/usr/bin/env python3
"""
udp_plot.py — Live aircraft state visualiser
Listens for JSON-encoded AircraftState datagrams on UDP 0.0.0.0:8000
and plots all fields in real time.

WSL2 SETUP (run once in an elevated PowerShell on the Windows host):
  $wsl_ip = (wsl hostname -I).Trim().Split()[0]
  netsh interface portproxy add v4tov4 `
        listenport=8000 listenaddress=127.0.0.1 `
        connectport=8000 connectaddress=$wsl_ip
  netsh advfirewall firewall add rule name="WSL UDP 8000" `
        dir=in action=allow protocol=UDP localport=8000

To remove the proxy later:
  netsh interface portproxy delete v4tov4 listenport=8000 listenaddress=127.0.0.1

Requirements (pip install --break-system-packages matplotlib):
  matplotlib

Usage:
  python3 udp_plot.py [--port 8000] [--history 300]
"""

import argparse
import json
import queue
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, fields

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------

@dataclass
class AircraftState:
    theta: float = 0.0            # pitch angle [deg]
    q: float = 0.0                # pitch rate [deg/s]
    phi: float = 0.0              # roll angle [deg]
    p: float = 0.0                # roll rate [deg/s]
    psi: float = 0.0              # heading [deg]
    r: float = 0.0                # yaw rate [deg/s]
    altitude_ft: float = 0.0     # altitude [ft]
    vertical_speed_fpm: float = 0.0  # vertical speed [ft/min]
    airspeed_kts: float = 0.0    # indicated airspeed [kt]
    slip_deg: float = 0.0        # beta/slip [deg]
    throttle: float = 0.0        # throttle ratio [0, 1]


FIELD_NAMES = [f.name for f in fields(AircraftState)]

# Group fields into subplot panels for visual coherence
PANELS = [
    ("Attitude",        ["theta", "phi", "psi"],              ["Pitch [deg]", "Roll [deg]", "Heading [deg]"]),
    ("Angular Rates",   ["q", "p", "r"],                      ["Pitch rate [°/s]", "Roll rate [°/s]", "Yaw rate [°/s]"]),
    ("Flight Params",   ["altitude_ft", "vertical_speed_fpm", "airspeed_kts"],
                        ["Altitude [ft]", "VS [fpm]", "IAS [kt]"]),
    ("Controls",        ["slip_deg", "throttle"],             ["Slip [deg]", "Throttle [0-1]"]),
]

COLORS = [
    "#4e9af1", "#f47c3c", "#6bcb77",   # panel 1
    "#e05c7e", "#b57bee", "#f5c842",   # panel 2
    "#48ccc9", "#f4845f", "#98d8c8",   # panel 3
    "#c9b99a", "#a3d977",              # panel 4
]

COLOR_MAP = {name: COLORS[i] for i, name in enumerate(FIELD_NAMES)}

# ---------------------------------------------------------------------------
# UDP receiver thread
# ---------------------------------------------------------------------------

def receiver_thread(host: str, port: int, q: queue.Queue, stop_event: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.5)
    try:
        sock.bind((host, port))
    except OSError as e:
        print(f"[ERROR] Cannot bind to {host}:{port} — {e}")
        stop_event.set()
        return

    print(f"[UDP] Listening on {host}:{port} …")

    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[RECV ERR] {e}")
            continue

        try:
            payload = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError as e:
            print(f"[JSON ERR] {e} | raw: {data[:80]}")
            continue

        # Accept either a flat dict or one nested under a key
        if not isinstance(payload, dict):
            continue
        # Some senders wrap it: {"state": {...}}
        if len(payload) == 1 and isinstance(next(iter(payload.values())), dict):
            payload = next(iter(payload.values()))

        q.put_nowait(payload)

    sock.close()
    print("[UDP] Socket closed.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Live aircraft state UDP plotter")
    parser.add_argument("--port",    type=int, default=8000, help="UDP port (default 8000)")
    parser.add_argument("--host",    default="0.0.0.0",      help="Bind address (default 0.0.0.0)")
    parser.add_argument("--history", type=int, default=300,  help="Samples to keep in view (default 300)")
    args = parser.parse_args()

    pkt_queue: queue.Queue = queue.Queue(maxsize=500)
    stop_event = threading.Event()

    t = threading.Thread(
        target=receiver_thread,
        args=(args.host, args.port, pkt_queue, stop_event),
        daemon=True,
    )
    t.start()

    # ------------------------------------------------------------------
    # Circular buffers: timestamps + one deque per field
    # ------------------------------------------------------------------
    N = args.history
    times: deque = deque(maxlen=N)
    buffers: dict[str, deque] = {name: deque(maxlen=N) for name in FIELD_NAMES}
    pkt_count = [0]
    last_pkt_time = [None]

    # ------------------------------------------------------------------
    # Build figure
    # ------------------------------------------------------------------
    matplotlib.rcParams.update({
        "axes.facecolor":   "#0d1117",
        "figure.facecolor": "#0d1117",
        "axes.edgecolor":   "#30363d",
        "axes.grid":        True,
        "grid.color":       "#21262d",
        "grid.linewidth":   0.6,
        "text.color":       "#c9d1d9",
        "axes.labelcolor":  "#8b949e",
        "xtick.color":      "#6e7681",
        "ytick.color":      "#6e7681",
        "font.family":      "monospace",
        "axes.titlesize":   9,
        "axes.labelsize":   8,
        "xtick.labelsize":  7,
        "ytick.labelsize":  7,
    })

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Aircraft State Monitor", color="#e6edf3", fontsize=13, fontweight="bold", y=0.98)
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35,
                           left=0.07, right=0.97, top=0.93, bottom=0.07)

    axes_grid: list[list[plt.Axes]] = []
    lines_grid: list[list[plt.Line2D]] = []

    for panel_idx, (title, field_keys, ylabels) in enumerate(PANELS):
        row, col = divmod(panel_idx, 2)
        inner_gs = gridspec.GridSpecFromSubplotSpec(
            len(field_keys), 1,
            subplot_spec=gs[row, col],
            hspace=0.15,
        )
        panel_axes = []
        panel_lines = []
        for sub_idx, (key, ylabel) in enumerate(zip(field_keys, ylabels)):
            share_x = panel_axes[0] if sub_idx > 0 else None
            ax = fig.add_subplot(inner_gs[sub_idx], sharex=share_x)
            ax.set_ylabel(ylabel, labelpad=3)
            ax.tick_params(labelbottom=(sub_idx == len(field_keys) - 1))
            if sub_idx == 0:
                ax.set_title(title, loc="left", pad=4,
                             color="#e6edf3", fontweight="bold")
            line, = ax.plot([], [], color=COLOR_MAP[key], linewidth=1.1, antialiased=True)
            panel_axes.append(ax)
            panel_lines.append(line)

        axes_grid.append(panel_axes)
        lines_grid.append(panel_lines)

    # Status bar text
    status_text = fig.text(0.01, 0.005, "Waiting for data…",
                           fontsize=7, color="#6e7681",
                           verticalalignment="bottom")

    # ------------------------------------------------------------------
    # Animation update
    # ------------------------------------------------------------------

    def update(_frame):
        # Drain all pending packets
        drained = 0
        while not pkt_queue.empty():
            try:
                payload = pkt_queue.get_nowait()
            except queue.Empty:
                break

            t_now = time.monotonic()
            times.append(t_now)
            last_pkt_time[0] = t_now
            pkt_count[0] += 1

            for name in FIELD_NAMES:
                val = payload.get(name, 0.0)
                try:
                    val = float(val)
                except (TypeError, ValueError):
                    val = 0.0
                buffers[name].append(val)

            drained += 1

        if not times:
            return []

        t_arr = list(times)
        t0 = t_arr[0]
        x = [t - t0 for t in t_arr]

        updated = []
        for panel_idx, (_, field_keys, _) in enumerate(PANELS):
            for sub_idx, key in enumerate(field_keys):
                ax = axes_grid[panel_idx][sub_idx]
                line = lines_grid[panel_idx][sub_idx]
                y = list(buffers[key])

                if len(x) != len(y):
                    # Trim to common length (can happen on first packet)
                    n = min(len(x), len(y))
                    x_plot, y_plot = x[-n:], y[-n:]
                else:
                    x_plot, y_plot = x, y

                line.set_data(x_plot, y_plot)

                if x_plot:
                    ax.set_xlim(x_plot[0], max(x_plot[-1], x_plot[0] + 1.0))
                if y_plot:
                    lo, hi = min(y_plot), max(y_plot)
                    margin = (hi - lo) * 0.12 if hi != lo else max(abs(hi) * 0.1, 0.5)
                    ax.set_ylim(lo - margin, hi + margin)

                updated.append(line)

        # Status bar
        age = ""
        if last_pkt_time[0] is not None:
            age = f"  |  last pkt {time.monotonic() - last_pkt_time[0]:.2f}s ago"
        status_text.set_text(
            f"pkts received: {pkt_count[0]}  |  "
            f"buf: {len(times)}/{N}  |  "
            f"port: {args.port}{age}"
        )

        return updated

    anim = FuncAnimation(
        fig, update,
        interval=50,       # 20 Hz refresh
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        stop_event.set()
        t.join(timeout=2.0)
        print("Exiting.")


if __name__ == "__main__":
    main()
