#!/usr/bin/env python3
"""
Real-time telemetry viewer and CSV session analyzer for ZephyrAir.
"""

import sys
import os
import signal
import time
import re
import csv
import argparse
import threading
import queue
from collections import deque
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

TELEMETRY_REGEX = re.compile(r"P:([-\d.]+),T:([-\d.]+),A:([-\d.]+)")
MAX_DISPLAY_POINTS = 100


class TelemetryDashboard:
    def __init__(self, data_source, alert_pa=101400.0, log_filename=None, is_serial=False):
        self.source = data_source
        self.is_serial = is_serial
        self.alert_pa = alert_pa
        self.data_queue = queue.Queue()
        self.running = True
        self.streaming_paused = False

        self.pressures = deque(maxlen=MAX_DISPLAY_POINTS)
        self.temps = deque(maxlen=MAX_DISPLAY_POINTS)
        self.altitudes = deque(maxlen=MAX_DISPLAY_POINTS)

        if log_filename is None:
            log_filename = f"telemetry_{int(time.time())}.csv"
        self.log_filename = log_filename
        self.csv_file = open(log_filename, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp_s", "pressure_pa", "temp_c", "altitude_m"])
        print(f"Logging telemetry to {log_filename}")

        self.fig, (self.ax_p, self.ax_t, self.ax_a) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        self.fig.canvas.manager.set_window_title("ZephyrAir Telemetry Dashboard")
        self.fig.suptitle("ZephyrAir: BMP390 Real-Time Telemetry", fontsize=13, fontweight="bold")

        self._init_axes()

        self.reader_thread = threading.Thread(target=self._reader_worker, daemon=True)
        self.reader_thread.start()

    def _init_axes(self):
        self.ax_p.set_ylabel("Pressure (Pa)", fontweight="bold", color="#007acc")
        self.ax_p.grid(True, linestyle="--", alpha=0.5)

        self.ax_t.set_ylabel("Temp (°C)", fontweight="bold", color="#e056fd")
        self.ax_t.grid(True, linestyle="--", alpha=0.5)

        self.ax_a.set_ylabel("Altitude (m)", fontweight="bold", color="#2ed573")
        self.ax_a.set_xlabel("Sample Window", fontweight="bold")
        self.ax_a.grid(True, linestyle="--", alpha=0.5)

    def _reader_worker(self):
        while self.running:
            try:
                line = self.source.readline()
                if not line:
                    time.sleep(0.01)
                    continue

                if isinstance(line, bytes):
                    line = line.decode("utf-8", errors="ignore")

                line = line.strip()
                if line:
                    self.data_queue.put(line)
            except Exception:
                break

    def send_shell_command(self, cmd):
        if self.is_serial and hasattr(self.source, "write"):
            try:
                self.source.write(f"{cmd}\r\n".encode("utf-8"))
                print(f"Sent shell command: {cmd}")
            except Exception as exc:
                print(f"Failed to send command: {exc}")

    def _on_key_press(self, event):
        if event.key == "s":
            out = "telemetry_snapshot.png"
            self.fig.savefig(out, dpi=150)
            print(f"Saved plot snapshot to {out}")
        elif event.key == "p":
            cmd = "sensor start" if self.streaming_paused else "sensor stop"
            self.streaming_paused = not self.streaming_paused
            self.send_shell_command(cmd)
        elif event.key == "1":
            self.send_shell_command("sensor odr 2")
        elif event.key == "2":
            self.send_shell_command("sensor odr 10")
        elif event.key == "3":
            self.send_shell_command("sensor odr 25")
        elif event.key == "f":
            self.send_shell_command("sensor fault 2")

    def update(self, _frame):
        updated = False

        while not self.data_queue.empty():
            line = self.data_queue.get_nowait()
            match = TELEMETRY_REGEX.search(line)
            if match:
                p_val = float(match.group(1))
                t_val = float(match.group(2))
                a_val = float(match.group(3))
                now = time.time()

                self.pressures.append(p_val)
                self.temps.append(t_val)
                self.altitudes.append(a_val)

                self.csv_writer.writerow([now, p_val, t_val, a_val])
                updated = True

        if not updated or len(self.pressures) == 0:
            return

        self.csv_file.flush()

        latest_p = self.pressures[-1]
        is_alert = latest_p >= self.alert_pa
        p_color = "#ff4757" if is_alert else "#007acc"
        p_status = " [HIGH PRESSURE ALERT]" if is_alert else ""

        self.ax_p.clear()
        self.ax_p.plot(self.pressures, color=p_color, linewidth=2)
        self.ax_p.axhline(self.alert_pa, color="#ff4757", linestyle=":", alpha=0.8, label=f"Alert ({self.alert_pa:.0f} Pa)")
        self.ax_p.set_ylabel("Pressure (Pa)", fontweight="bold", color=p_color)
        self.ax_p.set_title(f"Pressure: {latest_p:.2f} Pa{p_status}", loc="right", fontsize=10, color=p_color)
        self.ax_p.legend(loc="upper left", fontsize=8)
        self.ax_p.grid(True, linestyle="--", alpha=0.5)

        self.ax_t.clear()
        self.ax_t.plot(self.temps, color="#e056fd", linewidth=2)
        self.ax_t.set_ylabel("Temp (°C)", fontweight="bold", color="#e056fd")
        self.ax_t.set_title(f"Temp: {self.temps[-1]:.2f} °C", loc="right", fontsize=10, color="#e056fd")
        self.ax_t.grid(True, linestyle="--", alpha=0.5)

        self.ax_a.clear()
        self.ax_a.plot(self.altitudes, color="#2ed573", linewidth=2)
        self.ax_a.set_ylabel("Altitude (m)", fontweight="bold", color="#2ed573")
        self.ax_a.set_xlabel("Sample Window (Press 's' to save PNG)", fontweight="bold")
        self.ax_a.set_title(f"Altitude: {self.altitudes[-1]:.2f} m", loc="right", fontsize=10, color="#2ed573")
        self.ax_a.grid(True, linestyle="--", alpha=0.5)

        plt.tight_layout()

    def shutdown(self, *_args):
        if self.running:
            self.running = False
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
            print(f"\nDashboard closed. Session saved to {self.log_filename}")
        os._exit(0)

    def run(self):
        signal.signal(signal.SIGINT, self.shutdown)
        self.fig.canvas.mpl_connect("close_event", self.shutdown)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.show()
        self.shutdown()


def replay_csv(filepath, alert_pa):
    if not os.path.exists(filepath):
        print(f"Error: File '{filepath}' not found.")
        sys.exit(1)

    df = pd.read_csv(filepath)
    if df.empty:
        print(f"Error: File '{filepath}' contains no telemetry rows.")
        sys.exit(1)

    df["elapsed_s"] = df["timestamp_s"] - df["timestamp_s"].iloc[0]

    print(f"\n=== Session Summary: {filepath} ({len(df)} samples) ===")
    summary = df[["pressure_pa", "temp_c", "altitude_m"]].agg(["min", "max", "mean", "std"])
    print(summary.to_string(float_format=lambda x: f"{x:.2f}"))
    print("========================================================\n")

    fig, (ax_p, ax_t, ax_a) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title(f"ZephyrAir Replay — {os.path.basename(filepath)}")
    fig.suptitle(f"ZephyrAir Session Replay: {os.path.basename(filepath)}", fontsize=13, fontweight="bold")

    ax_p.plot(df["elapsed_s"], df["pressure_pa"], color="#007acc", linewidth=2, label="Pressure")
    ax_p.axhline(alert_pa, color="#ff4757", linestyle=":", alpha=0.8, label=f"Alert ({alert_pa:.0f} Pa)")
    alerts = df[df["pressure_pa"] >= alert_pa]
    if not alerts.empty:
        ax_p.scatter(alerts["elapsed_s"], alerts["pressure_pa"], color="#ff4757", zorder=5, s=20, label="Alert Exceeded")
    ax_p.set_ylabel("Pressure (Pa)", fontweight="bold", color="#007acc")
    ax_p.legend(loc="upper right", fontsize=8)
    ax_p.grid(True, linestyle="--", alpha=0.5)

    ax_t.plot(df["elapsed_s"], df["temp_c"], color="#e056fd", linewidth=2)
    ax_t.set_ylabel("Temp (°C)", fontweight="bold", color="#e056fd")
    ax_t.grid(True, linestyle="--", alpha=0.5)

    ax_a.plot(df["elapsed_s"], df["altitude_m"], color="#2ed573", linewidth=2)
    ax_a.set_ylabel("Altitude (m)", fontweight="bold", color="#2ed573")
    ax_a.set_xlabel("Elapsed Time (s)", fontweight="bold")
    ax_a.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="ZephyrAir Telemetry Dashboard")
    parser.add_argument("--port", type=str, help="Serial port path (e.g., /dev/pts/2 or COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--stdin", action="store_true", help="Read telemetry stream from stdin")
    parser.add_argument("--replay", type=str, metavar="CSV_FILE", help="Replay and analyze a recorded CSV session")
    parser.add_argument("--alert-pa", type=float, default=101400.0, help="High pressure alert threshold in Pa")

    args = parser.parse_args()

    if args.replay:
        replay_csv(args.replay, args.alert_pa)
        return

    if args.stdin:
        print("Reading telemetry stream from stdin...")
        source = sys.stdin
        is_serial = False
    elif args.port:
        import serial
        print(f"Opening serial port {args.port} at {args.baud} baud...")
        source = serial.Serial(args.port, args.baud, timeout=1)
        is_serial = True
    else:
        parser.print_help()
        sys.exit(1)

    app = TelemetryDashboard(source, alert_pa=args.alert_pa, is_serial=is_serial)
    app.run()


if __name__ == "__main__":
    main()