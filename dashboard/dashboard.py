#!/usr/bin/env python3
"""
ZephyrAir Telemetry Dashboard (v2 - Robust)
Real-time visualization of barometric pressure, temperature, and altitude.
"""

import sys
import time
import re
import csv
import argparse
import threading
import queue
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Regex to match: P:<val>,T:<val>,A:<val>
TELEMETRY_REGEX = re.compile(r"P:([-\d.]+),T:([-\d.]+),A:([-\d.]+)")

MAX_DISPLAY_POINTS = 100  # Show last 100 samples (~10 seconds at 10 Hz)


class TelemetryDashboard:
    def __init__(self, data_source, log_filename=None):
        self.source = data_source
        self.data_queue = queue.Queue()
        self.running = True

        self.pressures = deque(maxlen=MAX_DISPLAY_POINTS)
        self.temps = deque(maxlen=MAX_DISPLAY_POINTS)
        self.altitudes = deque(maxlen=MAX_DISPLAY_POINTS)

        # CSV Logging
        if log_filename is None:
            log_filename = f"telemetry_{int(time.time())}.csv"
        self.csv_file = open(log_filename, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp_s", "pressure_pa", "temp_c", "altitude_m"])
        print(f"[*] Logging to: {log_filename}")

        # Set up Figure and Subplots
        self.fig, (self.ax_p, self.ax_t, self.ax_a) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        self.fig.canvas.manager.set_window_title("ZephyrAir — Live Telemetry Dashboard")
        self.fig.suptitle("ZephyrAir: nRF5340 / BMP390 Real-Time Telemetry", fontsize=14, fontweight="bold")

        # Initial Plot Layout Setup (labels always visible!)
        self._init_axes()

        # Start a dedicated background thread to read from stdin/serial
        self.reader_thread = threading.Thread(target=self._reader_worker, daemon=True)
        self.reader_thread.start()

    def _init_axes(self):
        self.ax_p.set_ylabel("Pressure (Pa)", fontweight="bold", color="#007acc")
        self.ax_p.grid(True, linestyle="--", alpha=0.5)

        self.ax_t.set_ylabel("Temp (°C)", fontweight="bold", color="#e056fd")
        self.ax_t.grid(True, linestyle="--", alpha=0.5)

        self.ax_a.set_ylabel("Altitude (m)", fontweight="bold", color="#2ed573")
        self.ax_a.set_xlabel("Sample History", fontweight="bold")
        self.ax_a.grid(True, linestyle="--", alpha=0.5)

    def _reader_worker(self):
        """Continuously reads lines in the background and pushes into a queue."""
        while self.running:
            try:
                line = self.source.readline()
                if not line:
                    time.sleep(0.01)
                    continue

                # Handle both bytes (Serial) and string (stdin)
                if isinstance(line, bytes):
                    line = line.decode("utf-8", errors="ignore")

                line = line.strip()
                if line:
                    self.data_queue.put(line)
            except Exception as e:
                print(f"[Reader Error] {e}")
                break

    def update(self, frame):
        """Pulls all newly arrived data from the queue and updates the plot."""
        updated = False

        # Drain everything in the queue so far
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

                # Write to CSV
                self.csv_writer.writerow([now, p_val, t_val, a_val])
                updated = True

        if not updated or len(self.pressures) == 0:
            return

        self.csv_file.flush()

        # Re-plot Pressure
        self.ax_p.clear()
        self.ax_p.plot(self.pressures, color="#007acc", linewidth=2)
        self.ax_p.set_ylabel("Pressure (Pa)", fontweight="bold", color="#007acc")
        self.ax_p.set_title(f"Pressure: {self.pressures[-1]:.2f} Pa", loc="right", fontsize=10, color="#007acc")
        self.ax_p.grid(True, linestyle="--", alpha=0.5)

        # Re-plot Temperature
        self.ax_t.clear()
        self.ax_t.plot(self.temps, color="#e056fd", linewidth=2)
        self.ax_t.set_ylabel("Temp (°C)", fontweight="bold", color="#e056fd")
        self.ax_t.set_title(f"Temp: {self.temps[-1]:.2f} °C", loc="right", fontsize=10, color="#e056fd")
        self.ax_t.grid(True, linestyle="--", alpha=0.5)

        # Re-plot Altitude
        self.ax_a.clear()
        self.ax_a.plot(self.altitudes, color="#2ed573", linewidth=2)
        self.ax_a.set_ylabel("Altitude (m)", fontweight="bold", color="#2ed573")
        self.ax_a.set_xlabel("Sample History", fontweight="bold")
        self.ax_a.set_title(f"Altitude: {self.altitudes[-1]:.2f} m", loc="right", fontsize=10, color="#2ed573")
        self.ax_a.grid(True, linestyle="--", alpha=0.5)

        plt.tight_layout()

    def run(self):
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.show()
        self.running = False
        self.csv_file.close()


def main():
    parser = argparse.ArgumentParser(description="ZephyrAir Real-Time Telemetry Dashboard")
    parser.add_argument("--port", type=str, help="Serial port (e.g., /dev/pts/2 or COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--stdin", action="store_true", help="Read directly from stdin pipe")

    args = parser.parse_args()

    if args.stdin:
        print("[*] Reading telemetry stream from stdin pipe...")
        source = sys.stdin
    elif args.port:
        import serial
        print(f"[*] Opening serial port: {args.port} at {args.baud} baud...")
        source = serial.Serial(args.port, args.baud, timeout=1)
    else:
        print("Error: Specify either --stdin or --port <port_name>")
        sys.exit(1)

    app = TelemetryDashboard(source)
    app.run()


if __name__ == "__main__":
    main()