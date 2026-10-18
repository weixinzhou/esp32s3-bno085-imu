#!/usr/bin/env python3
"""Log the imu_v7_test firmware's CSV stream to a file and plot it live.

usage:
  python3 imu_logger.py                     auto-detect the board, log + plot
  python3 imu_logger.py -p /dev/cu.usbserial-XXXX
  python3 imu_logger.py --window 20         seconds of history on screen
  python3 imu_logger.py --no-plot           log only (Ctrl+C to stop)
  python3 imu_logger.py --reset             reset the board first, to log its bring-up too

The firmware prints one CSV line per sample and '#' lines for commentary
(column header, per-sensor rates). Anything else is its ESP_LOG output and is
echoed to the terminal, so bring-up errors stay visible.
"""
import argparse
import csv
import datetime
import os
import re
import sys
import threading
import time
from collections import deque

import serial
from serial.tools import list_ports

# Columns of one firmware data line, in order (see main/imu_stream.c).
FIELDS = [
    "t_ms",
    "qw", "qx", "qy", "qz",
    "yaw_deg", "pitch_deg", "roll_deg",
    "ax_mps2", "ay_mps2", "az_mps2",
    "gx_rps", "gy_rps", "gz_rps",
    "mx_uT", "my_uT", "mz_uT",
    "rv_status", "mag_status",
    "gqw", "gqx", "gqy", "gqz",               # game rotation vector (no magnetometer)
    "lx_mps2", "ly_mps2", "lz_mps2",          # linear acceleration, gravity removed
    "rch0_mV", "rch1_mV", "rch2_mV", "rch3_mV", "rch4_mV", "rch5_mV",   # resistance channels
    "vsup_mV",                                 # same divider with nothing connected: the supply
]
COL = {name: i for i, name in enumerate(FIELDS)}

ESPRESSIF_VID = 0x303A                       # native USB (XIAO ESP32-S3)
USB_UART_VIDS = {0x0403, 0x10C4, 0x1A86}     # FTDI, CP210x, CH34x bridges

ANSI = re.compile(r"\x1b\[[0-9;]*m")
ACCURACY = ["unreliable", "low", "medium", "high"]


def find_port():
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == ESPRESSIF_VID:
            return p.device
    for p in ports:
        if p.vid in USB_UART_VIDS:
            return p.device
    return None


def open_port(port, baud, reset):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.2
    # ESP32 boards' auto-reset circuit (and the S3's native USB, which mimics
    # it) resets the chip while RTS is asserted and DTR is not. The OS asserts
    # both on open and pyserial then applies DTR before RTS, so asking for
    # both deasserted passes through exactly that state. Both asserted is the
    # idle state, so opening the port leaves a running board alone.
    s.dtr = True
    s.rts = True
    s.open()
    if reset:
        s.dtr = False                        # RTS only: chip held in reset
        time.sleep(0.1)
        s.rts = False                        # both off: released, normal boot
    return s


class Reader(threading.Thread):
    """Reads lines off the port, writes data rows to CSV, keeps a plot buffer."""

    def __init__(self, ser, csv_path, history):
        super().__init__(daemon=True)
        self.ser = ser
        self.csv_path = csv_path
        self.buf = deque(maxlen=history)     # rows of floats, FIELDS order
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.count = 0
        self.rates = ""                      # latest '# rate Hz: ...' line
        self.t0 = time.time()

    def run(self):
        with open(self.csv_path, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["host_s"] + FIELDS)
            pending = b""
            last_flush = time.time()
            while not self.stop_evt.is_set():
                try:
                    # Not read(4096): that blocks for the full timeout and delivers
                    # samples in 0.2 s bursts. Take whatever has arrived.
                    chunk = self.ser.read(self.ser.in_waiting or 1)
                except serial.SerialException as e:
                    print(f"serial error: {e}", file=sys.stderr)
                    break
                pending += chunk
                *lines, pending = pending.split(b"\n")
                for raw in lines:
                    self._handle(raw.decode("utf-8", "replace"), writer)
                if time.time() - last_flush > 1.0:
                    fh.flush()
                    last_flush = time.time()

    def _handle(self, line, writer):
        line = ANSI.sub("", line).strip()
        if not line:
            return
        if line.startswith("#"):
            if line.startswith("# rate"):
                self.rates = line[2:]
                print(line)
            return
        parts = line.split(",")
        try:
            if len(parts) != len(FIELDS):
                raise ValueError
            row = [float(x) for x in parts]
        except ValueError:
            print(line)                      # firmware log output
            return
        writer.writerow([f"{time.time() - self.t0:.3f}"] + parts)
        with self.lock:
            # A step back in device time means the board rebooted: start the
            # plot afresh instead of drawing a line back across the window.
            if self.buf and row[0] < self.buf[-1][0] - 1000:
                self.buf.clear()
                print("(device restarted)")
            self.buf.append(row)
            self.count += 1

    def snapshot(self):
        with self.lock:
            return list(self.buf), self.count


# --------------------------------------------------------------------------
# Live plot

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
# Categorical slots 1-3 of the reference palette, validated all-pairs for
# CVD; the same axis keeps the same color in every panel.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]

# (y label, CSV columns, series names, live-value format)
PANELS = [
    ("Orientation [deg]", ["yaw_deg", "pitch_deg", "roll_deg"], ["yaw", "pitch", "roll"], "{:+8.1f}"),
    ("Acceleration [m/s²]", ["ax_mps2", "ay_mps2", "az_mps2"], ["x", "y", "z"], "{:+8.2f}"),
    ("Angular rate [rad/s]", ["gx_rps", "gy_rps", "gz_rps"], ["x", "y", "z"], "{:+8.3f}"),
    ("Magnetic field [µT]", ["mx_uT", "my_uT", "mz_uT"], ["x", "y", "z"], "{:+8.1f}"),
]


class LivePlot:
    def __init__(self, reader, window_s):
        import matplotlib.pyplot as plt
        from matplotlib.layout_engine import ConstrainedLayoutEngine

        self.reader = reader
        self.window_s = window_s
        plt.rcParams.update({
            "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
            "axes.edgecolor": GRID, "axes.labelcolor": TEXT_2,
            "xtick.color": TEXT_2, "ytick.color": TEXT_2,
            "text.color": TEXT, "font.size": 9,
        })
        self.fig, axes = plt.subplots(len(PANELS), 1, sharex=True, figsize=(11, 9))
        # Leave a strip at the bottom for the footer line.
        self.fig.set_layout_engine(ConstrainedLayoutEngine(rect=(0, 0.03, 1, 0.97)))
        self.panels = []
        for ax, (ylabel, cols, names, fmt) in zip(axes, PANELS):
            lines = [ax.plot([], [], color=c, lw=1.5, label=n)[0]
                     for c, n in zip(SERIES, names)]
            ax.set_ylabel(ylabel)
            ax.grid(True, color=GRID, lw=0.8)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            # The legend doubles as the direct label: it names each series in
            # text ink and carries its live value, so identity never rests on
            # color alone. It sits outside the plot so it never covers data,
            # in a monospace face so changing digits do not shift the layout.
            legend = ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False,
                               labelcolor=TEXT, prop={"family": "monospace", "size": 8})
            self.panels.append((ax, [COL[c] for c in cols], names, fmt, lines, legend))
        axes[-1].set_xlabel("device time [s]")
        self.title = self.fig.suptitle("waiting for data ...", color=TEXT, fontsize=10)
        self.footer = self.fig.text(0.01, 0.008, "", color=TEXT_2, fontsize=8)

    def update(self, _frame=None):
        rows, count = self.reader.snapshot()
        if not rows:
            return
        t_end = rows[-1][0] / 1000.0
        rows = [r for r in rows if r[0] / 1000.0 >= t_end - self.window_s]
        t = [r[0] / 1000.0 for r in rows]
        last = rows[-1]
        for ax, idx, names, fmt, lines, legend in self.panels:
            for i, name, line, text in zip(idx, names, lines, legend.get_texts()):
                line.set_data(t, [r[i] for r in rows])
                text.set_text(f"{name:<5}" + fmt.format(last[i]))
            ax.relim()
            ax.autoscale_view(scalex=False)
            ax.set_xlim(t_end - self.window_s, t_end)
        rv, mag = int(last[COL["rv_status"]]), int(last[COL["mag_status"]])
        self.title.set_text(
            f"{count} samples   ·   orientation accuracy: {ACCURACY[rv & 3]}"
            f"   ·   magnetometer calibration: {ACCURACY[mag & 3]}")
        self.footer.set_text(f"{self.reader.rates}   →  {self.reader.csv_path}")

    def show(self):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        self._anim = FuncAnimation(self.fig, self.update, interval=50,
                                   cache_frame_data=False)
        plt.show()


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="serial port (default: auto-detect)")
    ap.add_argument("-b", "--baud", type=int, default=921600,
                    help="UART console baud (ignored on native USB)")
    ap.add_argument("-o", "--out", help="CSV path (default: logs/imu_<time>.csv next to this script)")
    ap.add_argument("-w", "--window", type=float, default=10.0, help="seconds shown in the plot")
    ap.add_argument("--no-plot", action="store_true", help="log only")
    ap.add_argument("--duration", type=float, help="stop after this many seconds")
    ap.add_argument("--reset", action="store_true", help="reset the board when connecting")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("no ESP32 serial port found; pass one with -p")

    out = args.out
    if not out:
        logs = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(logs, exist_ok=True)
        out = os.path.join(logs, datetime.datetime.now().strftime("imu_%Y%m%d_%H%M%S.csv"))

    ser = open_port(port, args.baud, args.reset)
    # Keep enough rows for the window even at the firmware's full 100 Hz.
    reader = Reader(ser, out, history=int(args.window * 200) + 100)
    reader.start()
    print(f"logging {port} -> {out}")

    try:
        if args.no_plot:
            end = time.time() + args.duration if args.duration else None
            while reader.is_alive() and (end is None or time.time() < end):
                time.sleep(0.2)
        else:
            plot = LivePlot(reader, args.window)
            if args.duration:
                import matplotlib.pyplot as plt
                timer = plot.fig.canvas.new_timer(interval=int(args.duration * 1000))
                timer.add_callback(plt.close, plot.fig)
                timer.start()
            plot.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop_evt.set()
        reader.join(timeout=2)
        ser.close()
        print(f"saved {reader.count} samples to {out}")


if __name__ == "__main__":
    main()
