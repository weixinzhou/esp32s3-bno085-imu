#!/usr/bin/env python3
"""Live 3D orientation dashboard for the imu_v7_test firmware.

Takes the board's stream over Bluetooth LE (default) or the serial port and
pushes it to a browser page (web/index.html) over Server-Sent Events. BLE
needs bleak, serial needs pyserial; the page draws with three.js.

usage:
  python3 imu_dashboard.py                  find "IMU-BNO085" over BLE, open the browser
  python3 imu_dashboard.py --serial         use the USB serial port instead (auto-detect)
  python3 imu_dashboard.py -p /dev/cu.usbserial-XXXX   a specific serial port
  python3 imu_dashboard.py --replay latest  play back the newest recording
  python3 imu_dashboard.py --record         start recording to CSV right away

Whatever the source, a dropped link is retried until it comes back, so a
demo survives a bumped cable or the board going out of radio range.
On macOS the terminal app needs Bluetooth permission the first time
(System Settings > Privacy & Security > Bluetooth).
"""
import argparse
import asyncio
import csv
import datetime
import json
import math
import os
import queue
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

from imu_logger import ANSI, FIELDS, find_port, open_port

# BLE service of main/ble_link.c.
BLE_NAME = "IMU-BNO085"
SVC_UUID = "7a4c0001-5b2e-4f6d-9c3a-2e1d8b6f0a51"
DATA_UUID = "7a4c0002-5b2e-4f6d-9c3a-2e1d8b6f0a51"
LOG_UUID = "7a4c0003-5b2e-4f6d-9c3a-2e1d8b6f0a51"
# ble_sample_t: t_ms, rotation vector (Q14), accel (Q8), gyro (Q9), mag (Q4),
# two status bytes, game rotation vector (Q14), linear accel (Q8), then the
# resistance front end in ADS1115 counts (0.125 mV): RCH0-5 and the supply.
# 60 bytes; firmware from before the front end sent the first 46.
SAMPLE = struct.Struct("<I4h3h3h3hBB4h3h7h")
SAMPLE_OLD = struct.Struct("<I4h3h3h3hBB4h3h")

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(HERE, "web")
LOGS = os.path.join(HERE, "logs")
CONTENT_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript",
                 ".css": "text/css", ".svg": "image/svg+xml", ".png": "image/png"}


class Hub:
    """Fans lines out to every connected browser and, while recording, to CSV."""

    def __init__(self, port):
        self.port = port
        self.connected = False
        self.clients = set()
        self.lock = threading.Lock()
        self.csv_fh = None
        self.csv_writer = None
        self.csv_path = None
        self.rec_started = None
        self.rec_rows = 0
        self.t0 = time.time()

    # -- browsers -----------------------------------------------------------
    def subscribe(self):
        q = queue.Queue(maxsize=400)          # ~8 s at 50 Hz; a stalled tab just drops
        with self.lock:
            self.clients.add(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.discard(q)

    def publish(self, event, data):
        msg = f"event: {event}\ndata: {data}\n\n".encode()
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass

    def state(self):
        with self.lock:
            return {
                "port": self.port,
                "connected": self.connected,
                "recording": self.csv_writer is not None,
                "csv_path": self.csv_path,
                "rec_seconds": time.time() - self.rec_started if self.rec_started else 0,
                "rec_rows": self.rec_rows,
            }

    def set_connected(self, connected):
        self.connected = connected
        self.publish("state", json.dumps(self.state()))

    # -- recording ----------------------------------------------------------
    def start_recording(self):
        with self.lock:
            if self.csv_writer:
                return
            os.makedirs(LOGS, exist_ok=True)
            self.csv_path = os.path.join(
                LOGS, datetime.datetime.now().strftime("imu_%Y%m%d_%H%M%S.csv"))
            self.csv_fh = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_fh)
            self.csv_writer.writerow(["host_s"] + FIELDS)
            self.rec_started = time.time()
            self.rec_rows = 0
        print(f"recording -> {self.csv_path}")

    def stop_recording(self):
        with self.lock:
            if not self.csv_writer:
                return
            self.csv_fh.close()
            self.csv_fh = self.csv_writer = None
            self.rec_started = None
            print(f"saved {self.rec_rows} rows to {self.csv_path}")

    def record(self, parts):
        with self.lock:
            if self.csv_writer:
                self.csv_writer.writerow([f"{time.time() - self.t0:.3f}"] + parts)
                self.rec_rows += 1


def handle_line(hub, line):
    line = ANSI.sub("", line).strip()
    if not line:
        return
    if line.startswith("#"):
        if line.startswith("# rate"):
            hub.publish("rate", json.dumps(line[2:]))
        return
    parts = line.split(",")
    # 33 columns from current firmware; 26 and 19 from earlier builds, which
    # still play back.
    if len(parts) in (len(FIELDS), 26, 19) and parts[0].isdigit():
        hub.publish("s", line)
        if len(parts) == len(FIELDS):
            hub.record(parts)
        return
    print(line)                                  # firmware log output
    hub.publish("log", json.dumps(line))


def serial_loop(hub, baud, stop):
    """Read forever; on any serial error wait for the port to come back."""
    while not stop.is_set():
        try:
            ser = open_port(hub.port, baud, reset=False)
        except (serial.SerialException, OSError):
            time.sleep(1.0)
            continue
        print(f"connected to {hub.port}")
        hub.set_connected(True)
        pending = b""
        try:
            while not stop.is_set():
                # read() waits until it has every byte asked for or times out, so
                # read(4096) sat out the 0.2 s timeout and handed over ten samples
                # at a time: the page moved in 5 Hz jumps. Take what is there.
                pending += ser.read(ser.in_waiting or 1)
                *lines, pending = pending.split(b"\n")
                for raw in lines:
                    handle_line(hub, raw.decode("utf-8", "replace"))
        except (serial.SerialException, OSError) as e:
            print(f"serial lost ({e}); waiting for {hub.port} to come back ...")
        finally:
            try:
                ser.close()
            except Exception:
                pass
            hub.set_connected(False)


def resolve_replay(arg):
    """Turn --replay's argument into an existing file, or exit listing the options."""
    recordings = sorted(
        (os.path.join(LOGS, f) for f in os.listdir(LOGS) if f.endswith(".csv")),
        key=os.path.getmtime) if os.path.isdir(LOGS) else []
    if arg == "latest":
        if recordings:
            return recordings[-1]
    elif os.path.isfile(arg):
        return arg
    elif os.path.isfile(os.path.join(LOGS, arg)):          # bare file name from logs/
        return os.path.join(LOGS, arg)
    msg = [f"replay file not found: {arg}"]
    if recordings:
        msg.append(f"recordings in {LOGS} (newest last; '--replay latest' picks the newest):")
        msg += [f"  {os.path.basename(r)}" for r in recordings]
    else:
        msg.append(f"no recordings in {LOGS} yet; press R in the dashboard to make one")
    sys.exit("\n".join(msg))


def sample_to_line(data):
    """Turn one BLE sample into the same line the serial port prints."""
    full = len(data) >= SAMPLE.size
    v = (SAMPLE if full else SAMPLE_OLD).unpack(data[:SAMPLE.size if full else SAMPLE_OLD.size])
    q = [c / 16384 for c in v[1:5]]
    acc = [c / 256 for c in v[5:8]]
    gyr = [c / 512 for c in v[8:11]]
    mag = [c / 16 for c in v[11:14]]
    gq = [c / 16384 for c in v[16:20]]
    lin = [c / 256 for c in v[20:23]]
    r, i, j, k = q                                  # same Euler maths as the firmware
    yaw = math.degrees(math.atan2(2 * (r * k + i * j), 1 - 2 * (j * j + k * k)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (r * j - k * i)))))
    roll = math.degrees(math.atan2(2 * (r * i + j * k), 1 - 2 * (i * i + j * j)))
    return ",".join([str(v[0]), *(f"{c:.4f}" for c in q), f"{yaw:.2f}", f"{pitch:.2f}", f"{roll:.2f}",
                     *(f"{c:.3f}" for c in acc), *(f"{c:.4f}" for c in gyr), *(f"{c:.2f}" for c in mag),
                     str(v[14]), str(v[15]), *(f"{c:.4f}" for c in gq), *(f"{c:.3f}" for c in lin),
                     *(f"{c * 0.125:.1f}" for c in v[23:30])])


async def ble_loop(hub, stop):
    """Find the board by its service UUID (or name), stream, and reconnect forever."""
    from bleak import BleakClient, BleakScanner

    short_warned = False

    def on_data(_, data):
        nonlocal short_warned
        if len(data) >= SAMPLE_OLD.size:
            handle_line(hub, sample_to_line(bytes(data)))
        elif not short_warned:
            short_warned = True
            print(f"BLE packets are {len(data)} bytes, need {SAMPLE.size}: the link MTU is too small")

    while not stop.is_set():
        print(f"searching for {BLE_NAME} over Bluetooth ...")
        dev = await BleakScanner.find_device_by_filter(
            lambda d, ad: SVC_UUID in [u.lower() for u in ad.service_uuids] or d.name == BLE_NAME,
            timeout=5.0)
        if dev is None:
            continue
        lost = asyncio.Event()
        try:
            async with BleakClient(dev, disconnected_callback=lambda _: lost.set()) as client:
                hub.port = f"BLE {dev.name or BLE_NAME}"
                print(f"connected to {dev.name} ({dev.address}), MTU {client.mtu_size}")
                hub.set_connected(True)
                await client.start_notify(LOG_UUID, lambda _, d: handle_line(hub, bytes(d).decode("utf-8", "replace")))
                await client.start_notify(DATA_UUID, on_data)
                while not stop.is_set() and not lost.is_set():
                    await asyncio.sleep(0.25)
        except Exception as e:                         # bleak raises several types; all mean "retry"
            print(f"BLE link error: {e}")
        if hub.connected:
            print("BLE link lost; reconnecting ...")
        hub.set_connected(False)
        await asyncio.sleep(1.0)


def replay_loop(hub, path, stop):
    """Play a recorded CSV (host_s + device columns) back at its own pace, looped.

    A fallback for demos: if the board misbehaves on the day, the page still
    has real motion to show.
    """
    with open(path, newline="") as fh:
        rows = [r[1:] for r in csv.reader(fh)][1:]       # drop header and host_s
    if not rows:
        sys.exit(f"{path} has no data rows")
    hub.set_connected(True)
    print(f"replaying {len(rows)} rows from {path} (looping)")
    t_first, t_last = float(rows[0][0]), float(rows[-1][0])
    offset = 0.0
    while not stop.is_set():
        start_wall = time.time()
        for row in rows:
            if stop.is_set():
                return
            # Device timestamps set the pace. Each loop shifts them forward so
            # the page sees time keep running rather than a board reboot.
            t = float(row[0])
            delay = start_wall + (t - t_first) / 1000.0 - time.time()
            if delay > 0:
                time.sleep(delay)
            handle_line(hub, ",".join([str(int(t + offset))] + row[1:]))
        time.sleep(0.5)
        offset += t_last - t_first + 500


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):          # keep the terminal for the board's output
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj):
            self._send(200, json.dumps(obj).encode(), "application/json")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/events":
                return self._events()
            if path == "/state":
                return self._json(hub.state())
            if path == "/":
                path = "/index.html"
            # Only files inside web/, nothing above it.
            full = os.path.realpath(os.path.join(WEB, path.lstrip("/")))
            if not full.startswith(os.path.realpath(WEB) + os.sep) or not os.path.isfile(full):
                return self._send(404, b"not found", "text/plain")
            with open(full, "rb") as fh:
                body = fh.read()
            ctype = CONTENT_TYPES.get(os.path.splitext(full)[1], "application/octet-stream")
            self._send(200, body, ctype)

        def do_POST(self):
            if self.path.split("?", 1)[0] != "/record":
                return self._send(404, b"not found", "text/plain")
            length = int(self.headers.get("Content-Length") or 0)
            try:
                on = bool(json.loads(self.rfile.read(length) or b"{}").get("on"))
            except ValueError:
                return self._send(400, b"bad json", "text/plain")
            hub.start_recording() if on else hub.stop_recording()
            state = hub.state()
            hub.publish("state", json.dumps(state))
            self._json(state)

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            q = hub.subscribe()
            try:
                self.wfile.write(b"retry: 1000\n\n")
                self.wfile.write(f"event: state\ndata: {json.dumps(hub.state())}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        msg = q.get(timeout=10)
                    except queue.Empty:
                        msg = b": keepalive\n\n"
                    self.wfile.write(msg)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                hub.unsubscribe(q)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", action="store_true", help="read the USB serial port instead of BLE")
    ap.add_argument("-p", "--port", help="serial port to use (implies --serial)")
    ap.add_argument("-b", "--baud", type=int, default=921600,
                    help="UART console baud (ignored on native USB)")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--record", action="store_true", help="start recording to CSV immediately")
    ap.add_argument("--no-browser", action="store_true", help="do not open a browser tab")
    ap.add_argument("--replay", metavar="CSV",
                    help="play back a recorded CSV instead of a board: a path, a file name "
                         "in logs/, or 'latest' for the newest recording")
    args = ap.parse_args()

    stop = threading.Event()
    source = None
    if args.replay:
        path = resolve_replay(args.replay)
        hub = Hub("Replay " + os.path.basename(path))
        source = threading.Thread(target=replay_loop, args=(hub, path, stop), daemon=True)
    elif args.serial or args.port:
        port = args.port or find_port()
        if not port:
            sys.exit("no ESP32 serial port found; plug the board in, pass -p, or use BLE")
        hub = Hub(port)
        source = threading.Thread(target=serial_loop, args=(hub, args.baud, stop), daemon=True)
    else:
        hub = Hub(f"BLE {BLE_NAME}")
    if args.record:
        hub.start_recording()
    if source:
        source.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(hub))
    server.daemon_threads = True
    url = f"http://localhost:{args.http_port}/"
    print(f"dashboard: {url}   (Ctrl+C to quit)")
    if not args.no_browser:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        if source:
            server.serve_forever()
        else:
            # CoreBluetooth is happiest on the main thread, so BLE runs here
            # and the web server moves to a thread.
            threading.Thread(target=server.serve_forever, daemon=True).start()
            asyncio.run(ble_loop(hub, stop))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        hub.stop_recording()
        if not source:
            server.shutdown()          # serve_forever runs in a thread in BLE mode
        server.server_close()


if __name__ == "__main__":
    main()
