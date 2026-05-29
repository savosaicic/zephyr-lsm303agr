#!/usr/bin/env python3
"""
cal2d_gui -- control & visualize the LSM303AGR 2D fine-tune calibration.

Simplest possible stack: Python standard-library Tkinter for the UI (no
matplotlib/numpy) plus pyserial for UART. Talks to the Zephyr shell on the MCU.

It speaks two things over the serial link:
  * sends shell commands:  cal2d start | stop | apply | off | clear | show | status
  * parses two machine-readable telemetry lines the firmware emits (the "#T,"
    / "#R," payload is matched anywhere in a line, so any Zephyr log prefix is
    tolerated):
        #T,state,count,ax,ay,az,mx,my,mz,heading,applied
        #R,status,roll,pitch,yaw,hix,hiy,hiz,arc,radius,plane_res,circ_res,axis_vert

What you see:
  * a compass dial with the live tilt-compensated heading (base vs fine-tuned),
  * a magnetometer XY plot showing the swept points of a calibration capture,
    with the fitted hard-iron center + field-radius circle overlaid,
  * the computed fine-tune parameters with quality warnings,
  * a raw serial console with a command entry box.

Usage:
    pip install -r requirements.txt
    python cal2d_gui.py [--port /dev/ttyACM0] [--baud 115200]
"""

import argparse
import math
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

BAUD_DEFAULT = 115200
# Strip ANSI/VT100 escape sequences the Zephyr shell wraps log output in
# (colors like \x1b[0m, cursor moves like \x1b[8D, erases like \x1b[J).
ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
T_RE = re.compile(r"#T,([^\r\n]+)")
R_RE = re.compile(r"#R,([^\r\n]+)")
MAX_LIVE_POINTS = 600
LOG_MAX_LINES = 500


def tilt_comp_heading(ax, ay, az, mx, my, mz):
    """Tilt-compensated heading (deg), matching the firmware's DT0058 formula."""
    roll = math.atan2(ay, az)
    denom = ay * math.sin(roll) + az * math.cos(roll)
    pitch = math.atan(-ax / denom) if denom else 0.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    mxh = mx * cp + my * (sr * sp) + mz * (cr * sp)
    myh = my * cr + mz * (-sr)
    hdg = math.degrees(math.atan2(-myh, mxh))
    return hdg + 360.0 if hdg < 0 else hdg


class SerialLink:
    """Background serial reader; line strings are pushed onto a queue."""

    def __init__(self, line_queue):
        self.q = line_queue
        self.ser = None
        self._lock = threading.Lock()
        self._reader = None
        self._stop = threading.Event()

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baud):
        self.close()
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def close(self):
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def send(self, text):
        with self._lock:
            if self.connected:
                self.ser.write((text + "\r\n").encode())

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as exc:  # device unplugged, etc.
                self.q.put(("error", f"serial read failed: {exc}"))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.q.put(("line", line.decode(errors="replace").rstrip("\r")))


BASE_COLOR = "#f0a35e"   # orange: base (uncorrected) heading
FT_COLOR = "#39d98a"     # green: 2D fine-tuned heading


class CompassDial(tk.Canvas):
    """Compass needle(s), 0 deg = North = up, clockwise. Shows the base
    (orange) heading always, and the fine-tuned (green) heading overlaid when
    a fine-tune is applied, so the two can be compared."""

    TEXT_BAND = 64   # reserved vertical space below the dial for readouts

    def __init__(self, master, size=300, **kw):
        super().__init__(master, width=size, height=size, bg="#101418",
                         highlightthickness=0, **kw)
        self.size = size
        self.base = None      # orange heading (deg)
        self.ft = None        # green heading (deg), or None when not applied

    def update_heading(self, base, ft):
        self.base = base
        self.ft = ft
        self.redraw()

    def _arrow(self, cx, cy, r, heading, color, width):
        a = math.radians(heading)
        nx = cx + (r - 16) * math.sin(a)
        ny = cy - (r - 16) * math.cos(a)
        bx = cx - (r * 0.35) * math.sin(a)
        by = cy + (r * 0.35) * math.cos(a)
        self.create_line(bx, by, nx, ny, fill=color, width=width,
                         arrow=tk.LAST, arrowshape=(14, 16, 6))

    def redraw(self):
        self.delete("all")
        s = self.size
        cx = s / 2
        cy = (s - self.TEXT_BAND) / 2   # dial centered above the text band
        r = min(cx, cy) - 18
        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#3a4450",
                         width=2)
        for ang, label in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            a = math.radians(ang)
            lx = cx + (r - 12) * math.sin(a)
            ly = cy - (r - 12) * math.cos(a)
            self.create_text(lx, ly, text=label, fill="#7f8c99",
                             font=("TkDefaultFont", 10, "bold"))
        for ang in range(0, 360, 30):
            a = math.radians(ang)
            x0 = cx + r * math.sin(a)
            y0 = cy - r * math.cos(a)
            x1 = cx + (r - 6) * math.sin(a)
            y1 = cy - (r - 6) * math.cos(a)
            self.create_line(x0, y0, x1, y1, fill="#2a323b")

        if self.base is None:
            self.create_text(cx, cy, text="--", fill="#56606b",
                             font=("TkDefaultFont", 16))
            return

        # base (orange) always; fine-tuned (green) on top when applied
        self.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#cfd6dd",
                         outline="")
        if self.ft is not None:
            self._arrow(cx, cy, r, self.base, BASE_COLOR, 3)
            self._arrow(cx, cy, r, self.ft, FT_COLOR, 4)
            delta = ((self.ft - self.base + 180.0) % 360.0) - 180.0
            self.create_text(cx, s - 46, text=f"base {self.base:6.1f}°",
                             fill=BASE_COLOR, font=("TkDefaultFont", 12, "bold"))
            self.create_text(cx, s - 16,
                             text=f"fine {self.ft:6.1f}°   (Δ {delta:+.1f}°)",
                             fill=FT_COLOR, font=("TkDefaultFont", 12, "bold"))
        else:
            self._arrow(cx, cy, r, self.base, BASE_COLOR, 4)
            self.create_text(cx, s - 26, text=f"{self.base:6.1f}°  (base)",
                             fill=BASE_COLOR, font=("TkDefaultFont", 13, "bold"))


class MagPlot(tk.Canvas):
    """XY scatter of magnetometer samples + fitted center/circle overlay."""

    def __init__(self, master, size=320, **kw):
        super().__init__(master, width=size, height=size, bg="#101418",
                         highlightthickness=0, **kw)
        self.size = size
        self.live = []          # rolling recent points (faint)
        self.sweep = []         # points captured during the active sweep (bold)
        self.cur = None         # current point
        self.center = None      # (hix, hiy) fitted hard-iron
        self.radius = None      # fitted field radius

    def add_point(self, mx, my, capturing):
        self.cur = (mx, my)
        self.live.append((mx, my))
        if len(self.live) > MAX_LIVE_POINTS:
            self.live.pop(0)
        if capturing:
            self.sweep.append((mx, my))

    def begin_sweep(self):
        self.sweep = []
        self.center = None
        self.radius = None

    def set_fit(self, cx, cy, radius):
        self.center = (cx, cy)
        self.radius = radius

    def _bounds(self):
        pts = list(self.live) + list(self.sweep)
        if self.center is not None and self.radius:
            cx, cy = self.center
            r = self.radius
            pts += [(cx - r, cy - r), (cx + r, cy + r)]
        if not pts:
            return None
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
        # square, equal aspect, with margin
        spanx, spany = maxx - minx, maxy - miny
        span = max(spanx, spany, 1e-6) * 1.15
        ccx, ccy = (minx + maxx) / 2, (miny + maxy) / 2
        return ccx - span / 2, ccx + span / 2, ccy - span / 2, ccy + span / 2

    def redraw(self):
        self.delete("all")
        s = self.size
        m = 20
        b = self._bounds()
        if b is None:
            self.create_text(s / 2, s / 2, text="(no data)", fill="#56606b")
            return
        minx, maxx, miny, maxy = b

        def to_px(x, y):
            px = m + (x - minx) / (maxx - minx) * (s - 2 * m)
            py = s - m - (y - miny) / (maxy - miny) * (s - 2 * m)  # y up
            return px, py

        # axes through origin if visible
        if minx <= 0 <= maxx:
            ox, _ = to_px(0, miny)
            self.create_line(ox, m, ox, s - m, fill="#222a32")
        if miny <= 0 <= maxy:
            _, oy = to_px(minx, 0)
            self.create_line(m, oy, s - m, oy, fill="#222a32")

        for (x, y) in self.live:
            px, py = to_px(x, y)
            self.create_oval(px - 1, py - 1, px + 1, py + 1, fill="#37414c",
                             outline="")
        for (x, y) in self.sweep:
            px, py = to_px(x, y)
            self.create_oval(px - 2, py - 2, px + 2, py + 2, fill="#5aa9e6",
                             outline="")

        if self.center is not None and self.radius:
            cx, cy = self.center
            cpx, cpy = to_px(cx, cy)
            rpx = self.radius / (maxx - minx) * (s - 2 * m)
            self.create_oval(cpx - rpx, cpy - rpx, cpx + rpx, cpy + rpx,
                             outline="#39d98a", width=2)
            self.create_line(cpx - 6, cpy, cpx + 6, cpy, fill="#39d98a", width=2)
            self.create_line(cpx, cpy - 6, cpx, cpy + 6, fill="#39d98a", width=2)

        if self.cur is not None:
            px, py = to_px(*self.cur)
            self.create_oval(px - 4, py - 4, px + 4, py + 4, fill="#f0a35e",
                             outline="#ffffff")
        self.create_text(s / 2, 12, text="mag X-Y (soft-iron) -- sweep + fit",
                         fill="#7f8c99", font=("TkDefaultFont", 9))


class App:
    def __init__(self, root, port, baud):
        self.root = root
        self.q = queue.Queue()
        self.link = SerialLink(self.q)
        self.prev_state = 0
        root.title("cal2d -- 2D calibration control & visualization")
        root.configure(bg="#161b20")

        self._build_ui()
        if port:
            self.port_var.set(port)
        self.baud_var.set(str(baud))
        self.root.after(50, self._pump)

    # ----- UI -----
    def _build_ui(self):
        top = tk.Frame(self.root, bg="#161b20")
        top.pack(fill="x", padx=8, pady=6)

        tk.Label(top, text="Port", bg="#161b20", fg="#cfd6dd").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=22,
                                       values=self._ports())
        self.port_combo.pack(side="left", padx=4)
        tk.Button(top, text="↻", command=self._refresh_ports).pack(
            side="left")
        tk.Label(top, text="Baud", bg="#161b20", fg="#cfd6dd").pack(
            side="left", padx=(8, 0))
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        tk.Entry(top, textvariable=self.baud_var, width=8).pack(side="left",
                                                                padx=4)
        self.connect_btn = tk.Button(top, text="Connect",
                                     command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=8)
        self.status_lbl = tk.Label(top, text="disconnected", bg="#161b20",
                                   fg="#f0a35e")
        self.status_lbl.pack(side="left", padx=8)

        # control buttons
        ctl = tk.Frame(self.root, bg="#161b20")
        ctl.pack(fill="x", padx=8)
        self.cmd_btns = []
        for label, cmd in (("Start", "cal2d start"), ("Stop", "cal2d stop"),
                           ("Apply", "cal2d apply"), ("Off", "cal2d off"),
                           ("Show", "cal2d show"), ("Clear", "cal2d clear"),
                           ("Status", "cal2d status")):
            b = tk.Button(ctl, text=label,
                          command=lambda c=cmd: self.link.send(c))
            b.pack(side="left", padx=3, pady=4)
            self.cmd_btns.append(b)

        # middle: compass + mag plot + result
        mid = tk.Frame(self.root, bg="#161b20")
        mid.pack(fill="both", expand=True, padx=8, pady=4)

        self.compass = CompassDial(mid, size=320)
        self.compass.pack(side="left", padx=6)
        self.compass.redraw()
        self.magplot = MagPlot(mid, size=320)
        self.magplot.pack(side="left", padx=6)
        self.magplot.redraw()

        res = tk.Frame(mid, bg="#1b2228")
        res.pack(side="left", fill="both", expand=True, padx=6)
        tk.Label(res, text="Fine-tune result", bg="#1b2228", fg="#cfd6dd",
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w", padx=8,
                                                          pady=(8, 2))
        self.result_text = tk.Text(res, height=12, width=40, bg="#11161b",
                                   fg="#cfd6dd", relief="flat", wrap="word")
        self.result_text.pack(fill="both", expand=True, padx=8, pady=6)
        self.result_text.insert("end", "(no result yet)")
        self.result_text.configure(state="disabled")

        # capture state line
        self.cap_lbl = tk.Label(self.root, text="capture: idle", bg="#161b20",
                                fg="#7f8c99")
        self.cap_lbl.pack(anchor="w", padx=12)

        # console + entry
        con = tk.Frame(self.root, bg="#161b20")
        con.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        self.console = tk.Text(con, height=10, bg="#0c0f12", fg="#9fb0bf",
                               relief="flat", wrap="none")
        self.console.pack(fill="both", expand=True)
        entry_row = tk.Frame(con, bg="#161b20")
        entry_row.pack(fill="x")
        self.entry = tk.Entry(entry_row)
        self.entry.pack(side="left", fill="x", expand=True, pady=4)
        self.entry.bind("<Return>", self._send_entry)
        tk.Button(entry_row, text="Send", command=self._send_entry).pack(
            side="left", padx=4)

        self._set_controls_enabled(False)

    def _ports(self):
        return [p.device for p in list_ports.comports()]

    def _refresh_ports(self):
        self.port_combo["values"] = self._ports()

    def _set_controls_enabled(self, on):
        state = "normal" if on else "disabled"
        for b in self.cmd_btns:
            b.configure(state=state)

    # ----- connection -----
    def _toggle_connect(self):
        if self.link.connected:
            self.link.close()
            self.status_lbl.configure(text="disconnected", fg="#f0a35e")
            self.connect_btn.configure(text="Connect")
            self._set_controls_enabled(False)
            return
        port = self.port_var.get().strip()
        if not port:
            self._log("** select a serial port first\n")
            return
        try:
            self.link.open(port, int(self.baud_var.get()))
        except Exception as exc:
            self._log(f"** open failed: {exc}\n")
            return
        self.status_lbl.configure(text=f"connected {port}", fg="#39d98a")
        self.connect_btn.configure(text="Disconnect")
        self._set_controls_enabled(True)

    def _send_entry(self, *_):
        txt = self.entry.get().strip()
        if txt:
            self.link.send(txt)
            self.entry.delete(0, "end")

    # ----- telemetry pump -----
    def _pump(self):
        dirty_plot = False
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "error":
                    self._log(f"** {payload}\n")
                    self.link.close()
                    self.status_lbl.configure(text="disconnected", fg="#f0a35e")
                    self.connect_btn.configure(text="Connect")
                    self._set_controls_enabled(False)
                    continue
                line = ANSI_RE.sub("", payload).rstrip()
                if not line:
                    continue
                mt = T_RE.search(line)
                mr = R_RE.search(line)
                if mt:
                    dirty_plot |= self._handle_t(mt.group(1))
                elif mr:
                    self._handle_r(mr.group(1))
                    self._log(line + "\n")
                else:
                    self._log(line + "\n")
        except queue.Empty:
            pass
        if dirty_plot:
            self.compass.redraw()
            self.magplot.redraw()
        self.root.after(50, self._pump)

    def _handle_t(self, payload):
        f = payload.split(",")
        if len(f) < 10:
            return False
        try:
            state = int(f[0]); count = int(f[1])
            ax, ay, az = float(f[2]), float(f[3]), float(f[4])
            mx, my, mz = float(f[5]), float(f[6]), float(f[7])
            heading = float(f[8]); applied = int(f[9])
        except ValueError:
            return False
        if state == 1 and self.prev_state == 0:
            self.magplot.begin_sweep()
        self.prev_state = state
        self.magplot.add_point(mx, my, state == 1)
        # orange = base heading from the raw soft-iron mag (always computable);
        # green = the firmware's fine-tuned heading, only when applied.
        base_h = tilt_comp_heading(ax, ay, az, mx, my, mz)
        ft_h = heading if applied == 1 else None
        self.compass.update_heading(base_h, ft_h)
        self.cap_lbl.configure(
            text=f"capture: {'RUNNING' if state else 'idle'}  samples={count}"
                 f"   applied={'yes' if applied else 'no'}")
        return True

    def _handle_r(self, payload):
        f = payload.split(",")
        if len(f) < 12:
            return
        try:
            status = int(f[0])
            roll, pitch, yaw = float(f[1]), float(f[2]), float(f[3])
            hix, hiy, hiz = float(f[4]), float(f[5]), float(f[6])
            arc, radius = float(f[7]), float(f[8])
            plane_res, circ_res = float(f[9]), float(f[10])
            axis_vert = int(f[11])
        except ValueError:
            return

        lines = []
        if status != 0:
            why = {-1: "not enough samples", -2: "degenerate plane",
                   -3: "degenerate circle"}.get(status, "no/invalid result")
            lines.append(f"status: ERROR ({status}) {why}")
            self._set_result("\n".join(lines))
            return

        self.magplot.set_fit(hix, hiy, radius)
        lines.append("status: OK")
        lines.append(f"de-rotation  r/p/y = {roll:.2f} / {pitch:.2f} / "
                     f"{yaw:.2f} deg")
        lines.append(f"hard-iron    = [{hix:.4f} {hiy:.4f} {hiz:.4f}]")
        lines.append(f"arc span     = {arc:.1f} deg")
        lines.append(f"field radius = {radius:.4f}")
        lines.append(f"plane resid  = {plane_res:.4f}")
        lines.append(f"circle resid = {circ_res:.4f}")
        lines.append(f"rotation axis: {'vertical' if axis_vert else 'tilted'}")
        warns = []
        if arc < 30.0:
            warns.append("! arc < 30 deg: hard-iron ill-conditioned; "
                         "trust de-rotation, be cautious with hard-iron")
        if plane_res > 0.05:
            warns.append("! high plane residual: motion not a clean in-plane "
                         "rotation")
        if circ_res > 0.05:
            warns.append("! high circle residual: soft-iron off or noisy")
        if warns:
            lines.append("")
            lines.extend(warns)
        self._set_result("\n".join(lines))

    def _set_result(self, text):
        self.result_text.configure(state="normal")
        self.result_text.delete("1.0", "end")
        self.result_text.insert("end", text)
        self.result_text.configure(state="disabled")

    def _log(self, text):
        self.console.insert("end", text)
        # trim
        n = int(self.console.index("end-1c").split(".")[0])
        if n > LOG_MAX_LINES:
            self.console.delete("1.0", f"{n - LOG_MAX_LINES}.0")
        self.console.see("end")


def main():
    ap = argparse.ArgumentParser(description="2D calibration control GUI")
    ap.add_argument("--port", default=None, help="serial port (e.g. /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    args = ap.parse_args()

    root = tk.Tk()
    app = App(root, args.port, args.baud)
    try:
        root.mainloop()
    finally:
        app.link.close()


if __name__ == "__main__":
    main()
