# cal2d GUI

A tiny desktop app to **control and visualize the 2D fine-tune calibration**
over UART. Simplest possible stack: Python standard-library **Tkinter** for the
UI + **pyserial** for the serial link (no matplotlib/numpy).

![panes: compass · mag X-Y plot · result · serial console]

## What it does

- **Control** — buttons send the firmware's shell commands:
  `cal2d start / stop / apply / off / show / clear / status`, plus a free-text
  command box.
- **Visualize**
  - a **compass dial** with the live tilt-compensated heading: an **orange**
    needle for the base (uncorrected) heading is always shown, and once a
    fine-tune is applied a **green** needle for the 2D fine-tuned heading is
    overlaid alongside it (with the Δ between them) so you can compare;
  - a **magnetometer X-Y plot** showing the points swept during a capture
    (`cal2d start` … `cal2d stop`), with the fitted **hard-iron center** and
    **field-radius circle** overlaid once the result arrives;
  - a **result panel** with the de-rotation angles, hard-iron, arc span and
    fit-quality residuals, including the same warnings the firmware prints
    (short arc, non-planar motion, high residual).
- a raw **serial console** for everything else.

## Requirements

- The MCU running this project's firmware (it emits the `#T,`/`#R,` telemetry
  the GUI parses — see below). Build/flash per the top-level README.
- Python 3.8+ with Tkinter (ships with CPython on Windows/macOS; on Debian/
  Ubuntu install `python3-tk`).

```bash
pip install -r requirements.txt
python cal2d_gui.py                 # pick the port in the UI, or:
python cal2d_gui.py --port /dev/ttyACM0 --baud 115200
```

## Typical session

1. Connect to the board's serial port.
2. Watch the compass — `Apply` / `Off` toggle base vs fine-tuned heading.
3. Click **Start**, rotate the device ±20–30° in its mounting plane, click
   **Stop** (or let it auto-finish when the buffer fills). The swept arc is
   drawn live; the fitted circle + center and the parameters appear on stop.
4. Click **Apply** to use the fine-tune; the compass turns green.

## Telemetry protocol (emitted by the firmware)

The GUI matches the payload anywhere in a line, so any Zephyr log prefix
(`[ts] <inf> module:`) is tolerated.

```
#T,state,count,ax,ay,az,mx,my,mz,heading,applied      (every ~50 ms)
#R,status,roll,pitch,yaw,hix,hiy,hiz,arc,radius,plane_res,circ_res,axis_vert
```

- `#T` — `state` 0=idle/1=capturing, `count` captured samples, `ax..az` accel
  (m/s²), `mx..mz` soft-iron mag (pre-2D, so the sweep traces a circle),
  `heading` deg (reflects the fine-tune when applied), `applied` 0/1.
- `#R` — emitted on `cal2d stop`/`show` (and on auto-finish). `status` 0=OK,
  negative = error; angles in degrees, hard-iron/radius in mag units,
  `axis_vert` 0/1.

> Note: the X-Y plot shows the raw soft-iron mag projected onto sensor X-Y, with
> the hard-iron center/radius overlaid. For a near-horizontal mount this matches
> the swept circle directly; for a strongly tilted mount it's an approximate
> projection (the de-rotation is what the result panel reports).
