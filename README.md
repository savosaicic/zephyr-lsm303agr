# zephyr-lsm303agr

A minimal Zephyr RTOS application that reads accelerometer and magnetometer data
from an **LSM303AGR** sensor over I2C.

## Supported boards

| Board                          | Target                          | I2C pins                                |
| ------------------------------ | ------------------------------- | --------------------------------------- |
| Nordic nRF9151 DK              | `nrf9151dk/nrf9151/ns`          | Arduino header — SDA: P0.30, SCL: P0.31 |
| Makerdiary nRF9151 Connect Kit | `nrf9151_connectkit/nrf9151/ns` | SDA: P0.17, SCL: P0.18                  |

## Prerequisites

- Zephyr SDK / nRF Connect SDK toolchain (follow the [Getting Started](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) guide)
- A supported board with an I2C bus
- LSM303AGR wired to the board's I2C pins listed above, with:
    - Accelerometer at address `0x19`
    - Magnetometer at address `0x1E`

## Initialize the workspace

```bash
mkdir zephyr-lsm303agr-ws && cd zephyr-lsm303agr-ws
west init -m https://github.com/savosaicic/zephyr-lsm303agr --mr main
west update
pip install -r zephyr/scripts/requirements.txt
```

## Device tree overlay

Board overlays are provided for both supported boards under `boards/`:

```
boards/
  nrf9151dk_nrf9151_ns.overlay
  nrf9151_connectkit_nrf9151_ns.overlay
```

## Build & flash
```bash
# nrf9151dk/nrf9151/ns or nrf9151_connectkit/nrf9151/ns
BOARD=nrf9151dk/nrf9151/ns

cd zephyr-lsm303agr
west build -p always -b $BOARD .
west flash
```

Replace `nrf9151dk/nrf9151/ns` with your board target if different.

## Expected serial output

Connect to the board's serial port at **115200 baud** (`/dev/ttyACM0` on Linux, `/dev/tty.usbmodem*` on macOS).
You should see readings printed every second:

```
Accel: X=0.12 Y=-0.03 Z=9.81 m/s²
Mag:   X=0.23 Y=-0.11 Z=0.45 Gauss
Accel: X=0.11 Y=-0.02 Z=9.82 m/s²
Mag:   X=0.22 Y=-0.10 Z=0.44 Gauss
```

## Single-tap detection

Accelerometer fires `SENSOR_TRIG_TAP` on any axis; handler logs `TAP detected`.

Wire sensor **INT1** to a GPIO and set the pin in the board overlay.
Tune via `SENSOR_ATTR_SLOPE_TH` (threshold, m/s²) and
`SENSOR_ATTR_SLOPE_DUR` (ODR samples) in `main.c`.

## Magnetometer Calibration

This project applies **hard-iron** and **soft-iron** calibration
to the LSM303AGR magnetometer:

- **Hard-iron offsets**: constant bias from nearby magnetic sources
  (handled in hardware)
- **Soft-iron correction**: distortion from surrounding materials
  (handled in software)

### Calibration Data Source

Calibration values are currently **stubbed** in `calibration.c`:

- They come from a **MotionCal session** (external calibration tool)
- The tool computes:
    - A **hard-iron offset vector**
    - A **3×3 soft-iron correction matrix**

```c
struct mag_calibration {
  float hard_iron[3];
  float soft_iron[3][3];
};
```

These values are assumed to be precomputed by moving the sensor in all
orientations during calibration.

### How Calibration Is Applied

#### 1. Load Calibration into a runtime structure

```c
calibration_load(&cal);
```

#### 2. Apply Hard-Iron (Hardware)

```c
calibration_apply_hw(i2c, &cal);
```

- Converts hard-iron offsets from **Gauss** to **LSB**
- Writes them into LSM303AGR offset registers via I2C
- After this, all magnetometer readings are automatically hard-iron corrected
  by the chip

#### 3. Apply Soft-Iron (Software)

```c
calibration_apply_sw(&cal, mx, my, mz, &cx, &cy, &cz);
```

- Applies a **3×3 matrix transform** to the raw measurements
- Input must already be hard-iron corrected

### Calibration storage in production

In production, calibration coefficients could be stored in a dedicated
NVS flash partition as a `struct mag_calibration`:

```c
struct mag_calibration {
  float    hard_iron[3];
  float    soft_iron[3][3];
  uint8_t  signature[64];
};
```

We could add a signature field to ensure data integrity and authenticity.
The signature would cover everything before the `signature` field,
and firmware should refuse to use calibration data that fails verification.

The `calibration_load()` function would retrieve these values from NVS
instead of using stubbed values.

## 2D fine-tune calibration (install site)

The lab 3D calibration above is the **base** calibration. Once the device is
fixed in its final location, the surrounding ferromagnetic environment adds a
new hard-iron offset, and the magnetometer frame may have a small roll/pitch
misalignment relative to the accelerometer (installation error). A full 3D
re-calibration is impossible there — only a **limited in-plane rotation
(±20–30°)** is available.

This project implements ST design tip **DT0103** ("accelerometer-assisted 2D
calibration") to re-estimate, from that single limited sweep:

1. the **installation error** → a de-rotation matrix, and
2. the **residual hard-iron offset** → the center of the fitted circle,

assuming soft-iron is already known/applied (it is — from the lab calibration).

### Workflow (serial shell @115200)

```
cal2d start      # begin capturing; now rotate the device ±20–30° in its plane
cal2d stop       # finish, compute, and print the fine-tune
cal2d apply      # apply the fine-tune to the live stream (heading is printed)
cal2d off        # revert to base calibration only
cal2d show       # reprint the last result
cal2d clear      # discard the result
cal2d status     # capture / apply status
```

`cal2d stop` prints the de-rotation (roll/pitch/yaw), the residual hard-iron,
the captured **arc span**, and **fit-quality residuals**, with warnings when a
session is too short or the motion was not a clean in-plane rotation. The result
is held in RAM and applied live (no NVS persistence yet — see below).

### Desktop GUI

A small Tkinter + pyserial app under [`tools/cal2d_gui/`](tools/cal2d_gui/)
controls and **visualizes** the fine-tune over UART: a live heading compass, the
swept magnetometer X-Y points with the fitted hard-iron center/circle, the
result parameters, and buttons for all `cal2d` commands.

```bash
cd tools/cal2d_gui && pip install -r requirements.txt && python cal2d_gui.py
```

To feed it, the firmware emits two machine-readable telemetry lines (the GUI
matches the payload regardless of the Zephyr log prefix):

```
#T,state,count,ax,ay,az,mx,my,mz,heading,applied      (~20 Hz)
#R,status,roll,pitch,yaw,hix,hiy,hiz,arc,radius,plane_res,circ_res,axis_vert
```

`#T` streams every tick (`mx,my,mz` are the soft-iron mag, so a capture sweep
traces a circle); `#R` is emitted on `cal2d stop` / `show` and on auto-finish.

### What it compensates, and how it composes

At runtime the pipeline is:

```
raw mag → (chip) lab hard-iron → (sw) lab soft-iron → (sw) cal2d: subtract
          residual hard-iron, then de-rotate → tilt-compensated heading
```

The cal2d core receives **soft-iron-corrected** data and returns the *residual*
installation error + environmental hard-iron on top of the lab calibration.

### Accuracy and limitations (important)

The two outputs behave very differently on a short arc:

- **De-rotation (installation error / tilt):** well-conditioned even on a ~50°
  arc — this is the trustworthy output.
- **Hard-iron (circle center):** *ill-conditioned* on a short arc — center and
  radius are strongly correlated, so a few mGauss of noise can produce tens of
  mGauss of error. `cal2d stop` reports the arc span and residuals so a poor
  session can be rejected.

Validated envelope (host self-test, see below), for the install-site case
(short ±25° arc, 5 mG mag / 10 mg accel noise):

| Mount tilt from horizontal | Heading error |
| -------------------------- | ------------- |
| 0° (level)                 | ~1.9°         |
| 5°                         | ~2.6°         |
| 10°                        | ~4.6°         |
| 20°                        | ~7.9°         |

The implementation **defaults to the horizontal-plane path**, which is robust
for near-horizontal mounts (the intended use). DT0103's tilted-plane finalize
(accelerometer de-rotation + "Ycorr" Newton step) is included but **disabled by
default**: in validation it did not improve heading accuracy across tilt, and
the published MATLAB listing it derives from contains a derivative sign error
and an invalid root-selection jump (both noted in `src/cal2d.c`). It can be
enabled via `cal2d_params.force_horizontal = false` for experimentation. Large
tilts combined with a limited arc are a fundamental limitation of 2D
calibration, acknowledged by DT0103.

### Host self-test

The numerical core (`src/cal2d.c`) is pure C and is validated on the host
against DT0103's own simulation (known installation error + hard-iron recovered
exactly with no noise; graceful degradation with noise and tilt):

```bash
cd tests/cal2d && make run
```

### NVS persistence (future)

The fine-tune result is currently **log-only / applied in RAM**. Persisting it
to NVS alongside the base `struct mag_calibration` (see above) and loading it at
boot is a planned next step.
