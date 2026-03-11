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
