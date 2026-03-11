# zephyr-lsm303agr

A minimal Zephyr RTOS application that reads accelerometer and magnetometer data
from an **LSM303AGR** sensor over I2C.

## Prerequisites

- Zephyr SDK / nRF Connect SDK toolchain (follow the [Getting Started](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) guide)
- A supported board with an I2C bus
- LSM303AGR wired to the board's I2C bus (`SDA`/`SCL`) — on the **nRF9151 DK** these are pins **P0.30 (SDA)** and **P0.31 (SCL)** on the Arduino header — with:
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

A board overlay is required to enable the I2C bus and declare the LSM303AGR nodes. One is already provided for the **nRF9151 DK**:

```
boards/nrf9151dk_nrf9151_ns.overlay
```

For a different board, create a matching overlay file under `boards/` named after your board target (e.g. `boards/your_board.overlay`) with the following content, adjusting the I2C bus node label as needed:

```dts
&arduino_i2c {
    status = "okay";

    lsm303agr_accel: lsm303agr_accel@19 {
        compatible = "st,lis2dh";
        reg = <0x19>;
    };

    lsm303agr_mag: lsm303agr_mag@1e {
        compatible = "st,lis2mdl";
        reg = <0x1e>;
    };
};
```

## Build & flash
```bash
cd zephyr-lsm303agr
west build -b nrf9151dk/nrf9151/ns .
west flash
```

Replace `nrf9151dk/nrf9151/ns` with your board target if different.

## Expected serial output

Connect to the board's serial port at **115200 baud**. You should see readings printed every second:

```
Accel: X=0.12 Y=-0.03 Z=9.81 m/s²
Mag:   X=0.23 Y=-0.11 Z=0.45 Gauss
Accel: X=0.11 Y=-0.02 Z=9.82 m/s²
Mag:   X=0.22 Y=-0.10 Z=0.44 Gauss
```
