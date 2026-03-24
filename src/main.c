#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include "calibration.h"

LOG_MODULE_REGISTER(zephyr_lsm303agr);

const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_accel));
const struct device *mag   = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_mag));

static const struct device *i2c =
  DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm303agr_mag)));

int main(void)
{
  int ret;

  if (!device_is_ready(accel)) {
    LOG_ERR("Accelerometer not ready");
    return -1;
  }
  if (!device_is_ready(mag)) {
    LOG_ERR("Magnetometer not ready");
    return -1;
  }
  if (!device_is_ready(i2c)) {
    LOG_ERR("I2C bus not ready");
    return -1;
  }

  struct mag_calibration cal;
  ret = calibration_load(&cal);
  if (ret < 0) {
    LOG_ERR("calibration_load failed: %d", ret);
    return ret;
  }

  /*
   * Write hard-iron offsets to LSM303AGR hardware registers
   * After that, every reading is automatically hard-iron
   * corrected by the chip.
   */
  ret = calibration_apply_hw(i2c, &cal);
  if (ret < 0) {
    LOG_ERR("calibration_apply_hw failed: %d", ret);
    return ret;
  }

  LOG_INF("Calibration applied. Starting measurement loop.");

  struct sensor_value accel_x, accel_y, accel_z;
  struct sensor_value mag_x,   mag_y,   mag_z;

  while (1) {
    /* Read accelerometer */
    sensor_sample_fetch(accel);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_x);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_y);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_z);
    double dax = sensor_value_to_double(&accel_x);
    double day = sensor_value_to_double(&accel_y);
    double daz = sensor_value_to_double(&accel_z);

    /* Read magnetometer */
    sensor_sample_fetch(mag);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_X, &mag_x);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_Y, &mag_y);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_Z, &mag_z);
    double dmx = sensor_value_to_double(&mag_x);
    double dmy = sensor_value_to_double(&mag_y);
    double dmz = sensor_value_to_double(&mag_z);

    /* Apply soft-iron matrix */
    double cmx, cmy, cmz;
    calibration_apply_sw(&cal, dmx, dmy, dmz, &cmx, &cmy, &cmz);

    LOG_INF("Accel: X=%.2f Y=%.2f Z=%.2f m/s²", dax, day, daz);
    LOG_INF("Mag: X=%.3f Y=%.3f Z=%.3f G (hard-iron by chip)", dmx, dmy, dmz);
    LOG_INF("Mag: X=%.3f Y=%.3f Z=%.3f G (soft-iron applied)", cmx, cmy, cmz);
    LOG_INF("---");

    k_msleep(1000);
  }
  return 0;
}
