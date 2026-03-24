#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(zephyr_lsm303agr);

const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_accel));
const struct device *mag   = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_mag));

int main(void)
{
  if (!device_is_ready(accel)) {
    LOG_ERR("Accelerometer not ready");
    return -1;
  }

  if (!device_is_ready(mag)) {
    LOG_ERR("Magnetometer not ready");
    return -1;
  }

  struct sensor_value accel_x, accel_y, accel_z;
  struct sensor_value mag_x,   mag_y,   mag_z;

  while (1) {
    sensor_sample_fetch(accel);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_x);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_y);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_z);
    LOG_INF("Accel: X=%.2f Y=%.2f Z=%.2f m/s²",
           sensor_value_to_double(&accel_x), sensor_value_to_double(&accel_y),
           sensor_value_to_double(&accel_z));

    sensor_sample_fetch(mag);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_X, &mag_x);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_Y, &mag_y);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_Z, &mag_z);
    LOG_INF("Mag:   X=%.2f Y=%.2f Z=%.2f Gauss",
           sensor_value_to_double(&mag_x), sensor_value_to_double(&mag_y),
           sensor_value_to_double(&mag_z));

    k_msleep(1000);
  }
  return 0;
}
