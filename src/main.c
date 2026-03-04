#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_accel));
const struct device *mag   = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_mag));

int main(void)
{
  if (!device_is_ready(accel)) {
    printk("Accelerometer not ready\n");
    return -1;
  }

  if (!device_is_ready(mag)) {
    printk("Magnetometer not ready\n");
    return -1;
  }

  struct sensor_value accel_x, accel_y, accel_z;
  struct sensor_value mag_x,   mag_y,   mag_z;

  while (1) {
    sensor_sample_fetch(accel);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_x);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_y);
    sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_z);
    printk("Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n",
           sensor_value_to_double(&accel_x), sensor_value_to_double(&accel_y),
           sensor_value_to_double(&accel_z));

    sensor_sample_fetch(mag);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_X, &mag_x);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_Y, &mag_y);
    sensor_channel_get(mag, SENSOR_CHAN_MAGN_Z, &mag_z);
    printk("Mag:   X=%.2f Y=%.2f Z=%.2f Gauss\n",
           sensor_value_to_double(&mag_x), sensor_value_to_double(&mag_y),
           sensor_value_to_double(&mag_z));

    k_msleep(1000);
  }
  return 0;
}
