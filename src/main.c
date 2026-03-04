#include <zephyr/kernel.h>
#include <zephyr/device.h>

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

  while (1) {
    k_msleep(1000);
  }
  return 0;
}
