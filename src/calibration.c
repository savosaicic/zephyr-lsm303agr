#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "calibration.h"

LOG_MODULE_REGISTER(calibration);

#define LSM303AGR_MAG_ADDR 0x1Eu

/* Hard-iron offset registers */
#define REG_OFFSET_X_L 0x45u
#define REG_OFFSET_X_H 0x46u
#define REG_OFFSET_Y_L 0x47u
#define REG_OFFSET_Y_H 0x48u
#define REG_OFFSET_Z_L 0x49u
#define REG_OFFSET_Z_H 0x4Au

/*
 * Magnetic sensitivity: 1.5 mgauss/LSB (datasheet Table 3, M_So).
 * Used to convert Gauss offsets to 16-bit integer for the offset registers.
 */
#define MAG_SENS_GAUSS_PER_LSB 0.0015f

/* Stub calibration data (in Gauss) */
static const struct mag_calibration cal_stub = {
  .hard_iron = {-0.1691f, -0.1813f, 0.1774f},
  .soft_iron = {
    {+0.985f, +0.020f, +0.008f},
    {+0.020f, +1.026f, -0.014f},
    {+0.008f, -0.014f, +0.990f},
  }};

/*
 * Load magnetometer calibration values.
 * Currently returns stub values derived from a MotionCal session.
 */
int calibration_load(struct mag_calibration *cal)
{
  memcpy(cal, &cal_stub, sizeof(*cal));
  LOG_INF("Stub calibration active");
  LOG_INF("  hard-iron: X=%.4f Y=%.4f Z=%.4f G", (double)cal->hard_iron[0],
          (double)cal->hard_iron[1], (double)cal->hard_iron[2]);
  return 0;
}

/*
 * Write hard-iron offsets to LSM303AGR hardware registers
 */
int calibration_apply_hw(const struct device *i2c,
                         const struct mag_calibration *cal)
{
  if (!device_is_ready(i2c)) {
    LOG_ERR("I2C bus not ready");
    return -ENODEV;
  }

  static const uint8_t regs[3][2] = {
    {REG_OFFSET_X_L, REG_OFFSET_X_H},
    {REG_OFFSET_Y_L, REG_OFFSET_Y_H},
    {REG_OFFSET_Z_L, REG_OFFSET_Z_H},
  };
  static const char *const axis_name[] = {"X", "Y", "Z"};

  for (int i = 0; i < 3; i++) {
    int16_t lsb = (int16_t)(cal->hard_iron[i] / MAG_SENS_GAUSS_PER_LSB);

    int ret = i2c_reg_write_byte(i2c, LSM303AGR_MAG_ADDR, regs[i][0],
                                (uint8_t)(lsb & 0xFF));
    if (ret) {
      LOG_ERR("HW offset %s low byte write failed: %d", axis_name[i], ret);
      return ret;
    }

    ret = i2c_reg_write_byte(i2c, LSM303AGR_MAG_ADDR, regs[i][1],
                            (uint8_t)((lsb >> 8) & 0xFF));
    if (ret) {
      LOG_ERR("HW offset %s high byte write failed: %d", axis_name[i], ret);
      return ret;
    }

    LOG_DBG("  offset %s: %.4f G → %d LSB", axis_name[i],
            (double)cal->hard_iron[i], (int)lsb);
  }

  LOG_INF("Hard-iron offsets written to LSM303AGR hardware registers");
  return 0;
}

/*
 * Applies the 3×3 soft-iron matrix.
 * Input must already be hard-iron corrected.
 */
void calibration_apply_sw(const struct mag_calibration *cal, double rx,
                          double ry, double rz, double *cx, double *cy,
                          double *cz)
{
  const float (*w)[3] = cal->soft_iron;

  *cx = (double)w[0][0] * rx + (double)w[0][1] * ry + (double)w[0][2] * rz;
  *cy = (double)w[1][0] * rx + (double)w[1][1] * ry + (double)w[1][2] * rz;
  *cz = (double)w[2][0] * rx + (double)w[2][1] * ry + (double)w[2][2] * rz;
}
