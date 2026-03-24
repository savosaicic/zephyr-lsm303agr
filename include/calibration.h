#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <zephyr/device.h>

struct mag_calibration {
  float hard_iron[3];
  float soft_iron[3][3];
};

int calibration_load(struct mag_calibration *cal);

int calibration_apply_hw(const struct device *i2c,
                         const struct mag_calibration *cal);

void calibration_apply_sw(const struct mag_calibration *cal, double rx,
                          double ry, double rz, double *cx, double *cy,
                          double *cz);

#endif /* !CALIBRATION_H */
