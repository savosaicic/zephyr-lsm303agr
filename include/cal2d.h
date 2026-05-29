#ifndef CAL2D_H
#define CAL2D_H

/*
 * 2D fine-tune magnetometer calibration (ST DT0103).
 *
 * Accelerometer-assisted 2D calibration: from a single limited in-plane
 * rotation (a "fine-tune" sweep, e.g. +-20..30 deg) at the install site,
 * re-estimate
 *
 *   1. the magnetometer installation error (roll/pitch misalignment of the
 *      mag frame vs. the accelerometer frame) -> a de-rotation matrix, and
 *   2. the residual hard-iron offset of the local environment -> the center
 *      of the fitted circle.
 *
 * Soft-iron is assumed already known and applied to the input data (from the
 * lab 3D MotionCal calibration), so the algorithm internally treats soft-iron
 * as unity. Hard-iron returned here is the *additional* environmental offset,
 * on top of whatever the lab calibration already removes.
 *
 * This module is pure C (only <math.h>) so it can be unit-tested on the host.
 *
 * IMPORTANT — short-arc caveat:
 *   With only ~50 deg of arc the plane fit (installation error / tilt) stays
 *   well-conditioned, but the circle-center fit (hard-iron) is ill-conditioned:
 *   center and radius become strongly correlated, so a few mGauss of noise can
 *   produce tens of mGauss of hard-iron error. Trust the de-rotation; gate the
 *   hard-iron on `arc_span_deg` and `circle_residual` in the result.
 *
 * Matrix convention: row-vector post-multiplication, matching the DT0103
 * MATLAB. For a row vector v and matrix M, the transformed vector is
 *   out[j] = sum_i v[i] * M[i][j]    (i.e. out = v * M).
 */

#include <stddef.h>
#include <stdbool.h>

struct cal2d_params {
	/*
	 * Lab soft-iron matrix, kept for reference/logging only. The input data
	 * passed to cal2d_compute() is assumed to be ALREADY soft-iron corrected,
	 * so the core treats soft-iron as unity.
	 */
	float soft_iron[3][3];

	/*
	 * Accelerometer RMS noise in g units, used for the vertical-axis test
	 * (axis is treated as vertical when the accel barycenter spread is below
	 * 6 * acc_noise_rms). Increase to bias toward the simpler vertical-axis
	 * path; decrease to make the tilted-plane machinery trigger on smaller
	 * tilts. ~0.02 g is a sensible default for the LSM303AGR.
	 */
	float acc_noise_rms;

	/*
	 * false (default): circle fit (DT0103 f=2) — 3 parameters, best
	 * conditioned on a short arc, correct when soft-iron is already applied.
	 * true: rotated-ellipse fit (f=0) — 5 parameters, only if residual
	 * soft-iron must be absorbed; much worse on a short arc.
	 */
	bool use_ellipse_fit;

	/*
	 * Force the simple flatten-to-horizontal path (skip the tilted-plane
	 * accel de-rotation + Ycorr finalize) even when the rotation axis is
	 * detected as non-vertical. See cal2d.c / README for why this is the
	 * default-safe choice for near-horizontal mounts.
	 */
	bool force_horizontal;
};

struct cal2d_result {
	/* Installation-error de-rotation (row-vector convention: out = mag * derotM). */
	float derotM[3][3];

	/* Estimated residual hard-iron offset, sensor frame, same units as input. */
	float hard_iron[3];

	/* Reported installation error [roll, pitch, yaw] in radians. */
	float rpy_err[3];

	/* Extent of the captured sweep in degrees (de-rotated XY). */
	float arc_span_deg;

	/* Mag plane-fit RMS residual, normalized by the mean radius (unitless). */
	float plane_rms_mag;

	/* Circle-fit RMS radial residual, normalized by the radius (unitless). */
	float circle_residual;

	/* Fitted circle radius (field strength in the horizontal plane). */
	float field_radius;

	/* true => rotation axis treated as vertical; tilt machinery skipped. */
	bool axis_vertical;

	/* 0 = ok; <0 = error (see CAL2D_ERR_* below). */
	int status;
};

#define CAL2D_OK 0
#define CAL2D_ERR_TOO_FEW (-1)    /* not enough samples */
#define CAL2D_ERR_PLANE (-2)      /* degenerate mag plane fit */
#define CAL2D_ERR_CIRCLE (-3)     /* degenerate circle fit */

/* Minimum number of samples cal2d_compute() will accept. */
#define CAL2D_MIN_SAMPLES 8

/*
 * Compute the 2D fine-tune calibration from a captured sweep.
 *
 *   acc : N x 3 accelerometer samples (any consistent unit; only direction
 *         and relative spread matter — m/s^2 or g are both fine).
 *   mag : N x 3 magnetometer samples, ALREADY soft-iron (and lab hard-iron)
 *         corrected.
 *   n   : number of samples.
 *   p   : parameters (may be NULL for defaults).
 *   r   : result (filled on return; r->status carries the outcome).
 *
 * Returns r->status (0 on success, negative on error).
 */
int cal2d_compute(const float acc[][3], const float mag[][3], size_t n,
		  const struct cal2d_params *p, struct cal2d_result *r);

/*
 * Apply a computed result to one magnetometer sample:
 *   out = (mag - hard_iron) * derotM
 * Input must be in the same (soft-iron corrected) space as the capture.
 */
void cal2d_apply(const struct cal2d_result *r, double mx, double my, double mz,
		 double *ox, double *oy, double *oz);

/* Fill p with default parameters (unity soft-iron, 0.02 g noise, circle fit). */
void cal2d_params_default(struct cal2d_params *p);

#endif /* !CAL2D_H */
