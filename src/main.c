#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>

#include <math.h>

#include "calibration.h"
#include "cal2d.h"

LOG_MODULE_REGISTER(zephyr_lsm303agr);

const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_accel));
const struct device *mag   = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_mag));

static const struct device *i2c =
  DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm303agr_mag)));

/*
 * 2D fine-tune calibration (DT0103). The lab 3D MotionCal values (hard-iron in
 * the chip registers, soft-iron applied in software) are the base calibration.
 * At the install site the operator runs a single limited in-plane rotation:
 *
 *   cal2d start   -> rotate the device +-20..30 deg in its mounting plane
 *   cal2d stop    -> compute and print the fine-tune (de-rotation + hard-iron)
 *   cal2d apply   -> apply it to the live stream; cal2d off to revert
 *
 * The fine-tune sees soft-iron + lab-hard-iron corrected data and returns the
 * residual installation error and residual environmental hard-iron.
 */
#define CAL2D_MAX_SAMPLES 400
#define CAL2D_TICK_MS     50   /* ~20 Hz capture rate */
#define IDLE_PRINT_TICKS  1   /* decimate idle stream to ~1 Hz */

static struct mag_calibration cal;  /* lab base calibration */

static volatile bool cap_running;
static float  cap_acc[CAL2D_MAX_SAMPLES][3];
static float  cap_mag[CAL2D_MAX_SAMPLES][3];
static size_t cap_count;

static struct cal2d_result g_result;
static volatile bool g_result_valid;
static volatile bool g_apply;

/* Tilt-compensated heading (DT0058) from accel (m/s^2) + horizontal-frame mag. */
static double tilt_comp_heading(double ax, double ay, double az, double mx,
				double my, double mz)
{
	double roll = atan2(ay, az);
	double pitch = atan(-ax / (ay * sin(roll) + az * cos(roll)));
	double cr = cos(roll), sr = sin(roll);
	double cp = cos(pitch), sp = sin(pitch);

	/* rotate mag by euler2rotM([roll,pitch,0]) (row-vector convention) */
	double mxh = mx * (cp) + my * (sr * sp) + mz * (cr * sp);
	double myh = my * (cr) + mz * (-sr);
	double hdg = atan2(-myh, mxh) * 180.0 / 3.14159265358979323846;
	if (hdg < 0.0) {
		hdg += 360.0;
	}
	return hdg;
}

/*
 * Machine-readable result line for the GUI tool (tools/cal2d_gui). The GUI
 * extracts the "#R," payload from the line regardless of any log prefix:
 *   #R,status,roll,pitch,yaw,hix,hiy,hiz,arc_deg,radius,plane_res,circ_res,axis_vert
 * angles in degrees, hard-iron/radius in the mag units, axis_vert 0/1.
 */
static void cal2d_emit_result_line(int status)
{
	if (status == CAL2D_OK && g_result_valid) {
		const struct cal2d_result *r = &g_result;
		LOG_INF("#R,0,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%d",
			(double)(r->rpy_err[0] * 57.29578f),
			(double)(r->rpy_err[1] * 57.29578f),
			(double)(r->rpy_err[2] * 57.29578f),
			(double)r->hard_iron[0], (double)r->hard_iron[1],
			(double)r->hard_iron[2], (double)r->arc_span_deg,
			(double)r->field_radius, (double)r->plane_rms_mag,
			(double)r->circle_residual, r->axis_vertical ? 1 : 0);
	} else {
		LOG_INF("#R,%d,0,0,0,0,0,0,0,0,0,0,0", status);
	}
}

/*
 * Stop capturing and compute the fine-tune from the first n samples, storing
 * it in g_result on success. Shared by the 'cal2d stop' command and the
 * auto-stop path when the sample buffer fills. Returns a CAL2D_* status.
 */
static int cal2d_finish_capture(size_t n)
{
	cap_running = false;

	struct cal2d_params p;
	cal2d_params_default(&p);
	memcpy(p.soft_iron, cal.soft_iron, sizeof(p.soft_iron));

	struct cal2d_result r;
	int st = cal2d_compute(cap_acc, cap_mag, n, &p, &r);
	if (st == CAL2D_OK) {
		g_result = r;
		g_result_valid = true;
	}
	cal2d_emit_result_line(st);
	return st;
}

/* ------------------------------------------------------------------ */
/* cal2d shell commands                                               */
/* ------------------------------------------------------------------ */

static int cmd_cal2d_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (cap_running) {
		shell_warn(sh, "capture already running; use 'cal2d stop' first");
		return -EALREADY;
	}
	cap_count = 0;
	cap_running = true;
	shell_print(sh, "cal2d: capturing -- rotate the device +-20..30 deg in its");
	shell_print(sh, "       mounting plane, then run 'cal2d stop' (max %d samples)",
		    CAL2D_MAX_SAMPLES);
	return 0;
}

static int cmd_cal2d_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!cap_running) {
		shell_warn(sh, "no capture running; use 'cal2d start' first");
		return -EINVAL;
	}
	size_t n = cap_count;
	shell_print(sh, "cal2d: captured %u samples, computing...", (unsigned)n);

	int st = cal2d_finish_capture(n);
	if (st != CAL2D_OK) {
		const char *why = st == CAL2D_ERR_TOO_FEW ? "not enough samples"
			: st == CAL2D_ERR_PLANE ? "degenerate plane (no rotation?)"
			: st == CAL2D_ERR_CIRCLE ? "degenerate circle fit"
			: "unknown error";
		shell_error(sh, "cal2d: compute failed (%d): %s", st, why);
		return st;
	}

	const struct cal2d_result r = g_result;

	shell_print(sh, "cal2d: result");
	shell_print(sh, "  de-rotation roll/pitch/yaw = %.2f / %.2f / %.2f deg",
		    (double)(r.rpy_err[0] * 57.29578f),
		    (double)(r.rpy_err[1] * 57.29578f),
		    (double)(r.rpy_err[2] * 57.29578f));
	shell_print(sh, "  residual hard-iron = [%.4f %.4f %.4f] (subtracted, sensor frame)",
		    (double)r.hard_iron[0], (double)r.hard_iron[1],
		    (double)r.hard_iron[2]);
	shell_print(sh, "  arc span = %.1f deg | field radius = %.4f",
		    (double)r.arc_span_deg, (double)r.field_radius);
	shell_print(sh, "  plane residual = %.4f | circle residual = %.4f",
		    (double)r.plane_rms_mag, (double)r.circle_residual);
	shell_print(sh, "  rotation axis treated as %s",
		    r.axis_vertical ? "vertical" : "tilted");

	if (r.arc_span_deg < 30.0f) {
		shell_warn(sh, "  arc < 30 deg: hard-iron estimate is ill-conditioned"
				" -- trust the de-rotation, be cautious with hard-iron");
	}
	if (r.plane_rms_mag > 0.05f) {
		shell_warn(sh, "  high plane residual: motion was not a clean in-plane"
				" rotation");
	}
	if (r.circle_residual > 0.05f) {
		shell_warn(sh, "  high circle residual: soft-iron may be off, or noisy"
				" capture");
	}
	shell_print(sh, "  run 'cal2d apply' to use it on the live stream");
	return 0;
}

static int cmd_cal2d_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!g_result_valid) {
		shell_warn(sh, "no result yet; run 'cal2d start'/'cal2d stop' first");
		return -EINVAL;
	}
	g_apply = true;
	shell_print(sh, "cal2d: applying fine-tune to live stream");
	return 0;
}

static int cmd_cal2d_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	g_apply = false;
	shell_print(sh, "cal2d: fine-tune disabled on live stream");
	return 0;
}

static int cmd_cal2d_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!g_result_valid) {
		shell_print(sh, "cal2d: no result");
		return 0;
	}
	shell_print(sh, "cal2d: de-rot r/p/y = %.2f/%.2f/%.2f deg, HI = [%.4f %.4f %.4f],"
			" arc %.1f deg, applied=%s",
		    (double)(g_result.rpy_err[0] * 57.29578f),
		    (double)(g_result.rpy_err[1] * 57.29578f),
		    (double)(g_result.rpy_err[2] * 57.29578f),
		    (double)g_result.hard_iron[0], (double)g_result.hard_iron[1],
		    (double)g_result.hard_iron[2], (double)g_result.arc_span_deg,
		    g_apply ? "yes" : "no");
	cal2d_emit_result_line(CAL2D_OK);
	return 0;
}

static int cmd_cal2d_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	g_apply = false;
	g_result_valid = false;
	shell_print(sh, "cal2d: result cleared");
	return 0;
}

static int cmd_cal2d_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "cal2d: capturing=%s samples=%u result=%s applied=%s",
		    cap_running ? "yes" : "no", (unsigned)cap_count,
		    g_result_valid ? "valid" : "none", g_apply ? "yes" : "no");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	cal2d_cmds,
	SHELL_CMD(start, NULL, "Begin a fine-tune capture (then rotate +-20..30 deg)",
		  cmd_cal2d_start),
	SHELL_CMD(stop, NULL, "Finish capture and compute the 2D fine-tune",
		  cmd_cal2d_stop),
	SHELL_CMD(apply, NULL, "Apply the last fine-tune to the live stream",
		  cmd_cal2d_apply),
	SHELL_CMD(off, NULL, "Stop applying the fine-tune", cmd_cal2d_off),
	SHELL_CMD(show, NULL, "Print the last fine-tune result", cmd_cal2d_show),
	SHELL_CMD(clear, NULL, "Discard the last fine-tune result", cmd_cal2d_clear),
	SHELL_CMD(status, NULL, "Print capture/apply status", cmd_cal2d_status),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(cal2d, &cal2d_cmds,
		   "2D fine-tune magnetometer calibration (DT0103)", NULL);

/* ------------------------------------------------------------------ */

static void tap_handler(const struct device *dev,
                        const struct sensor_trigger *trig)
{
	LOG_INF("TAP detected");
}

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

	/* Single-tap detection on accelerometer */
	struct sensor_value tap_th;
	sensor_g_to_ms2(2, &tap_th);  /* ~2g threshold */
	ret = sensor_attr_set(accel, SENSOR_CHAN_ACCEL_XYZ,
	                      SENSOR_ATTR_SLOPE_TH, &tap_th);
	if (ret < 0) {
		LOG_ERR("tap threshold set failed: %d", ret);
		return ret;
	}

	struct sensor_value tap_dur = { .val1 = 3, .val2 = 0 }; /* ODR samples */
	ret = sensor_attr_set(accel, SENSOR_CHAN_ACCEL_XYZ,
	                      SENSOR_ATTR_SLOPE_DUR, &tap_dur);
	if (ret < 0) {
		LOG_ERR("tap duration set failed: %d", ret);
		return ret;
	}

	static const struct sensor_trigger tap_trig = {
		.type = SENSOR_TRIG_TAP,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	ret = sensor_trigger_set(accel, (struct sensor_trigger *)&tap_trig, tap_handler);
	if (ret < 0) {
		LOG_ERR("tap trigger set failed: %d", ret);
		return ret;
	}

	LOG_INF("Calibration applied. Starting measurement loop.");
	LOG_INF("Run 'cal2d start' / 'cal2d stop' to fine-tune at the install site.");

	struct sensor_value accel_x, accel_y, accel_z;
	struct sensor_value mag_x,   mag_y,   mag_z;
	unsigned int idle_ticks = 0;

	while (1) {
		/* Read accelerometer */
		sensor_sample_fetch(accel);
		sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_x);
		sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_y);
		sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_z);
		double dax = sensor_value_to_double(&accel_x);
		double day = sensor_value_to_double(&accel_y);
		double daz = sensor_value_to_double(&accel_z);

		/* Read magnetometer (hard-iron corrected in hardware) */
		sensor_sample_fetch(mag);
		sensor_channel_get(mag, SENSOR_CHAN_MAGN_X, &mag_x);
		sensor_channel_get(mag, SENSOR_CHAN_MAGN_Y, &mag_y);
		sensor_channel_get(mag, SENSOR_CHAN_MAGN_Z, &mag_z);
		double dmx = sensor_value_to_double(&mag_x);
		double dmy = sensor_value_to_double(&mag_y);
		double dmz = sensor_value_to_double(&mag_z);

		/* Lab soft-iron (the cal2d core expects soft-iron-corrected data) */
		double cmx, cmy, cmz;
		calibration_apply_sw(&cal, dmx, dmy, dmz, &cmx, &cmy, &cmz);

		/*
		 * Tilt-compensated heading. Use the 2D fine-tuned mag when a
		 * fine-tune is applied, otherwise the base (soft-iron) mag.
		 */
		double hmx = cmx, hmy = cmy, hmz = cmz;
		bool finetuned = g_result_valid && g_apply;
		if (finetuned) {
			cal2d_apply(&g_result, cmx, cmy, cmz, &hmx, &hmy, &hmz);
		}
		double hdg = tilt_comp_heading(dax, day, daz, hmx, hmy, hmz);

		if (cap_running) {
			if (cap_count < CAL2D_MAX_SAMPLES) {
				cap_acc[cap_count][0] = (float)dax;
				cap_acc[cap_count][1] = (float)day;
				cap_acc[cap_count][2] = (float)daz;
				cap_mag[cap_count][0] = (float)cmx;
				cap_mag[cap_count][1] = (float)cmy;
				cap_mag[cap_count][2] = (float)cmz;
				cap_count++;
			}
			if (cap_count >= CAL2D_MAX_SAMPLES) {
				/* buffer full: stop and compute here, no 'cal2d stop' needed */
				int st = cal2d_finish_capture(cap_count);
				if (st == CAL2D_OK) {
					LOG_INF("cal2d: buffer full (%d samples) -- auto-finished;"
						" run 'cal2d show' / 'cal2d apply'",
						CAL2D_MAX_SAMPLES);
				} else {
					LOG_ERR("cal2d: buffer full but compute failed (%d)", st);
				}
			}
		}

		/*
		 * Machine-readable telemetry for the GUI tool (and a human-glanceable
		 * heading). The GUI extracts the "#T," payload regardless of log prefix:
		 *   #T,state,count,ax,ay,az,mx,my,mz,heading,applied
		 * state 0=idle/1=capturing; mx,my,mz are soft-iron (pre-2D) so the GUI
		 * can plot the swept circle; heading reflects the fine-tune if applied.
		 */
		if (++idle_ticks >= IDLE_PRINT_TICKS) {
			idle_ticks = 0;
			LOG_INF("#T,%d,%u,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.1f,%d",
				cap_running ? 1 : 0, (unsigned)cap_count, dax, day, daz,
				cmx, cmy, cmz, hdg, finetuned ? 1 : 0);
		}

		k_msleep(CAL2D_TICK_MS);
	}
	return 0;
}
