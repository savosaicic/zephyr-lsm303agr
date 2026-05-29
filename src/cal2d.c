/*
 * 2D fine-tune magnetometer calibration (ST DT0103) — pure C core.
 *
 * Faithful port of the DT0103 reference MATLAB ("Compensating for magnetometer
 * installation error and hard-iron effects using accelerometer-assisted 2D
 * calibration", DT0103 Rev 1). The original MATLAB for each function is quoted
 * in a comment above the C port.
 *
 * Conventions:
 *   - Row-vector post-multiplication: out = v * M, i.e. out[j]=sum_i v[i]*M[i][j].
 *   - euler = [roll(phi), pitch(theta), yaw(psi)], radians, NED frame.
 *   - All internal math is double for numerical robustness; the public API
 *     stores float to match the sensor sample buffers.
 */

#include <math.h>
#include <string.h>

#include "cal2d.h"

/* ------------------------------------------------------------------ */
/* Small linear-algebra helpers                                       */
/* ------------------------------------------------------------------ */

/* C = A * B   (3x3, row-major) */
static void mat3_mul(const double A[3][3], const double B[3][3], double C[3][3])
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			double s = 0.0;
			for (int k = 0; k < 3; k++) {
				s += A[i][k] * B[k][j];
			}
			C[i][j] = s;
		}
	}
}

/* B = A^T  (3x3) */
static void mat3_transpose(const double A[3][3], double B[3][3])
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			B[i][j] = A[j][i];
		}
	}
}

/* out = v * M   (row vector times 3x3) */
static void vec3_mul_mat(const double v[3], const double M[3][3], double out[3])
{
	for (int j = 0; j < 3; j++) {
		out[j] = v[0] * M[0][j] + v[1] * M[1][j] + v[2] * M[2][j];
	}
}

/* out = M * v   (3x3 times column vector) */
static void mat_mul_vec3(const double M[3][3], const double v[3], double out[3])
{
	for (int i = 0; i < 3; i++) {
		out[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
	}
}

/*
 * Solve A x = b for a small dense system (n <= 5) with partial pivoting.
 * A is row-major n x n (destroyed). Returns 0 on success, -1 if singular.
 */
static int solve_linear(double *A, double *b, int n, double *x)
{
	for (int col = 0; col < n; col++) {
		/* partial pivot */
		int piv = col;
		double best = fabs(A[col * n + col]);
		for (int r = col + 1; r < n; r++) {
			double v = fabs(A[r * n + col]);
			if (v > best) {
				best = v;
				piv = r;
			}
		}
		if (best < 1e-18) {
			return -1;
		}
		if (piv != col) {
			for (int c = 0; c < n; c++) {
				double t = A[col * n + c];
				A[col * n + c] = A[piv * n + c];
				A[piv * n + c] = t;
			}
			double t = b[col];
			b[col] = b[piv];
			b[piv] = t;
		}
		/* eliminate */
		double diag = A[col * n + col];
		for (int r = col + 1; r < n; r++) {
			double f = A[r * n + col] / diag;
			if (f == 0.0) {
				continue;
			}
			for (int c = col; c < n; c++) {
				A[r * n + c] -= f * A[col * n + c];
			}
			b[r] -= f * b[col];
		}
	}
	/* back-substitution */
	for (int i = n - 1; i >= 0; i--) {
		double s = b[i];
		for (int c = i + 1; c < n; c++) {
			s -= A[i * n + c] * x[c];
		}
		x[i] = s / A[i * n + i];
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* DT0103 helper functions (pp. 7-9)                                  */
/* ------------------------------------------------------------------ */

/*
 * function [euler] = acc2euler(acc)
 *   x=acc(:,1); y=acc(:,2); z=acc(:,3);
 *   r=atan2(y,z);
 *   p=atan(-x./(y.*sin(r) + z.*cos(r)));
 *   y=0;
 *   euler=[r p y];
 */
static void acc2euler(const double acc[3], double euler[3])
{
	double x = acc[0], y = acc[1], z = acc[2];
	double r = atan2(y, z);
	double p = atan(-x / (y * sin(r) + z * cos(r)));
	euler[0] = r;
	euler[1] = p;
	euler[2] = 0.0;
}

/*
 * function M = euler2rotM(euler)   % NED, Roll(phi) Pitch(theta) Yaw(psi)
 *   m11= cos(theta)*cos(psi);
 *   m12= cos(theta)*sin(psi);
 *   m13=-sin(theta);
 *   m21=sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
 *   m22=sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
 *   m23=sin(phi)*cos(theta);
 *   m31=cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
 *   m32=cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
 *   m33=cos(phi)*cos(theta);
 */
static void euler2rotM(const double euler[3], double M[3][3])
{
	double phi = euler[0], theta = euler[1], psi = euler[2];
	double cph = cos(phi), sph = sin(phi);
	double cth = cos(theta), sth = sin(theta);
	double cps = cos(psi), sps = sin(psi);

	M[0][0] = cth * cps;
	M[0][1] = cth * sps;
	M[0][2] = -sth;
	M[1][0] = sph * sth * cps - cph * sps;
	M[1][1] = sph * sth * sps + cph * cps;
	M[1][2] = sph * cth;
	M[2][0] = cph * sth * cps + sph * sps;
	M[2][1] = cph * sth * sps - sph * cps;
	M[2][2] = cph * cth;
}

/*
 * function euler = rotM2euler(M)
 *   m13=min(+1,m13); m13=max(-1,m13);
 *   theta=-asin(m13);
 *   costh=cos(theta);
 *   if abs(costh)<0.001,   % singularity
 *     sgnth=sign(theta); phi=0;
 *     psi=-sgnth*atan2(-m32,+m22);
 *   else
 *     psi=atan2(m12/costh,m11/costh);
 *     phi=atan2(m23/costh,m33/costh);
 *   end;
 *   euler=[phi theta psi]; % roll pitch yaw
 */
static void rotM2euler(const double M[3][3], double euler[3])
{
	double m13 = M[0][2];
	if (m13 > 1.0) {
		m13 = 1.0;
	}
	if (m13 < -1.0) {
		m13 = -1.0;
	}
	double theta = -asin(m13);
	double costh = cos(theta);
	double phi, psi;

	if (fabs(costh) < 0.001) {
		double sgnth = (theta > 0.0) ? 1.0 : ((theta < 0.0) ? -1.0 : 0.0);
		phi = 0.0;
		psi = -sgnth * atan2(-M[2][1], M[1][1]);
	} else {
		psi = atan2(M[0][1] / costh, M[0][0] / costh);
		phi = atan2(M[1][2] / costh, M[2][2] / costh);
	}
	euler[0] = phi;
	euler[1] = theta;
	euler[2] = psi;
}

/*
 * function [n,p] = planefitls(xyz)   % least squares plane normal (no eigen)
 * Computes the unit normal of the best-fit plane through the points and,
 * via *rms_dist_out, the mean Euclidean distance of points from their
 * centroid (used by the caller for the vertical-axis / cluster test).
 * Returns 0 on success, -1 if the points are degenerate (D==0).
 */
static int planefitls(const float xyz[][3], size_t N, double n[3],
		      double centroid[3], double *rms_dist_out)
{
	double p[3] = {0.0, 0.0, 0.0};
	for (size_t i = 0; i < N; i++) {
		p[0] += (double)xyz[i][0];
		p[1] += (double)xyz[i][1];
		p[2] += (double)xyz[i][2];
	}
	p[0] /= (double)N;
	p[1] /= (double)N;
	p[2] /= (double)N;

	double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
	double dsum = 0.0;
	for (size_t i = 0; i < N; i++) {
		double x = (double)xyz[i][0] - p[0];
		double y = (double)xyz[i][1] - p[1];
		double z = (double)xyz[i][2] - p[2];
		xx += x * x;
		yy += y * y;
		zz += z * z;
		xy += x * y;
		xz += x * z;
		yz += y * z;
		dsum += sqrt(x * x + y * y + z * z);
	}

	if (centroid) {
		centroid[0] = p[0];
		centroid[1] = p[1];
		centroid[2] = p[2];
	}
	if (rms_dist_out) {
		*rms_dist_out = dsum / (double)N;
	}

	double Dx = yy * zz - yz * yz;
	double Dy = xx * zz - xz * xz;
	double Dz = xx * yy - xy * xy;

	double D = fabs(Dx);
	int flag = 0;
	if (fabs(Dy) > D) {
		D = fabs(Dy);
		flag = 1;
	}
	if (fabs(Dz) > D) {
		D = fabs(Dz);
		flag = 2;
	}
	if (D == 0.0) {
		n[0] = n[1] = n[2] = 0.0;
		return -1;
	}

	double a, b, c;
	switch (flag) {
	case 0:
		a = 1.0;
		b = (xz * yz - xy * zz) / Dx;
		c = (xy * yz - xz * yy) / Dx;
		break;
	case 1:
		a = (yz * xz - xy * zz) / Dy;
		b = 1.0;
		c = (xy * xz - yz * xx) / Dy;
		break;
	default: /* case 2 */
		a = (yz * xy - xz * yy) / Dz;
		b = (xz * xy - yz * xx) / Dz;
		c = 1.0;
		break;
	}
	double norm = sqrt(a * a + b * b + c * c);
	n[0] = a / norm;
	n[1] = b / norm;
	n[2] = c / norm;
	return 0;
}

/*
 * Both the circle fit (DT0103 ellipsoid_2D_fit, f=2) and the rotated-ellipse
 * fit (f=0) are computed inline in cal2d_compute() by streaming the de-rotated
 * points into normal equations, which avoids a large temporary buffer of
 * de-rotated coordinates. See the comments at those sites.
 *
 *   circle (f=2):  D=[x^2+y^2, 2x, 2y];        center=(-v2/v1, -v3/v1)
 *   ellipse (f=0): D=[x^2, y^2, 2xy, 2x, 2y];  center=-[v1 v3; v3 v2]\[v4;v5]
 */

/*
 * Solve for the yaw Ycorr that removes the spurious yaw introduced by the
 * tilted-plane de-rotation (DT0103 "Ycorr"). After the finalize
 *   derotM_final = derotM_inst * euler2rotM([0,0,Ycorr]) * aderotM',
 * the (1,2) entry of derotM_final is
 *   M12(b) = c1 cos b + c2 sin b + c3,
 * and we want M12 = 0 (zero yaw). This has two roots; the physical branch is
 * the one with M11(b) > 0 (yaw = 0, not pi).
 *
 * DT0103 solves this with a Newton iteration whose listing has a sign error in
 * the derivative and a "+pi" branch jump that is only valid when the two roots
 * are pi apart (they are not). We instead solve the trig equation in closed
 * form and pick the M11>0 branch deterministically.
 *
 *   derotM  : installation-error de-rotation (first row used) = derotM_inst
 *   aderotM : accel-derived de-rotation (rows 0 and 1 used)
 */
static double solve_ycorr(const double derotM[3][3], const double aderotM[3][3])
{
	double m11 = derotM[0][0], m12 = derotM[0][1], m13 = derotM[0][2];
	double a11 = aderotM[0][0], a12 = aderotM[0][1], a13 = aderotM[0][2];
	double a21 = aderotM[1][0], a22 = aderotM[1][1], a23 = aderotM[1][2];

	double c1 = a21 * m11 + a22 * m12;   /* M12 = c1 cos b + c2 sin b + c3 */
	double c2 = a22 * m11 - a21 * m12;
	double c3 = a23 * m13;

	double R = sqrt(c1 * c1 + c2 * c2);
	if (R < 1e-12) {
		return 0.0;   /* degenerate: no yaw ambiguity to resolve */
	}
	/* c1 cos b + c2 sin b = -c3  ->  R cos(b - phi) = -c3, phi = atan2(c2,c1) */
	double arg = -c3 / R;
	if (arg > 1.0) {
		arg = 1.0;
	}
	if (arg < -1.0) {
		arg = -1.0;
	}
	double phi = atan2(c2, c1);
	double d = acos(arg);
	double cand[2] = {phi + d, phi - d};

	/* pick the branch with the larger M11 (the M11>0, yaw=0 solution) */
	double best_b = cand[0];
	double best_m11 = -1e300;
	for (int k = 0; k < 2; k++) {
		double cb = cos(cand[k]), sb = sin(cand[k]);
		double M11 = cb * (a11 * m11 + a12 * m12) +
			     sb * (a12 * m11 - a11 * m12) + a13 * m13;
		if (M11 > best_m11) {
			best_m11 = M11;
			best_b = cand[k];
		}
	}
	return best_b;
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

void cal2d_params_default(struct cal2d_params *p)
{
	memset(p, 0, sizeof(*p));
	for (int i = 0; i < 3; i++) {
		p->soft_iron[i][i] = 1.0f;
	}
	p->acc_noise_rms = 0.02f;
	p->use_ellipse_fit = false;
	/*
	 * Default to the horizontal-plane path. The DT0103 tilted-plane finalize
	 * (accel de-rotation + Ycorr) did not improve heading accuracy in host
	 * validation across tilt, and the published listing it is based on has
	 * sign/branch errors; for the limited-arc fine-tune on a near-horizontal
	 * mount the simple flatten-to-horizontal path is more robust. Set
	 * force_horizontal=false to opt into the experimental tilted-plane path.
	 */
	p->force_horizontal = true;
}

void cal2d_apply(const struct cal2d_result *r, double mx, double my, double mz,
		 double *ox, double *oy, double *oz)
{
	double v[3] = {
		mx - (double)r->hard_iron[0],
		my - (double)r->hard_iron[1],
		mz - (double)r->hard_iron[2],
	};
	double M[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			M[i][j] = (double)r->derotM[i][j];
		}
	}
	double out[3];
	vec3_mul_mat(v, M, out);  /* out = (mag - HI) * derotM */
	*ox = out[0];
	*oy = out[1];
	*oz = out[2];
}

int cal2d_compute(const float acc[][3], const float mag[][3], size_t n,
		  const struct cal2d_params *p, struct cal2d_result *r)
{
	struct cal2d_params def;
	if (!p) {
		cal2d_params_default(&def);
		p = &def;
	}

	memset(r, 0, sizeof(*r));
	/* identity de-rotation as a safe default */
	for (int i = 0; i < 3; i++) {
		r->derotM[i][i] = 1.0f;
	}

	if (n < CAL2D_MIN_SAMPLES) {
		r->status = CAL2D_ERR_TOO_FEW;
		return r->status;
	}

	/* --- Step 1: rotation-axis estimation from accelerometer ---------- */
	double na[3], a_centroid[3], avmd;
	double aderotM[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	bool axis_vertical;

	if (planefitls(acc, n, na, a_centroid, &avmd) != 0) {
		/* accel points degenerate => treat axis as vertical */
		axis_vertical = true;
	} else {
		/* choose minimal rotation w.r.t. vertical: dot(na,[0,0,1])>=0 */
		if (na[2] < 0.0) {
			na[0] = -na[0];
			na[1] = -na[1];
			na[2] = -na[2];
		}
		/* cluster of accel points (spread below noise) => vertical axis */
		axis_vertical = (avmd < 6.0 * (double)p->acc_noise_rms);
		if (!axis_vertical) {
			double rpy_ax[3];
			acc2euler(na, rpy_ax);
			euler2rotM(rpy_ax, aderotM);  /* de-rotate accel to horizontal */
		}
	}
	r->axis_vertical = axis_vertical;

	/* --- Step 2: installation error from magnetometer plane ----------- */
	double nme[3], m_centroid[3], mvmd;
	if (planefitls(mag, n, nme, m_centroid, &mvmd) != 0) {
		r->status = CAL2D_ERR_PLANE;
		return r->status;
	}
	if (nme[2] < 0.0) {
		nme[0] = -nme[0];
		nme[1] = -nme[1];
		nme[2] = -nme[2];
	}
	double rpy_err[3];
	acc2euler(nme, rpy_err);  /* soft-iron assumed unity (already applied) */
	double derotM[3][3];
	euler2rotM(rpy_err, derotM);

	/* --- Step 3: de-rotate mag to horizontal, fit circle, find HI ----- */
	/*
	 * mvc = mag * derotM. To keep memory bounded (no temporary buffer of
	 * de-rotated points), we stream the de-rotated coordinates directly into
	 * the circle-fit normal equations and accumulate the mean Z, which is the
	 * height of the fitted plane = the Z coordinate of the center.
	 */
	double A[9] = {0};
	double bb[3] = {0};
	double meanZ = 0.0;
	for (size_t i = 0; i < n; i++) {
		double v[3] = {mag[i][0], mag[i][1], mag[i][2]};
		double w[3];
		vec3_mul_mat(v, derotM, w);  /* w = mag * derotM (horizontal) */
		double x = w[0], y = w[1];
		meanZ += w[2];

		double d0 = x * x + y * y;
		double d1 = 2.0 * x;
		double d2 = 2.0 * y;
		A[0] += d0 * d0;
		A[1] += d0 * d1;
		A[2] += d0 * d2;
		A[4] += d1 * d1;
		A[5] += d1 * d2;
		A[8] += d2 * d2;
		bb[0] += d0;
		bb[1] += d1;
		bb[2] += d2;
	}
	meanZ /= (double)n;
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];

	double o2[2];
	double radius;
	if (p->use_ellipse_fit) {
		/* Rotated-ellipse fit (f=0): stream the de-rotated points into the
		 * 5-parameter normal equations (second pass; the circle-fit pass
		 * above is unused in this mode). */
		double Ae[25] = {0};
		double be[5] = {0};
		for (size_t i = 0; i < n; i++) {
			double v[3] = {mag[i][0], mag[i][1], mag[i][2]};
			double w[3];
			vec3_mul_mat(v, derotM, w);
			double x = w[0], y = w[1];
			double d[5] = {x * x, y * y, 2 * x * y, 2 * x, 2 * y};
			for (int rr = 0; rr < 5; rr++) {
				for (int cc = 0; cc < 5; cc++) {
					Ae[rr * 5 + cc] += d[rr] * d[cc];
				}
				be[rr] += d[rr];
			}
		}
		double ve[5];
		if (solve_linear(Ae, be, 5, ve) != 0) {
			r->status = CAL2D_ERR_CIRCLE;
			return r->status;
		}
		double a11 = ve[0], a12 = ve[2], a22 = ve[1];
		double det = a11 * a22 - a12 * a12;
		if (fabs(det) < 1e-18) {
			r->status = CAL2D_ERR_CIRCLE;
			return r->status;
		}
		o2[0] = -(a22 * ve[3] - a12 * ve[4]) / det;
		o2[1] = -(-a12 * ve[3] + a11 * ve[4]) / det;
		radius = 0.0; /* filled below from residual loop */
	} else {
		double v[3];
		if (solve_linear(A, bb, 3, v) != 0 || fabs(v[0]) < 1e-18) {
			r->status = CAL2D_ERR_CIRCLE;
			return r->status;
		}
		o2[0] = -v[1] / v[0];
		o2[1] = -v[2] / v[0];
		double r2 = 1.0 / v[0] + o2[0] * o2[0] + o2[1] * o2[1];
		if (r2 <= 0.0) {
			r->status = CAL2D_ERR_CIRCLE;
			return r->status;
		}
		radius = sqrt(r2);
	}

	/* o3 = [o2x, o2y, meanZ]; HIE = derotM * o3   (back-rotate center) */
	double o3[3] = {o2[0], o2[1], meanZ};
	double hie[3];
	mat_mul_vec3(derotM, o3, hie);

	/*
	 * Keep the installation de-rotation (which flattens the mag circle to
	 * horizontal). The tilted finalize below replaces derotM with one that
	 * maps into the tilted accel plane, so all the circle/arc quality metrics
	 * must be evaluated against this flattened version, not the final one.
	 */
	double derotM_inst[3][3];
	memcpy(derotM_inst, derotM, sizeof(derotM_inst));

	/* --- Step 4: tilted-plane finalize (Newton Ycorr) ----------------- */
	if (!axis_vertical && !p->force_horizontal) {
		double Ycorr = solve_ycorr(derotM, aderotM);
		double yaw[3] = {0.0, 0.0, Ycorr};
		double yawrotM[3][3];
		euler2rotM(yaw, yawrotM);
		double aderotT[3][3];
		mat3_transpose(aderotM, aderotT);
		/* derotM = derotM * yawrotM * aderotM' */
		double tmp[3][3], newderot[3][3];
		mat3_mul(derotM, yawrotM, tmp);
		mat3_mul(tmp, aderotT, newderot);
		memcpy(derotM, newderot, sizeof(derotM));
		rotM2euler(derotM, rpy_err);
	}

	/* --- Quality metrics ---------------------------------------------- */
	/* plane residual (mag): RMS of dot(point-centroid, nme), normalized */
	double pres = 0.0;
	for (size_t i = 0; i < n; i++) {
		double dx = (double)mag[i][0] - m_centroid[0];
		double dy = (double)mag[i][1] - m_centroid[1];
		double dz = (double)mag[i][2] - m_centroid[2];
		double d = dx * nme[0] + dy * nme[1] + dz * nme[2];
		pres += d * d;
	}
	pres = sqrt(pres / (double)n);

	/*
	 * Circle residual + arc span, computed on de-rotated XY about o2.
	 * Arc span: take the mean angle as a reference, then the span is the
	 * range of the wrapped angular offsets from it (robust for arcs < 180 deg,
	 * which is exactly the limited-motion fine-tune case).
	 */
	double cres = 0.0;
	double rsum = 0.0;
	double cx = 0.0, sx = 0.0;
	for (size_t i = 0; i < n; i++) {
		double v[3] = {mag[i][0], mag[i][1], mag[i][2]};
		double w[3];
		vec3_mul_mat(v, derotM_inst, w);  /* flattened plane */
		double dx = w[0] - o2[0];
		double dy = w[1] - o2[1];
		double rr = sqrt(dx * dx + dy * dy);
		rsum += rr;
		double ang = atan2(dy, dx);
		cx += cos(ang);
		sx += sin(ang);
	}
	double meanr = rsum / (double)n;
	if (p->use_ellipse_fit) {
		radius = meanr;
	}
	double refang = atan2(sx, cx);
	double amin = 1e9, amax = -1e9;
	for (size_t i = 0; i < n; i++) {
		double v[3] = {mag[i][0], mag[i][1], mag[i][2]};
		double w[3];
		vec3_mul_mat(v, derotM_inst, w);  /* flattened plane */
		double dx = w[0] - o2[0];
		double dy = w[1] - o2[1];
		double rr = sqrt(dx * dx + dy * dy);
		cres += (rr - meanr) * (rr - meanr);
		double ang = atan2(dy, dx) - refang;
		while (ang > 3.14159265358979323846) {
			ang -= 2.0 * 3.14159265358979323846;
		}
		while (ang < -3.14159265358979323846) {
			ang += 2.0 * 3.14159265358979323846;
		}
		if (ang < amin) {
			amin = ang;
		}
		if (ang > amax) {
			amax = ang;
		}
	}
	cres = sqrt(cres / (double)n);

	/* --- fill result -------------------------------------------------- */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			r->derotM[i][j] = (float)derotM[i][j];
		}
	}
	r->hard_iron[0] = (float)hie[0];
	r->hard_iron[1] = (float)hie[1];
	r->hard_iron[2] = (float)hie[2];
	r->rpy_err[0] = (float)rpy_err[0];
	r->rpy_err[1] = (float)rpy_err[1];
	r->rpy_err[2] = (float)rpy_err[2];
	r->arc_span_deg = (float)((amax - amin) * 180.0 / 3.14159265358979323846);
	r->field_radius = (float)radius;
	r->plane_rms_mag = (float)(meanr > 0.0 ? pres / meanr : pres);
	r->circle_residual = (float)(meanr > 0.0 ? cres / meanr : cres);
	r->status = CAL2D_OK;
	return r->status;
}
