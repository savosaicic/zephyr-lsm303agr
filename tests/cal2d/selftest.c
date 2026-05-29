/*
 * Host self-test for the DT0103 2D fine-tune calibration core.
 *
 * Reproduces the DT0103 test.m simulation (quat2rotM rotation about a chosen
 * axis, installation error rotMerr, hard iron HI; soft-iron treated as already
 * applied, SI = unity) and feeds the synthesized accel/mag through
 * cal2d_compute(). The correctness criterion is convention-free:
 *
 *   the calibrated heading (tilt-compensated eCompass on the corrected mve)
 *   must match the heading the IDEAL, perfectly-aligned mag (nv) would give
 *   with the same accelerometer.
 *
 * Forward model, exactly as DT0103 test.m (column-vector math):
 *   nav: gravity aref=[0,0,1], field mref=[cos mi,0,sin mi]
 *   rotM   = quat2rotM(yaw_i about rotax)
 *   av     = rotM' * aref                          (accelerometer)
 *   nv     = rotM' * mref                          (ideal, aligned mag)
 *   mve    = rotMerr' * rotM' * mref + HI          (measured: misaligned + HI)
 *
 * Supported envelope (asserted): near-horizontal mount (rotation axis within
 * ~10 deg of vertical) with a limited arc — the install-site fine-tune case.
 * A full informational tilt sweep is printed too.
 *
 * Build & run:  cd tests/cal2d && make run
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cal2d.h"

#define PI 3.14159265358979323846
#define DEG (PI / 180.0)
#define NMAX 400

/* ---- generator helpers (same conventions as the core) ---- */

static void euler2rotM(const double e[3], double M[3][3])
{
	double phi = e[0], theta = e[1], psi = e[2];
	double cph = cos(phi), sph = sin(phi);
	double cth = cos(theta), sth = sin(theta);
	double cps = cos(psi), sps = sin(psi);
	M[0][0] = cth * cps;             M[0][1] = cth * sps;             M[0][2] = -sth;
	M[1][0] = sph*sth*cps - cph*sps; M[1][1] = sph*sth*sps + cph*cps; M[1][2] = sph*cth;
	M[2][0] = cph*sth*cps + sph*sps; M[2][1] = cph*sth*sps - sph*cps; M[2][2] = cph*cth;
}
static void quat2rotM(const double Q[4], double M[3][3])
{
	double qw = Q[0], qx = Q[1], qy = Q[2], qz = Q[3];
	double qw2=qw*qw, qx2=qx*qx, qy2=qy*qy, qz2=qz*qz, n = 1.0/(qw2+qx2+qy2+qz2), t1, t2;
	M[0][0]=(qx2-qy2-qz2+qw2)*n; M[1][1]=(-qx2+qy2-qz2+qw2)*n; M[2][2]=(-qx2-qy2+qz2+qw2)*n;
	t1=qx*qy; t2=qz*qw; M[0][1]=2*(t1+t2)*n; M[1][0]=2*(t1-t2)*n;
	t1=qx*qz; t2=qy*qw; M[0][2]=2*(t1+t2)*n; M[2][0]=2*(t1-t2)*n;
	t1=qy*qz; t2=qx*qw; M[1][2]=2*(t1+t2)*n; M[2][1]=2*(t1-t2)*n;
}
/* out = M^T * v  (matches MATLAB rotM'*vec) */
static void matT_mul_vec(const double M[3][3], const double v[3], double o[3])
{ for (int i=0;i<3;i++) o[i] = M[0][i]*v[0] + M[1][i]*v[1] + M[2][i]*v[2]; }

static uint32_t rng_state = 12345u;
static double urand(void){ rng_state = rng_state*1664525u + 1013904223u; return (double)(rng_state>>8)/(double)(1u<<24); }
static double gauss(void){ double u1=urand(),u2=urand(); if(u1<1e-12)u1=1e-12; return sqrt(-2.0*log(u1))*cos(2.0*PI*u2); }
static double wrap_pi(double a){ while(a>PI)a-=2*PI; while(a<=-PI)a+=2*PI; return a; }

/* ---- simulation ---- */

struct sim_cfg {
	double mi_deg, rerr_deg, perr_deg, HI[3], rotax[3];
	double arc_start_deg, arc_end_deg;
	int N;
	double mag_noise, acc_noise;
};

static void simulate(const struct sim_cfg *c, float acc[][3], float mag[][3],
		     float nv_out[][3])
{
	double mi = c->mi_deg * DEG;
	double aref[3] = {0, 0, 1};
	double mref[3] = {cos(mi), 0, sin(mi)};
	double rpyerr[3] = {c->rerr_deg*DEG, c->perr_deg*DEG, 0};
	double Rerr[3][3];
	euler2rotM(rpyerr, Rerr);

	double u[3] = {c->rotax[0], c->rotax[1], c->rotax[2]};
	double un = sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);
	u[0]/=un; u[1]/=un; u[2]/=un;

	for (int i = 0; i < c->N; i++) {
		double yaw = (c->N==1) ? c->arc_start_deg*DEG
			: (c->arc_start_deg + (c->arc_end_deg-c->arc_start_deg)*(double)i/(c->N-1))*DEG;
		double r2 = yaw/2;
		double Q[4] = {cos(r2), sin(r2)*u[0], sin(r2)*u[1], sin(r2)*u[2]};
		double rotM[3][3];
		quat2rotM(Q, rotM);

		double av[3], nv[3], mve[3];
		matT_mul_vec(rotM, aref, av);      /* av  = rotM' * aref */
		matT_mul_vec(rotM, mref, nv);      /* nv  = rotM' * mref (ideal) */
		matT_mul_vec(Rerr, nv, mve);       /* mve = rotMerr' * nv */
		for (int k=0;k<3;k++) mve[k] += c->HI[k];

		acc[i][0]=(float)(av[0]+c->acc_noise*gauss());
		acc[i][1]=(float)(av[1]+c->acc_noise*gauss());
		acc[i][2]=(float)(av[2]+c->acc_noise*gauss());
		mag[i][0]=(float)(mve[0]+c->mag_noise*gauss());
		mag[i][1]=(float)(mve[1]+c->mag_noise*gauss());
		mag[i][2]=(float)(mve[2]+c->mag_noise*gauss());
		if (nv_out){ nv_out[i][0]=(float)nv[0]; nv_out[i][1]=(float)nv[1]; nv_out[i][2]=(float)nv[2]; }
	}
}

/* tilt-compensated heading (DT0058) from a calibrated mag + its accel sample */
static double tc_heading(const struct cal2d_result *r, const float a[3], const float m[3])
{
	double ox, oy, oz;
	cal2d_apply(r, m[0], m[1], m[2], &ox, &oy, &oz);
	double roll = atan2(a[1], a[2]);
	double pitch = atan(-a[0] / (a[1]*sin(roll) + a[2]*cos(roll)));
	double e[3] = {roll, pitch, 0}, Rtc[3][3];
	euler2rotM(e, Rtc);
	double v[3] = {ox, oy, oz}, h[3];
	for (int j=0;j<3;j++) h[j] = v[0]*Rtc[0][j] + v[1]*Rtc[1][j] + v[2]*Rtc[2][j];
	return atan2(-h[1], h[0]);
}

/* RMS heading error vs the ideal aligned-mag eCompass (constant offset removed) */
static double heading_err(const struct cal2d_result *r, float acc[][3],
			  float mag[][3], float nv[][3], int N)
{
	struct cal2d_result ident;
	memset(&ident, 0, sizeof(ident));
	for (int k=0;k<3;k++) ident.derotM[k][k]=1.0f;
	double cs=0, sn=0;
	for (int i=0;i<N;i++){
		double d = wrap_pi(tc_heading(r,acc[i],mag[i]) - tc_heading(&ident,acc[i],nv[i]));
		cs+=cos(d); sn+=sin(d);
	}
	double mean = atan2(sn, cs), acc2=0;
	for (int i=0;i<N;i++){
		double d = wrap_pi(tc_heading(r,acc[i],mag[i]) - tc_heading(&ident,acc[i],nv[i]) - mean);
		acc2 += d*d;
	}
	return sqrt(acc2/N)/DEG;
}

static int g_fail;
static void expect(const char *what, bool ok, double val, double tol)
{
	printf("    [%s] %-42s (%.3f <= %.3f)\n", ok?"PASS":"FAIL", what, val, tol);
	if (!ok) g_fail = 1;
}

static double run(const char *name, const struct sim_cfg *c)
{
	static float acc[NMAX][3], mag[NMAX][3], nv[NMAX][3];
	rng_state = 12345u;
	simulate(c, acc, mag, nv);
	struct cal2d_params p;
	cal2d_params_default(&p);
	p.acc_noise_rms = (c->acc_noise>0)?(float)c->acc_noise:0.005f;
	struct cal2d_result r;
	cal2d_compute(acc, mag, c->N, &p, &r);
	double herr = heading_err(&r, acc, mag, nv, c->N);
	printf("== %s ==  status=%d arc=%.0f field_r=%.3f plane_res=%.4f circle_res=%.4f  herr=%.2f deg\n",
	       name, r.status, r.arc_span_deg, r.field_radius, r.plane_rms_mag, r.circle_residual, herr);
	return herr;
}

int main(void)
{
	struct sim_cfg base = {
		.mi_deg=60, .rerr_deg=8, .perr_deg=6, .HI={0.10,-0.20,0.30},
		.rotax={0,0,1}, .arc_start_deg=0, .arc_end_deg=315, .N=50,
		.mag_noise=0, .acc_noise=0,
	};

	printf("=== Supported envelope (near-horizontal mount, limited arc) ===\n\n");

	struct sim_cfg c;
	double h;

	c = base; /* vertical, full sweep, no noise -> exact */
	h = run("vertical full-sweep no-noise", &c);
	expect("heading err", h < 0.05, h, 0.05);

	c = base; c.mag_noise=0.005; c.acc_noise=0.010;
	h = run("vertical full-sweep noise", &c);
	expect("heading err", h < 2.0, h, 2.0);

	c = base; c.arc_start_deg=-25; c.arc_end_deg=25; c.N=30;
	h = run("short-arc +-25 vertical no-noise", &c);
	expect("heading err", h < 0.05, h, 0.05);

	c.mag_noise=0.005; c.acc_noise=0.010;
	h = run("short-arc +-25 vertical noise", &c);
	expect("heading err", h < 6.0, h, 6.0);

	/* near-horizontal: ~8 deg mount tilt, short arc, noise */
	c = base; c.arc_start_deg=-25; c.arc_end_deg=25; c.N=30;
	c.mag_noise=0.005; c.acc_noise=0.010;
	c.rotax[0]=0; c.rotax[1]=sin(8*DEG); c.rotax[2]=cos(8*DEG);
	h = run("near-horizontal 8deg short-arc noise", &c);
	expect("heading err", h < 6.0, h, 6.0);

	/* too-few-samples returns an error */
	{
		float a[4][3]={{0,0,1},{0,0,1},{0,0,1},{0,0,1}};
		float m[4][3]={{1,0,0},{0,1,0},{-1,0,0},{0,-1,0}};
		struct cal2d_result r;
		cal2d_compute(a, m, 4, NULL, &r);
		expect("too-few-samples rejected", r.status == CAL2D_ERR_TOO_FEW, r.status, CAL2D_ERR_TOO_FEW);
	}

	printf("\n=== Informational: heading error vs mount tilt ===\n");
	printf("(short arc +-25 deg, mag noise 5mG, acc noise 10mg -- the install-site case)\n");
	for (int ai=0; ai<=30; ai+=5) {
		double a = ai*DEG;
		struct sim_cfg t = base;
		t.arc_start_deg=-25; t.arc_end_deg=25; t.N=30;
		t.mag_noise=0.005; t.acc_noise=0.010;
		t.rotax[0]=0; t.rotax[1]=sin(a); t.rotax[2]=cos(a);
		static float acc[NMAX][3], mag[NMAX][3], nv[NMAX][3];
		rng_state=12345u; simulate(&t, acc, mag, nv);
		struct cal2d_params p; cal2d_params_default(&p); p.acc_noise_rms=0.010f;
		struct cal2d_result r; cal2d_compute(acc,mag,t.N,&p,&r);
		printf("  mount tilt %2d deg -> heading error %5.2f deg\n", ai, heading_err(&r,acc,mag,nv,t.N));
	}

	printf("\n%s\n", g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
	return g_fail ? 1 : 0;
}
