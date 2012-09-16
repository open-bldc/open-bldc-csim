  /*
 * Open-BLDC CSim - Open BrushLess DC Motor Controller C Simulator
 * Copyright (C) 2012 by Piotr Esden-Tempski <piotr@esden.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <math.h>
#include <gsl/gsl_errno.h>

#include "dyn_model.h"

#include "misc_utils.h"
#include "switch.h"

#ifndef DEBUG
//#define DEBUG
#endif

/* Simulator internal voltages vector. */
struct voltages_vector {
	double eu;
	double ev;
	double ew;
	double vu;
	double vv;
	double vw;
	double star;
};

int backemf(struct state_vector *sv, struct motor *m, double thetae_offset, double *bemf)
{
	double phase_thetae = norm_angle((sv->theta * (m->NbPoles / 2.)) + thetae_offset);
	double bemf_constant = vpradps_of_rpmpv(m->Kv); /* aka. ke in V/rad/s */
	double max_bemf = bemf_constant * sv->omega;
	int slice = angle_slice(phase_thetae, 12);
	*bemf = 0.;

	switch (slice) {
	case 0:
		*bemf = (max_bemf / (M_PI * (1./6.))) * phase_thetae;
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		*bemf = max_bemf;
		break;
	case 5:
	case 6:
		*bemf = -((max_bemf / (M_PI * (1./6.))) * (phase_thetae - M_PI));
		break;
	case 7:
	case 8:
	case 9:
	case 10:
		*bemf = -max_bemf;
		break;
	case 11:
		*bemf = (max_bemf / (M_PI * (1./6.))) * (phase_thetae - (2. * M_PI));
		break;
	default:
		printf("ERROR: angle out of bounds can not calculate bemf %f\n", phase_thetae);
		return GSL_FAILURE;
	}

#ifdef DEBUG
	printf("DEBUG: alpha %.5e bemf %.5e\n", phase_thetae, *bemf);
#endif

	return GSL_SUCCESS;
}

void voltages(double t, struct state_vector *sv, struct command_vector *cv, struct motor *m, struct voltages_vector *vv)
{
	/* Check which phases are excited. */

	bool hu = switch_get(&(cv->hu), t);
	bool lu = switch_get(&(cv->lu), t);
	bool hv = switch_get(&(cv->hv), t);
	bool lv = switch_get(&(cv->lv), t);
	bool hw = switch_get(&(cv->hw), t);
	bool lw = switch_get(&(cv->lw), t);

	bool pux = (hu || lu);
	bool pvx = (hv || lv);
	bool pwx = (hw || lw);

	vv->vu = 0.;
	vv->vv = 0.;
	vv->vw = 0.;
	vv->star = 0.;

	if (pux && pvx && pwx) {

		if (hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		if (hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		if (hw) {
			vv->vw = m->VDC / 2.;
		} else {
			vv->vw = -(m->VDC / 2.);
		}

		vv->star = (vv->vu + vv->vv + vv->vw - vv->eu - vv->ev - vv->ew) / 3.;

	} else if (pux && pvx) {

		if (hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		if (hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		vv->star = (vv->vu + vv->vv - vv->eu - vv->ev) / 2.;

		vv->vw = vv->ew + vv->star;

	} else if (pux && pwx) {

		if (hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		if (hw) {
			vv->vw = m->VDC / 2.;
		} else {
			vv->vw = -(m->VDC / 2.);
		}

		vv->star = (vv->vu + vv->vw - vv->eu - vv->ew) / 2.;

		vv->vv = vv->ev + vv->star;

	} else if (pvx && pwx) {

		if (hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		if (hw) {
			vv->vw = m->VDC / 2.;
		} else {
			vv->vw = -(m->VDC / 2.);
		}

		vv->star = (vv->vv + vv->vw - vv->ev - vv->ew) / 2.;

		vv->vu = vv->eu + vv->star;

	} else if (pux) {

		if (hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		vv->star = (vv->vu - vv->eu);

		vv->vv = vv->ev + vv->star;
		vv->vw = vv->ew + vv->star;

	} else if (pvx) {

		if (hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		vv->star = (vv->vv - vv->ev);

		vv->vu = vv->eu + vv->star;
		vv->vw = vv->ew + vv->star;

	} else if (pwx) {

		if (hw) {
			vv->vw = m->VDC / 2.;
		} else {
			vv->vw = -(m->VDC / 2.);
		}

		vv->star = (vv->vw - vv->ew);

		vv->vu = vv->eu + vv->star;
		vv->vv = vv->ev + vv->star;

	} else {
		/* XXX: this needs more precise checking I guess. :/ */
		vv->vu = vv->eu;
		vv->vv = vv->ev;
		vv->vw = vv->ew;

		vv->star = 0;
	}
}

void init_state(struct state_vector *sv)
{
  sv->iu = 0;
  sv->iv = 0;
  sv->iw = 0;
  sv->theta = 0;
  sv->omega = 0.000001;
}

int dyn(double t, const double asv[], double aov[], void *params)
{
	struct state_vector *sv = (struct state_vector *)asv;
	struct state_vector *sv_dot = (struct state_vector *)aov;
	struct parameters *p = (struct parameters *)params;
	double etorque, mtorque;
	struct voltages_vector vv;
	int ret;

#ifdef DEBUG
	printf("DEBUG: input t %.5e iu %.5e iv %.5e ew %5e th %5e om %5e\n", t, sv->iu, sv->iv, sv->iw, sv->theta, sv->omega);
#endif

	/* Calculate backemf voltages. */
	ret = backemf(sv, p->m, 0., &vv.eu);
	if (ret != GSL_SUCCESS) return ret;
	ret = backemf(sv, p->m, M_PI * (2. / 3.), &vv.ev);
	if (ret != GSL_SUCCESS) return ret;
	ret = backemf(sv, p->m, M_PI * (4. / 3.), &vv.ew);
	if (ret != GSL_SUCCESS) return ret;

#ifdef DEBUG
	printf("DEBUG: bemf %.5e %.5e %.5e\n", vv.eu, vv.ev, vv.ew);
#endif

	/* Electromagnetic torque. */
	if (sv->omega == 0) {
		printf("ERROR: input state vector omega equals 0!!!\n");
		return GSL_FAILURE;
	}
	etorque = ((vv.eu * sv->iu) + (vv.ev * sv->iv) + (vv.ew * sv->iw)) / sv->omega;

#ifdef DEBUG
	printf("DEBUG: etorque %.5e\n", etorque);
#endif

	/* Mechanical torque. */
	mtorque = ((etorque * (p->m->NbPoles / 2)) - (p->m->damping * sv->omega) - p->pv->torque);

#ifdef DEBUG
	printf("DEBUG: damping %.5e omega %.5e torque %.5e mtorque %.5e\n", p->m->damping, sv->omega, p->pv->torque, mtorque);
#endif

	if ((mtorque > 0) && (mtorque <= p->m->static_friction)) {
		mtorque = 0;
	} else if (mtorque > p->m->static_friction) {
		mtorque -= p->m->static_friction;
	} else if ((mtorque < 0) && (mtorque >= -(p->m->static_friction))) {
		mtorque = 0;
	} else if (mtorque < -(p->m->static_friction)) {
		mtorque += p->m->static_friction;
	}

	/* Position of the rotor */
	sv_dot->theta = sv->omega;

	/* Acceleration of the rotor. (omega_dot) */
	sv_dot->omega = mtorque / p->m->inertia;

	/* Calculate voltages. */
	voltages(t, sv, p->cv, p->m, &vv);

	/* Calculate dot currents. */
	sv_dot->iu = (vv.vu - (p->m->R * sv->iu) - vv.eu - vv.star) / (p->m->L - p->m->M);
	sv_dot->iv = (vv.vv - (p->m->R * sv->iv) - vv.ev - vv.star) / (p->m->L - p->m->M);
	sv_dot->iw = (vv.vw - (p->m->R * sv->iw) - vv.ew - vv.star) / (p->m->L - p->m->M);

#ifdef DEBUG
	printf("DEBUG: output %.5e %.5e %.5e %5e %5e %5e\n", t, sv_dot->iu, sv_dot->iv, sv_dot->iw, sv_dot->theta, sv_dot->omega);
#endif

	return GSL_SUCCESS;
}
