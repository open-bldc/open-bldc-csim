#include "dyn_model.h"

/* Simulator internal voltages vector. */
struct voltages_vector {
	double eu;
	double ev;
	double ew;
	double uv;
	double vv;
	double wv;
	double star;
};

double backemf(struct state_vector *sv, struct motor *m, double thetae_offset)
{
	double phase_thetae = fmod((sv->theta * (m->NbPoles / 2.)) + thetae_offset, M_PI * 2);
	double bemf_constant = vpradps_of_rpmpv(m->Kv); /* aka. ke in V/rad/s */
	double max_bemf = bemf_constant * sv->omega;
	double bemf = 0.;

	if ((0. <= phase_thetae) &&
	    (phase_thetae <= (M_PI * (1./6.)))) {
		bemf = (max_bemf / (M_PI * (1./6.))) * phase_thetae;
	} else if (((M_PI/6.) < phase_thetae) &&
		   (phase_thetae <= (M_PI * (5./6.)))) {
		bemf = max_bemf;
	} else if (((M_PI * (5./6.)) < phase_thetae) &&
		   (phase_thetae <= (M_PI * (7./6.)))) {
		bemf = -((max_bemf / (M_PI / 6.)) * (phase_thetae - M_PI));
	} else if (((M_PI * (7./6.)) < phase_thetae ) &&
		   (phase_thetae <= (math.pi * (11./6.)))) {
		bemf = -max_bemf;
	} else if (((M_PI * (11./6.)) < phase_thetae) &&
		(phase_thetae <= (2.0 * M_PI))) {
		bemf = (max_bemf/(M_PI/6.)) * (phase_thetae - (2. * M_PI));
	} else {
		printf("ERROR: angle out of bounds can not calculate bemf %f", phase_thetae);
	}

	return bemf;
}

void voltages(struct state_vector *sv, struct command_vector *cv, struct motor *m, struct voltages_vector *vv)
{
	/* Check which phases are excited. */
	bool pux = (cv->hu || cv->lu);
	bool pvx = (cv->hv || cv->lv);
	bool pwx = (cv->hw || cv->lw);

	vv->vu = 0.;
	vv->vv = 0.;
	vv->vw = 0.;
	vv->star = 0.;

	if (pux && pvx && pwx) {

		if (cv->hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		if (cv->hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		if (cv->hw) {
			vv->vw = m->VDC / 2.;
		} else {
			vv->vw = -(m->VDC / 2.);
		}

		vv->star = (vv->vu + vv->vv + vv->vw - vv->eu - vv->ev - vv->ew) / 3.;

	} else if (pux && pvx) {

		if (cv->hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		if (cv->hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		vv->star = (vv->vu + vv->vv - vv->eu - vv->ev) / 2.;

		vv->vw = vv->ew + vv->star;

	} else if (pux && pwx) {

		if (cv->hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		if (cv->hw) {
			vv->vw = m->VDC / 2.;
		} else {
			vv->vw = -(m->VDC / 2.);
		}

		vv->star = (vv->vu + vv->vw - vv->eu - vv->ew) / 2.;

		vv->vv = vv->ev + vv->star;

	} else if (pvx && pwx) {

		if (cv->hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		if (cv->hw) {
			vv->vw = m->VDC / 2.;
		} else {
			vv->vw = -(m->VDC / 2.);
		}

		vv->star = (vv->vv + vv->vw - vv->ev - vv->ew) / 2.;

		vv->vu = vv->eu + vv->star;

	} else if (pux) {

		if (cv->hu) {
			vv->vu = m->VDC / 2.;
		} else {
			vv->vu = -(m->VDC / 2.);
		}

		vv->star = (vv->vu - vv->eu);

		vv->vv = vv->ev + vv->star;
		vv->vw = vv->ew + vv->star;

	} else if (pvx) {

		if (cv->hv) {
			vv->vv = m->VDC / 2.;
		} else {
			vv->vv = -(m->VDC / 2.);
		}

		vv->star = (vv->vv - vv->ev);

		vv->vu = vv->eu + vv->star;
		vv->vw = vv->ew + vv->star;

	} else if (pwx) {

		if (cv->hw) {
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

int dyn(double t, const double asv[], double aov[], void *params)
{
	struct state_vector *sv = (struct state_vector *)asv;
	struct state_vector *sv_dot = (struct state_vector *)aov;
	struct parameters *p = (struct parameters *)params;
	double etorque, mtorque;
	struct voltages_vector vv;

	/* Calculate backemf voltages. */
	vv.eu = backemf(sv, p->m, 0.);
	vv.ev = backemf(sv, p->m, M_PI * (2. / 3.));
	vv.ew = backemf(sv, p->m, M_PI * (4. / 3.));

	/* Electromagnetic torque. */
	etorque = ((vv.eu * sv->iu) + (vv.ev * sv->iv) + (vv.ew * sv->iw)) / sv->omega;

	/* Mechanical torque. */
	mtorque = ((etorque * (p->m->NbPoles / 2)) - (p->m->damping * sv->omega) - p->pv->torque);

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
	voltages(sv, p->cv, p->m, &vv);

	/* Calculate dot currents. */
	sv_dot->iu = (vv.vu - (p->m->R * sv->iu) - vv.eu - vv.star) / (p->m->L - p->m->M);
	sv_dot->iv = (vv.vv - (p->m->R * sv->iv) - vv.ev - vv.star) / (p->m->L - p->m->M);
	sv_dot->iw = (vv.vw - (p->m->R * sv->iw) - vv.ew - vv.star) / (p->m->L - p->m->M);

	return GSL_SUCCESS;
}
