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

#ifndef _DYN_MODEL_H
#define _DYN_MODEL_H

/* Motor parameter struct. */
struct motor {
	double inertia;  /* aka 'J' in kg/(m^2) */
	double damping;  /* aka 'B' in Nm/(rad/s) */
	double static_friction; /* in Nm */
	double Kv;       /* motor constant in RPM/V */
	double L;        /* Coil inductance in H */
	double M;        /* Mutual coil inductance in H */
	double R;        /* Coil resistence in Ohm */
	double VDC;      /* Supply voltage */
	int NbPoles;     /* NbPoles / 2 = Number of pole pairs (you count the permanent magnets on the rotor to get NbPoles) */
};

/* State vector struct. */
struct state_vector {
	double theta; /* angle of the rotor */
	double omega; /* angular speed of the rotor */
	double iu;    /* phase u current */
	double iv;    /* phase v current */
	double iw;    /* phase w current */
};

/* Command vector struct. */
struct command_vector {
	bool hu; /* low side phase U switch */
	bool lu; /* high side phase U switch */
	bool hv; /* low side phase V switch */
	bool lv; /* high side phase V switch */
	bool hw; /* low side phase W switch */
	bool lw; /* high side phase W switch */
};

/* Perturbation vector struct. */
struct perturbation_vector {
	double torque; /* Torque applied to the shaft of the motor. */
};

/* Output vector struct. */
struct output_vector {
	double iu;    /* Phase U current. */
	double iv;    /* Phase V current. */
	double iw;    /* Phase W current. */
	double vu;    /* Phase U voltage. */
	double vv;    /* Phase V voltage. */
	double vw;    /* Phase W voltage. */
	double theta; /* Rotor angle. */
	double omega; /* Rotor speed. */
};

/* Parameter struct. */
struct parameters {
	struct motor *m;
	struct command_vector *cv;
	struct perturbation_vector *pv;
};

void init_state(struct state_vector *sv);
int dyn(double t, const double asv[], double aov[], void *params);

#endif /* _DYN_MODEL_H */
