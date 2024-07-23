/**
 * @file LADRC_height_velo.hpp
 *
 * LADRC height velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <drivers/drv_hrt.h>

namespace ns_adrc_height_{

class TD{
public:
	TD() {reset();};
	~TD() {};

	void set_td_coef(const float a0, const float a1)
	{
		_a0 = a0;
		_a1 = a1;
	}
	void set_td_ratio_frequency(const float xi, const float wn)
	{
		_a0 = wn*wn;
		_a1 = 2.0f * xi * wn;
	}
	void update(const float input, const float dt)
	{
		v = input;
		v1 += dt * v2;
		v2 += dt * (_a0 * (v - v1) - _a1 * v2);
	}

	float get_input_signal(void)
	{
		return v;
	}
	float get_tracking_signal(void)
	{
		return v1;
	}
	float get_differential_signal(void)
	{
		return v2;
	}

	void reset()
	{
		v = v1 = v2 = 0.0f;
	}

private:
	float v = 0.0f;
	float v1 = 0.0f;
	float v2 = 0.0f;

	float _a0 = 0.0f;
	float _a1 = 0.0f;
}; // class TD

class ErrorCombiner_h
{
public:
	ErrorCombiner_h() {};
	~ErrorCombiner_h() {};

	void set_error_combiner_coef(const float beta1, const float beta2)
	{
		_beta1 = beta1;
		_beta2 = beta2;
	}

	void set_output_limit(const float umin, const float umax)
	{
		_u_min = umin;
		_u_max = umax;
	}

	void set_distrub_gain(const float gain)
	{
		_disturb_gain = gain;
	}

	float update(const float v1d, const float v2d, const float z1, const float z2, const float disturb)
	{
		e1 = v1d - z1;
		e2 = v2d - z2;
		u0 = _beta1 * e1 + _beta2 * e2;
		u = u0 + _disturb_gain * disturb;
		u = math::constrain(u, _u_min, _u_max);
		return u;
	}

	void reset()
	{
		u0 = 0.0f;
		u = 0.0f;
		e1 = 0.0f;
		e2 = 0.0f;
	}

	float get_error1(void)
	{
		return e1;
	}
	float get_error2(void)
	{
		return e2;
	}
	float get_error_combine(void)
	{
		return u0;
	}
	float get_ladrc_output(void)
	{
		return u;
	}
private:
	float u0 = 0.0f;
	float u = 0.0f;
	float e1 = 0.0f;
	float e2 = 0.0f;

	float _u_min = 0.0f;
	float _u_max = 0.0f;

	float _beta1 = 0.0f;
	float _beta2 = 0.0f;

	float _disturb_gain = 1.0f;
}; // class ErrorCombiner_h

class ESO_h
{
public:
	ESO_h() {};
	~ESO_h() {};

	void set_eso_coef(const float b0, const float beta01, const float beta02, const float beta03)
	{
		_b0 = b0;
		_beta01 = beta01;
		_beta02 = beta02;
		_beta03 = beta03;
	}

	void set_eso_gain_cutoff_frequency(const float b0, const float wc)
	{
		_b0 = b0;
		_beta01 = 3.0f * wc;
		_beta02 = 3.0f * wc * wc;
		_beta03 = wc * wc * wc;
	}

	void set_disturb_limit(const float dmin, const float dmax)
	{
		_disturb_min = dmin;
		_disturb_max = dmax;
	}

	void update(const float u, const float y_, const float dt)
	{
		y = y_;
		const float ey = z1 - y;
		z1 += dt * (z2 - _beta01 * ey);
		z2 += dt * (z3 + _b0 * u - _beta02 * ey);
		z3 += dt * (-_beta03 * ey);

		distrub = -z3 / _b0;
		distrub = math::constrain(distrub, _disturb_min, _disturb_max);
	}

	float get_eso_reference(void)
	{
		return y;
	}
	float get_eso_state1(void)
	{
		return z1;
	}
	float get_eso_state2(void)
	{
		return z2;
	}
	float get_eso_state3(void)
	{
		return z3;
	}

	float get_eso_disturb()
	{
		return distrub;
	}

	void reset()
	{
		y = 0.0f;
		z1 = z2 = z3 = 0.0f;
		distrub = 0.0f;
	}

private:
	float y = 0.0f;
	float z1 = 0.0f;
	float z2 = 0.0f;
	float z3 = 0.0f;
	float distrub = 0.0f;

	float _disturb_min = 0.0f;
	float _disturb_max = 0.0f;

	float _b0 = 1e5f;

	float _beta01 = 0.0f;
	float _beta02 = 0.0f;
	float _beta03 = 0.0f;
}; // class ESO_h

class LADRC_hv
{
public:
	LADRC_hv() = default;
	~LADRC_hv() = default;

	TD 		td;
	ErrorCombiner_h ec;
	ESO_h 		eso;

	float update(const float v, const float y, const float dt)
	{
		td.update(v, dt);

		const float v1 = td.get_tracking_signal();
		const float v2 = td.get_differential_signal();

		const float z1 = eso.get_eso_state1();
		const float z2 = eso.get_eso_state2();
		const float disturb = eso.get_eso_disturb();

		const float u = ec.update(v1, v2, z1, z2, disturb);

		eso.update(u, y, dt);

		return u;
	}

	void reset()
	{
		td.reset();
		ec.reset();
		eso.reset();
	}
}; // class LADRC_hv

} // namespace ns_adrc_height_
