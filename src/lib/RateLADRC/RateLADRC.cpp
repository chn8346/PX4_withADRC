/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RateADRC.cpp
 */

#include <RateLADRC.hpp>
#include <px4_platform_common/defines.h>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

Vector3f RateLADRC::update(const Vector3f &rate, const Vector3f &rate_sp, const float dt, const bool landed)
{
	Vector3f torque;


	torque(0) = ratex.update(rate_sp(0), rate(0), dt);
	torque(1) = ratey.update(rate_sp(1), rate(1), dt);
	torque(2) = ratez.update(rate_sp(2), rate(2), dt);

	if (landed) {
		ratex.reset();
		ratey.reset();
		ratez.reset();
	}

	return torque;
}

void RateLADRC::record_adrc_status(ladrc_status_s &rate_status, LADRC &ladrc)
{
	rate_status.timestamp = hrt_absolute_time();

	rate_status.v = ladrc.td.get_input_signal();
	rate_status.v1 = ladrc.td.get_tracking_signal();
	rate_status.v2 = ladrc.td.get_differential_signal();

	rate_status.e1 = ladrc.ec.get_error1();
	rate_status.e2 = ladrc.ec.get_error2();
	rate_status.u0 = ladrc.ec.get_error_combine();
	rate_status.u  = ladrc.ec.get_ladrc_output();

	rate_status.y = ladrc.eso.get_eso_reference();
	rate_status.z1 = ladrc.eso.get_eso_state1();
	rate_status.z2 = ladrc.eso.get_eso_state2();
	rate_status.z3 = ladrc.eso.get_eso_state3();
	rate_status.disturb = ladrc.eso.get_eso_disturb();
}

void RateLADRC::record_rateloop_ladrc_status(ladrc_status_s &ratex_status, ladrc_status_s &ratey_status, ladrc_status_s &ratez_status)
{
	record_adrc_status(ratex_status, ratex);
	record_adrc_status(ratey_status, ratey);
	record_adrc_status(ratez_status, ratez);

}

