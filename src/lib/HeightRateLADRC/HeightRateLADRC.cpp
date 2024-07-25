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
 * @file HeightRateLADRC.cpp
 *
 * LADRC height velocity control.
 */


#include <HeightRateLADRC.hpp>
#include <px4_platform_common/defines.h>
#include <mathlib/math/Limits.hpp>
#include <drivers/drv_hrt.h>

using namespace matrix;

double HeightRateLADRC::update(const double &rate, const double &rate_sp,
				const float dt, const bool landed)
{
	double torque;

	torque = height_rate_controller.update(rate_sp, rate, dt);

	if (landed) {
		height_rate_controller.reset();
	}

	return torque;
}

void HeightRateLADRC::record_adrc_status(ladrc_status_s &height_ladrc_status, ns_adrc_height_::LADRC_hv &ladrc)
{
	height_ladrc_status.timestamp = hrt_absolute_time();

	height_ladrc_status.v = ladrc.td.get_input_signal();
	height_ladrc_status.v1 = ladrc.td.get_tracking_signal();
	height_ladrc_status.v2 = ladrc.td.get_differential_signal();

	height_ladrc_status.e1 = ladrc.ec.get_error1();
	height_ladrc_status.e2 = ladrc.ec.get_error2();
	height_ladrc_status.u0 = ladrc.ec.get_error_combine();
	height_ladrc_status.u  = ladrc.ec.get_ladrc_output();

	height_ladrc_status.y = ladrc.eso.get_eso_reference();
	height_ladrc_status.z1 = ladrc.eso.get_eso_state1();
	height_ladrc_status.z2 = ladrc.eso.get_eso_state2();
	height_ladrc_status.z3 = ladrc.eso.get_eso_state3();
	height_ladrc_status.disturb = ladrc.eso.get_eso_disturb();
}

void HeightRateLADRC::record_rateloop_ladrc_status(ladrc_status_s &height_ladrc_status)
{
	record_adrc_status(height_ladrc_status, height_rate_controller);
}

