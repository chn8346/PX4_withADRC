/**
 * @file HeightRateLADRC.hpp
 *
 * LADRC height velocity control.
 */

#pragma once

#include "LADRC_height_velo.hpp"
#include <mathlib/math/Limits.hpp>
#include <cmath>
// #include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/ladrc_status.h>

class HeightRateLADRC
{
public:
	ns_adrc_height_::LADRC_hv height_rate_controller;

	double update(const double &rate, const double &rate_sp,
			const float dt, const bool landed);

	void record_rateloop_ladrc_status(ladrc_status_s &height_ladrc_status);

private:
	void record_adrc_status(ladrc_status_s &height_ladrc_status, ns_adrc_height_::LADRC_hv &ladrc);
};
