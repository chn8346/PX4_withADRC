/**
 * @file RateADRC.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <LADRC.hpp>
#include <mathlib/math/Limits.hpp>
#include <cmath>
// #include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/ladrc_status.h>

class RateLADRC
{
public:
	LADRC ratex;
	LADRC ratey;
	LADRC ratez;

	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const float dt, const bool landed);

	void record_rateloop_ladrc_status(ladrc_status_s &ratex_status, ladrc_status_s &ratey_status, ladrc_status_s &ratez_status);

private:
	void record_adrc_status(ladrc_status_s &rate_status, LADRC &ladrc);
};
