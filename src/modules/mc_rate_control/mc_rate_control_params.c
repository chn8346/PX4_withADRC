/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file mc_rate_control_params.c
 *
 * Parameters for multicopter rate controller
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_ROLLRATE_K * (MC_ROLLRATE_P * error
 * 			     + MC_ROLLRATE_I * error_integral
 * 			     + MC_ROLLRATE_D * error_derivative)
 * Set MC_ROLLRATE_P=1 to implement a PID in the ideal form.
 * Set MC_ROLLRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_PITCHRATE_K * (MC_PITCHRATE_P * error
 * 			     + MC_PITCHRATE_I * error_integral
 * 			     + MC_PITCHRATE_D * error_derivative)
 * Set MC_PITCHRATE_P=1 to implement a PID in the ideal form.
 * Set MC_PITCHRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_YAWRATE_K * (MC_YAWRATE_P * error
 * 			     + MC_YAWRATE_I * error_integral
 * 			     + MC_YAWRATE_D * error_derivative)
 * Set MC_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set MC_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_K, 1.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);


/**
 * Rate control method
 *
 * @value 0 PID(default)
 * @value 1 LADRC
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RATE_METHOD, 1);

/**
 * ADRC TD damping ratio for roll
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_TD_XI, 1.0f);

/**
 * ADRC TD frequency for roll
 *
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_TD_FREQ, 60.0);

/**
 * ADRC state1 feedback gain for roll
 *
 * @min 0.0100
 * @max 0.2000
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_ERR_K1, 0.08);

/**
 * ADRC state2 feedback gain for roll
 *
 * @min 0.0
 * @max 0.1
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_ERR_K2, 0.002);


/**
 * ADRC disturb gain for roll
 *
 * @min 0.00000
 * @max 100.00000
 * @decimal 6
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_DGAIN, 0.01);


/**
 * ADRC disturb max amplitude for roll
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_DMAX, 0.1);

/**
 * ADRC output max amplitude for roll
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_UMAX, 0.5);

/**
 * ADRC ESO gain for roll
 *
 * @min 1
 * @max 1e6
 * @decimal 1
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_ESO_GAIN, 10000);

/**
 * ADRC ESO bandwidth for roll
 *
 * @unit rad/s
 * @min 1
 * @max 400
 * @decimal 0
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_R_ESO_BW, 80);


/**
 * ADRC TD damping ratio for pitch
 *
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_TD_XI, 1.0);

/**
 * ADRC TD frequency for pitch
 *
 * @min 10.0
 * @max 200.0
 * @decimal 0
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_TD_FREQ, 60.0);

/**
 * ADRC state1 feedback gain for pitch
 *
 * @min 0.01
 * @max 0.2
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_ERR_K1, 0.08);

/**
 * ADRC state2 feedback gain for pitch
 *
 * @min 0.0
 * @max 0.1
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_ERR_K2, 0.002);

/**
 * ADRC disturb max amplitude for pitch
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_DMAX, 0.1);





/**
 * ADRC disturb gain for pitch
 *
 * @min 0.0
 * @max 100.0
 * @decimal 6
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_DGAIN, 0.01);


/**
 * ADRC output max amplitude for pitch
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_UMAX, 0.5);

/**
 * ADRC ESO gain for pitch
 *
 * @min 1
 * @max 1e6
 * @decimal 0
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_ESO_GAIN, 10000);

/**
 * ADRC ESO bandwidth for pitch
 *
 * @unit rad/s
 * @min 1
 * @max 400
 * @decimal 0
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_P_ESO_BW, 80);







/**
 * ADRC TD damping ratio for yaw
 *
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_TD_XI, 2.0);

/**
 * ADRC TD frequency for yaw
 *
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_TD_FREQ, 20.0);

/**
 * ADRC state1 feedback gain for yaw
 *
 * @min 0.01
 * @max 0.5
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_ERR_K1, 0.15);

/**
 * ADRC state2 feedback gain for yaw
 *
 * @min 0.0
 * @max 0.1
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_ERR_K2, 0.00);

/**
 * ADRC disturb max amplitude for yaw
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_DMAX, 0.2);




/**
 * ADRC disturb gain for yaw
 *
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_DGAIN, 0.1);




/**
 * ADRC output max amplitude for yaw
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_UMAX, 0.5);

/**
 * ADRC ESO gain for yaw
 *
 * @min 1
 * @max 1e6
 * @decimal 0
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_ESO_GAIN, 3e3);

/**
 * ADRC ESO bandwidth for yaw
 *
 * @unit rad/s
 * @min 1
 * @max 200
 * @decimal 0
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_Y_ESO_BW, 40);



