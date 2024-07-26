/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Height control method
 *
 * @value 0 PID(default)
 * @value 1 LADRC
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_H_CTRL_METHOD, 0);

/**
 * ADRC Control amplify
 * @min 1.0
 * @max 1000.0
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_AMP, 50.0);

/**
 * ADRC TD damping ratio for Height Control
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_TD_XI, 1.0f);

/**
 * ADRC TD frequency for Height Control
 *
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_TD_FREQ, 60.0);

/**
 * ADRC state1 feedback gain for Height Control
 *
 * @min 0.0100
 * @max 0.2000
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_ERR_K1, 0.08);

/**
 * ADRC state2 feedback gain for Height Control
 *
 * @min 0.0
 * @max 0.1
 * @decimal 4
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_ERR_K2, 0.002);


/**
 * ADRC disturb gain for Height Control
 *
 * @min 0.00000
 * @max 100.00000
 * @decimal 6
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_DGAIN, 0.01);


/**
 * ADRC disturb max amplitude for Height Control
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_DMAX, 0.1);

/**
 * ADRC output max amplitude for Height Control
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_UMAX, 0.5);

/**
 * ADRC ESO gain for Height Control
 *
 * @min 1
 * @max 1e6
 * @decimal 1
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_ESO_GAIN, 10000);

/**
 * ADRC ESO bandwidth for Height Control
 *
 * @unit rad/s
 * @min 1
 * @max 400
 * @decimal 0
 * @group Multicopter ADRC Control
 */
PARAM_DEFINE_FLOAT(ADRC_H_ESO_BW, 80);

