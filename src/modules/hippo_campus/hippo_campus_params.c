/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
#include <systemlib/param/param.h>

/**
 * @file hippo_campus_params.c
 *
 * Choose between set of setpoints
 *
 *
 */


/**
 * Roll D Gain
 *
 * Choose between set of setpoints
 * 0: NESW
 * 1: Box
 * 2: Helix
 *
 */
PARAM_DEFINE_INT32(HC_MODE, 0);


/**
 * Thrust gain
 *
 * for simulation choose 0.1f
 * for hippoc choose 1.0f
 *
 */
PARAM_DEFINE_FLOAT(HC_T_SCALE, 1.0f);

/**
 * Manover time
 *
 * time a manover should be performed
 * or time to wait between setpoints
 *
 */
PARAM_DEFINE_INT32(HC_M_TIME, 5);

/**
 * Wait for setoint
 *
 * 0: dont wait until setpoint is reached (use HC_M_TIME)
 * 1: wait until setpoint is reached
 *
 */
PARAM_DEFINE_INT32(HC_WAIT, 1);


/**
 * Yaw Setpoint
 * for use with control mode 5
 *
 */
PARAM_DEFINE_INT32(HC_YAW, 0);

/**
 * Pitch Setpoint
 * for use with control mode 5
 *
 */
PARAM_DEFINE_INT32(HC_PITCH, 0);

/**
 * Roll Setpoint
 * for use with control mode 5
 *
 */
PARAM_DEFINE_INT32(HC_ROLL, 0);

