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
 * @file path_controller_params.c
 * Parameters for path controller for the Hippocampus.
 *
 * @author Nils Rottmann <Nils.Rottmann@tuhh.de>
 */


/**
 * K_P proportional position gain
 *
 * Factor for the position error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_PX, 1.0f);
PARAM_DEFINE_FLOAT(PC_K_PY, 1.0f);
PARAM_DEFINE_FLOAT(PC_K_PZ, 1.0f);

/**
 * K_V derivative position gain
 *
 * Factor for the position error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_VX, 1.0f);
PARAM_DEFINE_FLOAT(PC_K_VY, 1.0f);
PARAM_DEFINE_FLOAT(PC_K_VZ, 1.0f);

/**
 * K_P proportional angel gain
 *
 * Factor for the angular error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_RX, 0.0f);
PARAM_DEFINE_FLOAT(PC_K_RY, 1.0f);
PARAM_DEFINE_FLOAT(PC_K_RZ, 1.0f);

/**
 * K_W derivative angel gain
 *
 * Factor for the angel velocity error.
 *
 */
PARAM_DEFINE_FLOAT(PC_K_WX, 1.0f);
PARAM_DEFINE_FLOAT(PC_K_WY, 1.0f);
PARAM_DEFINE_FLOAT(PC_K_WZ, 1.0f);

/**
 * Mass + added Mass of the Hippocampus
 *
 */
PARAM_DEFINE_FLOAT(PC_m, 3.0f);

/**
 * Damping X-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_X_u, 5.39f);

/**
 * Damping Y-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_Y_v, 17.36f);

/**
 * Damping Z-direction
 *
 */
PARAM_DEFINE_FLOAT(PC_Z_w, 17.36f);

/**
 * Force scaling constant, should be the same as given in the simulation
 *
 */
PARAM_DEFINE_FLOAT(PC_K_F, 3.0f);

/**
 * Moment scaling constant, should be the same as given in the simulation
 *
 */
PARAM_DEFINE_FLOAT(PC_K_M, 0.02f);

/**
 * lifting arm for the Forces to generate Moments in pitch and yaw direction
 *
 */
PARAM_DEFINE_FLOAT(PC_L, 0.0481f);

/**
 * Operating Grade
 *
 */
PARAM_DEFINE_FLOAT(PC_OG, 1.0f);

/**
 * Desired Angle Roll
 *
 */
PARAM_DEFINE_FLOAT(PC_ROLL, 0.0f);

/**
 * Desired Angle Pitch
 *
 */
PARAM_DEFINE_FLOAT(PC_PITCH, 1.5f);

/**
 * Desired Angle Yaw
 *
 */
PARAM_DEFINE_FLOAT(PC_YAW, 0.0f);

