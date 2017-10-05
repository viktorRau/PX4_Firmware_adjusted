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


//*****************************
//* Point Function
//*****************************
/**
 * x coordinate for point control
 */
PARAM_DEFINE_FLOAT(TP_POINT_X, 10.0);

/**
 * y coordinate for point control
 */
PARAM_DEFINE_FLOAT(TP_POINT_Y, 20.0);

/**
 * z coordinate for point control
 */
PARAM_DEFINE_FLOAT(TP_POINT_Z, 5.0);

//******************************
//* Circle Function
//******************************
/**
 * radius
 */
PARAM_DEFINE_FLOAT(TP_CIRCLE_R, 4.0);

/**
 * Periodtime
 */
PARAM_DEFINE_FLOAT(TP_CIRCLE_T, 100.0);

//******************************
//* Spiral Function
//******************************
/**
 * descent rate per circle
 */
PARAM_DEFINE_FLOAT(TP_SPIRAL_DESC, 4.0);

//******************************
//* Random Trajectory Function
//******************************
/**
 * Velocity
 */
PARAM_DEFINE_FLOAT(TP_RANDOM_V, 0.3);

/**
 * Time Step Size
 */
PARAM_DEFINE_FLOAT(TP_RANDOM_DT, 0.02);
