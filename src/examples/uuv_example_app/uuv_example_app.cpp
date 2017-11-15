/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file uuv_example_app.cpp
 *
 * This file let the hippocampus drive in a circle and prints the orientation as well as the acceleration data.
 * The HippoCampus is an autonomous underwater vehicle (AUV) designed by the Technical University Hamburg-Harburg (TUHH).
 * https://www.tuhh.de/mum/forschung/forschungsgebiete-und-projekte/flow-field-estimation-with-a-swarm-of-auvs.html
 *
 * @author Nils Rottann <Nils.Rottmann@tuhh.de>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

// system libraries
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
// internal libraries
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

// Include uORB and the required topics for this app
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>               // this topic holds the orientation of the hippocampus
#include <uORB/topics/vehicle_local_position.h>         // this topic gives back the local position of the vehicle
#include <uORB/topics/logging_hippocampus.h>

extern "C" __EXPORT int uuv_example_app_main(int argc, char *argv[]);

int uuv_example_app_main(int argc, char *argv[])
{
	PX4_INFO("auv_hippocampus_example_app has been started!");

	/* subscribe to vehicle_attitude topic */
	int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	/* limit the update rate to 5 Hz */
	orb_set_interval(vehicle_attitude_sub_fd, 200);

    /* subscribe to vehicle_local_position topic */
	int vehicle_local_position_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	/* limit the update rate to 5 Hz */
	orb_set_interval(vehicle_local_position_sub_fd, 200);

	/* advertise to actuator_control topic */
	struct actuator_controls_s act;
	memset(&act, 0, sizeof(act));
	orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

	/* advertise to logging_hippocampus topic */
	struct logging_hippocampus_s log;
	memset(&log, 0, sizeof(log));
	orb_advert_t log_pub = orb_advertise(ORB_ID(logging_hippocampus), &log);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = vehicle_local_position_sub_fd,   .events = POLLIN },
		{ .fd = vehicle_attitude_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;

	for (int i = 0; i < 1000; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {

				/* obtain data */
				struct vehicle_attitude_s raw_att_state;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &raw_att_state);

				/* obtain data */
				struct vehicle_local_position_s raw_pos_state;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub_fd, &raw_pos_state);

				// get current rotation matrix from control state quaternions, the quaternions are generated by the
				// attitude_estimator_q application using the sensor data
				math::Quaternion q_att(raw_att_state.q[0], raw_att_state.q[1], raw_att_state.q[2],
						       raw_att_state.q[3]);     // control_state is frequently updated
				math::Matrix<3, 3> R =
					q_att.to_dcm();     // create rotation matrix for the quaternion when post multiplying with a column vector

				// orientation vectors
				math::Vector<3> x_B(R(0, 0), R(1, 0), R(2, 0));     // orientation body x-axis (in world coordinates)
				math::Vector<3> y_B(R(0, 1), R(1, 1), R(2, 1));     // orientation body y-axis (in world coordinates)
				math::Vector<3> z_B(R(0, 2), R(1, 2), R(2, 2));     // orientation body z-axis (in world coordinates)

                // store logging data for publishing
	            log.x = raw_pos_state.x;
	            log.y = raw_pos_state.y;
	            log.z = raw_pos_state.z;

	            log.xa[0] = x_B(0);
	            log.xa[1] = x_B(1);
	            log.xa[2] = x_B(2);

	            log.ya[0] = y_B(0);
	            log.ya[1] = y_B(1);
	            log.ya[2] = y_B(2);

	            log.za[0] = z_B(0);
	            log.za[1] = z_B(1);
	            log.za[2] = z_B(2);

	            orb_publish(ORB_ID(logging_hippocampus), log_pub, &log);

			}
		}

		// Give actuator input to the HippoC, this will result in a circle
		act.control[0] = 0.0f;      // roll
		act.control[1] = 0.0f;      // pitch
		act.control[2] = 0.4f;		// yaw
		act.control[3] = 0.05f;		// thrust
		orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);

	}


	PX4_INFO("Exiting uuv_example_app!");


	return 0;
}


