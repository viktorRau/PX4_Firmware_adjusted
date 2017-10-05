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

/**
 * @file trajectory_planner_main.cpp
 * Hippocampus trajectory_planner
 *
 * based on mc_att_control_main.cpp from
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * adjusted by
 * @author Nils Rottmann    <Nils.Rottmann@tuhh.de>
 *
 * This app creates a trajectory and publishes it to the setpoint topics
 */


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <random>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
// uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/parameter_update.h>
// system libraries
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
// internal libraries
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>

#include <memory.h>


// Hippocampus trajectory_planner
extern "C" __EXPORT int trajectory_planner_main(int argc, char *argv[]);

// the class from which an instance will be initiated by starting this application
class HippocampusTrajectoryPlanner
{
public:
	// Constructor
	HippocampusTrajectoryPlanner(char *type_traj);

	// Destructor, also kills the main task
	~HippocampusTrajectoryPlanner();

	// Start the multicopter attitude control task, @return OK on success.
	int	start();

private:

	bool	_task_should_exit;		// if true, task_main() should exit
	int		_control_task;			// task handle

	// topic subscription
	int		        _params_sub;			// parameter updates subscription

	// topic publications
	orb_advert_t    _v_traj_sp_pub;

	// topic structures, in this structures the data of the topics are stored
	struct trajectory_setpoint_s	        _v_traj_sp;			// vehicle attitude setpoint

	// performance counters
	perf_counter_t	_loop_perf;
	perf_counter_t	_controller_latency_perf;

	// STOMP, declare random generator
	std::default_random_engine generator;
  	std::normal_distribution<float> distribution;
    constexpr static int rand_n = 50;
  	int rand_counter;
  	math::Matrix<rand_n,2> rand_u;
  	math::Matrix<rand_n,rand_n> Sigma;
  	math::Vector<5> act_state;

	// time counter
	float t_ges;

	// trajectory type
	char type_array[100];

	struct {
		param_t point_x;
		param_t point_y;
		param_t point_z;
		param_t circle_r;
		param_t circle_t;
		param_t spiral_desc;
		param_t random_v;
		param_t random_dt;
	}		_params_handles;		// handles for to find parameters

	struct {
		float point_x;              // parameters, they will be filled when the function parameters_update is called
		float point_y;
		float point_z;
		float circle_r;
		float circle_t;
		float spiral_desc;
		float random_v;
		float random_dt;
	}		_params;

	// path controller.
	void		plan_trajectory(float dt);

	// different trajectories
	void        point();
	void        circle();
	void        spiral();
	void        random();

	math::Vector<5> KinematicModel(math::Vector<5> state_0, math::Vector<2> u);

	// Update our local parameter cache.
	int			parameters_update();                // checks if parameters have changed and updates them

	// Check for parameter update and handle it.
	void		parameter_update_poll();            // receives parameters

	// Shim for calling task_main from task_create.
	static void	task_main_trampoline(int argc, char *argv[]);

	// Main attitude control task.
	void		task_main();
};


namespace trajectory_planner
{
HippocampusTrajectoryPlanner	*g_control;
}

// constructor of class HippocampusPathControl
HippocampusTrajectoryPlanner::HippocampusTrajectoryPlanner(char *type_traj) :

	// First part is about function which are called with the constructor
	_task_should_exit(false),
	_control_task(-1),

	// subscriptions
	_params_sub(-1),

	// publications
	_v_traj_sp_pub(nullptr),

	// performance counters
	_loop_perf(perf_alloc(PC_ELAPSED, "path_controller")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),

	// STOMP, initialize distribution
	distribution(0.0, 1.0)

// here starts the allocation of values to the variables
{
	// define publication settings
	memset(&_v_traj_sp, 0, sizeof(_v_traj_sp));

	// set parameters to zero
	_params.point_x = 0;
	_params.point_y = 0;
	_params.point_z = 0;
	_params.circle_r = 0;
	_params.circle_t = 0;
	_params.spiral_desc = 0;
	_params.random_v = 0;
	_params.random_dt = 0;

	// random initializing
	rand_counter = 0;
	rand_u.zero();
	act_state.zero();
	Sigma.zero();
	for (int i=0; i<rand_n; i++) {
	    for(int l=0; l<(i+1); l++) {
	        Sigma(i,l) = 1+i-l;
	    }
	}
	Sigma = Sigma * 0.001;

	// set time counter to zero
	t_ges = 0.0f;

	// allocate trajectory type
	strcpy(&type_array[0], type_traj);

	if (!strcmp(type_array, "point")) {
		PX4_INFO("Trajectory Planner started! Modus Point!");

	} else if (!strcmp(type_array, "circle")) {
		PX4_INFO("Trajectory Planner started! Modus Circle!");
	}

	// allocate parameter handles
	_params_handles.point_x         = 	    param_find("TP_POINT_X");
	_params_handles.point_y         = 	    param_find("TP_POINT_Y");
	_params_handles.point_z         = 	    param_find("TP_POINT_Z");
	_params_handles.circle_r        = 	    param_find("TP_CIRCLE_R");
	_params_handles.circle_t        = 	    param_find("TP_CIRCLE_T");
    _params_handles.spiral_desc     = 	    param_find("TP_SPIRAL_DESC");
    _params_handles.random_v        = 	    param_find("TP_RANDOM_V");
    _params_handles.random_dt       = 	    param_find("TP_RANDOM_DT");

	// fetch initial parameter values
	parameters_update();
}

// destructor of class HippocampusPathControl
HippocampusTrajectoryPlanner::~HippocampusTrajectoryPlanner()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	trajectory_planner::g_control = nullptr;

	PX4_INFO("Trajectory Planner stopped!");
}

// updates parameters
int HippocampusTrajectoryPlanner::parameters_update()
{
	param_get(_params_handles.point_x, &_params.point_x);
	param_get(_params_handles.point_y, &_params.point_y);
	param_get(_params_handles.point_z, &_params.point_z);
	param_get(_params_handles.circle_r, &_params.circle_r);
	param_get(_params_handles.circle_t, &_params.circle_t);
	param_get(_params_handles.spiral_desc, &_params.spiral_desc);
	param_get(_params_handles.random_v, &_params.random_v);
	param_get(_params_handles.random_dt, &_params.random_dt);

	return OK;
}

// check for parameter updates
void HippocampusTrajectoryPlanner::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

// Thius function gives back a simple point
void HippocampusTrajectoryPlanner::point()
{
	// allocate position
	_v_traj_sp.x = _params.point_x;         // x
	_v_traj_sp.y = _params.point_y;         // y
	_v_traj_sp.z = _params.point_z;         // z

	_v_traj_sp.dx = 0.0f;       // dx/dt
	_v_traj_sp.dy = 0.0f;       // dy/dt
	_v_traj_sp.dz = 0.0f;       // dz/dt

	_v_traj_sp.ddx = 0.0f;
	_v_traj_sp.ddy = 0.0f;
	_v_traj_sp.ddz = 0.0f;

	_v_traj_sp.dddx = 0.0f;
	_v_traj_sp.dddy = 0.0f;
	_v_traj_sp.dddz = 0.0f;

	_v_traj_sp.roll = 0.0f;                  // no roll angle
	_v_traj_sp.droll = 0.0f;

	// publish setpoint data
	orb_publish(ORB_ID(trajectory_setpoint), _v_traj_sp_pub, &_v_traj_sp);
}

// This function gives back a circle
void HippocampusTrajectoryPlanner::circle()
{
	// trajectory
	float T_round = _params.circle_t;           // time required for one round of the circle

	if (T_round < 1.0f) {
		T_round = 100.0;
		PX4_INFO("No valid value for TP_CIRCLE_T! Set to 100!");
	}

	float ratio = (2 * M_PI / (double)T_round);
	float r = _params.circle_r;                 // radius

	// calculate sinus and cosinus one time
	float sinus = sinf(ratio * t_ges);
	float cosinus = cosf(ratio * t_ges);

	// calculate position and their derivations
	_v_traj_sp.x = r * sinus;               // x
	_v_traj_sp.y = r - cosinus * r;         // y
	_v_traj_sp.z = 0.0f;                    // z

	_v_traj_sp.dx = ratio * cosinus * r;    // dx/dt
	_v_traj_sp.dy = ratio * sinus * r;      // dy/dt
	_v_traj_sp.dz = 0.0f;                   // dz/dt

	_v_traj_sp.ddx = -ratio * ratio * sinus * r;
	_v_traj_sp.ddy = ratio * ratio * cosinus * r;
	_v_traj_sp.ddz = 0.0f;

	_v_traj_sp.dddx = -ratio * ratio * ratio * cosinus * r;
	_v_traj_sp.dddy = -ratio * ratio * ratio * sinus * r;
	_v_traj_sp.dddz = 0.0f;

	_v_traj_sp.roll = 0.0f;                  // no roll angle
	_v_traj_sp.droll = 0.0f;

	// publish setpoint data
	orb_publish(ORB_ID(trajectory_setpoint), _v_traj_sp_pub, &_v_traj_sp);

}

// This function gives back a down spiral
void HippocampusTrajectoryPlanner::spiral()
{
	// trajectory
	float T_round = _params.circle_t;           // time required for one round of the circle
	float desc = _params.spiral_desc;

	if (T_round < 1.0f) {
		T_round = 100.0;
		PX4_INFO("No valid value for TP_CIRCLE_T! Set to 100!");
	}

	float ratio = (2 * M_PI / (double)T_round);
	float r = _params.circle_r;                 // radius

	// calculate sinus and cosinus one time
	float sinus = sinf(ratio * t_ges);
	float cosinus = cosf(ratio * t_ges);

	// calculate position and their derivations
	_v_traj_sp.x = r * sinus;               // x
	_v_traj_sp.y = r - cosinus * r;         // y
	_v_traj_sp.z = desc * (t_ges/T_round);  // z

	_v_traj_sp.dx = ratio * cosinus * r;    // dx/dt
	_v_traj_sp.dy = ratio * sinus * r;      // dy/dt
	_v_traj_sp.dz = desc/T_round;           // dz/dt

	_v_traj_sp.ddx = -ratio * ratio * sinus * r;
	_v_traj_sp.ddy = ratio * ratio * cosinus * r;
	_v_traj_sp.ddz = 0.0f;

	_v_traj_sp.dddx = -ratio * ratio * ratio * cosinus * r;
	_v_traj_sp.dddy = -ratio * ratio * ratio * sinus * r;
	_v_traj_sp.dddz = 0.0f;

	_v_traj_sp.roll = 0.0f;                  // no roll angle
	_v_traj_sp.droll = 0.0f;

	// publish setpoint data
	orb_publish(ORB_ID(trajectory_setpoint), _v_traj_sp_pub, &_v_traj_sp);

}

void HippocampusTrajectoryPlanner::random()
{
    // parameter
    float rand_v = _params.random_v;

    if (rand_counter == 0) {
        // Generate Random Input Samples
		for (int i=0; i < rand_n; i++) {
            rand_u(i, 0) = distribution(generator);
		    rand_u(i, 1) = distribution(generator);
		}
		rand_u = Sigma*rand_u;		// Generate Mutlivariate Normal Distribution
    }

    act_state = KinematicModel(act_state, rand_u.get_rowValues(rand_counter));
    rand_counter = rand_counter + 1;
    if (rand_counter > (rand_n - 5)) {
        rand_counter = 0;
    }

    float phi = act_state(3);
    float psi = act_state(4);

    // calculate position and their derivations
	_v_traj_sp.x = act_state(0);    // x
	_v_traj_sp.y = act_state(1);    // y
	_v_traj_sp.z = act_state(2);    // z

	_v_traj_sp.dx = cosf(psi) * cosf(phi) * rand_v;    // dx/dt
	_v_traj_sp.dy = sinf(phi) * rand_v;                // dy/dt
	_v_traj_sp.dz = -sinf(psi) * cosf(phi) * rand_v;   // dz/dt

	_v_traj_sp.ddx = 0.0f;
	_v_traj_sp.ddy = 0.0f;
	_v_traj_sp.ddz = 0.0f;

	_v_traj_sp.dddx = 0.0f;
	_v_traj_sp.dddy = 0.0f;
	_v_traj_sp.dddz = 0.0f;

	_v_traj_sp.roll = 0.0f;                  // no roll angle
	_v_traj_sp.droll = 0.0f;

    // publish setpoint data
	orb_publish(ORB_ID(trajectory_setpoint), _v_traj_sp_pub, &_v_traj_sp);
}

// Kinematic Model for random trajectory generator
math::Vector<5> HippocampusTrajectoryPlanner::KinematicModel(math::Vector<5> state_0, math::Vector<2> u)
{
    // Parameter
    float rand_v = _params.random_v;
    float rand_dt = _params.random_dt;

	math::Vector<5> state_1;

	state_1(3) = state_0(3) + u(0) * rand_dt;		// update phi
	state_1(4) = state_0(4) + u(1) * rand_dt;		// update psi


	if (fabs(u(0)) < 0.001 && fabs(u(1)) < 0.001) {
		state_1(0) = state_0(0) + cosf(state_0(4))*cosf(state_0(3))*rand_dt*rand_v;
		state_1(1) = state_0(1) + sinf(state_0(3))*rand_dt*rand_v;
		state_1(2) = state_0(2) - sinf(state_0(4))*cosf(state_0(3))*rand_dt*rand_v;
	}
	else {
		float uu = u(0) + u(1);
		float u_12 = u(0) - u(1);
		float u_21 = u(1) - u(0);

		// update x coordinate
		float sin_min_1 = sinf(state_1(3) - state_1(4));
		float sin_min_0 = sinf(state_0(3) - state_0(4));
		float sin_plus_1 = sinf(state_1(3) + state_1(4));
		float sin_plus_0 = sinf(state_0(3) + state_0(4));
		state_1(0) = state_0(0) + (rand_v / 2) * ((sin_min_1 - sin_min_0)/u_12 + (sin_plus_1-sin_plus_0)/uu);

		// update y coordinate
		state_1(1) = state_0(1) + (rand_v/u(0)) * (cosf(state_0(3)) - cosf(state_1(3)));

		// update z coordinate
		float cos_min_1 = cosf(state_1(4) - state_1(3));
		float cos_min_0 = cosf(state_0(4) - state_0(3));
		float cos_plus_1 = cosf(state_1(3) + state_1(4));
		float cos_plus_0 = cosf(state_0(3) + state_0(4));
		state_1(2) = state_0(2) + (rand_v / 2) * ((cos_min_1 - cos_min_0)/u_21 + (cos_plus_1-cos_plus_0)/uu);
	}

	return state_1;

}


// This function calls the chosen trajectory function
void HippocampusTrajectoryPlanner::plan_trajectory(float dt)
{

	t_ges = t_ges + dt;                     // add time passed

	parameters_update();                    // update parameter

	if (!strcmp(type_array, "point")) {
		point();
	} else if (!strcmp(type_array, "circle")) {
		circle();
	} else if (!strcmp(type_array, "spiral")) {
		spiral();
	} else if (!strcmp(type_array, "random")) {
		random();
	}
}

// Just starts the task_main function
void HippocampusTrajectoryPlanner::task_main_trampoline(int argc, char *argv[])
{
	trajectory_planner::g_control->task_main();
}

// this is the main_task function which does the control task
void HippocampusTrajectoryPlanner::task_main()
{
	// subscriber
	_params_sub = orb_subscribe(ORB_ID(parameter_update));              // initialize parameter update

	// publisher
	_v_traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &_v_traj_sp);

	// initialize parameters cache
	parameters_update();

	while (!_task_should_exit) {

		perf_begin(_loop_perf);


		static uint64_t last_run = 0;
		float dt = (hrt_absolute_time() - last_run) / 1000000.0f;   // calculate the time delta_t between two runs
		last_run = hrt_absolute_time();

		// do trajectory_planning
		plan_trajectory(dt);

		// check for parameter updates
		parameter_update_poll();

		// wait for a 10ms
		usleep(10000);

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

// start function
int HippocampusTrajectoryPlanner::start()
{
	ASSERT(_control_task == -1);        // give a value -1

	// start the control task, performs any specific accounting, scheduler setup, etc.
	_control_task = px4_task_spawn_cmd("trajectory_planner",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&HippocampusTrajectoryPlanner::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

// main function
int trajectory_planner_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: trajectory_planner {point|circle|spiral|random|stop|status}");
		return 1;
	}

	// if a valid trajectory is chosen, only generate new class if none already exists
	if (!strcmp(argv[1], "point") || !strcmp(argv[1], "circle") || !strcmp(argv[1], "spiral") || !strcmp(argv[1], "random")) {


		if (trajectory_planner::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		// allocate new class HippocampusTrajectoryPlanner
		trajectory_planner::g_control = new HippocampusTrajectoryPlanner(argv[1]);

		// check if class has been allocated, if not, give back failure
		if (trajectory_planner::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		// if function start() can not be called, delete instance of HippocampusTrajectoryPlanner and allocate null pointer
		if (OK != trajectory_planner::g_control->start()) {
			delete trajectory_planner::g_control;
			trajectory_planner::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	// if command is start, check if class exists, if not can not stop anything
	if (!strcmp(argv[1], "stop")) {
		if (trajectory_planner::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		// if class exists, delete it and allocate null pointer
		delete trajectory_planner::g_control;
		trajectory_planner::g_control = nullptr;
		return 0;
	}

	// if command is status and class exists, give back running, else give back not running
	if (!strcmp(argv[1], "status")) {
		if (trajectory_planner::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}