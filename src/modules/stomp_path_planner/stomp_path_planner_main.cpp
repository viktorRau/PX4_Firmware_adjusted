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
 * @file stomp_path_planner_main.cpp
 * Hippocampus path controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * based on mc_att_control_main.cpp from
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * adjusted by
 * @author Nils Rottmann    <Nils.Rottmann@tuhh.de>
 *
 * The controller has two loops: P loop for position and angular error and PD loop for velocity and angular rate error.
 */


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <random>					// include c++ standard library in order to generate random numbers
#include <iostream>
#include <fstream>
// uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/logging_hippocampus.h>            // logging message to have everything in one at the same time steps
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


// Hippocampus path controller
extern "C" __EXPORT int stomp_path_planner_main(int argc, char *argv[]);

// the class from which an instance will be initiated by starting this application
class StompPathPlanner
{
public:
	// Constructor
	StompPathPlanner();

	// Destructor, also kills the main task
	~StompPathPlanner();

	// Start the multicopter attitude control task, @return OK on success.
	int	start();

private:

	bool	_task_should_exit;		// if true, task_main() should exit
	int		_control_task;			// task handle

	// topic subscriptions
	int		_v_att_sub;		// attitude and position subscription
	int     _v_pos_sub;
	int		_params_sub;			// parameter updates subscription

    // topic publications
	orb_advert_t    _v_traj_sp_pub;

	// topic structures, in this structures the data of the topics are stored
	struct vehicle_attitude_s			_v_att;		            // vehicle attitude
	struct vehicle_local_position_s	    _v_pos;		            // vehicle position
	struct trajectory_setpoint_s	    _v_traj_sp;			    // trajectory setpoint

	// performance counters
	perf_counter_t	_loop_perf;
	perf_counter_t	_controller_latency_perf;

	// time
	float               t_ges;
	float               time_counter;
	float               time_limit;
	float               t_zero;

	// goal position
	math::Vector<3> goal_position;

	// true if process has to be started new
	bool prc_new;

	// Number of Estimation Steps and of Iterations
	constexpr static int N_STOMP = 30;
	constexpr static int K_STOMP = 50;
	constexpr static int iter_STOMP = 50;

    // Allocate Space for generated Trajectory
    math::Matrix<N_STOMP+1, 6> gen_trajectory;         // the sixth entry holds the timestamps

	// STOMP, declare random generator
	std::default_random_engine generator;
  	std::normal_distribution<float> distribution;

	struct {
		param_t v_STOMP;
		param_t delta_T_STOMP;
	}		_params_handles;		// handles for to find parameters

	struct {
		float v_STOMP;
		float delta_T_STOMP;
	}		_params;

	// Kinematic Model for STOMP algorithm
	math::Vector<5>     KinematicModel(math::Vector<5> pose_0, math::Vector<2> u);

	// Cost Function for STOMP algorithm
	float       CostFunction(math::Vector<5> state, math::Vector<3> wp);

	// STOMP Trajectory Planner
	void        PSTOMP(math::Vector<5> state_ini);

	// Update Trajectory Setpoint Topic
	void        pub_traj(int index);

	// Update our local parameter cache.
	int			parameters_update();                // checks if parameters have changed and updates them

	// Check for parameter update and handle it.
	void		parameter_update_poll();            // receives parameters

	// Shim for calling task_main from task_create.
	static void	task_main_trampoline(int argc, char *argv[]);

	// Main attitude control task.
	void		task_main();
};


namespace stomp_path_planner
{
StompPathPlanner	*g_control;
}

// constructor of class StompPathPlanner
StompPathPlanner::StompPathPlanner() :

	// First part is about function which are called with the constructor
	_task_should_exit(false),
	_control_task(-1),

	// subscriptions
	_v_att_sub(-1),
	_v_pos_sub(-1),
	_params_sub(-1),

	// publications
	_v_traj_sp_pub(nullptr),

	// performance counters
	_loop_perf(perf_alloc(PC_ELAPSED, "stomp_path_planner")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),

	// STOMP, initialize distribution
	distribution(0.0, 1.0)

// here starts the allocation of values to the variables
{
	// define settings
	memset(&_v_att, 0, sizeof(_v_att));
    memset(&_v_pos, 0, sizeof(_v_pos));

	// initialize STOMP parameter
	_params.delta_T_STOMP = 0.0f;
	_params.v_STOMP = 0.0f;

    // time and counter
	t_ges = 0.0f;
	time_counter = 100.0f;
	time_limit = 0.0f;
	t_zero = 0.0f;

	// STOMP
	prc_new = true;

	// initialize to zero
	gen_trajectory.zero();
	goal_position.zero();

	// allocate parameter handles
	_params_handles.delta_T_STOMP	= 	param_find("ST_D_T");
	_params_handles.v_STOMP		    = 	param_find("ST_V");

	// fetch initial parameter values
	parameters_update();
}

// destructor of class HippocampusPathControl
StompPathPlanner::~StompPathPlanner()
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

	stomp_path_planner::g_control = nullptr;
}

// updates parameters
int StompPathPlanner::parameters_update()
{
	float v;

	param_get(_params_handles.v_STOMP, &v);
	_params.v_STOMP = v;
	param_get(_params_handles.delta_T_STOMP, &v);
	_params.delta_T_STOMP = v;

	return OK;
}

// check for parameter updates
void StompPathPlanner::parameter_update_poll()
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

// Kinematic Model for PSTOMP algorithm
math::Vector<5> StompPathPlanner::KinematicModel(math::Vector<5> state_0, math::Vector<2> u)
{
	math::Vector<5> state_1;

	float delta_T_STOMP = _params.delta_T_STOMP;
	float v_STOMP = _params.v_STOMP;

	if (fabs(u(0)) < 0.001 && fabs(u(1)) < 0.001) {
	    state_1(3) = state_0(3);                        // update phi
	    state_1(4) = state_0(4);                        // update psi
		state_1(0) = state_0(0) + cosf(state_0(4))*cosf(state_0(3))*delta_T_STOMP*v_STOMP;
		state_1(1) = state_0(1) + sinf(state_0(3))*delta_T_STOMP*v_STOMP;
		state_1(2) = state_0(2) - sinf(state_0(4))*cosf(state_0(3))*delta_T_STOMP*v_STOMP;
	} else if (fabs(u(0) - u(1)) < 0.001) {
        float uu = u(0);
        float C = state_0(4) - state_0(3);

        state_1(3) = state_0(3) + uu * delta_T_STOMP;		    // update phi
        state_1(4) = state_0(4) + uu * delta_T_STOMP;		    // update psi

        // update x coordinate
        state_1(0) = state_0(0) + (v_STOMP / 2) * (cosf(C)*delta_T_STOMP + (cosf(C)/(2*uu))*(sinf(2*state_1(3)) - sinf(2*state_0(3)))
                                            + (sinf(C)/(2*uu)) * (cosf(2*state_1(3)) - cosf(2*state_0(3))));

        // update y coordinate
		state_1(1) = state_0(1) + (v_STOMP/uu) * (cosf(state_0(3)) - cosf(state_1(3)));

		// update z coordinate
		state_1(2) = state_0(2) - (v_STOMP / 2) * (sinf(C)*delta_T_STOMP + (cosf(C)/(2*uu))*(cosf(2*state_0(3)) - cosf(2*state_1(3)))
                                            + (sinf(C)/(2*uu)) * (sinf(2*state_1(3)) - sinf(2*state_0(3))));
	} else if (fabs(u(0) + u(1)) < 0.001) {
	    float uu = u(0);
        float C = state_0(4) + state_0(3);

        state_1(3) = state_0(3) + uu * delta_T_STOMP;		    // update phi
        state_1(4) = state_0(4) - uu * delta_T_STOMP;		    // update psi

        // update x coordinate
        state_1(0) = state_0(0) + (v_STOMP / 2) * (cosf(C)*delta_T_STOMP + (cosf(C)/(2*uu))*(sinf(2*state_1(3)) - sinf(2*state_0(3)))
                                            + (sinf(C)/(2*uu)) * (cosf(2*state_0(3)) - cosf(2*state_1(3))));

        // update y coordinate
		state_1(1) = state_0(1) + (v_STOMP/uu) * (cosf(state_0(3)) - cosf(state_1(3)));

		// update z coordinate
		state_1(2) = state_0(2) - (v_STOMP / 2) * (sinf(C)*delta_T_STOMP + (cosf(C)/(2*uu))*(cosf(2*state_1(3)) - cosf(2*state_0(3)))
                                            + (sinf(C)/(2*uu)) * (sinf(2*state_1(3)) - sinf(2*state_0(3))));
	}
	else {
	    state_1(3) = state_0(3) + u(0) * delta_T_STOMP;		    // update phi
        state_1(4) = state_0(4) + u(1) * delta_T_STOMP;		    // update psi

		float uu = u(0) + u(1);
		float u_12 = u(0) - u(1);
		float u_21 = u(1) - u(0);
	
		// update x coordinate
		float sin_min_1 = sinf(state_1(3) - state_1(4));
		float sin_min_0 = sinf(state_0(3) - state_0(4));
		float sin_plus_1 = sinf(state_1(3) + state_1(4));
		float sin_plus_0 = sinf(state_0(3) + state_0(4));
		state_1(0) = state_0(0) + (v_STOMP / 2) * ((sin_min_1 - sin_min_0)/u_12 + (sin_plus_1-sin_plus_0)/uu);

		// update y coordinate
		state_1(1) = state_0(1) + (v_STOMP/u(0)) * (cosf(state_0(3)) - cosf(state_1(3)));

		// update z coordinate
		float cos_min_1 = cosf(state_1(4) - state_1(3));
		float cos_min_0 = cosf(state_0(4) - state_0(3));
		float cos_plus_1 = cosf(state_1(3) + state_1(4));
		float cos_plus_0 = cosf(state_0(3) + state_0(4));
		state_1(2) = state_0(2) + (v_STOMP / 2) * ((cos_min_1 - cos_min_0)/u_21 + (cos_plus_1-cos_plus_0)/uu);
	}

	return state_1;
	
}

// Cost Function for the STOMP algorithm
float StompPathPlanner::CostFunction(math::Vector<5> state, math::Vector<3> wp)
{
	// Calculate orientation error
    math::Vector<3> position(state(0), state(1), state(2));     // get actual position
	math::Vector<3> direct = wp - position;                     // get actual direction vector and normalize
    direct.normalize();
    float psi_des = atan2f(-direct(2), direct(0));              // calculate desired angles
    float phi_des = 0;
    if (fabs(cosf(psi_des)) < 0.0001) {
        phi_des = atan2f(direct(1), 0.0f);
    }
    else {
        phi_des = atan2f(direct(1), direct(0)/cosf(psi_des));
    }

    float phi_s3 = phi_des - state(3);
    float psi_s4 = psi_des - state(4);
    float ori_error = sqrt(phi_s3*phi_s3 + psi_s4*psi_s4);        // Orientation Error

	// Calculate position error
	math::Vector<3> pos_error_vec = position - wp;
	float pos_error = pos_error_vec.length();
    pos_error = 0.0f;

	// Calculate complete cost
	return (ori_error + pos_error);

}


// Calculate Trajectory, PSTOMP algorithm
void StompPathPlanner::PSTOMP(math::Vector<5> state_ini)
{
    // Scaling Factors
    float scale_cost = 0.1f;
    float scale_var = 0.01f;

	/*// get actual orientation
    math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Vector<3> orientation = q_att.to_euler();

	// get initial state only pitch and yaw are relevant
	math::Vector<5> state_ini;
	state_ini(0) = _ctrl_state.x_pos;
	state_ini(1) = _ctrl_state.y_pos;
	state_ini(2) = _ctrl_state.z_pos;
	state_ini(3) = orientation(1);
	state_ini(4) = orientation(2);*/

	/*// Generating Inversed Squared Backward finite difference Matrix
	math::Matrix<N_STOMP, N_STOMP> Sigma;			// Generating Backward Finite Difference Matrix
	Sigma.identity();
	for( int i = 1; i < N_STOMP; i = i + 1 ) {
      		Sigma(i, (i-1)) = -2;
   	}
	for( int i = 2; i < N_STOMP; i = i + 1 ) {
      		Sigma(i, (i-2)) = 1;
   	}	
	Sigma = Sigma.transposed() * Sigma;	    // Squaring Matrices
	Sigma = Sigma.inversed();				// Inverse Matrix
	Sigma = Sigma * scale_var;              // Scale Variance

	// Cholesky Decomposition of Sigma to bottom down Matrix
	float summe = 0;
	for (int i=0; i < N_STOMP; i++) {
    		for (int l=0; l < i; l++) {
      			summe = Sigma(i,l);
      			for (int k=0; k < (l-1); k++) {
        			summe = summe - Sigma(i,k) * Sigma(l,k);
        			if (i > l) { 
          				Sigma(i,l) = summe / Sigma(l,l);
				}   
        			else if (summe > 0) {                             
          				Sigma(i,i) = sqrt(summe);
				}
				else {
					warnx("Cholesky Decomposition: Matrix has to be symetric!");
				}
			}
		}
	}
  	for (int i=0; i < (N_STOMP-1); i++) {
    		for (int l=(i+1); l < N_STOMP; l++) {
      			Sigma(i,l) = 0;
		}
	}*/

    math::Matrix<N_STOMP, N_STOMP> Sigma;
	for (int i=0; i<N_STOMP; i++) {
	    for(int l=0; l<(i+1); l++) {
	        Sigma(i,l) = 1+i-l;
	    }
	}

	// Start the algorithm

	// initialize holders
	math::Matrix<N_STOMP, 2> u_ini;			    // initial input at the beginning of each iteration
	u_ini.zero();
	math::Matrix<N_STOMP, 2> u_delta;		    // Change for the inputs
	u_delta.zero();
	math::Matrix<N_STOMP+1, 5> trajectory;		// Matrix to store current trajectory, sixth entry are timestamps
	trajectory.zero();
	trajectory.set_row(0, state_ini);		    // Set inital state
	math::Vector<K_STOMP> costs;                // Costs for the K different trajectories
	costs.zero();
	float sum_prob = 0;                         // Sum of all Probabilities from the K trajectories
	
	// Iterate through K sampled Trajectories a certain number of iterations
	for (int ll=0; ll < iter_STOMP; ll++) {
	    for (int k=0; k < K_STOMP; k++) {
		    // Generate Random Input Samples
		    math::Matrix<N_STOMP, 2> rand_vec;	// Generate samples from a one dimensional Normal Distribution
		    for (int i=0; i < N_STOMP; i++) {
    		    rand_vec(i, 0) = distribution(generator);
			    rand_vec(i, 1) = distribution(generator);
		    }
		    rand_vec = Sigma*rand_vec*scale_var;		// Generate Mutlivariate Normal Distribution based on the inversed squared backward finite difference matrix

		    math::Matrix<N_STOMP, 2> u_eps;     // Adding Samples to initial inputs
		    u_eps.zero();
		    u_eps = u_ini + rand_vec;

		    // Calculate trajectory and their cost
		    for (int i=0; i < N_STOMP; i++) {
			    trajectory.set_row((i+1), KinematicModel(trajectory.get_rowValues(i), u_eps.get_rowValues(i)));   // Get next Point for Trajectory
			    costs(k) = costs(k) + CostFunction(trajectory.get_rowValues(i+1), goal_position);  // Add cost for this point
		    }

	        // Sum up all input over the K trajectories scaled with their probabilities
	        float tmp_prob = expf(-scale_cost*costs(k));
	        u_delta = u_delta + u_eps * tmp_prob;
	        sum_prob = sum_prob + tmp_prob;
	    }
	    // Generate new input signals
	    u_ini = u_ini + u_delta / sum_prob;
	}

    // Calculate Final Trajectory
    float sum_prob_end = 0;
    for (int i=0; i < N_STOMP; i++) {
	    trajectory.set_row((i+1), KinematicModel(trajectory.get_rowValues(i), u_ini.get_rowValues(i)));
	    sum_prob_end = sum_prob_end + scale_cost * CostFunction(trajectory.get_rowValues(i+1), goal_position);
    }
    PX4_INFO("End Cost:\t%f", (double)sum_prob_end);

    // Allocate data into class intern matrix gen_trajectory
    for (int jj=0; jj<5; jj++) {                                                // allocate trajectory data
        gen_trajectory.set_col(jj, trajectory.get_colValues(jj));
    }
    for (int jj=0; jj<(N_STOMP+1); jj++) {                                      // allocate Timestamps
        gen_trajectory(jj, 5) = jj * _params.delta_T_STOMP;
    }
}

// Publish data to trajectory_setpoint for controller
void StompPathPlanner::pub_traj(int index)
{
    // Angles
    float phi = gen_trajectory(index, 3);
    float psi = gen_trajectory(index, 4);

    // allocate values
	_v_traj_sp.x = gen_trajectory(index, 0);        // x
	_v_traj_sp.y = gen_trajectory(index, 1);        // y
	_v_traj_sp.z = gen_trajectory(index, 2);        // z

	_v_traj_sp.dx = cosf(psi) * cosf(phi) * _params.v_STOMP;    // dx/dt
	_v_traj_sp.dy = sinf(phi) * _params.v_STOMP;                // dy/dt
	_v_traj_sp.dz = -sinf(psi) * cosf(phi) * _params.v_STOMP;   // dz/dt

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

// Just starts the task_main function
void StompPathPlanner::task_main_trampoline(int argc, char *argv[])
{
	stomp_path_planner::g_control->task_main();
}

// this is the main_task function which does the control task
void StompPathPlanner::task_main()
{
	PX4_INFO("STOMP Path Planner has been started!");
	// subscribe to uORB topics
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));               // subscribe to attitude and position
	_v_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));         // subscribe to attitude and position
	_params_sub = orb_subscribe(ORB_ID(parameter_update));              // parameter update

    // publisher
	_v_traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &_v_traj_sp);

	// initialize parameters cache
	parameters_update();

	// load goal position
    //std::ifstream myfile("data.txt");
    //myfile >> goal_position(0) >> goal_position(1) >> goal_position(2);
    goal_position(0) = 10.0f;
    goal_position(1) = 15.0f;
    PX4_INFO("Goal Coordinates:\t%f\t%f\t%f", (double)goal_position(0), (double)goal_position(1), (double)goal_position(2));

    // Initial Vector for STOMP
    math::Vector<5> state_ini;
	state_ini.zero();

	// wakeup source: vehicle pose
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		// wait for up to 100ms for data, we try to poll the data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out - periodic check for _task_should_exit
		if (pret == 0) {
			PX4_INFO("Got no data in 100ms!");
			continue;
		}

		// this is undesirable but not much we can do - might want to flag unhappy status
		if (pret < 0) {
			warn("stomp_path_planner: poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		// run controller on pose changes
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;   // calculate the time delta_t between two runs
			last_run = hrt_absolute_time();

			// guard against too small (< 2ms) and too large (> 20ms) dt's
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			// copy control state
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
			orb_copy(ORB_ID(vehicle_local_position), _v_pos_sub, &_v_pos);

			// count time
			t_ges = t_ges + dt;

			// If process has to be started new, then get actual position
			if (prc_new) {
			    // get actual orientation
                math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	            math::Vector<3> orientation = q_att.to_euler();

	            // get initial state only pitch and yaw are relevant
	            state_ini(0) = _v_pos.x;
	            state_ini(1) = _v_pos.y;
	            state_ini(2) = _v_pos.z;
	            state_ini(3) = orientation(1);
	            state_ini(4) = orientation(2);

	            prc_new = false;
			}

			// do trajectory planning
			time_limit = N_STOMP * _params.delta_T_STOMP * 0.3f;
            if (time_counter > time_limit) {
                PSTOMP(state_ini);
                time_counter = 0.0f;
                t_zero = t_ges;
                PX4_INFO("Start STOMP algorithm with Initial Position:");
                PX4_INFO("\t%f\t%f\t%f\n%f\t%f\n",
                        (double)state_ini(0), (double)state_ini(1), (double)state_ini(2), (double)state_ini(3), (double)state_ini(4));
            } else {
                time_counter = time_counter + dt;
            }

            // publish setpoint data for control algorithm
            int index = (int)round((t_ges - t_zero)/_params.delta_T_STOMP) + 1;     // get correct index
            pub_traj(index);
            state_ini(0) = gen_trajectory(index, 0);
	        state_ini(1) = gen_trajectory(index, 1);
	        state_ini(2) = gen_trajectory(index, 2);
	        state_ini(3) = gen_trajectory(index, 3);
	        state_ini(4) = gen_trajectory(index, 4);
	        //PX4_INFO("\t%f\t%f\t%f\n",
            //            (double)gen_trajectory(index, 0), (double)gen_trajectory(index, 1), (double)gen_trajectory(index, 2));

			// check for parameter updates
			parameter_update_poll();
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

// start function
int StompPathPlanner::start()
{
	ASSERT(_control_task == -1);        // give a value -1

	// start the control task, performs any specific accounting, scheduler setup, etc.
	_control_task = px4_task_spawn_cmd("stomp_path_planner",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&StompPathPlanner::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

// main function
int stomp_path_planner_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: stomp_path_planner {start|stop|status}");
		return 1;
	}

	// if command is start, then first control if class exists already, if not, allocate new one
	if (!strcmp(argv[1], "start")) {

		if (stomp_path_planner::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		// allocate new class StompPathPlanner
		stomp_path_planner::g_control = new StompPathPlanner;

		// check if class has been allocated, if not, give back failure
		if (stomp_path_planner::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		// if function start() can not be called, delete instance of StompPathPlanner and allocate null pointer
		if (OK != stomp_path_planner::g_control->start()) {
			delete stomp_path_planner::g_control;
			stomp_path_planner::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	// if command is start, check if class exists, if not can not stop anything
	if (!strcmp(argv[1], "stop")) {
		if (stomp_path_planner::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		// if class exists, delete it and allocate null pointer
		delete stomp_path_planner::g_control;
		stomp_path_planner::g_control = nullptr;
		return 0;
	}

	// if command is status and class exists, give back running, else give back not running
	if (!strcmp(argv[1], "status")) {
		if (stomp_path_planner::g_control) {
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
