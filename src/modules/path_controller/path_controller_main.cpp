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
 * @file path_controller_main.cpp
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
// uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>               // orientation data
#include <uORB/topics/vehicle_local_position.h>         // position data
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
extern "C" __EXPORT int path_controller_main(int argc, char *argv[]);

// the class from which an instance will be initiated by starting this application
class HippocampusPathControl
{
public:
	// Constructor
	HippocampusPathControl();

	// Destructor, also kills the main task
	~HippocampusPathControl();

	// Start the multicopter attitude control task, @return OK on success.
	int	start();

private:

	bool	_task_should_exit;		// if true, task_main() should exit
	int		_control_task;			// task handle

	// topic subscriptions
	int		_v_att_sub;		        // orientation data
	int     _v_pos_sub;             // position data
	int		_params_sub;			// parameter updates subscription
	int		_v_traj_sp_sub;			// trajectory setpoint subscription

	// topic publications
	orb_advert_t	_actuators_0_pub;		    // attitude actuator controls publication
	orb_advert_t    _logging_hippocampus_pub;   // logging data publisher
	orb_id_t        _actuators_id;	            // pointer to correct actuator controls0 uORB metadata structure

	// topic structures, in this structures the data of the topics are stored
	struct actuator_controls_s			_actuators;			    // actuator controls
	struct logging_hippocampus_s        _logging_hippocampus;   // logging data
	struct vehicle_attitude_s		    _v_att;		            // attitude data
	struct vehicle_local_position_s		_v_pos;		            // attitude data
	struct trajectory_setpoint_s	    _v_traj_sp;			    // trajectory setpoint

	// performance counters
	perf_counter_t	_loop_perf;
	perf_counter_t	_controller_latency_perf;

	// storage vectors for position, velocity, acceleration and the derivative of the acceleration
	math::Vector<3>     _position_0;        // actual position
	math::Vector<3>     _position_1;        // previous position
	math::Vector<3>     _position_2;
	math::Vector<3>     _position_3;

	// time
	float               t_ges;
	float               counter;

	math::Matrix<3, 3>  _I;				// identity matrix

	struct {
		param_t K_PX;
		param_t K_PY;
		param_t K_PZ;
		param_t K_VX;
		param_t K_VY;
		param_t K_VZ;
		param_t K_RX;
		param_t K_RY;
		param_t K_RZ;
		param_t K_WX;
		param_t K_WY;
		param_t K_WZ;
		param_t m;
		param_t X_u;
		param_t Y_v;
		param_t Z_w;
		param_t K_F;
		param_t K_M;
		param_t L;
		param_t OG;
	}		_params_handles;		// handles for to find parameters

	struct {
		// gain matrices
		math::Matrix<3, 3> K_p;         // position
		math::Matrix<3, 3> K_v;         // velocity
		math::Matrix<3, 3> K_r;         // orientation
		math::Matrix<3, 3> K_w;         // angular velocity
		// Hippocampus object parameters
		float m;                        // mass and added mass
		math::Matrix<3, 3> D;           // Damping Matrix
		// Force and Moment scaling factors
		float k_F;
		float k_M;
		// Lifting arm
		float L;
		float OG;                       // operating grade
	}		_params;

	// actualizes position data
	void        actualize_position();

	// writes position data into variables
	void        get_position(math::Vector<3> &x, math::Vector<3> &dx, math::Vector<3> &ddx, math::Vector<3> &dddx,
				 double dt);

	// update actual trajectory setpoint
	void        trajectory_setpoint_poll();

	// path controller.
	void		path_control(float dt);

	// Update our local parameter cache.
	int			parameters_update();                // checks if parameters have changed and updates them

	// Check for parameter update and handle it.
	void		parameter_update_poll();            // receives parameters

	// Shim for calling task_main from task_create.
	static void	task_main_trampoline(int argc, char *argv[]);

	// Main attitude control task.
	void		task_main();
};


namespace path_controller
{
HippocampusPathControl	*g_control;
}

// constructor of class HippocampusPathControl
HippocampusPathControl::HippocampusPathControl() :

	// First part is about function which are called with the constructor
	_task_should_exit(false),
	_control_task(-1),

	// subscriptions
	_v_att_sub(-1),
	_v_pos_sub(-1),
	_params_sub(-1),
	_v_traj_sp_sub(-1),

	// publications
	_actuators_0_pub(nullptr),
	_logging_hippocampus_pub(nullptr),
	_actuators_id(nullptr),


	// performance counters
	_loop_perf(perf_alloc(PC_ELAPSED, "path_controller")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

// here starts the allocation of values to the variables
{
	// define publication settings
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_pos, 0, sizeof(_v_pos));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_logging_hippocampus, 0, sizeof(_logging_hippocampus));
	memset(&_v_traj_sp, 0, sizeof(_v_traj_sp));

	// set parameters to zero
	_params.K_p.zero();
	_params.K_v.zero();
	_params.K_r.zero();
	_params.K_w.zero();
	_params.m = 0.0f;
	_params.D.zero();
	_params.L = 0.0f;
	_params.OG = 0.0f;

	// set initial values of vectors to zero
	_position_0.zero();
	_position_1.zero();
	_position_2.zero();
	_position_3.zero();

	t_ges = 0.0;
	counter = 0.0;

	// allocate Identity matrix
	_I.identity();

	// allocate parameter handles
	_params_handles.K_PX	        = 	param_find("PC_K_PX");
	_params_handles.K_PY	        = 	param_find("PC_K_PY");
	_params_handles.K_PZ	        = 	param_find("PC_K_PZ");
	_params_handles.K_VX		    = 	param_find("PC_K_VX");
	_params_handles.K_VY		    = 	param_find("PC_K_VY");
	_params_handles.K_VZ		    = 	param_find("PC_K_VZ");
	_params_handles.K_RX		    = 	param_find("PC_K_RX");
	_params_handles.K_RY		    = 	param_find("PC_K_RY");
	_params_handles.K_RZ		    = 	param_find("PC_K_RZ");
	_params_handles.K_WX		    = 	param_find("PC_K_WX");
	_params_handles.K_WY		    = 	param_find("PC_K_WY");
	_params_handles.K_WZ		    = 	param_find("PC_K_WZ");
	_params_handles.m		        = 	param_find("PC_m");
	_params_handles.X_u		        = 	param_find("PC_X_u");
	_params_handles.Y_v		        = 	param_find("PC_Y_v");
	_params_handles.Z_w		        = 	param_find("PC_Z_w");
	_params_handles.K_F		        = 	param_find("PC_K_F");
	_params_handles.K_M		        = 	param_find("PC_K_M");
	_params_handles.L	            = 	param_find("PC_L");
	_params_handles.OG	            = 	param_find("PC_OG");

	// fetch initial parameter values
	parameters_update();
}

// destructor of class HippocampusPathControl
HippocampusPathControl::~HippocampusPathControl()
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

	path_controller::g_control = nullptr;
}

// updates parameters
int HippocampusPathControl::parameters_update()
{
	float v;

	param_get(_params_handles.K_PX, &v);
	_params.K_p(0,0) = v;
	param_get(_params_handles.K_PY, &v);
	_params.K_p(1,1) = v;
	param_get(_params_handles.K_PZ, &v);
	_params.K_p(2,2) = v;
	param_get(_params_handles.K_VX, &v);
	_params.K_v(0,0) = v;
	param_get(_params_handles.K_VY, &v);
	_params.K_v(1,1) = v;
	param_get(_params_handles.K_VZ, &v);
	_params.K_v(2,2) = v;
	param_get(_params_handles.K_RX, &v);
	_params.K_r(0,0) = v;
	param_get(_params_handles.K_RY, &v);
	_params.K_r(1,1) = v;
	param_get(_params_handles.K_RZ, &v);
	_params.K_r(2,2) = v;
	param_get(_params_handles.K_WX, &v);
	_params.K_w(0,0) = v;
	param_get(_params_handles.K_WY, &v);
	_params.K_w(1,1) = v;
	param_get(_params_handles.K_WZ, &v);
	_params.K_w(2,2) = v;
	param_get(_params_handles.m, &v);
	_params.m = v;
	param_get(_params_handles.X_u, &v);
	_params.D(0, 0) = v;
	param_get(_params_handles.Y_v, &v);
	_params.D(1, 1) = v;
	param_get(_params_handles.Z_w, &v);
	_params.D(2, 2) = v;
	param_get(_params_handles.K_F, &v);
	_params.k_F = v;
	param_get(_params_handles.K_M, &v);
	_params.k_M = v;
	param_get(_params_handles.L, &v);
	_params.L = v;
    param_get(_params_handles.OG, &v);
	_params.OG = v;

	return OK;
}

// check for parameter updates
void HippocampusPathControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();

		PX4_INFO("K_p:\t%8.2f\t%8.2f\t%8.2f",
			 (double)_params.K_p(0,0),
			 (double)_params.K_p(1,1),
			 (double)_params.K_p(2,2));
		PX4_INFO("K_v:\t%8.2f\t%8.2f\t%8.2f",
			 (double)_params.K_v(0,0),
			 (double)_params.K_v(1,1),
			 (double)_params.K_v(2,2));
		PX4_INFO("K_r:\t%8.2f\t%8.2f\t%8.2f",
			 (double)_params.K_r(0,0),
			 (double)_params.K_r(1,1),
			 (double)_params.K_r(2,2));
		PX4_INFO("K_w:\t%8.2f\t%8.2f\t%8.2f\n",
			 (double)_params.K_w(0,0),
			 (double)_params.K_w(1,1),
			 (double)_params.K_w(2,2));

	}
}


// actualizes the position data if receiving new data
void HippocampusPathControl::actualize_position()
{

	// write position data into vectors
	math::Vector<3> holder(_v_pos.x, _v_pos.y, _v_pos.z);

	// store position data
	_position_3 = _position_2;
	_position_2 = _position_1;
	_position_1 = _position_0;
	_position_0 = holder;
}


//Calculates the velocity and accelerations based on the position using differential quotient
void HippocampusPathControl::get_position(math::Vector<3> &x, math::Vector<3> &dx, math::Vector<3> &ddx,
		math::Vector<3> &dddx, double dt)
{
	x = _position_0;
	dx = (_position_0 - _position_1) / dt;
	ddx = (_position_0 - _position_1 * 2.0 + _position_2) / (dt * dt);
	dddx = (_position_0 - _position_1 * 2.0 + _position_2 * 2.0 - _position_3) / (2.0 * dt * dt * dt);
}

// Get the Setpoint from the trajectory planner
void HippocampusPathControl::trajectory_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_traj_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(trajectory_setpoint), _v_traj_sp_sub, &_v_traj_sp);
	}
}


/**
 * path controller.
 * Input: 'setpoints from trajectory planner', 'actual position and orientation'
 * Output: 'actuators_control'
 */
void HippocampusPathControl::path_control(float dt)
{
	// actualize setpoint data
	trajectory_setpoint_poll();

	// count time
	t_ges = t_ges + dt;

	//*******************************
	//  Declaration of Variables
	//*******************************
	// define error vectors
	math::Vector<3> e_p;            // position error
	math::Vector<3> e_v;            // velocity error
	math::Vector<3> e_r;            // orientation error
	math::Matrix<3, 3> e_r_matrix;  // orientation error matrix
	math::Vector<3> e_w;            // angular velocity error

	// get actual position data
	math::Vector<3> r;                      // actual position
	math::Vector<3> rd;                     // actual velocity
	math::Vector<3> rdd;                    // actual acceleration
	math::Vector<3> rddd;
	actualize_position();                   // record new position data
	get_position(r, rd, rdd, rddd, dt);     // estimate derivations using difference methods

	// get trajectory setpoint data
	math::Vector<3> r_T(_v_traj_sp.x, _v_traj_sp.y, _v_traj_sp.z);                  // desired position
	math::Vector<3> rd_T(_v_traj_sp.dx, _v_traj_sp.dy, _v_traj_sp.dz);              // desired velocity
	math::Vector<3> rdd_T(_v_traj_sp.ddx, _v_traj_sp.ddy, _v_traj_sp.ddz);          // desired acceleration
	math::Vector<3> rddd_T(_v_traj_sp.dddx, _v_traj_sp.dddy, _v_traj_sp.dddz);

	// desired force and outputs
	math::Vector<3> F_des;
	float u_1;
	math::Vector<3> u_24;

	// rotation matrices and angular velocity vectors
	math::Vector<3> w_BW(_v_att.rollspeed, _v_att.pitchspeed, _v_att.yawspeed);      // angular velocity
	math::Vector<3>
	w_BW_T;                                                                         // desired angular velocity
	math::Matrix<3, 3>
	R;                                                                           // actual rotation matrix
	math::Matrix<3, 3>
	R_des;                                                                       // desired rotation matrix

	//******************************

	// get current rotation matrix from control state quaternions, the quaternions are generated by the
	// attitude_estimator_q application using the sensor data
	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	// create rotation matrix for the quaternion when post multiplying with a column vector x'=R*x
	R = q_att.to_dcm();

	// transform angular velocity into World coordinate system since the given control state data are in body frame
	w_BW = R * w_BW;

	// orientation vectors
	math::Vector<3> x_B(R(0, 0), R(1, 0), R(2,
						0));                             // orientation body x-axis (in world coordinates)
	math::Vector<3> y_B(R(0, 1), R(1, 1), R(2,
						1));                             // orientation body y-axis (in world coordinates)
	math::Vector<3> z_B(R(0, 2), R(1, 2), R(2,
						2));                             // orientation body z-axis (in world coordinates)
	math::Vector<3> x_B_des;                                                    // orientation body x-axis desired
	math::Vector<3> y_B_des;                                                    // orientation body y-axis desired
	math::Vector<3> z_B_des;                                                    // orientation body z-axis desired
	math::Vector<3> z_C_des(0, -sinf(_v_traj_sp.roll), cosf(_v_traj_sp.roll));    // orientation C-Coordinate system desired
	math::Vector<3> y_C_des(0, cosf(_v_traj_sp.roll), sinf(_v_traj_sp.roll));     // orientation C-Coordinate system desired

	// projection on x_B
	math::Vector<3> h_w;
	math::Vector<3> h_w_des;

	// thrust input
	e_p = r - r_T;          // calculate position error
	e_v = rd - rd_T;        // calculate velocity error

	F_des = -_params.K_p * e_p - _params.K_v * e_v + rdd_T * _params.m + _params.D * rd_T; // calculate desired force

	u_1 = F_des * x_B;                                 // calculate desired thrust

	// calculate orientation vectors
	x_B_des = F_des / F_des.length();

	// check wether y_B_des is closer to the actual position or z_B_des
	// TO DO: Find a fix for singularity
	y_B_des = z_C_des % x_B_des;
	y_B_des = y_B_des / y_B_des.length();
	z_B_des = x_B_des % y_B_des;

	// calculate desired rotation matrix
	R_des.set_col(0, x_B_des);
	R_des.set_col(1, y_B_des);
	R_des.set_col(2, z_B_des);

	// extracting one rotation from the rotation matrix, necessary due to singularities in the (R_des^T * R - R^T * R_des) approach
	// rotation axis for rotation between x_B and x_B_des, in B coordinates (not normalized yet)
	e_r = R.transposed() * (x_B_des % x_B);

	// calculate the angle errors using norm of cross product and dot product
	float x_B_sin = e_r.length();
	float x_B_cos = x_B * x_B_des;

	// rotation matrix after pitch/yaw only rotation, thus between R and R_py are only a roll rotation left
	math::Matrix<3, 3> R_py;

	// check if x_B and x_B_des are non parallel, otherwise we would not have rotations pitch and yaw
	if (x_B_sin > 0.0f) {
		// calculate axis angle representation of the pitch/yaw rotation
		float e_R_angle = atan2f(x_B_sin, x_B_cos);
		math::Vector<3> e_R_axis = e_r / x_B_sin;           // normalize axis

		// get the error vector of the rotation in B coordinates
		e_r = e_R_axis * e_R_angle;

		// get the cross product matrix for e_R_axis to calculate the Rodrigues formula
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_axis(2);
		e_R_cp(0, 2) = e_R_axis(1);
		e_R_cp(1, 0) = e_R_axis(2);
		e_R_cp(1, 2) = -e_R_axis(0);
		e_R_cp(2, 0) = -e_R_axis(1);
		e_R_cp(2, 1) = e_R_axis(0);

		// rotation matrix after pitch/yaw only rotation, thus between R and R_py are only a roll rotation left, in World coordinates
		R_py = R * (_I + e_R_cp * x_B_sin + e_R_cp * e_R_cp * (1.0f - x_B_cos));

	} else {
		// zero pitch/yaw rotation
		R_py = R;
	}

	//R_py and R_des have the same X axis, calculate roll error
	math::Vector<3> z_B_py(R_py(0, 2), R_py(1, 2), R_py(2, 2));
	e_r(0) = atan2f((z_B_des % z_B_py) * x_B_des, z_B_py * z_B_des);

	// Calculate desired angular velocity
	h_w_des = (rddd_T * _params.m + _params.D * rdd_T - (x_B_des * ((rddd_T * _params.m + _params.D * rdd_T) * x_B_des)))
		  * (1 / u_1);
	//h_w_des = (rddd_T-x_B_des*(x_B_des*rddd_T))*(_params.m/u_1);

	double q_ang = -h_w_des * z_B_des;
	double r_ang = -h_w_des * y_B_des;
	double p_ang = _v_traj_sp.droll * x_B_des(0);
	w_BW_T = x_B_des * p_ang + y_B_des * q_ang + z_B_des * r_ang;

	// calculate the angular velocity error
	e_w = R.transposed() * w_BW - R.transposed() * w_BW_T;

	// calculate input over feedback loop
	u_24 = -_params.K_r * e_r - _params.K_w * e_w;

	// scale roll with 1/8
	u_24(0) = u_24(0) * (_params.L * _params.k_F / _params.k_M);

	// put all calculated control outputs in one vector
	math::Vector<4> u_ges(u_1, u_24(0), u_24(1), u_24(2));

	// Generating K matrix with scaling factors for Forces and Moments
	float K_SCALE[4][4] = {
		{_params.k_F, _params.k_F, _params.k_F, _params.k_F},
		{ -_params.k_M, _params.k_M, -_params.k_M, _params.k_M},
		{ -_params.k_F * _params.L, -_params.k_F * _params.L, _params.k_F * _params.L, _params.k_F * _params.L},
		{_params.k_F * _params.L, -_params.k_F * _params.L, -_params.k_F * _params.L, _params.k_F * _params.L}
	};
	math::Matrix<4, 4> K_scale(K_SCALE);

	math::Vector<4> omega_des = K_scale.inversed() * u_ges;

	// Generating C_mix matrix for recalculation to Mixer Inputs
	float C_MIX[4][4] = {
		{ -1, -1, 1, 1},
		{1, -1, -1, 1},
		{ -1, 1, -1, 1},
		{1, 1, 1, 1}
	};
	math::Matrix<4, 4> C_mix(C_MIX);

	// calculate the desired rotor velocities
	math::Vector<4> mix_input = C_mix.inversed() * omega_des;

	// Reduce Input signal by a certain percentage
	mix_input = mix_input * _params.OG;

	// give the inputs to the actuators
	_actuators.control[0] = mix_input(0);           // roll
	_actuators.control[1] = mix_input(1);           // pitch
	_actuators.control[2] = mix_input(2);           // yaw
	_actuators.control[3] = mix_input(3);           // thrust

	// store logging data for publishing
	_logging_hippocampus.x = r(0);
	_logging_hippocampus.y = r(1);
	_logging_hippocampus.z = r(2);
	_logging_hippocampus.xd = r_T(0);
	_logging_hippocampus.yd = r_T(1);
	_logging_hippocampus.zd = r_T(2);

	_logging_hippocampus.xa[0] = x_B(0);
	_logging_hippocampus.xa[1] = x_B(1);
	_logging_hippocampus.xa[2] = x_B(2);

	_logging_hippocampus.ya[0] = y_B(0);
	_logging_hippocampus.ya[1] = y_B(1);
	_logging_hippocampus.ya[2] = y_B(2);

	_logging_hippocampus.za[0] = z_B(0);
	_logging_hippocampus.za[1] = z_B(1);
	_logging_hippocampus.za[2] = z_B(2);

	_logging_hippocampus.xad[0] = x_B_des(0);
	_logging_hippocampus.xad[1] = x_B_des(1);
	_logging_hippocampus.xad[2] = x_B_des(2);

	_logging_hippocampus.yad[0] = y_B_des(0);
	_logging_hippocampus.yad[1] = y_B_des(1);
	_logging_hippocampus.yad[2] = y_B_des(2);

	_logging_hippocampus.zad[0] = z_B_des(0);
	_logging_hippocampus.zad[1] = z_B_des(1);
	_logging_hippocampus.zad[2] = z_B_des(2);

	_logging_hippocampus.t = t_ges;

	// Debugging
	if (counter < t_ges) {

		counter = counter + 0.5f;            // every 0.5 seconds

		/*PX4_INFO("e_p:\t%8.2f\t%8.2f\t%8.2f",
			 (double)e_p(0),
			 (double)e_p(1),
			 (double)e_p(2));
		PX4_INFO("e_v:\t%8.2f\t%8.2f\t%8.2f",
			 (double)e_v(0),
			 (double)e_v(1),
			 (double)e_v(2));
		PX4_INFO("e_r:\t%8.2f\t%8.2f\t%8.2f\n",
			 (double)e_r(0),
			 (double)e_r(1),
			 (double)e_r(2));
		PX4_INFO("e_w:\t%8.2f\t%8.2f\t%8.2f\n",
			 (double)e_w(0),
			 (double)e_w(1),
			 (double)e_w(2));*/
	}

}

// Just starts the task_main function
void HippocampusPathControl::task_main_trampoline(int argc, char *argv[])
{
	path_controller::g_control->task_main();
}

// this is the main_task function which does the control task
void HippocampusPathControl::task_main()
{

	PX4_INFO("Path Controller has been started!");
	// subscribe to uORB topics
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_v_traj_sp_sub = orb_subscribe(ORB_ID(trajectory_setpoint));

	// initialize parameters cache
	parameters_update();

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
			warn("path_controller: poll error %d, %d", pret, errno);
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

			// copy position and orientation data
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
            orb_copy(ORB_ID(vehicle_local_position), _v_pos_sub, &_v_pos);

			// do path control
			path_control(dt);

			// publish logging data
			if (_logging_hippocampus_pub != nullptr) {
				orb_publish(ORB_ID(logging_hippocampus), _logging_hippocampus_pub, &_logging_hippocampus);

			} else {
				_logging_hippocampus_pub = orb_advertise(ORB_ID(logging_hippocampus), &_logging_hippocampus);
			}

			// publish actuator timestamps
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _v_att.timestamp;

			if (_actuators_0_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
				perf_end(_controller_latency_perf);

			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}


			_actuators_id = ORB_ID(actuator_controls_0);

			// check for parameter updates
			parameter_update_poll();
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

// start function
int HippocampusPathControl::start()
{
	ASSERT(_control_task == -1);        // give a value -1

	// start the control task, performs any specific accounting, scheduler setup, etc.
	_control_task = px4_task_spawn_cmd("path_controller",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&HippocampusPathControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

// main function
int path_controller_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: path_controller {start|stop|status}");
		return 1;
	}

	// if command is start, then first control if class exists already, if not, allocate new one
	if (!strcmp(argv[1], "start")) {

		if (path_controller::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		// allocate new class HippocampusPathControl
		path_controller::g_control = new HippocampusPathControl;

		// check if class has been allocated, if not, give back failure
		if (path_controller::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		// if function start() can not be called, delete instance of HippocampusPathControl and allocate null pointer
		if (OK != path_controller::g_control->start()) {
			delete path_controller::g_control;
			path_controller::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	// if command is start, check if class exists, if not can not stop anything
	if (!strcmp(argv[1], "stop")) {
		if (path_controller::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		// if class exists, delete it and allocate null pointer
		delete path_controller::g_control;
		path_controller::g_control = nullptr;
		return 0;
	}

	// if command is status and class exists, give back running, else give back not running
	if (!strcmp(argv[1], "status")) {
		if (path_controller::g_control) {
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