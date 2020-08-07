#include <kinematics.h>

/**
 * This function is used to setup a robot object. 
 * \param kinematic_group_number : specifies what type of robot it is
 * \param lengths : points to the link lenghts of specified robot
 * \param robot_axis : Nessesary information required to run a robot. Please read the axis_data class declaration.
 * This function sets up internal arrays and copies data into internal variables and essentially sets up the robot. 
 */
robot::robot(int kinematic_group_number, double *lengths, robot_data robot_axis) {
	kinematic_group = kinematic_group_number;
	if (kinematic_group == scara_robot) {
		//SCARA Robot
	 scara_a1 = lengths[0]; //Link 1 length
	 scara_a2 = lengths[1]; //Link 2 length
	 scara_d4 = lengths[2]; //Link 3 length
	 // Denavit_Hartenberg Parameters : [ a ,   Alpha , d , Theta ]
		dh_params.dh_parameters[0][0] = scara_a1; dh_params.dh_parameters[0][1] = 0.0; dh_params.dh_parameters[0][2] = 0.0;      dh_params.dh_parameters[0][3] = 0.0;
		dh_params.dh_parameters[1][0] = scara_a2; dh_params.dh_parameters[1][1] = PI;  dh_params.dh_parameters[1][2] = 0.0;      dh_params.dh_parameters[1][3] = PI_DIV_2;
		dh_params.dh_parameters[2][0] = 0.0;      dh_params.dh_parameters[2][1] = 0.0; dh_params.dh_parameters[2][2] = 0.0;      dh_params.dh_parameters[2][3] = 0.0;
		dh_params.dh_parameters[3][0] = 0.0;      dh_params.dh_parameters[3][1] = 0.0; dh_params.dh_parameters[3][2] = scara_d4; dh_params.dh_parameters[3][3] = 0.0;
		dh_params.dh_parameters[4][0] = 0.0;      dh_params.dh_parameters[4][1] = 0.0; dh_params.dh_parameters[4][2] = 0.0;      dh_params.dh_parameters[4][3] = 0.0;
		dh_params.dh_parameters[5][0] = 0.0;      dh_params.dh_parameters[5][1] = 0.0; dh_params.dh_parameters[5][2] = 0.0;      dh_params.dh_parameters[5][3] = 0.0;

	}
	else if (kinematic_group == SixAxis_anthropomorphic_robot) {
		// 6 Axis Robot
	}

	robotaxis_data = robot_axis;
	execute_done = false;
	pulse_num = 0;
	buffer_pt = 0;
	j1_fs_limit_exceeded = false; j1_rs_limit_exceeded = false;
	j2_fs_limit_exceeded = false; j2_rs_limit_exceeded = false;
	j3_fs_limit_exceeded = false; j3_rs_limit_exceeded = false;
	j4_fs_limit_exceeded = false; j4_rs_limit_exceeded = false;
	j5_fs_limit_exceeded = false; j5_rs_limit_exceeded = false;
	j6_fs_limit_exceeded = false; j6_rs_limit_exceeded = false;
};


/**
* This function will home/datum the robot. In robotics, "homing can be defined as that behavior, which enables a robot to return to its initial
(home) position, after traveling a certain distance along an arbitrary path".[<a href = "https://www.ics.forth.gr/_pdf/brochures/tr287_argyros_homing.pdf"> Reference [1] </a>]

*This function will move the robot to its defualt/home position. This will happen one axis at a time, starting with the base axis, the controller will look for limit switches for each axis.
*If any axis does not have a limit switch, set the I/O number to -1 and the controller will assume that the axis is aleady at home postiion. 
*/
void robot::home(void) {
	// Homing Routine
	// Move each axis until sensor input is detected.
	target_point home_j1 = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	target_point home_j2_plus = { 0.0, 270.0, 0.0, 0.0, 0.0, 0.0 };
	
	double temp_speed;
	
	temp_speed = robotaxis_data.joint[j1].SPEED;
	robotaxis_data.joint[j1].SPEED = 20.0;
		
//	MOVEJ(home_j1, &buffer_pt, 0);
//	handle();
//	if (j1_fs_limit_exceeded) j1_fs_limit_exceeded = false;
//	else if (j1_rs_limit_exceeded) j1_rs_limit_exceeded = false;
//	target_point j1h = { robotaxis_data.joint[j1].position - 30, 0, 0, 0, 0, 0 };
//	MOVEJ(j1h, &buffer_pt, 0);
//	handle();
//	robotaxis_data.joint[j1].SPEED = temp_speed;
	
//	temp_speed = robotaxis_data.joint[j2].SPEED;
//	robotaxis_data.joint[j2].SPEED = 50.0;
//	MOVEJ(home_j2_plus, &buffer_pt, 0);
//	handle();
//	if (j2_fs_limit_exceeded) j2_fs_limit_exceeded = false;
//	else if (j2_rs_limit_exceeded) j2_rs_limit_exceeded = false;
//	double new_targ_p = robotaxis_data.joint[j2].position - 85.5;
//	target_point j2h = {0, new_targ_p, 0, 0, 0, 0 };
//	MOVEJ(j2h, &buffer_pt, 0);
//	handle();
//	robotaxis_data.joint[j2].SPEED = temp_speed;
	
	temp_speed = robotaxis_data.joint[j3].SPEED;
	robotaxis_data.joint[j3].SPEED = 5.0;
	target_point home_j3_plus = { 0.0, robotaxis_data.joint[j2].position, 100.0, 0, 0, 0 };
	MOVEJ(home_j3_plus, &buffer_pt, 0);
	handle();
	if (j3_fs_limit_exceeded) j3_fs_limit_exceeded = false;
	else if (j3_rs_limit_exceeded) j3_rs_limit_exceeded = false;
	
	temp_speed = robotaxis_data.joint[j4].SPEED;
	robotaxis_data.joint[j4].SPEED = 5.0;
	target_point home_j4 = { 0.0, robotaxis_data.joint[j2].position, robotaxis_data.joint[j3].position, 14.1, 0.0, 0.0 };
	MOVEJ(home_j4, &buffer_pt, 0);
	handle();
	
	
	HOME_COMPLETE = 1;
	inv_theta.m[j1] = 0.0; inv_theta.m[j2] = 90.0; inv_theta.m[j3] = robotaxis_data.joint[j3].position;
	inv_theta.m[j4] = 0.0; inv_theta.m[j5] = 0.0;  inv_theta.m[j6] = 0.0;
	fwd_theta.m[j1] = 0.0; fwd_theta.m[j2] = 90.0; fwd_theta.m[j3] = robotaxis_data.joint[j3].position;
	fwd_theta.m[j4] = 0.0; fwd_theta.m[j5] = 0.0;  fwd_theta.m[j6] = 0.0;
	robotaxis_data.joint[j1].position = 0.0;
	robotaxis_data.joint[j2].position = 90.0;
	robotaxis_data.joint[j3].position = robotaxis_data.joint[j3].position;
	robotaxis_data.joint[j4].position = 0.0;
	robotaxis_data.joint[j5].position = 0.0;
	robotaxis_data.joint[j6].position = 0.0;
	forward_kin();
};

/**
 *This function will enable all the axes of the robot and ready the robot for movement. It will turn ON enable pin specified in the robot_data structure.
 */
void robot::activate(void){
	if (kinematic_group == scara_robot) {
		for (int i = 0; i < 4; i++)
			HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[i].GPIO_BASE), robotaxis_data.joint[i].enable_pin, GPIO_PIN_SET); // Turn ON Enable Pin of all robot axes
	}
	else if (kinematic_group == SixAxis_anthropomorphic_robot) {
		for (int i = 0; i < 6; i++)
			HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[i].GPIO_BASE), robotaxis_data.joint[i].enable_pin, GPIO_PIN_SET); // Turn OFF Enable Pin of all robot axes
	}
	else{
		;
	}
};

/**
 *This function will deactivate all the axes of the robot and disable the robot for any movement. It will turn OFF enable pin specified in the robot_data structure.
 */
void robot::deactivate(void){
	if (kinematic_group == scara_robot) {
		for (int i = 0; i < 4; i++)
			HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[i].GPIO_BASE), robotaxis_data.joint[i].enable_pin, GPIO_PIN_RESET);
	}
	else if (kinematic_group == SixAxis_anthropomorphic_robot) {
		for (int i = 0; i < 6; i++)
			HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[i].GPIO_BASE), robotaxis_data.joint[i].enable_pin, GPIO_PIN_RESET);
	}
	else {
		;
	}
};
/**
 * This function will schedule all the nessesary functions required to keep the robot functional.
 * Internally it calls various routines that then actually run the functions nessesary. 
 */
void robot::handle(void){
		trajectory_routine();
};


/* Kinematics and Dynamics
 * 
 *
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * This function will run all the kinematics functions required to be run every tick.
 */
void robot::kinematics_routine(void){
	forward_kin();
	if (trajecotry_mode == 2) inverse_kin();
};

/**
 * This function executes Forward Kinematics and returns the Transformation matrix. 
 * \param theta : Input angles of each joint of the robot.
 * 
 * Internally it multiples DH matrices to get the final Transformation matrices
 */
void robot::forward_kin(void) {
	
	if (kinematic_group == scara_robot) {
		/// [https : //www.gnu.org/software/gsl/doc/html/blas.html]
		// Calculate DH matrices for each link
//		dhmat(dh_params.dh_parameters[0][0], dh_params.dh_parameters[0][1], dh_params.dh_parameters[0][2], fwd_theta.m[j1], &A1);
//		dhmat(dh_params.dh_parameters[1][0], dh_params.dh_parameters[1][1], dh_params.dh_parameters[1][2], fwd_theta.m[j2], &A2);
//		dhmat(dh_params.dh_parameters[2][0], dh_params.dh_parameters[2][1], fwd_theta.m[j3], dh_params.dh_parameters[2][2], &A3);
//		dhmat(dh_params.dh_parameters[3][0], dh_params.dh_parameters[3][1], dh_params.dh_parameters[3][2], fwd_theta.m[j4], &A4);

		// Multiply matrices to get Transformation Matrix
		static double t1, t2, d3, t4, st1, st2, ct1, ct2, st4, ct4;
		t1 = fwd_theta.m[j1]*PI_BY_ONE80;
		t2 = fwd_theta.m[j2]*PI_BY_ONE80;
		d3 = fwd_theta.m[j3];
		t4 = fwd_theta.m[j4]*PI_BY_ONE80;
		st1 = sinf(t1); st2 = sinf(t2); st4 = sinf(t4);
		ct1 = cosf(t1); ct2 = cosf(t2); ct4 = cosf(t4);
		
		T.m[0][0] = ct4*(ct1*ct2 - 1.0*st1*st2) + st4*(ct1*st2 + ct2*st1);
		T.m[0][1] = ct4*(ct1*st2 + ct2*st1) - 1.0*st4*(ct1*ct2 - 1.0*st1*st2);
		T.m[0][2] = 0.00000000000000012246467991473532071737640294584*ct1*st2 + 0.00000000000000012246467991473532071737640294584*ct2*st1;
		T.m[0][3] = scara_a1*ct1 + d3*(0.00000000000000012246467991473532071737640294584*ct1*st2 + 0.00000000000000012246467991473532071737640294584*ct2*st1) + scara_d4*(0.00000000000000012246467991473532071737640294584*ct1*st2 + 0.00000000000000012246467991473532071737640294584*ct2*st1) + scara_a2*ct1*ct2 - 1.0*scara_a2*st1*st2;
		
		T.m[1][0] = ct4*(ct1*st2 + ct2*st1) - 1.0*st4*(ct1*ct2 - 1.0*st1*st2);
		T.m[1][1] = -1.0*ct4*(ct1*ct2 - 1.0*st1*st2) - 1.0*st4*(ct1*st2 + ct2*st1);
		T.m[1][2] = 0.00000000000000012246467991473532071737640294584*st1*st2 - 0.00000000000000012246467991473532071737640294584*ct1*ct2;
		T.m[1][3] = scara_a1*st1 - 1.0*d3*(0.00000000000000012246467991473532071737640294584*ct1*ct2 - 0.00000000000000012246467991473532071737640294584*st1*st2) - 1.0*scara_d4*(0.00000000000000012246467991473532071737640294584*ct1*ct2 - 0.00000000000000012246467991473532071737640294584*st1*st2) + scara_a2 *ct1*st2 + scara_a2 *ct2*st1;
		
		T.m[2][0] = 0.00000000000000012246467991473532071737640294584*st4;
		T.m[2][1] = 0.00000000000000012246467991473532071737640294584*ct4;
		T.m[2][2] = -1.0;
		T.m[2][3] = d3;//-1.0*d3 - 1.0*scara_d4;
		
		T.m[3][0] = 0.0;
		T.m[3][1] = 0.0;
		T.m[3][2] = 0.0;
		T.m[3][3] = 1.0;

	}
	else if (kinematic_group == SixAxis_anthropomorphic_robot) {
		// Calculate DH matrices for each link
//		A1 = dhmat(dh_params.dh_parameters[0][0], dh_params.dh_parameters[0][1], dh_params.dh_parameters[0][2], fwd_theta.m[j1]);
//		A2 = dhmat(dh_params.dh_parameters[1][0], dh_params.dh_parameters[1][1], dh_params.dh_parameters[1][2], fwd_theta.m[j2]);
//		A3 = dhmat(dh_params.dh_parameters[2][0], dh_params.dh_parameters[2][1], dh_params.dh_parameters[2][2], fwd_theta.m[j3]);
//		A4 = dhmat(dh_params.dh_parameters[3][0], dh_params.dh_parameters[3][1], dh_params.dh_parameters[3][2], fwd_theta.m[j4]);
//		A5 = dhmat(dh_params.dh_parameters[4][0], dh_params.dh_parameters[4][1], dh_params.dh_parameters[4][2], fwd_theta.m[j5]);
//		A6 = dhmat(dh_params.dh_parameters[5][0], dh_params.dh_parameters[5][1], dh_params.dh_parameters[5][2], fwd_theta.m[j6]);

		// Multiply matrices to get Transformation Matrix

	}
};


/**
 *This function will perform Inverse Kinematics for specified kinematic group. 
 *It uses a geometric approach.
 *\param T : Transformation matrix of the end effector
 */
void robot::inverse_kin(void) {
	if (kinematic_group == scara_robot) {
		//SCARA Robot
		static double alpha, beta;
	    position.m[x] = T.m[x][3]; /// X coordinate of end effector
		position.m[y] = T.m[y][3]; /// Y coordinate of end effector
		position.m[z] = T.m[z][3]; /// Z coordinate of end effector 
		inv_theta.m[j2] = acosf((double)((position.m[x]*position.m[x]) + (position.m[y]*position.m[y]) - (scara_a1*scara_a1) - (scara_a2*scara_a2)) / (2*scara_a1*scara_a2));
		alpha = atan2f(position.m[y], position.m[x]);
		beta = atan2f(scara_a2*sinf(inv_theta.m[j2]), scara_a1 + scara_a2*cosf(inv_theta.m[j2]));
		inv_theta.m[j1] = alpha - beta;
		inv_theta.m[j3] = position.m[z];// + scara_d4;
		inv_theta.m[j4] = inv_theta.m[j1] + inv_theta.m[j2] - atan2f(T.m[0][1], T.m[0][0]);
		
		inv_theta.m[j1] *= ONE80_BY_PI;
		inv_theta.m[j2] *= ONE80_BY_PI;
		inv_theta.m[j4] *= ONE80_BY_PI;
	}
	else if (kinematic_group == SixAxis_anthropomorphic_robot) {
		// 6 axis anthropomorphic Robot
	}
};


/**
 *This function will convert a given quaternion into euler angles.
 */
void robot::quat_2_eul(void){
	
};

/**
 *This function will convert given Euler angles into a quaternion.
 */
void robot::eul_2_quat(void) {
	
};

/**
 *This function returns a Denavit - Hartenburg Matrix. 
 *"Denavit–Hartenberg parameters (also called DH parameters) are the four parameters associated with a particular convention for attaching reference frames to the links of a spatial kinematic chain, or robot manipulator."
 *
 *The following four transformation parameters are known as D–H parameters:
 *\param a     :  length of the common normal (aka a, but if using this notation, do not confuse with alpha ). Assuming a revolute joint, this is the radius about previous z.
 *\param alpha : angle about common normal, from old z axis to new z axis
 *\param d     : offset along previous {\displaystyle z}z to the common normal
 *\param theta : angle about previous z, from to old x to new x
 * 
 *[https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters]
 */
void robot::dhmat(double a, double alpha, double d, double theta, matrix4x4 *targetm) {
	
	targetm->m[0][0] = cosf(theta); targetm->m[0][1] = -sinf(theta)*cosf(alpha); targetm->m[0][2] = sinf(theta)*sinf(alpha);  targetm->m[0][3] = a*cosf(theta);
	targetm->m[1][0] = sinf(theta); targetm->m[1][1] = cosf(theta)*cosf(alpha);  targetm->m[1][2] = -cosf(theta)*sinf(alpha); targetm->m[1][3] = a*sinf(theta);
	targetm->m[2][0] = 0.0;         targetm->m[2][1] = sinf(alpha);              targetm->m[2][2] = cosf(alpha);              targetm->m[2][3] = d;
	targetm->m[3][0] = 0.0;         targetm->m[3][1] = 0.0;                      targetm->m[3][2] = 0.0;                      targetm->m[3][3] = 1.0;

};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Linear move for the robot. The robot will move in a straight line in cartesian space. 
 * \param target_point : specifies the target point the user wants the robot to go to. \ref target_point
 * Internally it loads the target point and move into buffers for the trajectory planner to process. 
 */
void robot::MOVEL(target_point p, int *buffer_pointer){
	if (*buffer_pointer < 6) {
		if (move_buffer[*buffer_pointer] == -1 || move_buffer[*buffer_pointer] == 0) {
			move_buffer[*buffer_pointer] = type_movel;
			point_buffer[*buffer_pointer] = p;
			axis_num_buffer[*buffer_pointer] = 0;
			*buffer_pointer += 1;
		}
	}
};

/**
 * Joint move for the robot. The robot will move in a straight line in joint space. 
 * \param target_point : specifies the target point the user wants the robot to go to. \ref target_point
 * Internally it loads the target point and move into buffers for the trajectory planner to process. 
 */
void robot::MOVEJ(target_point p, int *buffer_pointer, int axisp) {
	if (*buffer_pointer < 6) {
		if (move_buffer[*buffer_pointer] == -1 || move_buffer[*buffer_pointer] == 0) {
			move_buffer[*buffer_pointer] = type_movej;
			point_buffer[*buffer_pointer] = p;
			axis_num_buffer[*buffer_pointer] = axisp;
			*buffer_pointer += 1;
		}
	}
};

/**
 * Joint move for the robot. The robot will move in a circle in cartesian space. 
 * \param target_point : specifies the target point the user wants the robot to go to. \ref target_point. These points are the mid point and the end point of the circle. 
 * Internally it loads the target point and move into buffers for the trajectory planner to process. 
 */
void robot::MOVEC(target_point p, target_point q, int *buffer_pointer) {
	if (*buffer_pointer < 6) {
		if (move_buffer[*buffer_pointer] == 0) {
			move_buffer[*buffer_pointer] = type_movec;
			move_buffer[*buffer_pointer + 1] = type_movec;
			point_buffer[*buffer_pointer] = p;
			point_buffer[*buffer_pointer + 1] = q;
			axis_num_buffer[*buffer_pointer] = 0;
			axis_num_buffer[*buffer_pointer + 1] = 0;
			*buffer_pointer += 1;
		}
	}
};

void robot::EmergencyStop(void){
	// Do something to stop the robot	
	deactivate();
};


/* Trajectory Generator
 *
 *
 *
 *
 **/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * This function will generate grajectory based on loaded moves and points in the buffer. 
 * If the buffer is empty it will not do anything.
 * Internlly it runs different trajecotry generation algorithms to genereate a trajectory based to jerk.
 */
void robot::trajectory_routine(void){
	static double f_accel[3];
	static double c_accel[3];
	static double d_accel[3];
	for (int i = 0; i < 6; i++) { // Scan the move buffer
		if(move_buffer[i] == -1) { // If buffer is empty, dont do anything
			break; //yeet
		}
		else if(move_buffer[i] == type_movej) {
			trajecotry_mode = 1; // Switch to joint mode
			if(abs(point_buffer[i].X - robotaxis_data.joint[j1].position) > 0.1 ) { //Dont process any tiny movements what wouldnt result in a physcial rotation
				
				vector3 start_pos = { robotaxis_data.joint[j1].position, 0, 0 };
				vector3 end_pos = { point_buffer[i].X, 0, 0 };
				double target_dist;
				dist(start_pos, end_pos, &target_dist);
				temp_p = start_pos;
				temp_target = point_buffer[i].X;
				pulses_to_execute = 0.0;
				trap_trajec(0, robotaxis_data.joint[j1].SPEED, robotaxis_data.joint[j1].ACCEL, robotaxis_data.joint[j1].DECEL, target_dist, &f_accel[j1], &c_accel[j1], &d_accel[j1]);
				double accel_stack[3] = {f_accel[j1], c_accel[j1], d_accel[j1]};
				TickType_t start_t = xTaskGetTickCount();
				while (!motion_routine(start_t, j1, accel_stack) && !j1_fs_limit_exceeded && !j1_rs_limit_exceeded){
//					if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j1].GPIO_BASE_fslim, robotaxis_data.joint[j1].fs_limit_sw) == 1) {
//						j1_fs_limit_exceeded = true;
//					}
//					else if(HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j1].GPIO_BASE_rslim, robotaxis_data.joint[j1].rs_limit_sw) == 1){
//						j1_rs_limit_exceeded = true;
//					}
				}
			}
			if(abs(point_buffer[i].Y - robotaxis_data.joint[j2].position) > 0.1) {
				vector3 start_pos = { robotaxis_data.joint[j2].position, 0, 0 };
				vector3 end_pos = { point_buffer[i].Y, 0, 0 };
				double target_dist;
				dist(start_pos, end_pos, &target_dist);
				temp_p = start_pos;
				temp_target = point_buffer[i].Y;
				pulses_to_execute = 0.0;
				trap_trajec(0, robotaxis_data.joint[j2].SPEED, robotaxis_data.joint[j2].ACCEL, robotaxis_data.joint[j2].DECEL, target_dist, &f_accel[j2], &c_accel[j2], &d_accel[j2]);
				double accel_stack[3] = { f_accel[j2], c_accel[j2], d_accel[j2] };
				TickType_t start_t = xTaskGetTickCount();
				while (!motion_routine(start_t, j2, accel_stack) && !j2_fs_limit_exceeded && !j2_rs_limit_exceeded) {
					if (j2_fs_status == false) {
						if(end_pos.m[0] > start_pos.m[0])
							j2_fs_limit_exceeded = true;
					}
					else if (j2_rs_status == false) {
						j2_rs_limit_exceeded = true;
					}
				}
			}
			if(abs(point_buffer[i].Z - robotaxis_data.joint[j3].position) > 0.1) {
				vector3 start_pos = { robotaxis_data.joint[j3].position, 0, 0 };
				vector3 end_pos = { point_buffer[i].Z, 0, 0 };
				double target_dist;
				dist(start_pos, end_pos, &target_dist);
				temp_p = start_pos;
				temp_target = point_buffer[i].Z;
				pulses_to_execute = 0.0;
				trap_trajec(0, robotaxis_data.joint[j3].SPEED, robotaxis_data.joint[j3].ACCEL, robotaxis_data.joint[j3].DECEL, target_dist, &f_accel[j3], &c_accel[j3], &d_accel[j3]);
				double accel_stack[3] = { f_accel[j3], c_accel[j3], d_accel[j3] };
				TickType_t start_t = xTaskGetTickCount();
				while (!motion_routine(start_t, j3, accel_stack) && !j3_fs_limit_exceeded && !j3_rs_limit_exceeded) {
//					if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j3].GPIO_BASE_fslim, robotaxis_data.joint[j3].fs_limit_sw) == 0) {
//						j3_fs_limit_exceeded = true;
//					}
//					else if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j3].GPIO_BASE_rslim, robotaxis_data.joint[j3].rs_limit_sw) == 0) {
//						j3_rs_limit_exceeded = true;
//					}
				}
			}
			if(abs(point_buffer[i].U - robotaxis_data.joint[j4].position) > 0.1) {
				vector3 start_pos = { robotaxis_data.joint[j4].position, 0, 0 };
				vector3 end_pos = { point_buffer[i].U, 0, 0 };
				double target_dist;
				dist(start_pos, end_pos, &target_dist);
				temp_p = start_pos;
				temp_target = point_buffer[i].U;
				pulses_to_execute = 0.0;
				trap_trajec(0, robotaxis_data.joint[j4].SPEED, robotaxis_data.joint[j4].ACCEL, robotaxis_data.joint[j4].DECEL, target_dist, &f_accel[j4], &c_accel[j4], &d_accel[j4]);
				double accel_stack[3] = { f_accel[j4], c_accel[j4], d_accel[j4] };
				TickType_t start_t = xTaskGetTickCount();
				while (!motion_routine(start_t, j4, accel_stack) && !j4_fs_limit_exceeded && !j4_rs_limit_exceeded) {
//					if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j4].GPIO_BASE_fslim, robotaxis_data.joint[j4].fs_limit_sw) == 0) {
//						j4_fs_limit_exceeded = true;
//					}
//					else if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j4].GPIO_BASE_rslim, robotaxis_data.joint[j4].rs_limit_sw) == 0) {
//						j4_rs_limit_exceeded = true;
//					}
				}
			}
			fwd_theta.m[j1] = robotaxis_data.joint[j1].position;
			fwd_theta.m[j2] = robotaxis_data.joint[j2].position;
			fwd_theta.m[j3] = robotaxis_data.joint[j3].position;
			fwd_theta.m[j4] = robotaxis_data.joint[j4].position;
			forward_kin();
		}
		else if(move_buffer[i] == type_movel) {
			trajecotry_mode = 2; // Cartesian mode
			forward_kin();
			vector3 start_pos = { T.m[0][3], T.m[1][3], T.m[2][3] };
			vector3 end_pos = {point_buffer[i].X, point_buffer[i].Y, point_buffer[i].Z};
			double target_dist; bool insignificant_motion = false;
			if (i == 2) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			}
			else if(i == 5){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			}
			dist(start_pos, end_pos, &target_dist);
			if (target_dist < 0.1){
				insignificant_motion = true;
			}
			trap_trajec(0, robotaxis_data.LINEAR_SPEED, robotaxis_data.LINEAR_ACCEL, robotaxis_data.LINEAR_DECEL, target_dist, &f_accel[j1], &c_accel[j1], &d_accel[j1]);
			double accel_stack[3] = { f_accel[j1], c_accel[j1], d_accel[j1] };
			TickType_t start_t = xTaskGetTickCount();
			trajec1_s = 0;
			inv_kin_buffer_pt = i;
			while (!insignificant_motion && !motion_routine(start_t, j1, accel_stack)  && !j1_fs_limit_exceeded && !j1_rs_limit_exceeded && !j2_fs_limit_exceeded && !j2_rs_limit_exceeded && !j3_fs_limit_exceeded && !j3_rs_limit_exceeded && !j4_fs_limit_exceeded && !j4_rs_limit_exceeded) {
//				if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j1].GPIO_BASE_fslim, robotaxis_data.joint[j1].fs_limit_sw) == 0) {
//					j1_fs_limit_exceeded = true;
//				}
//				else if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j1].GPIO_BASE_rslim, robotaxis_data.joint[j1].rs_limit_sw) == 0) {
//					j1_rs_limit_exceeded = true;
//				}
//				if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j2].GPIO_BASE_fslim, robotaxis_data.joint[j2].fs_limit_sw) == 0) {
//					j2_fs_limit_exceeded = true;
//				}
//				else if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j2].GPIO_BASE_rslim, robotaxis_data.joint[j2].rs_limit_sw) == 0) {
//					j2_rs_limit_exceeded = true;
//				}
//				if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j3].GPIO_BASE_fslim, robotaxis_data.joint[j3].fs_limit_sw) == 0) {
//					j3_fs_limit_exceeded = true;
//				}
//				else if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j3].GPIO_BASE_rslim, robotaxis_data.joint[j3].rs_limit_sw) == 0) {
//					j3_rs_limit_exceeded = true;
//				}
//				if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j4].GPIO_BASE_fslim, robotaxis_data.joint[j4].fs_limit_sw) == 0) {
//					j4_fs_limit_exceeded = true;
//				}
//				else if (HAL_GPIO_ReadPin((GPIO_TypeDef *)robotaxis_data.joint[j4].GPIO_BASE_rslim, robotaxis_data.joint[j4].rs_limit_sw) == 0) {
//					j4_rs_limit_exceeded = true;
//				}
			}
			fwd_theta.m[j1] = robotaxis_data.joint[j1].position;
			fwd_theta.m[j2] = robotaxis_data.joint[j2].position;
			fwd_theta.m[j3] = robotaxis_data.joint[j3].position;
			fwd_theta.m[j4] = robotaxis_data.joint[j4].position;
			forward_kin();
			
		}
		else if(move_buffer[i] == type_movec) {
			;
		}
	}
	buffer_pt = 0;
	point_buffer[j1] = { 0, 0, 0, 0, 0, 0 };
	point_buffer[j2] = point_buffer[j1];
	point_buffer[j3] = point_buffer[j1];
	point_buffer[j4] = point_buffer[j1];
	point_buffer[j5] = point_buffer[j1];
	point_buffer[j6] = point_buffer[j1];
	move_buffer[j1] = -1; 
	move_buffer[j2] = -1;
	move_buffer[j3] = -1;
	move_buffer[j4] = -1;
	move_buffer[j5] = -1;
	move_buffer[j6] = -1;
};


/**
 * Calculates the distance between any 2 given 3D points. The output is stored in *target. 
 * Both the input points need to be of type Array3f.
 * \param p1 : point 1
 * \param p2 : point 2
 * \param target : The function will calculate the distance between p1 and p2 and save it in target. 
 */
void robot::dist(vector3 p1, vector3 p2, double *target) {
	*target = (sqrtf(pow(p2.m[0] - p1.m[0], 2) + pow(p2.m[1] - p1.m[1], 2) + pow(p2.m[2] - p1.m[2], 2)));
};

void robot::find_point_on_line(vector3 p1, vector3 p2, double distance, vector3 *output_point) {
	double dist_between_points; vector3 norm_vec;
	dist_between_points = (sqrtf(pow(p2.m[0] - p1.m[0], 2) + pow(p2.m[1] - p1.m[1], 2) + pow(p2.m[2] - p1.m[2], 2)));
	norm_vec = { (double)(p2.m[0] - p1.m[0]) / dist_between_points, (double)(p2.m[1] - p1.m[1]) / dist_between_points, (double)(p2.m[2] - p1.m[2]) / dist_between_points };
	output_point->m[0] = p1.m[0] + norm_vec.m[0]*distance;
	output_point->m[1] = p1.m[1] + norm_vec.m[1]*distance;
	output_point->m[2] = p1.m[2] + norm_vec.m[2]*distance;
};


/**
 *This function calcualtes the coefficients of a quintic polynomial that will generate a trajecotry starting at time t1, position x1, speed, v1 and acceleration a1 and end at time t2, position p2, speed v2 and acceleration a2.
 *\warning  : This solver can and will generate invalid outputs for invalid inputs. Please use the trajectory generator to generate a trajctory. 
 *\param t1 : Start time for the trajectory
 *\param p1 : Start position for the trajectory
 *\param v1 : Start velocity for the trajecotry
 *\param a1 : Start acceleration for the trajectory
 *\param jerk1 : Start jerk for the trajectory
 *\param t2 : End time for the trajectory
 *\param p2 : End position for the trajectory
 *\param v2 : End velocity for the trajecotr
 *\param a2 : End acceleration for the trajectory
 *\param jerk2 :End jerkfor the trajectory
 */
void robot :: heptic_solver(double t1, double x1, double v1, double a1, double jerk1, double t2, double x2, double v2, double a2, double jerk2, vector8 *target_m) {
	target_m->m[7] =  (double)-(120*x1 - 120*x2 - 60*t1*v1 - 60*t1*v2 + 60*t2*v1 + 60*t2*v2 + 12*a1*pow(t1,2) + 12*a1*pow(t2,2) - 12*a2*pow(t1,2) - 12*a2*pow(t2,2) - jerk1*pow(t1,3) + jerk1*pow(t2,3) - jerk2*pow(t1,3) + jerk2*pow(t2,3) - 3*jerk1*t1*pow(t2,2) + 3*jerk1*pow(t1,2)*t2 - 3*jerk2*t1*pow(t2,2) + 3*jerk2*pow(t1,2)*t2 - 24*a1*t1*t2 + 24*a2*t1*t2) / (6*pow(t1 - t2 , 7));
	target_m->m[6] =  (double) - (420*x1 - 420*x2 - 216*t1*v1 - 204*t1*v2 + 216*t2*v1 + 204*t2*v2 + 45*a1*pow(t1, 2) + 45*a1*pow(t2, 2) - 39*a2*pow(t1, 2) - 39*a2*pow(t2, 2) - 4*jerk1*pow(t1, 3) + 4*jerk1*pow(t2, 3) - 3*jerk2*pow(t1, 3) + 3*jerk2*pow(t2, 3) - 12*jerk1*t1*pow(t2, 2) + 12*jerk1*pow(t1, 2)*t2 - 9*jerk2*t1*pow(t2, 2) + 9*jerk2*pow(t1, 2)*t2 - 90*a1*t1*t2 + 78*a2*t1*t2) / (6*pow(t1 - t2, 6)); 
	target_m->m[5] =  (double) - (168*x1 - 168*x2 - 90*t1*v1 - 78*t1*v2 + 90*t2*v1 + 78*t2*v2 + 20*a1*pow(t1, 2) + 20*a1*pow(t2, 2) - 14*a2*pow(t1, 2) - 14*a2*pow(t2, 2) - 2*jerk1*pow(t1, 3) + 2*jerk1*pow(t2, 3) - jerk2*pow(t1, 3) + jerk2*pow(t2, 3) - 6*jerk1*t1*pow(t2, 2) + 6*jerk1*pow(t1, 2)*t2 - 3*jerk2*t1*pow(t2, 2) + 3*jerk2*pow(t1, 2)*t2 - 40*a1*t1*t2 + 28*a2*t1*t2) / (2*pow(t1 - t2, 5));
	target_m->m[4] =  (double) - (210*x1 - 210*x2 - 120*t1*v1 - 90*t1*v2 + 120*t2*v1 + 90*t2*v2 + 30*a1*pow(t1, 2) + 30*a1*pow(t2, 2) - 15*a2*pow(t1, 2) - 15*a2*pow(t2, 2) - 4*jerk1*pow(t1, 3) + 4*jerk1*pow(t2, 3) - jerk2*pow(t1, 3) + jerk2*pow(t2, 3) - 12*jerk1*t1*pow(t2, 2) + 12*jerk1*pow(t1, 2)*t2 - 3*jerk2*t1*pow(t2, 2) + 3*jerk2*pow(t1, 2)*t2 - 60*a1*t1*t2 + 30*a2*t1*t2) / (6*pow(t1 - t2, 4));
	target_m->m[3] =  (double)jerk1 / 6;
	target_m->m[2] =  (double)a1 / 2;
	target_m->m[1] = v1;
	target_m->m[0] = x1;
};


/**
 * Acceleration Pulse Algorithm is a part of the trajectory generator, this is one of 2 algorithms it uses to generate a jerk bound trajecotry.
 * \param a_max   : Maximum allowed acceleration
 * \param j_max   : Maximum allowed jerk
 * \param v1      : Start Velocity
 * \param v2      : End Velocity
 * \param p1      : Start Position
 * \param D       : Total Distance to travel
 * \param coeffs1 : Output coefficients for first segment
 * \param coeffs2 : Output coefficients for second segment
 * \param coeffs3 : Output coefficients for third segment
 * \param dt      : Time segment for first and second coefficients
 * \param t_end   : Time segment for third coefficients
 * \param Dap     : Total distance travelled by this algorithm 
 */
void robot::APA(double a_max, double j_max, double v1, double v2, double p1, double D, vector6 coeffs1,vector6 coeffs2, vector6 coeffs3, double *dt, double *t_end, double *Dap) {
	
	bool mode = 0;
	if (v1 > v2){
		mode = 1;
	}
	
	if (mode == 1) {
		static double v = v1;
		v1 = v2;
		v2 = v;
	}	
	
	double t_max = PI*a_max / (2*j_max);
	double V = v2 - v1;

	double Vmin = t_max*a_max;
	double Dmin = a_max*(pow(t_max,2)) + 2*v1*t_max;
	double dT, a_peak;
	if (D < Dmin && V > Vmin) {
		/// dt^3 + (pi*V)/j_max * dt - (pi*D)/(2*j_max) = 0
		/// x^3 + a*x^2 + b*x + c = 0
		double a = 0; 
		double b = (PI*V) / j_max; 
		double c =  -(PI*D) / (2*j_max);
		double Q = (pow(a, 2) - 3*b) / 9; 
		double R = (2*pow(a, 2) - 9*a*b + 27*c) / 54;
		double theta = acos(R / cbrt(pow(Q, 3)));
		double x1 = -2*sqrt(Q)*cos(theta / 3) - a / 3;
		double x2 = -2*sqrt(Q)*cos((theta + 2*PI) / 3) - a / 3;
		double x3 = -2*sqrt(Q)*cos((theta - 2*PI) / 3) - a / 3;
		dT = x1;
		a_peak = 2*j_max*dT / PI;
	}
	else if (D > Dmin && V < Vmin) {
		dT = sqrt((PI*V) / (2*j_max));
		a_peak = 2*j_max*dT / PI;
	}
	else if (D < Dmin && V < Vmin) {
		double a = 0; 
		double b = (PI*V) / j_max; 
		double c =  -(PI*D) / (2*j_max);
		double Q = (pow(a, 2) - 3*b) / 9; 
		double R = (2*pow(a, 2) - 9*a*b + 27*c) / 54;
		double theta = acos(R / cbrt(pow(Q, 3)));
		double x1 = -2*sqrt(Q)*cos(theta / 3) - a / 3;
		double x2 = -2*sqrt(Q)*cos((theta + 2*PI) / 3) - a / 3;
		double x3 = -2*sqrt(Q)*cos((theta - 2*PI) / 3) - a / 3;
		dT = x1;
		a_peak = 2*j_max*dT / PI;

		if (V < a_peak*dT) {
			dT = sqrt((PI*V) / (2*j_max));
			a_peak = 2*j_max*dT / PI;
		}
	}
	else {
		a_peak = a_max;
		dT = t_max;
	}
	
	Vmin = dT*a_peak;
	Dmin = a_peak*pow(dT,2) + 2*v1*dT;
    
	double Va = v1 + (a_peak*dT) / 2;
	double Vb = v2 - (a_peak*dT) / 2;
    
	double D1 = a_peak *pow(dT,2)* (0.25 - (1 / PI_SQUARED)) + v1*dT;
	double D3 = a_peak*pow(dT,2)*(0.25 - (1 / PI_SQUARED)) + Vb*dT;
	
	*dt = dT;
	*Dap = D1 + D3;
	
	if (mode == 0) {
//		*coeffs1 = quintic_solver(0, 0, v1, 0, dT, D1, Va, a_peak);
//		*coeffs2 = quintic_solver(dT, D1, Va, a_peak, 2*dT, D1 + D3, v1 + Vmin, 0);
	}
	else{
		vector3 tq;
		tq.m[0] = 0;
		tq.m[1] = dT;
		tq.m[2] = 2*dT;
		vector4 pq = { 0, D1, D1 + D3 };
		vector3 vq = { v1, Va, v2 };
		vector3 aq = { 0, a_max, 0 };
		vector3 tq1, pq1, vq1, aq1;
		
		for (int i = 0 ; i < 3; i++) {
			tq1.m[i] = tq.m[3] - tq.m[3 - i + 1];
			pq1.m[i] = pq.m[3] - pq.m[3 - i + 1];
			vq1.m[i] = vq.m[3 - i + 1];
			aq1.m[i] = -aq.m[3 - i + 1];
		}
		
		//quintic_solver(tq1.m[0], p1 + pq1.m[0], vq1.m[0], aq1.m[1], tq1.m[1], p1 + pq1.m[1], vq1.m[1], aq1.m[1], &coeffs1);
		//quintic_solver(tq1.m[1], p1 + pq1.m[1], vq1.m[1], aq1.m[3], tq1.m[2], p1 + pq1.m[2], vq1.m[2], aq1.m[2], &coeffs2);
	}
	//quintic_solver(2*dT, D1 + D3, v1 + Vmin, 0, 2*dT + (D - D1 - D3) / (v1 + Vmin), D, v1 + Vmin, 0, &coeffs3);
	*t_end = 2*dT + (D - D1 - D3) / (v1 + Vmin);
};


/**
 * Sustained Acceleration Pulse Algorithm is a part of the trajectory generator, this is one of 2 algorithms it uses to generate a jerk bound trajecotry.
 * \param p1      : Start position
 * \param v1      : Start velocity
 * \param p2      : End position
 * \param v2      : End velocity
 * \param a_max   : Maximum allowed Acceleration
 * \param t_max   : Maximum allowed time
 * \param coeffs1 : Output coefficients for first segment
 * \param coeffs2 : Output coefficients for second segment
 * \param coeffs3 : Output coefficients for third segment
 * \param coeffs4 : Output coefficients for fourth segment
 * \param t_glide : Time segment for second coefficient
 * \param e_end   : Time segment for fourth coefficient
 * \param Dsap    : Total distance traveleld by algorithm
 */
void robot::SAPA(double p1, double v1, double p2, double v2, double a_max, double t_max, vector6 coeffs1,vector6 coeffs2, vector6 coeffs3, vector6 coeffs4, double *t_glide, double *t_end, double *Dsap) {
	
	bool mode = 0;
	if (v1 > v2) {
		mode = 1;
	}
	
	if (mode == 1) {
		static double v = v1;
		v1 = v2;
		v2 = v;
	}	
	
	double D = abs(p2 - p1);
	double Va = v1 + (a_max*t_max) / 2;
	double Vb = v2 - (a_max*t_max) / 2;
	double D1 = a_max * pow(t_max, 2) * (0.25 - (1 / PI_SQUARED)) + v1*t_max;
	double D2 = (pow(Vb, 2) - pow(Va, 2)) / (2*a_max);
	double D3 = a_max*(pow(t_max, 2))*(0.25 - (1 / PI_SQUARED)) + Vb*t_max;
	
	if (D1 + D2 + D3 - D > 0) {
		Vb = -(a_max*t_max) + sqrt(pow(a_max*t_max, 2) - 2*a_max*(((a_max*pow(t_max, 2)) / 2) + v1*t_max - pow(Va, 2) / (2*a_max) - D));
		v2 = Vb + a_max*t_max / 2;
		D2 = (pow(Vb, 2) - pow(Va, 2)) / (2*a_max);
		D3 = pow(a_max*t_max,2)*(0.25 + (1 / (PI_SQUARED))) + Vb*t_max;
	}
	
	*Dsap = D1 + D2 + D3;
	*t_glide = (Vb - Va) / a_max;
	double tq2 = t_max;
	double tq3 = t_max + *t_glide;
	double tq4 = tq3 + t_max;
	double t_end_glide = (D - *Dsap) / v2;
	*t_end = tq4 + t_end_glide;
	
	if (mode == 0) {
		//quintic_solver(0, 0, v1, 0, t_max, D1, Va, a_max, &coeffs1);
		//quintic_solver(t_max, D1, Va, a_max, tq2 + *t_glide, D1 + D2, Vb, a_max, &coeffs2);
		//quintic_solver(tq2 + *t_glide, D1 + D2, Vb, a_max, tq3 + t_max, D1 + D2 + D3, v2, 0, &coeffs3);
		//quintic_solver(tq3 + t_max, D1 + D2 + D3, v2, 0, tq4 + t_end_glide, D, v2, 0, &coeffs4);
	}
	else {
		vector4 tq, pq, vq, aq;  
		tq.m[0] = 0;  tq.m[1] = tq2;   tq.m[2] = tq3;     tq.m[3] = tq4;          tq.m[4] = *t_end;
		pq.m[0] = 0;  pq.m[1] = D1;    pq.m[2] = D1 + D2; pq.m[3] = D1 + D2 + D3; pq.m[4] = D;
 		vq.m[0] = v1; vq.m[1] = Va;    vq.m[2] = Vb;      vq.m[3] = v2;           vq.m[4] = v2;
		aq.m[0] = 0;  aq.m[1] = a_max; aq.m[2] = a_max;   aq.m[3] = 0;            aq.m[4] = 0;
		vector6 tq1, pq1, vq1, aq1;

		for (int i = 0; i < 4; i++) {
			tq1.m[i] = tq.m[4] - tq.m[4 - i + 1];
			pq1.m[i] = pq.m[4] - pq.m[4 - i + 1];
			vq1.m[i] = vq.m[4 - i + 1];
			aq1.m[i] = -aq.m[4 - i + 1];
		}
		
		//quintic_solver(tq1.m[1], p1 + pq1.m[1], vq1.m[1], aq1.m[1], tq1.m[2], p1 + pq1.m[2], vq1.m[2], aq1.m[2], &coeffs1);
		//quintic_solver(tq1.m[2], p1 + pq1.m[2], vq1.m[2], aq1.m[2], tq1.m[3], p1 + pq1.m[3], vq1.m[3], aq1.m[3], &coeffs2);
		//quintic_solver(tq1.m[3], p1 + pq1.m[3], vq1.m[3], aq1.m[4], tq1.m[4], p1 + pq1.m[4], vq1.m[4], aq1.m[3], &coeffs3);
		//quintic_solver(tq1.m[4], p1 + pq1.m[4], vq1.m[4], aq1.m[4], tq1.m[5], p1 + pq1.m[5], vq1.m[5], aq1.m[5], &coeffs4);
	}
};

/**
 *Generates a trajectory based on CAM
 *A cam is a rotating or sliding piece in a mechanical linkage used especially in transforming rotary motion into linear motion
 *[https://en.wikipedia.org/wiki/Cam]
 *\param t0           : Start time
 *\param p0           : Start position
 *\param v0           : Start velocity
 *\param a0           : Start acceleration
 *\param t1           : End time
 *\param p1           : End position
 *\param v1           : End velocity
 *\param a1           : End acceleration
 *\param time         : time at which the output point needs to be calculated at
 *\param output_point : output point will be stored here
 */
void robot::CAM_trajecotry(double t0, double p0, double v0, double t1, double p1, double v1, double time, vector8 *output_point) {
	
	vector8 coeffs1;
	heptic_solver(t0, p0, v0, 0, 0, t1, p1, v1, 0, 0, &coeffs1);
	*output_point = coeffs1;
};


/*
 *Generates factors for accel, cruise and decel phase.
 *\param x1            : start postiion
 *\param v1            : velocity
 *\param a1            : acceleration
 *\param a2            : deceleration
 *\param x2            : end position
 *\param factor_accel  : This factor specifies how long the acceleration phase lasts.
 *\param factor_cruise : This factor specifies how lomg the cruise phase lasts.
 *\param factor_decel  : This factor specifies how long the deceleration phase lasts.
 **/
void  robot::trap_trajec(double x1, double v1, double a1, double a2, double x2, double *factor_accel, double *factor_cruise, double *factor_decel) {
	double total_dist, cruise_dist;
	neg_dist = false;
	
	total_dist = x2 - x1;
	if (total_dist < 0){
		total_dist *= -1; //porocess only positive dist
		neg_dist = true;
	}
	double decel_dist = 0.5 * v1 * v1 / a2;
	double accel_dist = 0.5 * v1 * v1 / a1;
	if (accel_dist + decel_dist > total_dist) { // no cruise phase. Recalcualte accel_dist based on decel_dist
		cruise_dist = 0;
	}
	else{
		cruise_dist = total_dist - accel_dist - decel_dist;
	}
	*factor_accel = v1 / a1;
	*factor_cruise = cruise_dist / v1;
	*factor_decel = v1 / a2;
};

/**
 *Generates a point using a 5th order / quintic polynomial.
 *\param coeffs : coefficients of the polynomial
 *\param time   : Time at whcih the point needs to be calculated
 *\param outp   : Output will be saved here 
 */
void robot::poly_7(vector8 coeffs, double time, double *outp) {
	
	*outp = coeffs.m[7]*pow(time, 7) + coeffs.m[6]*pow(time, 6) + coeffs.m[5]*pow(time, 5) + coeffs.m[4]*pow(time, 4) + coeffs.m[3]*pow(time, 3) + coeffs.m[2]*pow(time, 2) + coeffs.m[1]*time + coeffs.m[0];
	
};


/* Motion Generator
 *
 *
 *
 *
 *//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *This function will take the output position generated by the trajectory planner and actually execute it on the robot. It will generate pulses + dir to do this. 
 */
bool robot::motion_routine(TickType_t start_time, int axis_p, double factor_list[3]) {
	switch (trajecotry_mode) {
	case 1: { //Joint Mode
			double factor1, s, t_time;
			long int c_time;
			
			t_time = (double)(factor_list[0] + factor_list[1] + factor_list[2]);
			c_time = xTaskGetTickCountFromISR() - start_time;
			factor1 = (double)c_time / (configTICK_RATE_HZ) ;
			if (factor1 < t_time) {
				if (factor1 <= factor_list[0]) { // Accelerate
 					s = 0.5 * robotaxis_data.joint[axis_p].ACCEL*factor1*factor1;
				}
				else if (factor1 > factor_list[0] && factor1 < factor_list[0] + factor_list[1]) { // Cruise
					s = 0.5 * robotaxis_data.joint[axis_p].ACCEL*factor_list[0]*factor_list[0] + robotaxis_data.joint[axis_p].SPEED*(factor1 - factor_list[0]);
				}
				else if(factor1 >= factor_list[0] + factor_list[1] && factor1 < factor_list[0] + factor_list[1] + factor_list[2]){ // Decelerate
					s = 0.5 * robotaxis_data.joint[axis_p].ACCEL*factor_list[0]*factor_list[0] + robotaxis_data.joint[axis_p].SPEED*factor_list[1] + robotaxis_data.joint[axis_p].SPEED*(factor1 - factor_list[0] - factor_list[1]) - 0.5 * robotaxis_data.joint[axis_p].DECEL*(factor1 - factor_list[0] - factor_list[1])*(factor1 - factor_list[0] - factor_list[1]);
				}
				move(axis_p, s);
				return 0;
			}
			else {
				factor1 = t_time;
				s = 0.5 * robotaxis_data.joint[axis_p].ACCEL*factor_list[0]*factor_list[0] + robotaxis_data.joint[axis_p].SPEED*factor_list[1] + robotaxis_data.joint[axis_p].SPEED*(factor1 - factor_list[0] - factor_list[1]) - 0.5 * robotaxis_data.joint[axis_p].DECEL*(factor1 - factor_list[0] - factor_list[1])*(factor1 - factor_list[0] - factor_list[1]);
				move(axis_p, s);
				robotaxis_data.joint[axis_p].position = temp_p.m[0];
				return 1;
			}
		}
		break;    
	case 2: { // Cartesian Trajectory
		double factor1, s, t_time;
		long int c_time;

		t_time = (double)(factor_list[0] + factor_list[1] + factor_list[2]);
		c_time =  xTaskGetTickCountFromISR() - start_time;
		factor1 = (double)c_time / (configTICK_RATE_HZ);
		if (factor1 < t_time) {
			if (factor1 <= factor_list[0]) { // Accelerate
				s = 0.5 * robotaxis_data.LINEAR_ACCEL*factor1*factor1;
			}
			else if (factor1 > factor_list[0] && factor1 <= factor_list[0] + factor_list[1]) { // Cruise
				s = 0.5 * robotaxis_data.LINEAR_ACCEL*factor_list[0]*factor_list[0] + robotaxis_data.LINEAR_SPEED*(factor1 - factor_list[0]);
			}
			else if (factor1 > factor_list[0] + factor_list[1] && factor1 < factor_list[0] + factor_list[1] + factor_list[2]) { // Decelerate
				s = 0.5 * robotaxis_data.LINEAR_ACCEL*factor_list[0]*factor_list[0] + robotaxis_data.LINEAR_SPEED*factor_list[1] + robotaxis_data.LINEAR_SPEED*(factor1 - factor_list[0] - factor_list[1]) - 0.5 * robotaxis_data.LINEAR_DECEL*(factor1 - factor_list[0] - factor_list[1])*(factor1 - factor_list[0] - factor_list[1]);
			}
			move_invk(s);
			return 0;
		}
		else {
			s = 0.5 * robotaxis_data.LINEAR_ACCEL*factor_list[0]*factor_list[0] + robotaxis_data.LINEAR_SPEED*factor_list[1] + robotaxis_data.LINEAR_SPEED*(factor1 - factor_list[0] - factor_list[1]) - 0.5 * robotaxis_data.LINEAR_DECEL*(factor1 - factor_list[0] - factor_list[1])*(factor1 - factor_list[0] - factor_list[1]);
			move_invk(s);
			static vector3 current_pos, start_p, end_p;
			start_p = { T.m[0][3], T.m[1][3], T.m[2][3] };
			end_p = { point_buffer[j1].X, point_buffer[j1].Y, point_buffer[j1].Z };
			find_point_on_line(start_p, end_p, s, &current_pos);
			T.m[0][3] = current_pos.m[0];
			T.m[1][3] = current_pos.m[1];
			T.m[2][3] = current_pos.m[2];
			return 1;
		}
		}
		break;
	}
};

void robot::move(int axis_p, double target_position) {
	
	static vector3 current_target_pos, start_p, end_p;

	start_p = { robotaxis_data.joint[axis_p].position, 0, 0 };
	end_p = { temp_target, 0, 0 };
	find_point_on_line(start_p, end_p, target_position, &current_target_pos);
	
	static double diff_amount;
	diff_amount  = current_target_pos.m[0] -  temp_p.m[0];
	
	execute_pulses(axis_p, (double)diff_amount*robotaxis_data.joint[axis_p].UNITS);
	temp_p = current_target_pos;

};


void robot::move_invk(double target_position) {
	static vector3 current_target_pos, start_p, end_p;
	start_p = { T.m[0][3], T.m[1][3], T.m[2][3] };
	end_p = { point_buffer[inv_kin_buffer_pt].X, point_buffer[inv_kin_buffer_pt].Y, point_buffer[inv_kin_buffer_pt].Z };
	find_point_on_line(start_p, end_p, target_position, &current_target_pos);
	T.m[0][3] = current_target_pos.m[0];
	T.m[1][3] = current_target_pos.m[1];
	T.m[2][3] = current_target_pos.m[2];
	inverse_kin();
	static double diff_amount_j1, diff_amount_j2, diff_amount_j3, diff_amount_j4;
	diff_amount_j1  = inv_theta.m[j1] - robotaxis_data.joint[j1].position;
	diff_amount_j2  = inv_theta.m[j2] - robotaxis_data.joint[j2].position;
	diff_amount_j3  = inv_theta.m[j3] - robotaxis_data.joint[j3].position;
	diff_amount_j4  = (inv_theta.m[j4] - robotaxis_data.joint[j4].position)*(double)(5.1 / 90.0);
	
	execute_all_pulses(diff_amount_j1*robotaxis_data.joint[j1].UNITS, diff_amount_j2*robotaxis_data.joint[j2].UNITS, diff_amount_j3*robotaxis_data.joint[j3].UNITS*3, diff_amount_j4*robotaxis_data.joint[j4].UNITS);
	
	robotaxis_data.joint[j1].position = inv_theta.m[j1];
	robotaxis_data.joint[j2].position = inv_theta.m[j2];
	robotaxis_data.joint[j3].position = inv_theta.m[j3];
	robotaxis_data.joint[j4].position = inv_theta.m[j4];
	T.m[j1][3] = start_p.m[j1];
	T.m[j2][3] = start_p.m[j2];
	T.m[j3][3] = start_p.m[j3];
}

/**
 **This function will move the specified axis in forward direction forever until the motion is interrupted
 *\param axis_num what axis to perform motion on
 */
void robot::forward(int axis_num) {
	double time_to_speed = robotaxis_data.joint[axis_num].SPEED / robotaxis_data.joint[axis_num].ACCEL;
};


/**
 *This function will intrrupt any motion and stop any movement
 *\pram axis_num : what axis to stop 
 */
void robot::interrupt_motion(int axis_num){
	stop_motion = true;
};


/**
 *This function is the last layer. It will exeucte the pulses. 
 *\param axis_num : Axis number to move
 *\param amount : Amount of pulses to send
 */
void robot::execute_pulses(int axis_num, double amount){
	amount *= ((double)robotaxis_data.joint[axis_num].direction) ? -1.0 : 1.0;
	if (amount < 0) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[axis_num].GPIO_BASE), robotaxis_data.joint[axis_num].dir_pin, GPIO_PIN_SET);
		amount = -1.0*amount;
	}
	else if ((amount > 0.00001)){
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[axis_num].GPIO_BASE), robotaxis_data.joint[axis_num].dir_pin, GPIO_PIN_RESET);
	}

	amount += pulses_to_execute;
	if (amount < 1.0) pulses_to_execute += amount;
	else pulses_to_execute = amount - (int)amount;
	amount = (int)amount;
	
	for (int i = 0; i < 2*amount; i++) {
		if (last_pulse == true) {
			HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[axis_num].GPIO_BASE), robotaxis_data.joint[axis_num].pulse_pin, GPIO_PIN_SET);
			last_pulse = false;
		}
		else {
			HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[axis_num].GPIO_BASE), robotaxis_data.joint[axis_num].pulse_pin, GPIO_PIN_RESET);
			last_pulse = true;
		}
	}
	last_pulse = false;
};

void robot::execute_all_pulses(double amount_j1, double amount_j2, double amount_j3, double amount_j4){
	signed long j1_tracker, j2_tracker, j3_tracker, j4_tracker;
	bool movement_completed = false;
	bool j1_move_completed  = false;
	bool j2_move_completed  = false;
	bool j3_move_completed  = false;
	bool j4_move_completed  = false;
	j1_tracker = 0;
	j2_tracker = 0;
	j3_tracker = 0;
	j4_tracker = 0;
	amount_j1 *= ((double)robotaxis_data.joint[j1].direction) ? -1.0 : 1.0;
	amount_j2 *= ((double)robotaxis_data.joint[j2].direction) ? -1.0 : 1.0;
	amount_j3 *= ((double)robotaxis_data.joint[j3].direction) ? -1.0 : 1.0;
	amount_j4 *= ((double)robotaxis_data.joint[j4].direction) ? -1.0 : 1.0;
	if (amount_j1 < 0) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j1].GPIO_BASE), robotaxis_data.joint[j1].dir_pin, GPIO_PIN_SET);
		amount_j1 = -1*amount_j1;
	}
	else if((amount_j1 > 0.00001)) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j1].GPIO_BASE), robotaxis_data.joint[j1].dir_pin, GPIO_PIN_RESET);
	}
	if (amount_j2 < 0) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j2].GPIO_BASE), robotaxis_data.joint[j2].dir_pin, GPIO_PIN_SET);
		amount_j2 = -1*amount_j2;
	}
	else if ((amount_j2 > 0.00001)) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j2].GPIO_BASE), robotaxis_data.joint[j2].dir_pin, GPIO_PIN_RESET);
	}
	if (amount_j3 < 0) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j3].GPIO_BASE), robotaxis_data.joint[j3].dir_pin, GPIO_PIN_SET);
		amount_j3 = -1*amount_j3;
	}
	else if ((amount_j3 > 0.00001)) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j3].GPIO_BASE), robotaxis_data.joint[j3].dir_pin, GPIO_PIN_RESET);
	}
	if (amount_j4 < 0) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j4].GPIO_BASE), robotaxis_data.joint[j4].dir_pin, GPIO_PIN_SET);
		amount_j4 = -1*amount_j4;
	}
	else if ((amount_j4 > 0.00001)) {
		HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j4].GPIO_BASE), robotaxis_data.joint[j4].dir_pin, GPIO_PIN_RESET);
	}
	
	amount_j1 += pulses_pending_j1;
	if (amount_j1 < 1.0) pulses_pending_j1 += amount_j1;
	else pulses_pending_j1 = amount_j1 - (int)amount_j1;
	amount_j1 = (int)amount_j1;
	
	amount_j2 += pulses_pending_j2;
	if (amount_j2 < 1.0) pulses_pending_j2 += amount_j2;
	else pulses_pending_j2 = amount_j2 - (int)amount_j2;
	amount_j2 = (int)amount_j2;
	
	amount_j3 += pulses_pending_j3;
	if (amount_j3 < 1.0) pulses_pending_j3 += amount_j3;
	else pulses_pending_j3 = amount_j3 - (int)amount_j3;
	amount_j3 = (int)amount_j3;
	
	amount_j4 += pulses_pending_j4;
	if (amount_j4 < 1.0) pulses_pending_j4 += amount_j4;
	else pulses_pending_j4 = amount_j4 - (int)amount_j4;
	amount_j4 = (int)amount_j4;
	
	while(!movement_completed){
		if (j1_tracker < 2*amount_j1) {
			if (last_pulsej1 == true) {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j1].GPIO_BASE), robotaxis_data.joint[j1].pulse_pin, GPIO_PIN_SET);
				last_pulsej1 = false;
			}
			else {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j1].GPIO_BASE), robotaxis_data.joint[j1].pulse_pin, GPIO_PIN_RESET);
				last_pulsej1 = true;
			}
			j1_tracker++;
		}
		else{
			j1_move_completed = true;
			last_pulsej1 = false;
		}
		if (j2_tracker < 2*amount_j2) {
			if (last_pulsej1 == true) {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j2].GPIO_BASE), robotaxis_data.joint[j2].pulse_pin, GPIO_PIN_SET);
				last_pulsej2 = false;
			}
			else {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j2].GPIO_BASE), robotaxis_data.joint[j2].pulse_pin, GPIO_PIN_RESET);
				last_pulsej2 = true;
			}
			j2_tracker++;
		}
		else {
			j2_move_completed = true;
			last_pulsej2 = false;
		}
		if (j3_tracker < 2*amount_j3) {
			if (last_pulsej3 == true) {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j3].GPIO_BASE), robotaxis_data.joint[j3].pulse_pin, GPIO_PIN_SET);
				last_pulsej3 = false;
			}
			else {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j3].GPIO_BASE), robotaxis_data.joint[j3].pulse_pin, GPIO_PIN_RESET);
				last_pulsej3 = true;
			}
			j3_tracker++;
		}
		else {
			j3_move_completed = true;
			last_pulsej3 = false;
		}
		if (j4_tracker < 2*amount_j4) {
			if (last_pulsej4 == true) {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j4].GPIO_BASE), robotaxis_data.joint[j4].pulse_pin, GPIO_PIN_SET);
				last_pulsej4 = false;
			}
			else {
				HAL_GPIO_WritePin(((GPIO_TypeDef *) robotaxis_data.joint[j4].GPIO_BASE), robotaxis_data.joint[j4].pulse_pin, GPIO_PIN_RESET);
				last_pulsej4 = true;
			}
			j4_tracker++;
		}
		else {
			j4_move_completed = true;
			last_pulsej4 = false;
		}
		if(j1_move_completed && j2_move_completed && j3_move_completed && j4_move_completed){
			movement_completed = true;
		}
	}
};

void robot::floatPIDcal(double *setpoint, double *actual_position, double *output){
	static double pre_error = 0;
	static double integral = 0;
	double error;
	double derivative;

	//CaculateP,I,D
	error = *setpoint - *actual_position;
	//Incaseof errortoosmall then stop integration
	if(abs(error) > epsilon){
		integral = integral + error*delta_t;
	}
	derivative = (error - pre_error) / delta_t;
	*output = Kp*error + Ki*integral + Kd*derivative;
	//Saturation Filter
	if(*output > MAX){
		*output = MAX;
	}
	else if(*output < MIN){
		*output = MIN;
	}
	//Update error
	pre_error = error;
};