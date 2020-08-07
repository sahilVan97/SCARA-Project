#ifndef  kinematics_h
#define  kinematics_h

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string>
#include <iostream>
#include <math.h>

/// [https://www.daniweb.com/programming/software-development/threads/289812/can-t-link-gsl-properly#post1247197]
/**
 *This structure stores the Denavit - Hartenbug parameter in a floating point array.
 */
 struct robot_def {
	double dh_parameters[6][4];
};


/**
 *This structure stores the target point floating point variables x, y, z, u, v, w.
 */
struct target_point {
	double X, Y, Z, U, V, W;
};

struct matrix4x4 {
	double m[4][4];
};


struct vector8 {
	double m[8];
};

struct vector6 {
	double m[6];
};

struct vector4 {
	double m[4];
};

struct vector3 {
	double m[3];
};
/*
 *This structure is to save data for an axis
 */
struct axis_data {
	double UNITS; /// Pulses per revolution for  axis. Must be in degrees
	double fs_limit; /// Forward Software Limit
	double rs_limit; /// Reverse Software Limit
	int GPIO_BASE_fslim; /// GPIO port number for axis I/O
	int fs_limit_sw; /// Forward Limit switch Input number
	int GPIO_BASE_rslim; /// GPIO port number for axis I/O
	int rs_limit_sw; /// Reverser Limit switch Input number
	int GPIO_BASE; /// GPIO port number for axis I/O
	int pulse_pin; /// Pulse pin input number
	int dir_pin; /// direction pin input number;
	int enable_pin; /// Enable pin input number
	double SPEED; /// Speed of axis
	double ACCEL; /// Acceleration of axis
	double DECEL; /// Deceleration of axis
	double JERK; /// Jerk of axis.
	bool direction = 0; /// Direction of rotation for axis
	
	double position; /// current position
	//GPIO_TypeDef *GPIO_PORT = ((GPIO_TypeDef *) GPIOA_BASE);
};


/**
 *This stucture consists of all the robot axes \ref axis_data and other robot_specific paramters
 */
struct robot_data {
	axis_data joint[6];
	double LINEAR_SPEED; /// Speed for Cartesian/Linear movements
	double LINEAR_ACCEL;
	double LINEAR_DECEL;
	double ORIENTATION_SPEED; /// Speed for Orientation
	double JOINT_SPEED; /// Speed for Joint movements
};


/**
 *This is the master class that should be used to setup a robot. This class has Kinematics, Trajectory Planning and Motion Planning inside it. 
 */
class robot {
public:
	#define x 0 
	#define y 1
	#define z 2 
	#define j1 0 
	#define j2 1
	#define j3 2
	#define j4 3
	#define j5 4 
	#define j6 5 
	robot(int kinematic_group_number, double *lengths, robot_data robot_axis);
	
	void handle(void);
	
	void home(void);
	
	void activate(void);
	
	void deactivate(void);
	
	void MOVEL(target_point p, int *buffer_pointer);
	
	void MOVEJ(target_point p,  int *buffer_pointer, int axisp = -1);
	
	void MOVEC(target_point p, target_point q, int *buffer_pointer);
	
	void EmergencyStop(void);
	bool HOME_COMPLETE = false;
	
	int trajecotry_mode;
	bool j1_status = true;
	bool j2_rs_status = true;
	bool j2_fs_status = true;
	bool j3_rs_status = true;
	bool j3_fs_Status = true;
	int buffer_pt;
protected:
	/* Kinemaitcs and Dynamics Functions
	 **/
	#define buffer_ptr 0
	int home_status = 0;
	void kinematics_routine(void);
	void forward_kin(void);
	void inverse_kin(void);
	void jacobian(void);
	void dhmat(double a, double alpha, double d, double theta, matrix4x4 *targetm);
	void quat_2_eul(void);
	void eul_2_quat(void);
	
	matrix4x4 T;
	
	vector6 inv_theta;
	vector6 fwd_theta;
	int kinematic_group; 
	double scara_a1, scara_a2, scara_d4;
	robot_def dh_params;
	matrix4x4 A1, A2, A3, A4, A5, A6;
	matrix4x4 T01, T02, T03, T04, T05, T06;
	vector3 position, orientation; // Current end effector cartesian position/orientation. 
	robot_data robotaxis_data; // Robot data for interal usage
	bool j1_fs_limit_exceeded, j1_rs_limit_exceeded;
	bool j2_fs_limit_exceeded, j2_rs_limit_exceeded;
	bool j3_fs_limit_exceeded, j3_rs_limit_exceeded;
	bool j4_fs_limit_exceeded, j4_rs_limit_exceeded;
	bool j5_fs_limit_exceeded, j5_rs_limit_exceeded;
	bool j6_fs_limit_exceeded, j6_rs_limit_exceeded;

	#define type_movej 100 
	#define type_movel 101
	#define type_movec 102
	#define scara_robot 0
	#define SixAxis_anthropomorphic_robot 1
	
	int inv_kin_buffer_pt = 0;
	/* Trajectory Generator Functions
	 **/
	void trajectory_routine(void);
	
	void dist(vector3 p1, vector3 p2, double *target);
	
	void find_point_on_line(vector3 p1, vector3 p2, double distance, vector3 *output_point);
	
	void heptic_solver(double t1, double x1, double v1, double a1, double jerk1, double t2, double x2, double v2, double a2, double jerk2, vector8 *target_m);
	
	void APA(double a_max, double j_max, double v1, double v2, double p1, double D, vector6 coeffs1, vector6 coeffs2, vector6 coeffs3, double *dt, double *t_end, double *Dap);
	
	void SAPA(double p1, double v1, double p2, double v2, double a_max, double t_max, vector6 coeffs1, vector6 coeffs2, vector6 coeffs3, vector6 coeffs4, double *t_glide, double *t_end, double *Dsap);
	
	void CAM_trajecotry(double t0, double p0, double v0, double t1, double p1, double v1, double time, vector8 *output_point);
	
	void trap_trajec(double x1, double v1, double a1, double a2, double x2, double *factor_accel, double *factor_cruise, double *factor_decel);
	bool neg_dist;
	void poly_7(vector8 coeffs, double time, double *outp);
	
	int move_buffer[6];
	target_point point_buffer[6];
	int axis_num_buffer[6];
	
	/*Motion Functions
	 **/
	TickType_t xLastWakeTime;
	bool motion_routine(TickType_t start_time, int axis_p, double factor_list[3]);
	void move_invk(double target_position);
	void move(int axis_p, double target_position);
	void forward(int axis_num);
	void reverse(int axis_num);
	void interrupt_motion(int axis_num);
	void execute_pulses(int axis_num, double amount);
	void execute_all_pulses(double amount_j1, double amount_j2, double amount_j3, double amount_j4);
	void floatPIDcal(double *setpoint, double *actual_position, double *output);
	double trajec1_s;
	double diff_delta, pulses_to_execute;
	double pulses_pending_j1, pulses_pending_j2, pulses_pending_j3, pulses_pending_j4;
	bool last_pulse, execute_done, stop_motion, last_pulsej1, last_pulsej2, last_pulsej3, last_pulsej4;
	long int pulse_num;
	vector3 temp_p;
	double temp_target;
	
	#define PI 3.14159265358979323846
	#define PI_DIV_2 1.57079632679
	#define PI_BY_ONE80 0.01745329251
	#define ONE80_BY_PI 57.2957795131
	#define PI_SQUARED 9.86960440109	
	#define epsilon 0.01
	#define delta_t 0.001
	#define Kp 1
	#define Ki 1
	#define Kd 1
	#define MIN 2 // No single pulse should than larger than 2degrees
	#define MAX 2
#
/*!
	\var HOME_COMPLETE
	\brief Flag to confirm if homing is finished.
	\var inv_theta  
	\brief Joint angles calculated by inverse kinematics
	\var inv_theta  
	\brief Joint angles calculated by inverse kinematics
	\var fwd_theta   
	\brief Joint angles calculated by forward kinematics
	\var T      
	\brief Transformation Matrix of the robot end effector
	\var scara_a1
	\brief Link lengths specific to the SCARA robot.
	\var scara_a2  
	\brief Link lengths specific to the SCARA robot. 
	\var scara_d4    
	\brief Link lengths specific to the SCARA robot.
	\var dh_params  
	\brief DH parameters for internal use
	\var A1    
	\brief Joint 1 Matrix
	\var A2            
	\brief Joint 2 Matrix
	\var A3        
	\brief Joint 3 Matrix
	\var A4   
	\brief Joint 4 Matrix
	\var A5  
	\brief Joint 5 Matrix
	\var A6  
	\brief Joint 6 Matrix
	\var T01   
	\brief Transformation matrices between axis 0 and 1. Used in Jacobians calcualtions.
	\var T02    
	\brief Transformation matrices between axis 0 and 2. Used in Jacobians calcualtions.
	\var T03    
	\brief Transformation matrices between axis 0 and 3. Used in Jacobians calcualtions.
	\var T04    
	\brief Transformation matrices between axis 0 and 4. Used in Jacobians calcualtions.
	\var T05      
	\brief Transformation matrices between axis 0 and 5. Used in Jacobians calcualtions.
	\var T06          
	\brief Transformation matrices between axis 0 and 6. Used in Jacobians calcualtions.
	\var T06      
	\brief Transformation matrices between axis 0 and 6. Used in Jacobians calcualtions.
	\var position 
	\brief Current end effector position.
	\var orientaion   
	\brief Current end effector orientation.
	\var robotaxis_data 
	\brief Robot data for interal usage
	\def x         
	\brief to address x coordinate in arrays.
	\def y          
	\brief to address y coordinate in arrays.
	\def z    
	\brief to address z coordinate in arrays.
	\def j1 
	\brief to address first joint in arrays.
	\def j2 
	\brief to address second joint in arrays.
	\def j3 
	\brief to address third joint in arrays.
	\def j4 
	\brief to address fourth joint in arrays.
	\def j5 
	\brief to address fifth joint in arrays.
	\def j6 
	\brief to address sixth joint in arrays.
	\def type_movej
	\brief Joint Move type to co-relate \ref point_buffer and \ref move_type. This is used when MOVEJ is called.
	\def type_movel
	\brief Joint Move type to co-relate \ref point_buffer and \ref move_type. This is used when MOVEL is called.
	\def type_movec
	\brief Joint Move type to co-relate \ref point_buffer and \ref move_type. This is used when MOVEC is called.
	\def move_buffer
	\brief Move buffer. This will contain what type of move is loaded
	\def point_buffer
	\brief Points buffer. This will contain the actual target points
	\def axis_num_buffer
	\brief Buffer to hold what axis a move needs to  be executed on
	*/ 
};


#endif