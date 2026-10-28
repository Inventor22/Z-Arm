
class Scara
{
public:
	/*
	1) int generation.
	=1 Z-Arm low configuration
	=2 Z-Arm high configuration
	2) float z_travel
	Set the up and down stroke to be 210 or 310 according to the actual model, and you need to pass positive value;

	=0 communication has not yet been established,this initialization is unsuccessful;
	=1 initializing;
	=2 generation parameter error;
	=3 encoder value error;
	=11 controlled by the mobile terminal, this initialization is not successful
	=12 z_travel transmission error
	*/
	int initial(int generation, float z_travel);	

	/*
	1) float *x, coordinate value of x (mm)
	2) float *y, coordinate value of y (mm)
	3) float *z, coordinate value of z (mm)
	4) float *angle1, angle value of joint 1 (deg)
	5) float *angle2, angle value of joint 2 (deg)
	6) float * rotation, angle value of joint 4 (deg)
	7) bool *communicate_success,
	=0 communication has not been connected
	=1 communication has been established
	8) bool *initial_finish,
	=0 initialization successful
	=1 initialization unsuccessful
	9) bool *servo_off_flag,
	=0 servo not closed
	=1 servo closed
	10) bool *move_flag,
	=0 the robot arm is in standby state
	=1 the robot arm is in motion state
	*/
	void get_scara_param(
		float *x, float *y, float*z,
		float *angle1,
		float *angle2,
		float *rotation,
		bool *communicate_success,
		bool *initial_finish,
		bool *servo_off_flag,
		bool *move_flag);

	// Initialize server
	void net_port_initial_auto();

	/*	
	Set J1 J2 joint arm length
	float l1
	J1 joint length, reserved parameters, you must introduce 200,
	float l2
	J1 joint length, introduce 200 generally for those with J4 rotation joint, and set according to the actual situation for those with no J4 rotation joint
	*/
	void  set_arm_length(float l1, float l2);
		
	/*
	Change attitude

	float speed
	The joint speed (deg/s) when transforming attitude, and the difference of the
	joint angles between the two attitudes will be judged.
	At the same time, divided by speed to get the motion time of each joint,
	and the longer time is the final movement time

	=0 the robot arm is running other instructions, this command is invalid
	=1 this command goes into effect, and the robot arm begins to move
	=2 the incoming speed is less than or equal to 0
	=3 not initialized yet
	=4 can't reach by the other attitude
	=6 servo not opened
	=11 mobile terminal is controlling

	*/
	int change_attitude(float speed);

	/*
	Uniaxial motion
	
	int axis
	Enter 1 or 2 or 3 or 4, corresponding to joint 1- joint 4 respectively
	float distance
	Moving distance relative to current position,
	When axis=3, distance unit is mm;when axis=1 or 2 or 4, distance unit is deg
	float speed
	Moving speed,
	When axis=3, speed unit is mm/s;when axis=1 or 2 or 4, speed unit is deg/s
	bool interpolation,
	Interpolation method,
	=0, polynomial interpolation
	=1, linear interpolation

	=0 the robotic arm is running other instructions, this command is invalid
	=1 this command goes into effect, and the robotic arm begins to move
	=2 setting speed is less than or equal to zero
	=3 not initialized yet
	=4 cannot reach the target position
	=5 output shaft number parameter error
	=6 the robotic arm servo not opened
	=11 mobile terminal is controlling
	*/
	int single_axis_move(
		int axis,
		float distance,
		float speed,
		bool interpolation);

	/*
	Represent four degrees of freedom x(mm), y(mm), z(mm), r(deg) of all the point coordinates
	in a section of trajectory with four float arrays, and indicate the total number of points
	and running speed, introduce into the trail_move function;
	Note: the linear distance between two adjacent points in the trajectory should be equal to 1mm

	intpoint_number
	Number of points to be executed
	float *x
	The first address of x coordinate array, and the unit of data in the array is mm
	float *y
	The first address of y coordinate array, and the unit of data in the array is mm
	float *z
	The first address of z coordinate array, and the unit of data in the array is mm
	float *r
	The first address of r coordinate array, and the unit of data in the array is deg
	float speed
	Running speed

	=0 the robotic arm is running other instructions, this command is invalid
	=1 this command goes into effect, and the robotic arm begins to move
	=2 setting speed is less than or equal to zero
	=3 not initialized yet
	=4 the first point in the trajectory goes beyond bounds
	=6 the robotic arm servo not opened
	=11 mobile terminal in controlling
	*/
	int trail_move(
		int point_number,
		float *x,
		float *y,
		float *z,
		float *r,
		float speed);

	/*
	float angle1
	The absolute angle of the target point joint 1, unit is deg
	float angle2
	The absolute angle of the target point joint 2, unit is deg
	float z
	The absolute coordinate of target point joint 3, unit is mm
	float rotation
	The absolute coordinate of target point joint 4, unit is deg
	float speed
	Running speed unit
	Judge the difference of the joint angles between the current position and the target point, divided by speed at the same time, to get the movement time of each joint, and take the longer time as the final movement time, and then inversely calculate the actual running speed of each joint

	=0 the robotic arm is running other instructions, this command is invalid
	=1 this command goes into effect, and the robotic arm begins to move
	=2 setting speed is less than or equal to zero
	=3 not initialized yet
	=4 the position point goes beyond bounds
	=6 the robotic arm servo not opened
	=11 mobile terminal is controlling
	*/
	int set_angle_move(
		float angle1,
		float angle2,
		float z,
		float rotation,
		float speed);

	/*
	float goal_x
	X coordinate value of the target point, unit is mm
	float goal_y
	Y coordinate value of the target point, unit is mm
	float goal_z
	Z coordinate value of the target point, unit is mm
	float goal rotation z
	J4 angle value of the target point, unit is deg
	float speed
	running speed mm/s
	float acceleration
	acceleration value in T shape interpolation,
	valid only when interpolation=2;
	int interpolation
	1 is s curve interpolation, and 2 is T curve interpolation
	intmove_mode
	=1 is MOVEJ
	The trajectory from the current position to the target position is a straight line (if it can arrive)
	=2 is MOVEL
	Each joint moves from the current position to the target position, and the intermediate movement trajectory is generally not a straight line

	=0 the robotic arm is running other instructions, this command is invalid
	=1 this command goes into effect, and the robot arm begins to move
	=2 setting speed is less than or equal to zero
	=3 not initialized yet
	=4 in the MOVEL movement, the intermediate process points go out of bounds and it cannot arrive, and the robotic arm will stop moving
	=6 robotic arm servo not opened
	=7 in the MOVEL movement, any intermediate process point cannot arrive by the robotic arm's current attitude (attitude), and the robotic arm will stop moving
	=8 setting acceleration is less than or equal to zero
	=9 interpolation mode parameter error
	=10 move_mode move mode error
	=11 mobile terminal is controlling
	*/
	int set_position_move(
		float goal_x,
		float goal_y,
		float goal_z,
		float rotation,
		float speed,
		float acceleration,
		int interpolation,
		int move_mode);
};