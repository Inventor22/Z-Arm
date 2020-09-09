#ifndef SCARA_H
#define SCARA_H

#include "Scara.h"

#endif // !SCARA_H

int Scara::initial(int generation, float z_travel)
{
	return 0;
}

void Scara::get_scara_param(float * x, float * y, float * z, float * angle1, float * angle2, float * rotation, bool * communicate_success, bool * initial_finish, bool * servo_off_flag, bool * move_flag)
{
}

void Scara::net_port_initial_auto()
{
}

void Scara::set_arm_length(float l1, float l2)
{
}

int Scara::change_attitude(float speed)
{
	return 0;
}

int Scara::single_axis_move(int axis, float distance, float speed, bool interpolation)
{
	return 0;
}

int Scara::trail_move(int point_number, float * x, float * y, float * z, float * r, float speed)
{
	return 0;
}

int Scara::set_angle_move(float angle1, float angle2, float z, float rotation, float speed)
{
	return 0;
}

int Scara::set_position_move(float goal_x, float goal_y, float goal_z, float rotation, float speed, float acceleration, int interpolation, int move_mode)
{
	return 0;
}
