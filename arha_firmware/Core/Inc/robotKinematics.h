#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <stdbool.h>

/* Link lengths for the 2-DOF planar arm (meters) */
extern const float lengths[2];


// task space
float *inverse_kinematics(float *pos);
float *inverse_velocity(float *joints, float *endeffector_vel);


// joint space
float* get_end_effector_position(float *joints);
float *get_endeffector_velocity(float *joints, float *joint_vels);

#endif /* ROBOT_KINEMATICS_H */