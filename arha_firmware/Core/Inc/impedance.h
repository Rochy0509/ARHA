#ifndef IMPEDANCE_H
#define IMPEDANCE_H

void impendance_task_init(void);

/*
* Placeholder for forward kinematics function to get end-effector position.
* In a real implementation, this would compute the position based on current joint states.
*/
float* get_end_effector_position();

#endif // IMPEDANCE_H