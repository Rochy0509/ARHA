#include "robotKinematics.h"
#include <math.h>
/* use standard libm functions instead of CMSIS-DSP variants to avoid
   requiring the DSP library at link time */


// lengths in meters
const float lengths[2] = {0.1f, 0.1f};
float *inverse_kinematics(float *pos)
{
    static float joints[2];

    float x = pos[0];
    float y = pos[1];
    float L1 = lengths[0];
    float L2 = lengths[1];

    // calc theta 1
    joints[1] = acosf((x * x + y * y - L1 * L1 - L2 * L2) / (2.0f * L1 * L2));
    joints[0] = atan2f(y, x) - atan2f(L2 * sinf(joints[1]), L1 + L2 * cosf(joints[1]));
    return joints;
}


float *get_end_effector_position(float *joints)
{
    static float pos[3] = {0.0f, 0.0f, 0.0f};
    pos[0] = lengths[0] * cosf(joints[0]) + lengths[1] * cosf(joints[0] + joints[1]);
    pos[1] = lengths[0] * sinf(joints[0]) + lengths[1] * sinf(joints[0] + joints[1]);
    return pos;
}

float *inverse_velocity(float *joints, float *endeffector_vel)
{
    /* Compute joint velocities for desired end-effector velocity using damping
     * least squares to handle near-singular configurations. */
    static float joint_vels[2];
    float theta1 = joints[0];
    float theta2 = joints[1];

    float J00 = -lengths[0] * sinf(theta1) - lengths[1] * sinf(theta1 + theta2);
    float J01 = -lengths[1] * sinf(theta1 + theta2);
    float J10 =  lengths[0] * cosf(theta1) + lengths[1] * cosf(theta1 + theta2);
    float J11 =  lengths[1] * cosf(theta1 + theta2);

    const float lambda = 0.05f;

    /* J * J^T */
    float JJt00 = J00 * J00 + J01 * J01;
    float JJt01 = J00 * J10 + J01 * J11;
    float JJt11 = J10 * J10 + J11 * J11;

    /* form (J J^T + λ² I) */
    float M00 = JJt00 + lambda * lambda;
    float M01 = JJt01;
    float M11 = JJt11 + lambda * lambda;

    float det = M00 * M11 - M01 * M01;
    if (fabsf(det) < 1e-9f) {
        joint_vels[0] = 0.0f;
        joint_vels[1] = 0.0f;
        return joint_vels;
    }

    /* inverse of 2×2 matrix M */
    float invM00 =  M11 / det;
    float invM01 = -M01 / det;
    float invM10 = -M01 / det;
    float invM11 =  M00 / det;

    /* tmp = invM * endeffector_vel */
    float tmp0 = invM00 * endeffector_vel[0] + invM01 * endeffector_vel[1];
    float tmp1 = invM10 * endeffector_vel[0] + invM11 * endeffector_vel[1];

    /* joint_vels = J^T * tmp */
    joint_vels[0] = J00 * tmp0 + J10 * tmp1;
    joint_vels[1] = J01 * tmp0 + J11 * tmp1;
    return joint_vels;
}

float *get_endeffector_velocity(float *joints, float *joint_vels)
{
    static float vel[3] = {0.0f, 0.0f, 0.0f};
    float theta1 = joints[0];
    float theta2 = joints[1];
    float d1      = joint_vels[0];
    float d2      = joint_vels[1];

    /* Jacobian for 2-link planar arm */
    float J00 = -lengths[0] * sinf(theta1) - lengths[1] * sinf(theta1 + theta2);
    float J01 = -lengths[1] * sinf(theta1 + theta2);
    float J10 =  lengths[0] * cosf(theta1) + lengths[1] * cosf(theta1 + theta2);
    float J11 =  lengths[1] * cosf(theta1 + theta2);

    vel[0] = J00 * d1 + J01 * d2; // x velocity
    vel[1] = J10 * d1 + J11 * d2; // y velocity
    return vel;
}

// float *get_endeffector_velocity(float *joints, float *joint_vels)
// {
//     static float vel[3] = {0.0f, 0.0f, 0.0f};
//     /* Compute end-effector velocity using Jacobian */
//     float theta1 = joints[0];
//     float theta2 = joints[1];
//     float dtheta1 = joint_vels[0];
//     float dtheta2 = joint_vels[1];

//     /* Jacobian for 2-link planar arm */
//     float J[4] = {
//         -lengths[0] * arm_sin_f32(theta1) - lengths[1] * arm_sin_f32(theta1 + theta2), -lengths[1] * arm_sin_f32(theta1 + theta2),
//          lengths[0] * arm_cos_f32(theta1) + lengths[1] * arm_cos_f32(theta1 + theta2),  lengths[1] * arm_cos_f32(theta1 + theta2)
//     };

//     vel[0] = J[0] * dtheta1 + J[1] * dtheta2; // x velocity
//     vel[1] = J[2] * dtheta1 + J[3] * dtheta2; // y velocity
//     return vel;
// }