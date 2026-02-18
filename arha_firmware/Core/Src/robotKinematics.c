#include "robotKinematics.h"
#include <math.h>
#include <arm_math.h>

/* Link lengths for the 2-DOF planar arm (meters) */
const float lengths[2] = {0.1f, 0.1f};


float *inverse_kinematics(float *pos)
{
    static float joints[2];

    float x = pos[0];
    float y = pos[1];
    float L1 = lengths[0];
    float L2 = lengths[1];

    /* law of cosines for theta2 */
    float r2 = x * x + y * y;
    float cos_theta2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;
    float theta2 = acosf(cos_theta2);

    /* theta1 = atan2(y,x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2)) */
    float k1 = L1 + L2 * arm_cos_f32(theta2);
    float k2 = L2 * arm_sin_f32(theta2);
    float theta1 = atan2f(y, x) - atan2f(k2, k1);

    joints[0] = theta1;
    joints[1] = theta2;
    return joints;
}

float *inverse_velocity(float *joints, float *endeffector_vel)
{
    /* Compute joint velocities for desired end-effector velocity using damped least squares (DLS)
     * This is more robust near singularities and reduces large spikes from J^{-1}.
     */
    static float joint_vels[2];
    float theta1 = joints[0];
    float theta2 = joints[1];

    float J00 = -lengths[0] * arm_sin_f32(theta1) - lengths[1] * arm_sin_f32(theta1 + theta2);
    float J01 = -lengths[1] * arm_sin_f32(theta1 + theta2);
    float J10 =  lengths[0] * arm_cos_f32(theta1) + lengths[1] * arm_cos_f32(theta1 + theta2);
    float J11 =  lengths[1] * arm_cos_f32(theta1 + theta2);

    /* Damped least squares: pinv = J^T * inv(J*J^T + lambda^2 I) */
    const float lambda = 0.05f; /* damping (tunable) */

    float JJt00 = J00*J00 + J01*J01;
    float JJt01 = J00*J10 + J01*J11;
    float JJt11 = J10*J10 + J11*J11;

    float M00 = JJt00 + lambda*lambda;
    float M01 = JJt01;
    float M11 = JJt11 + lambda*lambda;

    float detM = M00 * M11 - M01 * M01;
    if (fabsf(detM) < 1e-9f) {
        joint_vels[0] = 0.0f;
        joint_vels[1] = 0.0f;
        return joint_vels;
    }

    /* inv(M) */
    float invM00 =  M11 / detM;
    float invM01 = -M01 / detM;
    float invM10 = -M01 / detM;
    float invM11 =  M00 / detM;

    /* tmp = invM * endeffector_vel */
    float tmp0 = invM00 * endeffector_vel[0] + invM01 * endeffector_vel[1];
    float tmp1 = invM10 * endeffector_vel[0] + invM11 * endeffector_vel[1];

    /* joint_vels = J^T * tmp */
    joint_vels[0] = J00 * tmp0 + J10 * tmp1;
    joint_vels[1] = J01 * tmp0 + J11 * tmp1;
    return joint_vels;
}

float *get_end_effector_position(float *joints)
{
    static float pos[3] = {0.0f, 0.0f, 0.0f};
    pos[0] = lengths[0] * arm_cos_f32(joints[0]) + lengths[1] * arm_cos_f32(joints[0] + joints[1]);
    pos[1] = lengths[0] * arm_sin_f32(joints[0]) + lengths[1] * arm_sin_f32(joints[0] + joints[1]);
    return pos;
}

float *get_endeffector_velocity(float *joints, float *joint_vels)
{
    static float vel[3] = {0.0f, 0.0f, 0.0f};
    /* Compute end-effector velocity using Jacobian */
    float theta1 = joints[0];
    float theta2 = joints[1];
    float dtheta1 = joint_vels[0];
    float dtheta2 = joint_vels[1];

    /* Jacobian for 2-link planar arm */
    float J[4] = {
        -lengths[0] * arm_sin_f32(theta1) - lengths[1] * arm_sin_f32(theta1 + theta2), -lengths[1] * arm_sin_f32(theta1 + theta2),
         lengths[0] * arm_cos_f32(theta1) + lengths[1] * arm_cos_f32(theta1 + theta2),  lengths[1] * arm_cos_f32(theta1 + theta2)
    };

    vel[0] = J[0] * dtheta1 + J[1] * dtheta2; // x velocity
    vel[1] = J[2] * dtheta1 + J[3] * dtheta2; // y velocity
    return vel;
}