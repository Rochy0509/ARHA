#include "posControl.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor/motor_control.h"
#include "motor/myactuator.h"
#include "robotKinematics.h"
#include <string.h>

/* Pos control task configuration */
#define POS_CONTROL_TASK_NAME "PosControl"
#define POS_CONTROL_TASK_STACK (512U * 2U)
#define POS_CONTROL_TASK_PRIORITY (osPriorityAboveNormal)
#define POS_CONTROL_PERIOD_MS 1U /* control loop period in ms */

static void pos_control_task(void *argument);
static osThreadId_t posControlTaskHandle = NULL;

void init_pos_control(void)
{
    /* Create RTOS thread for position control */
    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(attr));
    attr.name = POS_CONTROL_TASK_NAME;
    attr.stack_size = POS_CONTROL_TASK_STACK;
    attr.priority = POS_CONTROL_TASK_PRIORITY;

    posControlTaskHandle = osThreadNew(pos_control_task, NULL, &attr);
    /* posControlTaskHandle will be NULL on failure */
}

static void pos_control_task(void *argument)
{
    (void)argument;
    // motor_clear_errors_all();
    // motor_rezero_position(LIMB_RIGHT_ARM, 2);
    // motor_rezero_position(LIMB_RIGHT_ARM, 6);
    motor_set_position(LIMB_RIGHT_ARM, 2, 0.0);
    motor_set_position(LIMB_RIGHT_ARM, 6, 0.0);
    osDelay(2000); /* Allow motors to reach initial position */



    float target_pos[2] = {0.0f, 0.1f}; /* Example target position */
    float *joints = inverse_kinematics(target_pos);
    motor_set_position(LIMB_RIGHT_ARM, 2, joints[0]);
    motor_set_position(LIMB_RIGHT_ARM, 6, joints[1]);
    // motor_set_velocity(LIMB_RIGHT_ARM, 2, 0.5);
    /* Periodic control loop */
    for (;;)
    {
        osDelay(POS_CONTROL_PERIOD_MS);
    }
}
