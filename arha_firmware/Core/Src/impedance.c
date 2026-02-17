#include "impedance.h"
#include "motor_control.h"
#include "string.h"
#include "FreeRTOS.h"
#include <math.h>

#include <arm_math.h>

#include "main.h" // for huart3
#include <stdio.h>


static void impedance_control_loop(void *pvParameters) {
    float32_t desired_pos = 0.0f;
    float32_t desired_vel = 0.0f;
    float32_t stiffness = 10.0f; // Nm/rad
    float32_t damping = 1.0f;    // Nms/rad

    float32_t current_pos = 0.0f;
    float32_t current_vel = 0.0f;
    float32_t current_force = 0.0f;

    float32_t pos_error = 0.0f;
    float32_t vel_error = 0.0f;
    while (1) {
        
        pos_error = desired_pos - current_pos;
        vel_error = desired_vel - current_vel;

        vTaskDelay(pdMS_TO_TICKS(10)); // Run at 100 Hz
    }
}


float* get_end_effector_position() {
    // implement Forward Kin
    static float pos[3] = {0.0f, 0.0f, 0.0f};
    return pos;
}

void impendance_task_init(void) {
    xTaskCreate(impedance_control_loop, "ImpedanceControl", 256, NULL, 24, NULL);
}
