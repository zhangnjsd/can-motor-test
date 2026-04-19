#include "rad_task_fun.h"
#include "controller.h"
#include "main.h"

#define PRE_TIME_MS 5

void rad_task_fun(void *argument) {
    PID_t pid_angle = {0};
    float target_angle_ecd;
    
    while (current_angle == 0) {}
    target_angle_ecd = (float)current_angle;
    
    PID_Improve_Init(
        &pid_angle, 
        150.0f,
        0.0f,
        8.0f,

        0.4f,
        0.0f,
        0.0f,

        0.0f,
        0.0f,

        0.12f,
        0.12f,
        2,
        OutputFilter | DerivativeFilter
    );

    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;) {
        float angle_measure = (float)current_angle;
        float speed_ref = PID_Calculate(&pid_angle, angle_measure, target_angle_ecd);
        target_angle_speed_ref = speed_ref;
        target_angle_dbg = target_angle_ecd;

        next_wake_tick += PRE_TIME_MS;
        osDelayUntil(next_wake_tick);
    }
}