#include "resolve_task_fun.h"
#include "controller.h"
#include "main.h"

#define PRE_TIME_MS 1

void rad_task_fun(void *argument) {
    
    PID_t pid_angle = {0};
    PID_t pid_speed = {0};

    float target_angle_ecd;
    uint8_t target_locked = 0;
    
    while (current_angle == 0) {}
    target_angle_ecd = (float)current_angle;

    // ? Speed init
    PID_Improve_Init(
        &pid_speed, 
        25000.0f,
        8000.0f,
        1.2f,

        30.0f,
        1.2f,
        4.0f,

        80.0f,
        200.0f,

        0.05f,
        0.03f,
        8,
        Integral_Limit | Trapezoid_Intergral | DerivativeFilter | OutputFilter
    );
    
    
    // ? Angle init
    PID_Improve_Init(
        &pid_angle, 
        150.0f,
        0.0f,
        1.2f,

        0.4f,
        0.0f,
        0.0f,

        0.0f,
        0.0f,

        0.12f,
        0.12f,
        0,
        OutputFilter | DerivativeFilter
    );

    uint32_t next_wake_tick = osKernelGetTickCount();
    for (;;) {
        // * Speed range
        if (can_rx_fresh && !target_locked) {
            target_locked = 1;
        }

        if (target_locked) {
            float speed_measure = (float)((int16_t)current_speed);
            float speed_ref = target_angle_speed_ref;
            target_speed_dbg = speed_ref;

            float current_ref = PID_Calculate(&pid_speed, speed_measure, speed_ref);
            volt[0] = (int16_t)float_constrain(current_ref, -25000.0f, 25000.0f);
        } else {
            volt[0] = 0;
            target_speed_dbg = 0.0f;
        }

        // * Angle range
        float angle_measure = (float)current_angle;
        float speed_ref = PID_Calculate(&pid_angle, angle_measure, target_angle_ecd);
        target_angle_speed_ref = speed_ref;
        target_angle_dbg = target_angle_ecd;

        next_wake_tick += PRE_TIME_MS;
        osDelayUntil(next_wake_tick);
    }
}