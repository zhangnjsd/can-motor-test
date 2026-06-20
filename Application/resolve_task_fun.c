#include "resolve_task_fun.h"
#include "controller.h"
#include "main.h"
#include "recv_task_fun.h"

#define PRE_TIME_MS 1
#define DT7_CHANNEL_MAX 660.0f
#define ECD_ROUND 8191.0f
#define ECD_HALF_ROUND (ECD_ROUND / 2.0f)

static float ecd_format(float angle)
{
    while (angle < 0.0f) {
        angle += ECD_ROUND;
    }
    while (angle >= ECD_ROUND) {
        angle -= ECD_ROUND;
    }
    return angle;
}

static float ecd_shortest_error(float measure, float ref)
{
    float err = ref - measure;
    if (err > ECD_ROUND / 2.0f) {
        err -= ECD_ROUND;
    } else if (err < -ECD_ROUND / 2.0f) {
        err += ECD_ROUND;
    }
    return err;
}

void resolve_task_fun(void *argument) {
    
    PID_t pid_angle = {0};
    PID_t pid_speed = {0};

    float target_angle_ecd;
    float start_angle_ecd;
    uint8_t target_locked = 0;
    
    // ! Prevert boost!
    while (current_angle == 0) {}
    start_angle_ecd = (float)current_angle;
    target_angle_ecd = start_angle_ecd;

    // ? Speed args
    PID_Improve_Init(
        &pid_speed, 
        25000.0f,
        10000.0f,
        0.80f,

        32.0f,
        0.85f,
        0.0f,

        120.0f,
        240.0f,

        0.0f,
        0.0f,
        8,
        Integral_Limit | Trapezoid_Intergral
    );
    
    
    // ? Angle args
    PID_Improve_Init(
        &pid_angle, 
        400.0f,
        60.0f,
        0.30f,

        1.20f,
        0.01f,
        0.00f,

        120.0f,
        240.0f,

        0.0f,
        0.0f,
        0,
        Integral_Limit | ChangingIntegrationRate
    );

    uint32_t next_wake_tick = osKernelGetTickCount();
    for (;;) {
        float channel0 = float_constrain((float)dt7_data.channel0, -DT7_CHANNEL_MAX, DT7_CHANNEL_MAX);
        float target_offset_ecd = channel0 / DT7_CHANNEL_MAX * ECD_HALF_ROUND;
        target_angle_ecd = ecd_format(start_angle_ecd + target_offset_ecd);

        // * Angle loop (outer loop — MUST run first in cascade)
        //   Angle PID output = speed reference for the inner speed loop
        float angle_measure = (float)current_angle;
        float angle_err = ecd_shortest_error(angle_measure, target_angle_ecd);
        float speed_ref = PID_Calculate(&pid_angle, angle_measure, angle_measure + angle_err);
        target_angle_speed_ref = speed_ref;
        target_angle_dbg = target_angle_ecd;

        // * Speed loop (inner loop — uses fresh angle PID output)
        if (can_rx_fresh && !target_locked) {
            target_locked = 1;
        }

        if (target_locked) {
            float speed_measure = (float)((int16_t)current_speed);
            float speed_ref_cmd = target_angle_speed_ref;
            target_speed_dbg = speed_ref_cmd;

            float current_ref = PID_Calculate(&pid_speed, speed_measure, speed_ref_cmd);
            volt[0] = (int16_t)float_constrain(current_ref, -25000.0f, 25000.0f);
        } else {
            volt[0] = 0;
            target_speed_dbg = 0.0f;
        }

        next_wake_tick += PRE_TIME_MS;
        osDelayUntil(next_wake_tick);
    }
}