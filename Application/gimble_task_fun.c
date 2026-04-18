#include "gimble_task_fun.h"
#include "controller.h"

#define TARGET_SPEED_RPM 100.0f

void gimble_task_fun(void *argument) {
    // For transferring data to motor
    can_filter_init();
    // VULT_DEF: -25000 ~ 25000
    // Only 1 motor, set volt[0] as variable, others are 0
    PID_t pid_angle, pid_speed;
    int16_t volt[4] = {0};
    float target_angle_ecd = 0.0f;
    uint8_t target_locked = 0;

    PID_Improve_Init(
        &pid_speed, 
        25000.0f,                       // max_out: 输出给电机的最大电流值或PWM值
        5000.0f,                 // intergral_limit: 速度环积分限幅，防止大幅过冲
        1.0f,                          // deadband: 速度波动小于1可以忽略

        15.0f,
        0.5f,
        0.01f,

        10.0f,                               // A: 变速积分区间1
        50.0f,                               // B: 变速积分区间2 (误差>50时不积分)

        0.01f,                    // output_lpf_rc: 输出滤波，让电机运转更丝滑
        0.05f,                 // derivative_lpf_rc: 微分滤波，减小速度噪声
        10,                           // ols_order: 速度本身波动大，用10次采样最小二乘平滑
        1
    );
    PID_Improve_Init(
        &pid_angle, 
        8000.0f,                        // max_out: 这里的输出是内环的期望速度，不要超过电机最大转速
        0.0f,                    // intergral_limit: 角度环一般不用积分，给0
        5.0f,                           // deadband: 设置死区为5

        1.2f,
        0.0f,
        0.1f,

        0.0f,
        0.0f,

        0.0f,
        0.1f,                   // derivative_lpf_rc: 角度微分滤波
        20,                             // ols_order: 位置信号相对稳定，采样数可多点
        1
    );
    for (;;) {
        // 内环使用速度环，外环使用角度环
        // 速度环使用 PI/PID 控制，速度环误差不可过大
        // 角度环使用 P/PD 控制，能完全收敛与目标值重合 (~5编码值)

        if (can_rx_fresh && !target_locked) {
            target_angle_ecd = (float)current_angle;
            target_locked = 1;
        }

        target_angle_dbg = target_angle_ecd;

        if (target_locked) {
            float speed_measure = (float)((int16_t)current_speed);
            float speed_ref = TARGET_SPEED_RPM;
            target_speed_dbg = speed_ref;

            float current_ref = PID_Calculate(&pid_speed, speed_measure, speed_ref);
            volt[0] = (int16_t)float_constrain(current_ref, -25000.0f, 25000.0f);
        } else {
            volt[0] = 0;
            target_speed_dbg = 0.0f;
        }

        volt[1] = 0;
        volt[2] = 0;
        volt[3] = 0;
        CAN_Send(&hcan1, volt, CAN_ID);

        osDelay(10);
    }
}