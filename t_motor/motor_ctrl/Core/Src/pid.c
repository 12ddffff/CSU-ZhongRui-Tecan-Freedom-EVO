#include "pid.h"

void Speed_PID_Init(Speed_PID_TypeDef *pid, float Kp, float Ki, float Kd, int32_t max_out, int32_t max_i)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->max_output = max_out;
    pid->max_integral = max_i;

    pid->target = 0;
    pid->feedback = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
    pid->output = 0;
}

int32_t Speed_PID_Calc(Speed_PID_TypeDef *pid)
{
    // 1. 计算误差
    pid->err = pid->target - pid->feedback;

    // 2. 比例项
    int32_t P = pid->Kp * pid->err;

    // 3. 积分项 + 积分限幅
    pid->integral += pid->err;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    int32_t I = pid->Ki * pid->integral;

    // 4. 微分项
    int32_t D = pid->Kd * (pid->err - pid->last_err);
    pid->last_err = pid->err;

    // 5. 计算输出
    pid->output = P + I + D;

    // 6. 输出限幅（防止PWM超限）
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    return pid->output;
}

void Position_PID_Init(Position_PID_TypeDef *pid, float Kp, float Ki, float Kd, int32_t max_speed, int32_t max_i)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    // 位置环的最大输出，实际上就是限制了电机在跑长距离时的“最高限速”
    pid->max_output = max_speed; 
    pid->max_integral = max_i;

    pid->target = 0;
    pid->feedback = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
    pid->output = 0;
}

int32_t Position_PID_Calc(Position_PID_TypeDef *pid)
{
    // 1. 计算位置误差
    pid->err = pid->target - pid->feedback;

    // 2. 比例项
    int32_t P = pid->Kp * pid->err;

    // 3. 积分项 + 积分限幅 (虽然位置环通常不用 I，但为了结构完整保留)
    pid->integral += pid->err;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    int32_t I = pid->Ki * pid->integral;

    // 4. 微分项
    int32_t D = pid->Kd * (pid->err - pid->last_err);
    pid->last_err = pid->err;

    // 5. 计算输出 (这个输出其实就是算出来的期望速度)
    pid->output = P + I + D;

    // 6. 输出限幅（极其重要：防止位置偏差太大时，要求电机以无限大的速度运行）
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    return pid->output;
}
