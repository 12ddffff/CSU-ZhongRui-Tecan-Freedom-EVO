#ifndef PID_H

#define PID_H
#include "main.h"
typedef struct
{
    int32_t target;     // 目标速度（5ms脉冲数）
    int32_t feedback;   // 反馈速度（5ms脉冲数）
    int32_t err;        // 当前误差
    int32_t last_err;   // 上一次误差
    
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数

    int32_t integral;   // 积分累加
    int32_t max_integral;// 积分限幅
    int32_t output;     // 输出值
    int32_t max_output; // 最大输出限幅（PWM最大值）
} Speed_PID_TypeDef;

// 初始化速度环PID
void Speed_PID_Init(Speed_PID_TypeDef *pid, float Kp, float Ki, float Kd, int32_t max_out, int32_t max_i);

// 速度环PID计算
int32_t Speed_PID_Calc(Speed_PID_TypeDef *pid);

typedef struct
{
    int32_t target;       // 目标位置（累计脉冲数，比如 10000 个脉冲代表 10mm）
    int32_t feedback;     // 反馈位置（编码器当前累计脉冲数）
    int32_t err;          // 当前位置误差
    int32_t last_err;     // 上一次误差
    
    float Kp;             // 比例系数 (位置环通常只用 Kp！)
    float Ki;             // 积分系数 (位置环极少用，通常设为 0)
    float Kd;             // 微分系数 (通常设为 0，除非机械惯量极大需要加一点点 D 来抑制超调)

    int32_t integral;     // 积分累加
    int32_t max_integral; // 积分限幅
    
    int32_t output;       // 输出值（**注意：这里的输出将直接作为速度环的 target**）
    int32_t max_output;   // 最大输出限幅（**限制电机的最高运行速度**）
} Position_PID_TypeDef;

// 声明函数
void Position_PID_Init(Position_PID_TypeDef *pid, float Kp, float Ki, float Kd, int32_t max_speed, int32_t max_i);
int32_t Position_PID_Calc(Position_PID_TypeDef *pid);

#endif
