#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "tim.h"

#define DIR_FORWARD  1   // 正转
#define DIR_BACKWARD 0   // 反转

#define Left_Motor   0   //左路电机
#define Right_Motor  1   //右路电机

#define Z_HOME_PWM              600     // 回零PWM，建议低速低力矩
#define Z_BACKOFF_PWM           350     // 回退PWM
#define Z_HOME_TIMEOUT_MS       10000    // 最大回零时间
#define Z_STALL_CHECK_MS        500      // 每50ms检查一次编码器变化
#define Z_STALL_CONFIRM_COUNT   10       // 连续5次几乎不动才认为堵转
#define Z_STALL_DELTA           2       // 50ms内编码器变化小于3，认为几乎不动
#define Z_BACKOFF_TIME_MS       200     // 回退时间，先用时间法，后面可改成位置法

extern volatile uint8_t position_loop_enable;   // 位置环使能
extern volatile uint8_t speed_loop_enable;      // 速度环使能
extern volatile uint8_t z_homed;                // Z轴是否回零完成
extern volatile int16_t  Encoder_NewCnt;
extern int32_t Encoder_TotalCnt;
void motor_init(uint8_t motor_id);
void motor_ctrl(uint8_t motor_id, uint8_t direction, uint16_t speed);
void PID_Clear(void);
#endif
