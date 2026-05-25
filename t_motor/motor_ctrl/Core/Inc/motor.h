#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "tim.h"

#define DIR_FORWARD  1   // 正转
#define DIR_BACKWARD 0   // 反转

#define Left_Motor   0   //左路电机
#define Right_Motor  1   //右路电机

void motor_init(uint8_t motor_id);
void motor_ctrl(uint8_t motor_id, uint8_t direction, uint16_t speed);

#endif
