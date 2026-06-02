#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "tim.h"

#define DIR_FORWARD   1U
#define DIR_BACKWARD  0U

#define Left_Motor    0U
#define Right_Motor   1U

#define MOTOR_PWM_MAX            3599U

/* Z axis homing parameters */
#define Z_HOME_PWM               600U
#define Z_BACKOFF_PWM            350U
#define Z_HOME_TIMEOUT_MS        10000U
#define Z_STALL_CHECK_MS         500U
#define Z_STALL_CONFIRM_COUNT    10U
#define Z_STALL_DELTA            2
#define Z_BACKOFF_TIME_MS        200U

extern volatile uint8_t position_loop_enable;
extern volatile uint8_t speed_loop_enable;
extern volatile uint8_t z_homed;
extern volatile int16_t Encoder_NewCnt;
extern volatile int32_t Encoder_TotalCnt;

void motor_init(uint8_t motor_id);
void motor_ctrl(uint8_t motor_id, uint8_t direction, uint16_t speed);
void PID_Clear(void);

#endif
