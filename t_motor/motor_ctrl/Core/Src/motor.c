#include "motor.h"

volatile uint8_t position_loop_enable = 1;
volatile uint8_t speed_loop_enable = 1;
volatile uint8_t z_homed = 0;

void motor_init(uint8_t motor_id)
{
    if (motor_id == Left_Motor)
    {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    }
    else if (motor_id == Right_Motor)
    {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    }
    else
    {
        return;
    }

    position_loop_enable = 0;
    speed_loop_enable = 0;
    z_homed = 0;

    motor_ctrl(motor_id, DIR_FORWARD, 0);
    PID_Clear();

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    Encoder_NewCnt = 0;
    Encoder_TotalCnt = 0;

    HAL_Delay(100);

    motor_ctrl(motor_id, DIR_FORWARD, Z_HOME_PWM);

    uint32_t start_time = HAL_GetTick();
    uint32_t last_check_time = HAL_GetTick();
    int32_t last_encoder_cnt = Encoder_TotalCnt;
    uint8_t stall_count = 0;

    while (1)
    {
        if (HAL_GetTick() - start_time > Z_HOME_TIMEOUT_MS)
        {
            motor_ctrl(motor_id, DIR_FORWARD, 0);
            position_loop_enable = 0;
            speed_loop_enable = 0;
            z_homed = 0;
            return;
        }

        if (HAL_GetTick() - last_check_time >= Z_STALL_CHECK_MS)
        {
            last_check_time = HAL_GetTick();

            int32_t now_encoder_cnt = Encoder_TotalCnt;
            int32_t delta = now_encoder_cnt - last_encoder_cnt;
            last_encoder_cnt = now_encoder_cnt;

            if (delta >= 0 && delta < Z_STALL_DELTA)
            {
                stall_count++;
            }
            else
            {
                stall_count = 0;
            }

            if (stall_count >= Z_STALL_CONFIRM_COUNT)
            {
                break;
            }
        }
    }

    motor_ctrl(motor_id, DIR_FORWARD, 0);
    HAL_Delay(100);

    motor_ctrl(motor_id, DIR_BACKWARD, Z_BACKOFF_PWM);
    HAL_Delay(Z_BACKOFF_TIME_MS);
    motor_ctrl(motor_id, DIR_BACKWARD, 0);

    HAL_Delay(200);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    Encoder_NewCnt = 0;
    Encoder_TotalCnt = 0;

    PID_Clear();

    z_homed = 1;
    position_loop_enable = 1;
    speed_loop_enable = 1;
}

void motor_ctrl(uint8_t motor_id, uint8_t direction, uint16_t speed)
{
    if (speed > MOTOR_PWM_MAX)
    {
        speed = MOTOR_PWM_MAX;
    }

    if (motor_id == Right_Motor)
    {
        HAL_GPIO_WritePin(motor2_dir_GPIO_Port,
                          motor2_dir_Pin,
                          (direction == DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    }
    else if (motor_id == Left_Motor)
    {
        HAL_GPIO_WritePin(motor1_dir_GPIO_Port,
                          motor1_dir_Pin,
                          (direction == DIR_FORWARD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    }
}
