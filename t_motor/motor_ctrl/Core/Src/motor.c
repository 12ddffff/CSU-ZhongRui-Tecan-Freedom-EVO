#include "motor.h"



void motor_init(uint8_t motor_id)
{

    if(motor_id == Left_Motor)
    {
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    }
    else if(motor_id == Right_Motor)
    {
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    }


    motor_ctrl(motor_id, DIR_FORWARD, 0);
}


//speed·¶Î§ÊÇ0-3600
void motor_ctrl(uint8_t motor_id, uint8_t direction, uint16_t speed)
{

    if (motor_id == Right_Motor)
    {
        if (direction == DIR_FORWARD)
        {

            HAL_GPIO_WritePin(motor2_dir_GPIO_Port, motor2_dir_Pin, GPIO_PIN_SET);

        }
        else
        {
            HAL_GPIO_WritePin(motor2_dir_GPIO_Port, motor2_dir_Pin, GPIO_PIN_RESET);

        }

        
         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    }
    else if (motor_id == Left_Motor)
    {
        if (direction == DIR_FORWARD)
        {
            HAL_GPIO_WritePin(motor1_dir_GPIO_Port, motor1_dir_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(motor1_dir_GPIO_Port, motor1_dir_Pin, GPIO_PIN_RESET);
        }

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    }
}

