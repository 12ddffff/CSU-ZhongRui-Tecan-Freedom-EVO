#include "motor.h"

Motor_State_TypeDef z_motor;

static int32_t MotorAbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

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

    z_motor.position_loop_enable = 0;
    z_motor.speed_loop_enable = 0;
    z_motor.z_homed = 0;

    motor_ctrl(motor_id, DIR_FORWARD, 0);
    PID_Clear();

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    z_motor.encoder_delta = 0;
    z_motor.encoder_total = 0;
    z_motor.speed_rps = 0.0f;
    z_motor.current_angle_deg = 0.0f;
    z_motor.target_angle_deg = 0.0f;
}

void motor_home(uint8_t motor_id)
{
    if (motor_id != Left_Motor && motor_id != Right_Motor)
    {
        return;
    }

    z_motor.position_loop_enable = 0;
    z_motor.speed_loop_enable = 0;
    z_motor.z_homed = 0;

    motor_ctrl(motor_id, DIR_FORWARD, 0);
    PID_Clear();

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    z_motor.encoder_delta = 0;
    z_motor.encoder_total = 0;

    HAL_Delay(100);

    motor_ctrl(motor_id, DIR_FORWARD, Z_HOME_PWM);

    uint32_t start_time = HAL_GetTick();
    uint32_t last_check_time = HAL_GetTick();
    int32_t last_encoder_cnt = z_motor.encoder_total;
    uint8_t stall_count = 0;

    while (1)
    {
        if (HAL_GetTick() - start_time > Z_HOME_TIMEOUT_MS)
        {
            motor_ctrl(motor_id, DIR_FORWARD, 0);
            z_motor.position_loop_enable = 0;
            z_motor.speed_loop_enable = 0;
            z_motor.z_homed = 0;
            return;
        }

        if (HAL_GetTick() - last_check_time >= Z_STALL_CHECK_MS)
        {
            last_check_time = HAL_GetTick();

            int32_t now_encoder_cnt = z_motor.encoder_total;
            int32_t delta = now_encoder_cnt - last_encoder_cnt;
            last_encoder_cnt = now_encoder_cnt;

            if (MotorAbsI32(delta) < Z_STALL_DELTA)
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
    z_motor.encoder_delta = 0;
    z_motor.encoder_total = 0;

    PID_Clear();

    z_motor.z_homed = 1;
    z_motor.position_loop_enable = 0;
    z_motor.speed_loop_enable = 0;
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
