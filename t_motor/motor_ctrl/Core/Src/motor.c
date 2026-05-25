#include "motor.h"

volatile uint8_t position_loop_enable = 1;   // 位置环使能
volatile uint8_t speed_loop_enable = 1;      // 速度环使能
volatile uint8_t z_homed = 0;                // Z轴是否回零完成

//基于现在的接线  Z轴移液臂向上移动编码期数值增加
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
		

		//往编码器数值增大方向初始化
		
		
		// -------------------------------
    // 1. 关闭闭环，防止PID干扰回零
    // -------------------------------
    position_loop_enable = 0;
    speed_loop_enable = 0;

    motor_ctrl(motor_id, DIR_FORWARD, 0);

    PID_Clear();

    z_homed = 0;

    // 清除编码器计数
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    Encoder_NewCnt = 0;
    Encoder_TotalCnt = 0;

    HAL_Delay(100);

    // -------------------------------
    // 2. Z轴向上低速回零
    // 当前接线：Z轴向上，编码器数值增加
    // 所以回零方向使用 DIR_FORWARD
    // -------------------------------
    motor_ctrl(motor_id, DIR_FORWARD, Z_HOME_PWM);

    uint32_t start_time = HAL_GetTick();
    uint32_t last_check_time = HAL_GetTick();

    int32_t last_encoder_cnt = Encoder_TotalCnt;
    uint8_t stall_count = 0;

    while (1)
    {
        // 超时保护
        if (HAL_GetTick() - start_time > Z_HOME_TIMEOUT_MS)
        {
            motor_ctrl(motor_id, DIR_FORWARD, 0);
            z_homed = 0;

            // 这里可以设置错误标志
            // z_error = Z_HOME_TIMEOUT;

            position_loop_enable = 0;
            speed_loop_enable = 0;

            return;
        }

        // 每 Z_STALL_CHECK_MS 检查一次编码器变化
        if (HAL_GetTick() - last_check_time >= Z_STALL_CHECK_MS)
        {
            last_check_time = HAL_GetTick();

            int32_t now_encoder_cnt = Encoder_TotalCnt;
            int32_t delta = now_encoder_cnt - last_encoder_cnt;

            last_encoder_cnt = now_encoder_cnt;

            // 向上回零时，正常情况下 delta 应该为正
            // 如果连续多次 delta 很小，说明已经顶到机械上限，编码器不再变化
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

    // -------------------------------
    // 3. 检测到堵转，停止电机
    // -------------------------------
    motor_ctrl(motor_id, DIR_FORWARD, 0);
    HAL_Delay(100);

    // -------------------------------
    // 4. 向下回退一点，离开机械硬限位
    // -------------------------------
    motor_ctrl(motor_id, DIR_BACKWARD, Z_BACKOFF_PWM);
    HAL_Delay(Z_BACKOFF_TIME_MS);
    motor_ctrl(motor_id, DIR_BACKWARD, 0);

    HAL_Delay(200);

    // -------------------------------
    // 5. 回零完成后清零编码器和位置
    // 注意：清零点是在“回退后的位置”
    // -------------------------------
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    Encoder_NewCnt = 0;
    Encoder_TotalCnt = 0;

    

    PID_Clear();

    z_homed = 1;

    // -------------------------------
    // 6. 恢复闭环
    // -------------------------------
    position_loop_enable = 1;
    speed_loop_enable = 1;

		
		
    
		

//    motor_ctrl(motor_id, DIR_BACKWARD, 500);
//    HAL_Delay(1000);


//    motor_ctrl(motor_id, DIR_FORWARD, 0);
		
}


//speed范围是0-3600
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
    else if (motor_id == 2)
    {

    }
}

