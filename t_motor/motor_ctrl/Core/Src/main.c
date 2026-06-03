/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_CPR               2800.0f
#define RX_BUFFER_SIZE            64U
#define Z_AXIS_ENCODER_SIGN       (-1)
#define AUTO_HOME_ON_BOOT         1U

#define STATUS_SEND_PERIOD_MS     20U
#define DATA_PROCESS_PERIOD_MS    15U

#define SPEED_KP_INIT             10.0f
#define SPEED_KI_INIT             1.2f
#define SPEED_KD_INIT             0.0f
#define SPEED_MAX_PWM             3599
#define SPEED_MAX_INTEGRAL        2500
#define SPEED_TARGET_LIMIT        230.0f

#define POSITION_KP_INIT          0.08f
#define POSITION_KI_INIT          0.0f
#define POSITION_KD_INIT          0.0f
#define POSITION_MAX_SPEED        130
#define POSITION_MAX_INTEGRAL     0
#define POSITION_TARGET_LIMIT_DEG 3600.0f
#define POSITION_DEADBAND_COUNT   4
#define SPEED_STOP_DEADBAND_COUNT 2

#define DEFAULT_TARGET_ANGLE_DEG  180.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t uart_dma_tick = 0;
uint32_t data_process_tick = 0;

char tx_buffer[96];
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t uart_rx_length = 0;
volatile uint8_t uart_rx_frame_ready = 0;
char safe_buffer[RX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DataProcess_Task(void);
void PID_Clear(void);
static int32_t AngleToEncoderCount(float angle_deg);
static float EncoderCountToAngle(int32_t encoder_count);
static float ClampFloat(float value, float min_value, float max_value);
static int32_t AbsI32(int32_t value);
static void UART_StartReceiveToIdle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    Speed_PID_Init(&z_motor.speed_pid,
                   SPEED_KP_INIT,
                   SPEED_KI_INIT,
                   SPEED_KD_INIT,
                   SPEED_MAX_PWM,
                   SPEED_MAX_INTEGRAL);
    Position_PID_Init(&z_motor.position_pid,
                      POSITION_KP_INIT,
                      POSITION_KI_INIT,
                      POSITION_KD_INIT,
                      POSITION_MAX_SPEED,
                      POSITION_MAX_INTEGRAL);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim4);
    motor_init(Right_Motor);

#if AUTO_HOME_ON_BOOT
    motor_home(Right_Motor);
#endif

    z_motor.target_angle_deg = DEFAULT_TARGET_ANGLE_DEG;
    z_motor.position_pid.target = AngleToEncoderCount(z_motor.target_angle_deg);
#if AUTO_HOME_ON_BOOT
    if (z_motor.z_homed != 0)
    {
        z_motor.position_loop_enable = 1;
        z_motor.speed_loop_enable = 1;
    }
#endif
    UART_StartReceiveToIdle();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (HAL_GetTick() - uart_dma_tick >= STATUS_SEND_PERIOD_MS)
        {
            uart_dma_tick = HAL_GetTick();

            if (huart1.gState == HAL_UART_STATE_READY)
            {
                int tx_len;
                int32_t speed_feedback;
                int32_t speed_target;
                float speed_rps;
                float current_angle_deg;
                float target_angle_deg;

                __disable_irq();
                speed_feedback = z_motor.speed_pid.feedback;
                speed_target = z_motor.speed_pid.target;
                speed_rps = z_motor.speed_rps;
                current_angle_deg = z_motor.current_angle_deg;
                target_angle_deg = z_motor.target_angle_deg;
                __enable_irq();

                tx_len = snprintf(tx_buffer, sizeof(tx_buffer), "pid:%ld,%ld,%.2f,%.2f,%.2f\n",
                                  (long)speed_feedback,
                                  (long)speed_target,
                                  speed_rps,
                                  current_angle_deg,
                                  target_angle_deg);
                if (tx_len > 0)
                {
                    if (tx_len >= (int)sizeof(tx_buffer))
                    {
                        tx_len = sizeof(tx_buffer) - 1;
                    }
                    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buffer, (uint16_t)tx_len);
                }
            }
        }

        if (HAL_GetTick() - data_process_tick >= DATA_PROCESS_PERIOD_MS)
        {
            data_process_tick = HAL_GetTick();
            DataProcess_Task();
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4)
    {
        return;
    }

    z_motor.encoder_delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    z_motor.speed_pid.feedback = z_motor.encoder_delta;
    z_motor.speed_rps = (float)z_motor.encoder_delta / 5.0f / ENCODER_CPR * 1000.0f;
    z_motor.encoder_total += z_motor.encoder_delta;

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    z_motor.position_pid.feedback = z_motor.encoder_total;
    z_motor.current_angle_deg = EncoderCountToAngle(z_motor.position_pid.feedback);

    if (z_motor.speed_loop_enable == 0)
    {
        return;
    }

    if (z_motor.position_loop_enable != 0)
    {
        int32_t position_error = z_motor.position_pid.target - z_motor.position_pid.feedback;

        if (AbsI32(position_error) <= POSITION_DEADBAND_COUNT)
        {
            z_motor.position_pid.integral = 0;
            z_motor.position_pid.last_err = 0;
            z_motor.speed_pid.target = 0;
            z_motor.speed_pid.integral = 0;

            if (AbsI32(z_motor.speed_pid.feedback) <= SPEED_STOP_DEADBAND_COUNT)
            {
                z_motor.speed_pid.output = 0;
                z_motor.pwm = 0;
                motor_ctrl(Right_Motor, DIR_FORWARD, 0);
                return;
            }
        }
        else
        {
            z_motor.speed_pid.target = Position_PID_Calc(&z_motor.position_pid);
        }
    }

    z_motor.pwm = Speed_PID_Calc(&z_motor.speed_pid);

    if (z_motor.pwm >= 0)
    {
        motor_ctrl(Right_Motor, DIR_FORWARD, z_motor.pwm);
    }
    else
    {
        motor_ctrl(Right_Motor, DIR_BACKWARD, -z_motor.pwm);
    }
}

void PID_Clear(void)
{
    z_motor.position_pid.target = 0;
    z_motor.position_pid.feedback = 0;
    z_motor.position_pid.err = 0;
    z_motor.position_pid.last_err = 0;
    z_motor.position_pid.integral = 0;
    z_motor.position_pid.output = 0;

    z_motor.speed_pid.target = 0;
    z_motor.speed_pid.feedback = 0;
    z_motor.speed_pid.err = 0;
    z_motor.speed_pid.last_err = 0;
    z_motor.speed_pid.integral = 0;
    z_motor.speed_pid.output = 0;

    z_motor.pwm = 0;
}

static int32_t AngleToEncoderCount(float angle_deg)
{
    return (int32_t)(angle_deg / 360.0f * ENCODER_CPR * (float)Z_AXIS_ENCODER_SIGN);
}

static float EncoderCountToAngle(int32_t encoder_count)
{
    return (float)encoder_count / ENCODER_CPR * 360.0f * (float)Z_AXIS_ENCODER_SIGN;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static int32_t AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void UART_StartReceiveToIdle(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void DataProcess_Task(void)
{
    float target = 0.0f;
    char parse_buffer[RX_BUFFER_SIZE];
    uint16_t rx_length;

    if (uart_rx_frame_ready == 0)
    {
        return;
    }

    __disable_irq();
    rx_length = uart_rx_length;
    if (rx_length >= RX_BUFFER_SIZE)
    {
        rx_length = RX_BUFFER_SIZE - 1;
    }
    memcpy(parse_buffer, safe_buffer, rx_length);
    parse_buffer[rx_length] = '\0';
    uart_rx_frame_ready = 0;
    __enable_irq();

    switch (parse_buffer[0])
    {
        case 'T':
            if (sscanf(parse_buffer, "T_speed:%f", &target) == 1)
            {
                target = ClampFloat(target, -SPEED_TARGET_LIMIT, SPEED_TARGET_LIMIT);
                __disable_irq();
                z_motor.position_loop_enable = 0;
                z_motor.speed_loop_enable = 1;
                z_motor.speed_pid.target = (int32_t)target;
                z_motor.speed_pid.integral = 0;
                z_motor.speed_pid.last_err = 0;
                __enable_irq();
            }
            break;

        case 'P':
            if (sscanf(parse_buffer, "P_speed:%f", &target) == 1)
            {
                __disable_irq();
                z_motor.speed_pid.Kp = target;
                z_motor.speed_pid.integral = 0;
                __enable_irq();
            }
            else if (sscanf(parse_buffer, "P_position:%f", &target) == 1)
            {
                __disable_irq();
                z_motor.position_pid.Kp = target;
                z_motor.position_pid.integral = 0;
                __enable_irq();
            }
            break;

        case 'I':
            if (sscanf(parse_buffer, "I_speed:%f", &target) == 1)
            {
                __disable_irq();
                z_motor.speed_pid.Ki = target;
                z_motor.speed_pid.integral = 0;
                __enable_irq();
            }
            else if (sscanf(parse_buffer, "I_position:%f", &target) == 1)
            {
                __disable_irq();
                z_motor.position_pid.Ki = target;
                z_motor.position_pid.integral = 0;
                __enable_irq();
            }
            break;

        case 'D':
            if (sscanf(parse_buffer, "D_speed:%f", &target) == 1)
            {
                __disable_irq();
                z_motor.speed_pid.Kd = target;
                z_motor.speed_pid.integral = 0;
                __enable_irq();
            }
            else if (sscanf(parse_buffer, "D_position:%f", &target) == 1)
            {
                __disable_irq();
                z_motor.position_pid.Kd = target;
                z_motor.position_pid.integral = 0;
                __enable_irq();
            }
            break;

        case 'A':
            if (sscanf(parse_buffer, "A_position:%f", &target) == 1)
            {
                target = ClampFloat(target, -POSITION_TARGET_LIMIT_DEG, POSITION_TARGET_LIMIT_DEG);
                __disable_irq();
                z_motor.position_loop_enable = 1;
                z_motor.speed_loop_enable = 1;
                z_motor.target_angle_deg = target;
                z_motor.position_pid.target = AngleToEncoderCount(z_motor.target_angle_deg);
                z_motor.position_pid.integral = 0;
                z_motor.position_pid.last_err = 0;
                z_motor.speed_pid.integral = 0;
                z_motor.speed_pid.last_err = 0;
                __enable_irq();
            }
            break;

        case 'M':
            if (sscanf(parse_buffer, "M_speed:%f", &target) == 1)
            {
                target = ClampFloat(target, 0.0f, (float)MOTOR_PWM_MAX);
                __disable_irq();
                z_motor.speed_pid.max_output = (int32_t)target;
                z_motor.speed_pid.integral = 0;
                __enable_irq();
            }
            else if (sscanf(parse_buffer, "M_position:%f", &target) == 1)
            {
                target = ClampFloat(target, 0.0f, SPEED_TARGET_LIMIT);
                __disable_irq();
                z_motor.position_pid.max_output = (int32_t)target;
                z_motor.position_pid.integral = 0;
                z_motor.speed_pid.integral = 0;
                __enable_irq();
            }
            break;

        case 'S':
            if (strcmp(parse_buffer, "S_stop") == 0)
            {
                __disable_irq();
                z_motor.position_loop_enable = 0;
                z_motor.speed_loop_enable = 0;
                PID_Clear();
                __enable_irq();
                motor_ctrl(Right_Motor, DIR_FORWARD, 0);
            }
            break;

        case 'Z':
            if (strcmp(parse_buffer, "Z_home") == 0)
            {
                motor_home(Right_Motor);
            }
            break;

        default:
            break;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart1)
    {
        return;
    }

    if (Size >= RX_BUFFER_SIZE)
    {
        Size = RX_BUFFER_SIZE - 1;
    }

    if (uart_rx_frame_ready == 0)
    {
        memcpy(safe_buffer, rx_buffer, Size);
        safe_buffer[Size] = '\0';
        uart_rx_length = Size;
        uart_rx_frame_ready = 1;
    }

    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    UART_StartReceiveToIdle();
}
/* USER CODE END 4 */



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
