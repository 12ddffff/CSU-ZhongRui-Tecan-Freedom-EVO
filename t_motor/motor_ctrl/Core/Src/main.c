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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int16_t  Encoder_NewCnt;
int32_t Encoder_TotalCnt = 0;
float R_S = 0.0;
Speed_PID_TypeDef speed_pid;
Position_PID_TypeDef position_pid;
int16_t PWM = 0;
uint32_t uart_dma_tick = 0;
uint32_t data_process_tick = 0;
char tx_buffer[64];
float angle = 0;
float target_angle = 0;
float ENCODER_CPR = 2800.0f;
float total_angle; //总角度  距离






#define RX_BUFFER_SIZE  64

uint8_t rx_buffer[RX_BUFFER_SIZE];

volatile uint16_t uart_rx_length = 0;
volatile uint8_t uart_rx_frame_ready = 0;

char safe_buffer[RX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DataProcess_Task(void);

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
    motor_init(Right_Motor);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    Speed_PID_Init(&speed_pid, 10.0f, 1.2f, 0.0f, 3599, 2500);
    Position_PID_Init(&position_pid, 0.08f, 0.0f, 0.0f, 130, 0);
    position_pid.target = 1400;
    //speed_pid.target = 50;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);

	// 可选：关闭半传输中断，防止接收一半就触发回调
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    //24V电压90%占空比编码器输出值为230
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        if (HAL_GetTick() - uart_dma_tick >= 20)
        {
            uart_dma_tick = HAL_GetTick();

            if (huart1.gState == HAL_UART_STATE_READY)
            {
                angle = (float) position_pid.feedback / 2800 *360;
                sprintf((char *)tx_buffer,"pid:%d,%d,%.2f,%.2f,%.2f\n",speed_pid.feedback,speed_pid.target,R_S,angle,target_angle);
                HAL_UART_Transmit_DMA(&huart1,(uint8_t *)tx_buffer, strlen((char *)tx_buffer));
            }
        }
				
				if (HAL_GetTick() - data_process_tick >= 15)
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
//每5ms中断一次
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM4)
    {
        Encoder_NewCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        speed_pid.feedback = Encoder_NewCnt;
        R_S =(float)Encoder_NewCnt/5/2800*1000;
        //只读一相的上升沿和下降沿
        //5ms读到的数值除以5，再除以2800(一圈的输出值),乘1000，结果是转/s
        //PID运算就用5ms读数值去闭环，不用换算，要显示转速时候再换
        Encoder_TotalCnt += Encoder_NewCnt;
			
				//total_angle = Encoder_TotalCnt/2800 * 360;
			
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        position_pid.feedback = Encoder_TotalCnt;

			
			 // 闭环关闭时，只更新编码器，不做PID输出   //现在的方案回零速度环也关了，或者也可以改成回零时候速度环开着
			//但是开速度环有个问题  堵转时候积分会一致累加  不如开环
        if (position_loop_enable == 0 || speed_loop_enable == 0)
        {
            return;
        }
				
        speed_pid.target = Position_PID_Calc(&position_pid);
				
			

        PWM = Speed_PID_Calc(&speed_pid);
        if(PWM > 0)
        {
            motor_ctrl(Right_Motor,DIR_FORWARD,PWM);
        }
        else
        {
            motor_ctrl(Right_Motor,DIR_BACKWARD,-PWM);
        }

    }
}





 void PID_Clear(void)
{
    position_pid.target = 0;
    position_pid.feedback = 0;
    position_pid.err = 0;
    position_pid.last_err = 0;
    position_pid.integral = 0;
    position_pid.output = 0;

    speed_pid.target = 0;
    speed_pid.feedback = 0;
    speed_pid.err = 0;
    speed_pid.last_err = 0;
    speed_pid.integral = 0;
    speed_pid.output = 0;

    PWM = 0;
}


void DataProcess_Task(void)
{
    float target = 0;

    if (uart_rx_frame_ready == 0)
    {
        return;
    }

    uart_rx_frame_ready = 0;

    uint16_t rx_length = uart_rx_length;

    if (rx_length >= RX_BUFFER_SIZE)
    {
        rx_length = RX_BUFFER_SIZE - 1;
    }

    memcpy(safe_buffer, rx_buffer, rx_length);
    safe_buffer[rx_length] = '\0';

    switch (safe_buffer[0])
    {
        case 'T':
            if (sscanf(safe_buffer, "T_speed:%f", &target) == 1)
            {
                speed_pid.target = (int32_t)target;
            }
            break;

        case 'P':
            sscanf(safe_buffer, "P_speed:%f", &speed_pid.Kp);
            break;

        case 'I':
            sscanf(safe_buffer, "I_speed:%f", &speed_pid.Ki);
            break;

        case 'D':
            sscanf(safe_buffer, "D_speed:%f", &speed_pid.Kd);
            break;

        case 'A':
            if (sscanf(safe_buffer, "A_position:%f", &target_angle) == 1)
            {
                position_pid.target = (int32_t)(-(target_angle / 360.0f * 2800.0f));
							//将输入取反 以满足上位机给定数值增大而Z轴向下
            }
            break;
						


				
					break;

        default:
            break;
    }

    memset(rx_buffer, 0, RX_BUFFER_SIZE);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        uart_rx_length = Size;
        uart_rx_frame_ready = 1;
    }
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
