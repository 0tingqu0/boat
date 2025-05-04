/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nrf24l01.h"
#include "driver.h"
#include "gps.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// #define ADC_CHANNELS     4     // 摇杆通道数
// #define FILTER_WINDOW 8
uint16_t values[4];
char message[20] = "";

int RX_BUFF[4] = {0};       // 改为int类型存储解析后的数据
char TowardsData[10] = "";  // 方向
char ForwardsData[10] = ""; // 油门

char remode1[50] = "";
char remode2[50] = "";
char data_ready = 0;

uint8_t g_TxMode = 0, g_UartRxFlag = 0;
char g_UartRxBuffer[100] = {0};
char g_RF24L01RxBuffer[20] = {0};
uint8_t conversion = 0;        // nrf24l01转换标志
uint8_t g_RF24L01TxBuffer[20]; // nRF24L01发送缓冲区

extern uint8_t gps_buffer[512];  // GPS数据缓冲区
extern uint8_t gps_buffer_index; // GPS数据缓冲区索引
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void parse_input_str(char *str, int arr[4])
{
  char *token = strtok(str, ",");
  for (int i = 0; i < 4 && token != NULL; i++)
  {
    arr[i] = atoi(token);
    token = strtok(NULL, ",");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t i = 0;
  uint32_t j = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //  HAL_SYSTICK_IRQHandler();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // 启动定时器中断
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // 设置占空比50%
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // 设置占空比50%

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // 设置占空比50%
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // 设置占空比50%

  //	OLED_Init();

  // RF24L01引脚初始化
  NRF24L01_Gpio_Init();

  // 检测nRF24L01
  while (NRF24L01_check_DMA() == 0)
    ;
  RF24L01_Init_DMA();
  RF24L01_Set_Mode_DMA(MODE_TX);            // 发送模式
  
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (0 != NRF24L01_RxPacket_DMA(g_RF24L01RxBuffer)) // 接收字节
    {
      parse_input_str(g_RF24L01RxBuffer, RX_BUFF); // 解析接收数据

      update_motion_control(RX_BUFF); // 更新电机控制

      i = 0; // 计数器清零

      // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    else
    {
      i++;
      conversion++; // nrf24l01转换标志
      if (i >= 10)
      {
        quiescent(); // 进入静止状态
        i = 0;
      }
      if (conversion > 3) // 这里的condition是一个条件判断，具体条件需要根据实际情况定义
      {

        RF24L01_Set_Mode_DMA(MODE_TX); // 发送模式
                                       
        do
        {
          snprintf(gps_buffer , 512 , "%d" , g_RF24L01TxBuffer[0]=j);
          if (NRF24L01_TxPacket_DMA(gps_buffer, 20) == TX_OK) // 发送数据
          {
            conversion = 0; // 重置转换标志
          }
          j++;
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        } while (conversion >= 3); // 发送数据

        RF24L01_Set_Mode_DMA(MODE_RX); // 接收模式
      }
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
