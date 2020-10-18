/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "string.h"
#include "stdio.h"

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
UART_HandleTypeDef huart1;
QueueHandle_t xMyQueue;
/* USER CODE BEGIN PV */
char *pcStrings;
uint8_t ucReceiveData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void vSenderTask(void *pvParameters);
void vReceiverTask(void *pvParameters);


/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &ucReceiveData, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xQueue */
  xMyQueue = xQueueCreate(5, sizeof(uint16_t));
  if (xMyQueue == 0)
  {
    pcStrings = "Unable to create Unsigned Short\r\n\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStrings, strlen(pcStrings), 200);
  }
  else
  {
    pcStrings = "Unsigned Short Queue created successfuly\r\n\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStrings, strlen(pcStrings), 200);
  }
  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Sender1Task */
  xTaskCreate(vSenderTask, "Sender 1", 128, (void *)111, 1, NULL);

  /* creation of Sender2Task */
  //xTaskCreate(vSenderTask, "Sender 2", 128, (void *)222, 2, NULL);

  /* creation of ReceiverTask */
  xTaskCreate(vReceiverTask, "Receiver", 128, NULL, 3, NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, &ucReceiveData, 1);
  uint16_t usValueToSend = 12345;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  char *pcStringSendFromISR = "Sent from ISR\r\n\r\n";
  if (ucReceiveData == 'r')
  {
    if (xQueueSendToFrontFromISR(xMyQueue, &usValueToSend, &xHigherPriorityTaskWoken) == pdPASS)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)pcStringSendFromISR, strlen(pcStringSendFromISR), 100);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_SenderTask */
/**
  * @brief  Function implementing the SenderTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SenderTask */
void vSenderTask(void *pvParameters)
{
  /* USER CODE BEGIN 5 */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(2000);
  uint32_t ulValueToSend;
  char *pcStringFromSender; 
  ulValueToSend = (uint32_t)pvParameters;
  /* Infinite loop */
  for(;;)
  {
    if (xQueueSendToFront(xMyQueue, &ulValueToSend, portMAX_DELAY)==pdPASS)
    {
      if (ulValueToSend == 111)
      {
        pcStringFromSender = "Successfully sent the number to the queue\n Leaving Sender 1\r\n\r\n"; 
      }
      else
      {
        pcStringFromSender = "Successfully sent the number to the queue\n Leaving Sender 2\r\n\r\n";         
      }   
    }
    HAL_UART_Transmit(&huart1, (uint8_t *) pcStringFromSender, strlen(pcStringFromSender), 500);
    vTaskDelay(xTicksToWait);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReceiverTask */
/**
* @brief Function implementing the ReceiverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiverTask */
void vReceiverTask(void *pvParameters)
{
  /* USER CODE BEGIN ReceiverTask */
  uint16_t usReceiveValue = 0;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(3000);
  char *pcStringFromReceiver;
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(xMyQueue, &usReceiveValue, portMAX_DELAY)!=pdPASS)
    {
      pcStringFromReceiver = "Error in Receiving from Queue\r\n\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)pcStringFromReceiver, strlen(pcStringFromReceiver), 300);
    }
    else
    {
      sprintf(pcStringFromReceiver,"Successfully Received the number %d to the Queue\n Leaving Receiver Task\r\n\r\n", usReceiveValue);
      HAL_UART_Transmit(&huart1, (uint8_t *)pcStringFromReceiver, strlen(pcStringFromReceiver), 500);
    } 
    vTaskDelay(xTicksToWait);      
  }
  /* USER CODE END ReceiverTask */
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
