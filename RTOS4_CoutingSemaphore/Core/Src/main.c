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
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stdio.h"
#include "string.h"
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

TaskHandle_t xTask1Handle;
TaskHandle_t xTask2Handle;
TaskHandle_t xTask3Handle;
TaskHandle_t xTask4Handle;
SemaphoreHandle_t CountingSemaphoreHandle = NULL;
/* USER CODE BEGIN PV */
char *pcString;
uint16_t usRecource[3] = {111, 222, 333};
uint8_t ucIndex = 0;
uint8_t ucReceiveChar;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void vStartTask1(void *pvParameters);
void vStartTask2(void *pvParameters);
void vStartTask3(void *pvParameters);
void vStartTask4(void *pvParameters);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
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
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &ucReceiveChar, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of CountingSemaphore */
  CountingSemaphoreHandle = xSemaphoreCreateCounting(3, 0);
  if (CountingSemaphoreHandle != NULL)
  {
    pcString = "Counting semaphore created successfuly.\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcString, strlen(pcString), 200);
  }
  else
  {
    pcString = "Unable to creat counting semaphore.\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcString, strlen(pcString), 200);
  }
  
  

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  xTaskCreate(vStartTask1, "Task 1", 128, NULL, 1, &xTask1Handle);

  /* creation of Task2 */
  xTaskCreate(vStartTask2, "Task 2", 128, NULL, 2, &xTask2Handle);

  /* creation of Task3 */
  xTaskCreate(vStartTask3, "Task 3", 128, NULL, 3, &xTask3Handle);  

  /* creation of Task4 */
  xTaskCreate(vStartTask4, "Task 4", 128, NULL, 4, &xTask4Handle);
  if ((xTask1Handle && xTask2Handle && xTask3Handle && xTask4Handle) !=NULL)
  {
    pcString = "All Task created successfully.\r\n\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcString, strlen(pcString), 200);
  }
  else
  {
    pcString = "All Task didn't created.\r\n\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcString, strlen(pcString), 200);
  }
  
  

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
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  HAL_UART_Receive_IT(&huart1, &ucReceiveChar, 1);
  if (ucReceiveChar == 'r')
  {
    xSemaphoreGiveFromISR(CountingSemaphoreHandle, &xHigherPriorityTaskWoken);   
    xSemaphoreGiveFromISR(CountingSemaphoreHandle, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(CountingSemaphoreHandle, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(CountingSemaphoreHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void vStartTask1(void *pvParameters)
{
  /* USER CODE BEGIN 5 */
  char *pcStringTask1;
  /* Infinite loop */
  for(;;)
  {
    pcStringTask1 = pvPortMalloc(20*sizeof(pcStringTask1));
    sprintf(pcStringTask1, "Entered task 1\r\nAbout to acquired the semaphore\r\nTokens avaiable are: %d\r\n\r\n"
            , (uint8_t)uxSemaphoreGetCount(CountingSemaphoreHandle));
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask1, strlen(pcStringTask1), 500);
    xSemaphoreTake(CountingSemaphoreHandle, portMAX_DELAY);
    sprintf(pcStringTask1, "Leaving Task 1\r\nData accessed is: %d\r\nNot releasing the semaphore\r\n\r\n"
            , usRecource[ucIndex]);
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask1, strlen(pcStringTask1), 500);
    if (ucIndex < 2)
    {
      ucIndex++;
    }
    else
    {
      ucIndex = 0;
    }
    vPortFree(pcStringTask1);
    vTaskDelay(pdMS_TO_TICKS(3000));

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void vStartTask2(void *pvParameters)
{
  /* USER CODE BEGIN StartTask2 */
  char *pcStringTask2;
  /* Infinite loop */
  for(;;)
  {
    pcStringTask2 = pvPortMalloc(20*sizeof(pcStringTask2));
    sprintf(pcStringTask2, "Entered task 2\r\nAbout to acquired the semaphore\r\nTokens avaiable are: %d\r\n\r\n"
            , (uint8_t)uxSemaphoreGetCount(CountingSemaphoreHandle));
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask2, strlen(pcStringTask2), 500);
    xSemaphoreTake(CountingSemaphoreHandle, portMAX_DELAY);
    sprintf(pcStringTask2, "Leaving Task 2\r\nData accessed is: %d\r\nNot releasing the semaphore\r\n\r\n"
            , usRecource[ucIndex]);
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask2, strlen(pcStringTask2), 500);
    if (ucIndex < 2)
    {
      ucIndex++;
    }
    else
    {
      ucIndex = 0;
    }
    vPortFree(pcStringTask2);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END StartTask2 */
}

/* USER CODE BEGIN Header_StartTask3 */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask3 */
void vStartTask3(void *pvParameters)
{
  /* USER CODE BEGIN StartTask3 */
  char *pcStringTask3;
  /* Infinite loop */
  for(;;)
  {
    pcStringTask3 = pvPortMalloc(20*sizeof(pcStringTask3));
    sprintf(pcStringTask3, "Entered task 3\r\nAbout to acquired the semaphore\r\nTokens avaiable are: %d\r\n\r\n"
            , (uint8_t)uxSemaphoreGetCount(CountingSemaphoreHandle));
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask3, strlen(pcStringTask3), 500);
    xSemaphoreTake(CountingSemaphoreHandle, portMAX_DELAY);
    sprintf(pcStringTask3, "Leaving Task 3\r\nData accessed is: %d\r\nNot releasing the semaphore\r\n\r\n"
            , usRecource[ucIndex]);
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask3, strlen(pcStringTask3), 500);
    if (ucIndex < 2)
    {
      ucIndex++;
    }
    else
    {
      ucIndex = 0;
    }
    vPortFree(pcStringTask3);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  /* USER CODE END StartTask3 */
}

/* USER CODE BEGIN Header_StartTask4 */
/**
* @brief Function implementing the Task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask4 */
void vStartTask4(void *pvParameters)
{
  /* USER CODE BEGIN StartTask4 */
  char *pcStringTask4;
  /* Give 3 semaphore at beginning */
  // xSemaphoreGive(CountingSemaphoreHandle);
  // xSemaphoreGive(CountingSemaphoreHandle);
  // xSemaphoreGive(CountingSemaphoreHandle);

  /* Infinite loop */
  for(;;)
  {
    pcStringTask4 = pvPortMalloc(20*sizeof(pcStringTask4));
    sprintf(pcStringTask4, "Entered task 4\r\nAbout to acquired the semaphore\r\nTokens avaiable are: %d\r\n\r\n"
            , (uint8_t)uxSemaphoreGetCount(CountingSemaphoreHandle));
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask4, strlen(pcStringTask4), 500);
    xSemaphoreTake(CountingSemaphoreHandle, portMAX_DELAY);
    sprintf(pcStringTask4, "Leaving Task 4\r\nData accessed is: %d\r\nNot releasing the semaphore\r\n\r\n"
            , usRecource[ucIndex]);
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStringTask4, strlen(pcStringTask4), 500);
    if (ucIndex < 2)
    {
      ucIndex++;
    }
    else
    {
      ucIndex = 0;
    }
    vPortFree(pcStringTask4);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
  /* USER CODE END StartTask4 */
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
