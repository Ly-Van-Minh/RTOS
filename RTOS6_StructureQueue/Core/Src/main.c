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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
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
typedef struct
{
  char *pcString;
  uint32_t ulCount;
  uint16_t usLargeValue;
}MyStruct_t;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
QueueHandle_t xStructureQueue;
uint16_t usIndex1 = 0, usIndex2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void vSender1Task(void *pvParameters);
void vSender2Task(void *pvParameters);
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
  /* creation and verify of xStructureQueue */
  xStructureQueue = xQueueCreate(2, sizeof(MyStruct_t));
  if (xStructureQueue == 0)
  {
    char *pcNotSuccessString = "Unable to create Structue Queue\n\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcNotSuccessString, strlen(pcNotSuccessString), 200);
  }
  else
  {
    char *pcSuccessString = "Structure Queue created successfully\n\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)pcSuccessString, strlen(pcSuccessString), 200);
  }
  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of vSender1Task */
  xTaskCreate(vSender1Task, "Sender 1", 128, NULL, 2, NULL);

  /* creation of vSender2Task */
  xTaskCreate(vSender2Task, "Sender 2", 128, NULL, 2, NULL);

  /* creation of vReceiverTask */
  xTaskCreate(vReceiverTask, "Receiver", 128, NULL, 1, NULL);

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
  huart1.Init.BaudRate = 115200;
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_vSender1Task */
/**
  * @brief  Function implementing the vSender1Task thread.
  * @param  pvParameters: Not used
  * @retval None
  */
/* USER CODE END Header_vSender1Task */
void vSender1Task(void *pvParameters)
{
  /* USER CODE BEGIN 5 */
  MyStruct_t *pxStructToSend;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(2000);
  char *pcString1 = "Entered Sender 1 Task\n About to send to the queue\n\n";
  char *pcString2 = "Successfully sent the to the queue\n Leaving Sender 1Task\n\n\n"; 
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)pcString1, strlen(pcString1), 200);
    pxStructToSend = pvPortMalloc(sizeof(MyStruct_t));
    pxStructToSend->ulCount = usIndex1 + 1; 
    pxStructToSend->usLargeValue = 1000 + usIndex1*100;
    pxStructToSend->pcString = "Hello from Sender 1";
    if (xQueueSendToBack(xStructureQueue, &pxStructToSend, portMAX_DELAY) == pdPASS)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)pcString2, strlen(pcString2), 200);
    }
    usIndex1++;
    vTaskDelay(xTicksToWait);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vSender2Task */
/**
* @brief Function implementing the vSender2Task thread.
* @param pvParameters: Not used
* @retval None
*/
/* USER CODE END Header_vSender2Task */
void vSender2Task(void *pvParameters)
{
  /* USER CODE BEGIN StartTask02 */
  MyStruct_t *pxStructToSend;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(2000);
  char *pcString1 = "Entered Sender 1 Task\n About to send to the queue\n\n";
  char *pcString2 = "Successfully sent the to the queue\n Leaving Sender 1Task\n\n\n";
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)pcString1, strlen(pcString1), 200);
    pxStructToSend = pvPortMalloc(sizeof(MyStruct_t));
    pxStructToSend->ulCount = usIndex2 + 1; 
    pxStructToSend->usLargeValue = 1000 + usIndex2*100;
    pxStructToSend->pcString = "Hello from Sender 2";
    if (xQueueSendToBack(xStructureQueue, &pxStructToSend, portMAX_DELAY) == pdPASS)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)pcString2, strlen(pcString2), 200);
    }
    usIndex2++;
    vTaskDelay(xTicksToWait);
  }
  /* USER CODE END vSender2Task */
}

/* USER CODE BEGIN Header_vReceiverTask */
/**
* @brief Function implementing the vReceiverTask thread.
* @param pvParameters: Not used
* @retval None
*/
/* USER CODE END Header_vReceiverTask */
void vReceiverTask(void *pvParameters)
{
  /* USER CODE BEGIN pvParameters */
  MyStruct_t *xStructReceive;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(3000);
  char *pcString1 = "Entered Receiver Task\n About to Receive from the queue\n\n";
  char *pcString2;
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)pcString1, strlen(pcString1), 200);
    if (xQueueReceive(xStructureQueue, &xStructReceive, portMAX_DELAY) == pdPASS)
    {
      pcString2 = pvPortMalloc(100 *sizeof(char));
      sprintf(pcString2, "Receice from queue:\n Count = %d,\n Large Value = %d, String = %s",xStructReceive->ulCount,
              xStructReceive->usLargeValue, xStructReceive->pcString);
      HAL_UART_Transmit(&huart1, (uint8_t *)pcString2, strlen(pcString2), 300);
      vPortFree(pcString2);
    }
    vPortFree(xStructReceive);
    vTaskDelay(xTicksToWait);
  }
  /* USER CODE END vReceiverTask */
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
