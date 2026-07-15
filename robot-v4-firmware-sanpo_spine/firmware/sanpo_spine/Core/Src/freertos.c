/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "error.h"
#include "usbd_cdc.h"
#include "loop_function.h"
#include "spi.h"
#include "usart.h"
#include "i2c.h"
#include "adc.h"
#include "mpu6050.h"
#include "usart.h"
#include "wd_control.h"
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
/* USER CODE BEGIN Variables */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
osThreadId_t wdControlTaskHandle;
static StaticTask_t wdControlTaskControlBlock;
static uint32_t wdControlTaskStack[512];
const osThreadAttr_t wdControlTask_attributes = {
  .name = "wdControlTask",
  .cb_mem = &wdControlTaskControlBlock,
  .cb_size = sizeof(wdControlTaskControlBlock),
  .stack_mem = wdControlTaskStack,
  .stack_size = sizeof(wdControlTaskStack),
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cdcRXTask */
osThreadId_t cdcRXTaskHandle;
const osThreadAttr_t cdcRXTask_attributes = {
  .name = "cdcRXTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canaTXTask */
osThreadId_t canaTXTaskHandle;
const osThreadAttr_t canaTXTask_attributes = {
  .name = "canaTXTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataRXTask */
osThreadId_t dataRXTaskHandle;
const osThreadAttr_t dataRXTask_attributes = {
  .name = "dataRXTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cdcTXTask */
osThreadId_t cdcTXTaskHandle;
const osThreadAttr_t cdcTXTask_attributes = {
  .name = "cdcTXTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spiRXTask06 */
osThreadId_t spiRXTask06Handle;
const osThreadAttr_t spiRXTask06_attributes = {
  .name = "spiRXTask06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usartTXTask */
osThreadId_t usartTXTaskHandle;
const osThreadAttr_t usartTXTask_attributes = {
  .name = "usartTXTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2cRXTask */
osThreadId_t i2cRXTaskHandle;
const osThreadAttr_t i2cRXTask_attributes = {
  .name = "i2cRXTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adcRXTask */
osThreadId_t adcRXTaskHandle;
const osThreadAttr_t adcRXTask_attributes = {
  .name = "adcRXTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rs485ATXTask */
osThreadId_t rs485ATXTaskHandle;
const osThreadAttr_t rs485ATXTask_attributes = {
  .name = "rs485ATXTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rs485BTXTask */
osThreadId_t rs485BTXTaskHandle;
const osThreadAttr_t rs485BTXTask_attributes = {
  .name = "rs485BTXTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canbTXTask */
osThreadId_t canbTXTaskHandle;
const osThreadAttr_t canbTXTask_attributes = {
  .name = "canbTXTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartWDControlTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCDCRXTask(void *argument);
void StartCANATXTask(void *argument);
void StartDataRXTask(void *argument);
void StartCDCTXTask(void *argument);
void StartSPIRXTask(void *argument);
void StartUSARTTXTask(void *argument);
void StartI2CRXTask(void *argument);
void StartADCRXTask(void *argument);
void StartRS485ATXTask(void *argument);
void StartRS485BTXTask(void *argument);
void StartCANBTXTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of cdcRXTask */
  cdcRXTaskHandle = osThreadNew(StartCDCRXTask, NULL, &cdcRXTask_attributes);

  /* creation of canaTXTask */
  canaTXTaskHandle = osThreadNew(StartCANATXTask, NULL, &canaTXTask_attributes);

  /* creation of dataRXTask */
  dataRXTaskHandle = osThreadNew(StartDataRXTask, NULL, &dataRXTask_attributes);

  /* creation of cdcTXTask */
  cdcTXTaskHandle = osThreadNew(StartCDCTXTask, NULL, &cdcTXTask_attributes);

  /* creation of spiRXTask06 */
  spiRXTask06Handle = osThreadNew(StartSPIRXTask, NULL, &spiRXTask06_attributes);

  /* creation of usartTXTask */
  usartTXTaskHandle = osThreadNew(StartUSARTTXTask, NULL, &usartTXTask_attributes);

  /* creation of i2cRXTask */
  i2cRXTaskHandle = osThreadNew(StartI2CRXTask, NULL, &i2cRXTask_attributes);

  /* creation of adcRXTask */
  adcRXTaskHandle = osThreadNew(StartADCRXTask, NULL, &adcRXTask_attributes);

  /* creation of rs485ATXTask */
  rs485ATXTaskHandle = osThreadNew(StartRS485ATXTask, NULL, &rs485ATXTask_attributes);

  /* creation of rs485BTXTask */
  rs485BTXTaskHandle = osThreadNew(StartRS485BTXTask, NULL, &rs485BTXTask_attributes);

  /* creation of canbTXTask */
  canbTXTaskHandle = osThreadNew(StartCANBTXTask, NULL, &canbTXTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /*
   * Static allocation keeps the 1 kHz dry-run control task independent from
   * the tight generated heap. If creation still fails, cdcTXTask runs a
   * visible fallback tick so the PC tool can report the degraded mode.
   */
  wdControlTaskHandle = osThreadNew(StartWDControlTask, NULL, &wdControlTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  init_task(&hspi3,
		  RS485A_RE_GPIO_Port,
			RS485A_RE_Pin,
			&huart1,
			&hdma_usart1_rx,
			RS485B_RE_GPIO_Port,
			RS485B_RE_Pin,
			&huart6,
			&hdma_usart6_rx,
			&hUsbDeviceFS,
			&canaTXTaskHandle,
			&canbTXTaskHandle,
			&rs485ATXTaskHandle,
			&rs485BTXTaskHandle);

/* *********************************************************
* This task is part of a complete real-time framework.
* The HAL and RTOS are already initialized before this runs.
* USB device is initialized here via MX_USB_DEVICE_Init().
* The following handles are already defined and ready to use:
*   hspi3, huart1/huart6, hdma_usart1_rx/hdma_usart6_rx,
*   RS485A_RE_GPIO_Port/Pin, RS485B_RE_GPIO_Port/Pin.
* Replace the loop below with your system bring-up or services.
* Remove the function remove_me() before adding your code.
* ********************************************************/

  for(;;)
  {
	  osDelay(1);
  }


  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCDCRXTask */
/**
* @brief Function implementing the cdcRXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCDCRXTask */
void StartCDCRXTask(void *argument)
{
  /* USER CODE BEGIN StartCDCRXTask */

/* *********************************************************
* This task is part of a complete real-time framework.
* USB CDC device handle hUsbDeviceFS is available and ready.
* USBD_CDC_* APIs and the existing CDC helper functions
* Replace the loop below with your custom CDC RX logic.
* ********************************************************/

  for(;;)
  {
	cdc_rx_loop();
  }
  /* USER CODE END StartCDCRXTask */
}

/* USER CODE BEGIN Header_StartCANATXTask */
/**
* @brief Function implementing the canaTXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANATXTask */
void StartCANATXTask(void *argument)
{
  /* USER CODE BEGIN StartCANATXTask */
/* *********************************************************
 * This task is part of a complete real-time framework.
 * CAN handles hcan1 are already defined and ready.
 * hcan1 for CAN-1/3
 * Use them to publish outbound CAN frames or routing logic.
 * Existing code can be replaced.
 * ********************************************************/

  for(;;)
  {
	cana_tx_loop(&hcan1);
  }
  /* USER CODE END StartCANATXTask */
}

/* USER CODE BEGIN Header_StartDataRXTask */
/**
* @brief Function implementing the dataRXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataRXTask */
void StartDataRXTask(void *argument)
{
  /* USER CODE BEGIN StartDataRXTask */

/* *********************************************************
 * This task is part of a complete real-time framework.
 * The task is intended for inbound data processing/dispatch.
 * You can replace it with your own data aggregation/decoder logic.
 * ********************************************************/


  for(;;)
  {
  	data_rx_loop();
  }
  /* USER CODE END StartDataRXTask */
}

/* USER CODE BEGIN Header_StartCDCTXTask */
/**
* @brief Function implementing the cdcTXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCDCTXTask */
void StartCDCTXTask(void *argument)
{
  /* USER CODE BEGIN StartCDCTXTask */

/* *********************************************************
 * This task is part of a complete real-time framework.
 * USB CDC device handle hUsbDeviceFS is available and ready.
 * USBD_CDC_SetTxBuffer / USBD_CDC_TransmitPacket can be used
 * Replace the loop below with your custom CDC TX logic.
 * ********************************************************/

  for(;;)
  {
  	if (wdControlTaskHandle == NULL) {
  		wd_control_note_control_task_fallback();
  		wd_control_service_1khz();
  	}
  	if (!wd_control_usb_tx_poll()) {
  		cdc_tx_loop();
  	} else {
  		osDelay(1);
  	}
  }
  /* USER CODE END StartCDCTXTask */
}

/* USER CODE BEGIN Header_StartSPIRXTask */
/**
* @brief Function implementing the spiRXTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPIRXTask */
void StartSPIRXTask(void *argument)
{
  /* USER CODE BEGIN StartSPIRXTask */
/* *********************************************************
 * This task is part of a complete real-time framework.
 * SPI handle hspi3 and routing targets (hcan1/hcan2, huart1/huart6)
 * hcan1 for CAN-1/3
 * hcan2 for CAN-2/4
 * huart1 for RS485-1/3
 * huart6 for RS485-2/4
 * are already defined and ready to use.
 * You can use this task to read SPI payloads and forward them.
 * Replace the loop below with your own SPI RX pipeline.
 * ********************************************************/

  for(;;)
  {
  	spi_rx_loop(&hcan1, &hcan2, &huart1, &huart6);
  }
  /* USER CODE END StartSPIRXTask */
}

/* USER CODE BEGIN Header_StartUSARTTXTask */
/**
* @brief Function implementing the usartTXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSARTTXTask */
void StartUSARTTXTask(void *argument)
{
  /* USER CODE BEGIN StartUSARTTXTask */

/* *********************************************************
 * This task is part of a complete real-time framework.
 * UART handle huart3 (and DMA handles in usart.h) are ready.
 * Use HAL_UART_Transmit or HAL_UART_Transmit_DMA as needed.
 * Replace the loop below with your custom UART TX logic.
 * Remove the function remove_me() before adding your code.
 * ********************************************************/

  for(;;)
  {
	  osDelay(1);
  }


  /* USER CODE END StartUSARTTXTask */
}

/* USER CODE BEGIN Header_StartI2CRXTask */
/**
* @brief Function implementing the i2cRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2CRXTask */
void StartI2CRXTask(void *argument)
{
  /* USER CODE BEGIN StartI2CRXTask */

/* *********************************************************
 * This task is part of a complete real-time framework.
 * I2C handle hi2c2 is available and initialized.
 * UART handle huart3 is also available for debug/output.
 * You can read sensors (e.g., MPU6050) and publish results.
 * Replace the loop below with your custom I2C RX logic.
 * Remove the function remove_me() before adding your code.
 * ********************************************************/

	for(;;)
	{
		osDelay(1);
	}

//  for(;;)
//  {
//	uint8_t mpu_len = 14;
//	uint8_t mpu_data[14] = {0};
//  	if(MPU_Init(&hi2c2)==0)
//  	{
//  		MPU_Read_Len(&hi2c2, 0x3B, mpu_len, mpu_data);
//  		HAL_UART_Transmit_DMA(&huart3, mpu_data, mpu_len);
//  	}
//		HAL_Delay(1000);
//  }


  /* USER CODE END StartI2CRXTask */
}

/* USER CODE BEGIN Header_StartADCRXTask */
/**
* @brief Function implementing the adcRXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCRXTask */
void StartADCRXTask(void *argument)
{
  /* USER CODE BEGIN StartADCRXTask */
/* *********************************************************
 * This task is part of a complete real-time framework.
 * ADC handles hadc1 and hadc2 are already defined and ready.
 * Use HAL_ADC_Start / HAL_ADC_PollForConversion to sample data,
 * then filter, scale, or publish the results as needed.
 * Replace the loop below with your custom ADC sampling logic.
 * Remove the function remove_me() before adding your code.
 * ********************************************************/
	for(;;)
	{
		osDelay(1);
	}
//  for(;;)
//  {
//  	HAL_ADC_Start(&hadc1);//开始ADC采集
//  	HAL_ADC_PollForConversion(&hadc1,500);//等待采集结束
//
//
//  	HAL_ADC_Start(&hadc2);//开始ADC采集
//  	HAL_ADC_PollForConversion(&hadc2,500);//等待采集结束
//
//  	// HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&value_adc, value_len);
//
//  	HAL_Delay(1000);
//
//  }

  /* USER CODE END StartADCRXTask */
}

/* USER CODE BEGIN Header_StartRS485ATXTask */
/**
* @brief Function implementing the rs485ATXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRS485ATXTask */
void StartRS485ATXTask(void *argument)
{
  /* USER CODE BEGIN StartRS485ATXTask */
/* *********************************************************
 * This task is part of a complete real-time framework.
 * UART handle huart1 is ready for RS485-1/3 transmission.
 * RS485A_RE_GPIO_Port/Pin are available for driver enable control.
 * Replace the loop below with your custom RS485-1/3 TX logic.
 * ********************************************************/

  for(;;)
  {
	rs485a_tx_loop(&huart1, RS485A_RE_GPIO_Port, RS485A_RE_Pin);
  }
  /* USER CODE END StartRS485ATXTask */
}

/* USER CODE BEGIN Header_StartRS485BTXTask */
/**
* @brief Function implementing the rs485BTXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRS485BTXTask */
void StartRS485BTXTask(void *argument)
{
  /* USER CODE BEGIN StartRS485BTXTask */
/* *********************************************************
 * This task is part of a complete real-time framework.
 * UART handle huart6 is ready for RS485-2/4 transmission.
 * RS485B_RE_GPIO_Port/Pin are available for driver enable control.
 * Replace the loop below with your custom RS485-2/4 TX logic.
 * ********************************************************/

  for(;;)
  {
	rs485b_tx_loop(&huart6, RS485B_RE_GPIO_Port, RS485B_RE_Pin);
  }
  /* USER CODE END StartRS485BTXTask */
}

/* USER CODE BEGIN Header_StartCANBTXTask */
/**
* @brief Function implementing the canbTXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANBTXTask */
void StartCANBTXTask(void *argument)
{
  /* USER CODE BEGIN StartCANBTXTask */
/* *********************************************************
 * This task is part of a complete real-time framework.
 * CAN handles hcan2 are already defined and ready.
 * hcan2 for CAN-2/4
 * Use them to publish outbound CAN frames or routing logic.
 * Existing code can be replaced.
 * ********************************************************/

  for(;;)
  {
	canb_tx_loop(&hcan2);
  }
  /* USER CODE END StartCANBTXTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartWDControlTask(void *argument)
{
  TickType_t last_wake_tick;

  (void)argument;
  wd_control_init();
  last_wake_tick = xTaskGetTickCount();
  for (;;)
  {
    wd_control_service_1khz();
    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(1));
  }
}

/* USER CODE END Application */
