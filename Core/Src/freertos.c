/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // pawel
#include <string.h> // pawel
#include "bh1750.h" // pawel
#include "lux_buffer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1; // pawel
extern LuxBuffer_t luxBuffer;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include <stdint.h>
#include <string.h>

void FrameService(uint8_t* ramka, uint8_t dlugosc);

typedef enum {
    FSM_WAIT_START = 1,
    FSM_WAIT_SENDER = 2,
    FSM_WAIT_RECEIVER = 3,
    FSM_WAIT_LEN = 4,
    FSM_WAIT_DATA = 5,
    FSM_WAIT_CRC = 6,
    FSM_WAIT_STOP = 7,
	FSM_DONE = 8
} FSM_State;

#define MAX_DATA_LEN 64

// Statusy
typedef enum {
    FRAME_NONE = 0,     // brak ramki
    FRAME_OK,
    FRAME_ERR_LEN,
    FRAME_ERR_CRC
} frame_status_t;

typedef struct {
    uint8_t sender;
    uint8_t receiver;
    uint8_t length;
    uint8_t data[MAX_DATA_LEN];
    uint8_t crc;
    frame_status_t status;
} Frame_t;

FSM_State fsm_state = FSM_WAIT_START;
Frame_t frame;
frame_status_t frame_status = FRAME_NONE;
uint8_t frame_ready = 0;
uint8_t value;
uint8_t data_index = 0;
uint8_t crc_index = 0;
uint8_t crc_value = 0;
uint8_t crc_array[3];
uint8_t computed_crc = 0;

uint8_t znak;
char m = '1';

void FSM_ProcessByte(uint8_t rx)
{
    switch(fsm_state)
    {
        case FSM_WAIT_START:
            if(rx == ':') {
                fsm_state = FSM_WAIT_SENDER;
                computed_crc = rx; // CRC liczymy od startu
                data_index = 0;
                crc_index = 0;
                m = '1';
                HAL_UART_Transmit(&huart2, &m, 1, HAL_MAX_DELAY);
            }
            break;

        case FSM_WAIT_SENDER:
            frame.sender = rx;
            computed_crc += rx;
            fsm_state = FSM_WAIT_RECEIVER;
            m = '2';
            HAL_UART_Transmit(&huart2, &m, 1, HAL_MAX_DELAY);
            break;

        case FSM_WAIT_RECEIVER:
			frame.receiver = rx;
			computed_crc += rx;
			fsm_state = FSM_WAIT_LEN;
			m = '3';
			HAL_UART_Transmit(&huart2, &m, 1, HAL_MAX_DELAY);
            break;

        case FSM_WAIT_LEN:
        	if (!(rx >= '0' && rx <= '9'))
        	{
        		frame.status = FRAME_ERR_LEN;
        		fsm_state = FSM_DONE;
        		break;
        	}
			value = rx - '0';   // konwersja z ascii na wartość liczbową
			/*
			if(value > MAX_DATA_LEN) {
				// błąd długości -> reset FSM
				fsm_state = FSM_WAIT_START;
				break;
			}
			*/
			frame.length = value;
			computed_crc += rx;
			data_index = 0;
			fsm_state = (frame.length > 0) ? FSM_WAIT_DATA : FSM_WAIT_CRC;
			m = '4';
			HAL_UART_Transmit(&huart2, &m, 1, HAL_MAX_DELAY);
            break;

        case FSM_WAIT_DATA:
            frame.data[data_index++] = rx;
            computed_crc += rx;
            if(data_index >= frame.length)
                fsm_state = FSM_WAIT_CRC;
            m = '5';
            HAL_UART_Transmit(&huart2, &m, 1, HAL_MAX_DELAY);
            break;

        case FSM_WAIT_CRC:
			crc_array[crc_index++] = rx; // zapisanie każdej cyfry do tablicy
			if(crc_index < 3)
			{
				break;
			}
			crc_value = (crc_array[0]-'0')*100 +
			            (crc_array[1]-'0')*10  +
			            (crc_array[2]-'0'); // konwersja
			if(computed_crc == crc_value) {
				frame.crc = crc_value;
				frame.status = FRAME_OK;
			    fsm_state = FSM_WAIT_START;
			    frame_ready  = 1;   // tylko sygnał
			} else {
				// błąd CRC -> reset
			    frame.status = FRAME_ERR_CRC;
			    fsm_state = FSM_WAIT_START;
			}
            break;
    }
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
uint32_t SensorTaskBuffer[ 512 ];
osStaticThreadDef_t SensorTaskControlBlock;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .cb_mem = &SensorTaskControlBlock,
  .cb_size = sizeof(SensorTaskControlBlock),
  .stack_mem = &SensorTaskBuffer[0],
  .stack_size = sizeof(SensorTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FrameTask */
osThreadId_t FrameTaskHandle;
uint32_t FrameTaskBuffer[ 128 ];
osStaticThreadDef_t FrameTaskControlBlock;
const osThreadAttr_t FrameTask_attributes = {
  .name = "FrameTask",
  .cb_mem = &FrameTaskControlBlock,
  .cb_size = sizeof(FrameTaskControlBlock),
  .stack_mem = &FrameTaskBuffer[0],
  .stack_size = sizeof(FrameTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorQueue */
osMessageQueueId_t SensorQueueHandle;
uint8_t SensorQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t SensorQueueControlBlock;
const osMessageQueueAttr_t SensorQueue_attributes = {
  .name = "SensorQueue",
  .cb_mem = &SensorQueueControlBlock,
  .cb_size = sizeof(SensorQueueControlBlock),
  .mq_mem = &SensorQueueBuffer,
  .mq_size = sizeof(SensorQueueBuffer)
};
/* Definitions for FrameQueue */
osMessageQueueId_t FrameQueueHandle;
uint8_t FrameQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t FrameQueueControlBlock;
const osMessageQueueAttr_t FrameQueue_attributes = {
  .name = "FrameQueue",
  .cb_mem = &FrameQueueControlBlock,
  .cb_size = sizeof(FrameQueueControlBlock),
  .mq_mem = &FrameQueueBuffer,
  .mq_size = sizeof(FrameQueueBuffer)
};
/* Definitions for SensorTimer */
osTimerId_t SensorTimerHandle;
osStaticTimerDef_t myTimer01ControlBlock;
const osTimerAttr_t SensorTimer_attributes = {
  .name = "SensorTimer",
  .cb_mem = &myTimer01ControlBlock,
  .cb_size = sizeof(myTimer01ControlBlock),
};
/* Definitions for SensorMutex */
osMutexId_t SensorMutexHandle;
osStaticMutexDef_t SensorMutexControlBlock;
const osMutexAttr_t SensorMutex_attributes = {
  .name = "SensorMutex",
  .cb_mem = &SensorMutexControlBlock,
  .cb_size = sizeof(SensorMutexControlBlock),
};
/* Definitions for TestEvent */
osEventFlagsId_t TestEventHandle;
osStaticEventGroupDef_t TestEventControlBlock;
const osEventFlagsAttr_t TestEvent_attributes = {
  .name = "TestEvent",
  .cb_mem = &TestEventControlBlock,
  .cb_size = sizeof(TestEventControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartSensorTask(void *argument);
void StartFrameTask(void *argument);
void Callback01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of SensorMutex */
  SensorMutexHandle = osMutexNew(&SensorMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of SensorTimer */
  SensorTimerHandle = osTimerNew(Callback01, osTimerPeriodic, NULL, &SensorTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SensorQueue */
  SensorQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &SensorQueue_attributes);

  /* creation of FrameQueue */
  FrameQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &FrameQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of FrameTask */
  FrameTaskHandle = osThreadNew(StartFrameTask, NULL, &FrameTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of TestEvent */
  TestEventHandle = osEventFlagsNew(&TestEvent_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
	char uart_buf[64]; // pawel
	uint16_t lux_x10;
	uint16_t frequency = 1000;
	float lux; // pawel
	char ch;

	LuxBuffer_Init(&luxBuffer);

    vTaskDelay(pdMS_TO_TICKS(200));

    /* INIT BH1750 */
    BH1750_Init(&hi2c1);

    vTaskDelay(pdMS_TO_TICKS(10));

    /* start pomiaru */
    BH1750_StartContHR(&hi2c1);

    /* CZEKAJ NA PIERWSZY POMIAR (najważniejsze!) */
    vTaskDelay(pdMS_TO_TICKS(180));

  for(;;)
  {
      if (BH1750_ReadLux(&hi2c1, &lux) == HAL_OK)
      {
          /* Konwersja float -> int (lux * 10) */
          lux_x10 = (uint16_t)(lux * 10.0f);

          /* Zapis do bufora */
          LuxBuffer_Push(&luxBuffer, lux_x10);

          LuxBuffer_Get(&luxBuffer, 1, &lux); /// nadpisuje obecny pomiar

    	  int len = snprintf(uart_buf, sizeof(uart_buf), "Lux: %d\r\n", lux_x10);
		  	  // ----- dane z pomiaru ------ //
		  	for (int i = 0; i < len; i++)
		  	{
		  	    ch = uart_buf[i];
		  	    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
		  	}

    	      //HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, 1, HAL_MAX_DELAY);
      }

      //vTaskDelay(pdMS_TO_TICKS(500));
      for(int i = 0; i <= frequency; i++) {
          osDelay(1);
      }
    //HAL_UART_Transmit(&huart2, &pomiar, 1, HAL_MAX_DELAY);
    //
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartFrameTask */
/**
* @brief Function implementing the FrameTask thread.
* @param argument: Not used
* @retval None
*/
void ObslugaRamki(uint8_t* ramka, uint8_t dlugosc){
	HAL_UART_Transmit(&huart2, '!', 1, HAL_MAX_DELAY);
}
/* USER CODE END Header_StartFrameTask */
void StartFrameTask(void *argument)
{
  /* USER CODE BEGIN StartFrameTask */
  /* Infinite loop */
	uint8_t ch;

	    // powitanie
	    char msg[] = "START USART2 Polling\r\n";
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
  for(;;)
  {
	  // odbiór 1 bajtu (blocking)
	  if (HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) == HAL_OK) { // jeśli coś odbierze to idziemy dalej
		  FSM_ProcessByte(ch);
	  }

      /* Task śpi – nie polluje agresywnie */
      if (frame_ready)
      {
          /* Krytyczna sekcja – bardzo krótka */
          taskENTER_CRITICAL();
          frame_ready = 0;
          frame_t local_frame = frame;   // kopia lokalna
          taskEXIT_CRITICAL();

          /* === ANALIZA RAMKI === */
          Frame_Handle(&local_frame);
      }
      osDelay(1);
  }
  /* USER CODE END StartFrameTask */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

