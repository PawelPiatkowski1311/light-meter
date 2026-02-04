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
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32f3xx_ll_usart.h"
#include "lux_buffer.h"


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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define USART_TXBUF_LEN 1512
#define USART_RXBUF_LEN 128
uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];

/* Globalny bufor pomiarów */
LuxBuffer_t luxBuffer;

__IO int USART_Tx_Empty = 0;
__IO int USART_Tx_Busy  = 0;
__IO int USART_Rx_Empty = 0;
__IO int USART_Rx_Busy  = 0;

uint8_t t_USART_kbhit(){
	if(USART_Rx_Empty == USART_Rx_Busy){
         return 0;
     }else{
         return 1;
     }
} // USART_kbhit
/*
int16_t t_USART_getchar(){
     int16_t tmp;
     if(USART_Rx_Empty != USART_Rx_Busy){
         tmp = USART_RxBuf[USART_Rx_Empty];
         USART_Rx_Empty++;
         if(USART_Rx_Empty >= USART_RXBUF_LEN) USART_Rx_Empty = 0;
         return tmp;
     } else return -1;
} // USART_getchar
*/
/*
uint8_t t_USART_getline(char *buf){
     static uint8_t bf[128];
     static uint8_t b_idx = 0;
     int i;
     uint8_t ret;
     while(!t_USART_kbhit());
     bf[b_idx] = t_USART_getchar();
     if(bf[b_idx] == 0x0A){ // LF
         if((bf[b_idx-1] == 0x0D) || (bf[b_idx] == 13)){
             bf[b_idx] = 0;
             for(i = 0; i < b_idx; i++)
                 buf[i] = bf[i]; // przepisz do bufora
             ret = b_idx;
             b_idx = 0;
             return ret; // odebrano linię
         } else { // jeśli tekst dłuższy to zawijamy - trudno
             b_idx++;
             if(b_idx >= 128) b_idx = 0;
         }
     }
     return 0;
} // USART_getline
*/

void USART_fsend(char *format, ...){
     char tmp_rs[128];
     int i;
     __IO int idx;
     va_list arglist;
     va_start(arglist, format);
     vsprintf(tmp_rs, format, arglist);
     va_end(arglist);
     idx = USART_Tx_Empty;
     for(i = 0; i < strlen(tmp_rs); i++){
         USART_TxBuf[idx++] = tmp_rs[i];
         if(idx >= USART_TXBUF_LEN) idx = 0;
     }
     __disable_irq();
     if((USART_Tx_Empty == USART_Tx_Busy) && (LL_USART_IsActiveFlag_TXE(USART2) == SET)){ // sprawdzić dodatkowo zajętość bufora nadajnika
         uint8_t tmp = USART_TxBuf[USART_Tx_Busy];
         USART_Tx_Busy++;
         if(USART_Tx_Busy >= USART_TXBUF_LEN) USART_Tx_Busy = 0;
         HAL_UART_Transmit_IT(&huart2, &tmp, 1);
     } else {
         USART_Tx_Empty = idx;
     }
     __enable_irq();
} // fsend

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
     if(huart == &huart2){
         if(USART_Tx_Empty != USART_Tx_Busy){
             uint8_t tmp = USART_TxBuf[USART_Tx_Busy];
             USART_Tx_Busy++;
             if(USART_Tx_Busy >= USART_TXBUF_LEN) USART_Tx_Busy = 0;
             HAL_UART_Transmit_IT(&huart2, &tmp, 1);
         }
     }
}

// RX calback (wywolany przy odbiorze znaku)
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
     if(huart == &huart2){
         if(++USART_Rx_Empty >= USART_RXBUF_LEN) USART_Rx_Empty = 0;
         HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_Rx_Empty], 1);
     }
}
*/
// Globalny bufor
//#define USART_RXBUF_LEN 32
uint8_t USART_RxBuf[USART_RXBUF_LEN];
volatile uint16_t USART_Rx_Head = 0; // miejsce do zapisu
volatile uint16_t USART_Rx_Tail = 0; // miejsce do odczytu

// Callback HAL UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // restart odbioru na bieżący indeks
        HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_Rx_Head], 1);

        // dopiero po wywołaniu Receive_IT zwiększamy indeks
        uint16_t next = USART_Rx_Head + 1;
        if(next >= USART_RXBUF_LEN) next = 0;
        USART_Rx_Head = next;
    }
}

// Pobranie znaku z bufora
int t_USART_getchar(void)
{
    if (USART_Rx_Tail == USART_Rx_Head) return -1; // brak danych
    int ch = USART_RxBuf[USART_Rx_Tail++];
    if(USART_Rx_Tail >= USART_RXBUF_LEN) USART_Rx_Tail = 0;
    return ch;
}


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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
