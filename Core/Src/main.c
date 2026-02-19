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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bh1750.h"
#include "frame.h"
#include "frame_handler.h"
#include "lux_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    FSM_WAIT_START = 1,
    FSM_WAIT_SENDER,
    FSM_WAIT_RECEIVER,
    FSM_WAIT_LEN,
    FSM_WAIT_DATA,
    FSM_WAIT_CRC,
} FSM_State;

typedef enum {
    SENSOR_BOOT_DELAY = 0,
    SENSOR_SEND_POWER_ON,
    SENSOR_WAIT_POWER_ON,
    SENSOR_SEND_RESET,
    SENSOR_WAIT_RESET,
    SENSOR_SEND_CONT_H_RES,
    SENSOR_WAIT_CONT_H_RES,
    SENSOR_FIRST_MEAS_DELAY,
    SENSOR_READ_REQUEST,
    SENSOR_WAIT_READ,
} SensorState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_TXBUF_LEN 1512u
#define USART_RXBUF_LEN 128u

#define I2C_EVT_TX_DONE (1u << 0)
#define I2C_EVT_RX_DONE (1u << 1)
#define I2C_EVT_ERR     (1u << 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Bufory pierścieniowe UART.
 * ISR zapisuje/odczytuje pojedyncze bajty, a kod główny dokłada dane do TX.
 * Dzięki temu transmisja jest nieblokująca i odporna na krótkie skoki obciążenia.
 */
static uint8_t usart_tx_buf[USART_TXBUF_LEN];
static uint8_t usart_rx_buf[USART_RXBUF_LEN];

static volatile uint16_t usart_tx_head = 0;
static volatile uint16_t usart_tx_tail = 0;
static volatile uint16_t usart_rx_head = 0;
static volatile uint16_t usart_rx_tail = 0;
static volatile uint32_t usart_rx_overflow = 0;

static uint8_t uart_tx_byte_in_flight = 0;
static uint8_t uart_rx_byte_in_flight = 0;

static volatile uint32_t i2c_events = 0;
static uint8_t bh1750_rx_buf[2];

/* Dane współdzielone przez moduły aplikacyjne:
 * - luxBuffer: historia pomiarów (bufor cykliczny),
 * - frequency: okres próbkowania w milisekundach, modyfikowany komendą SETTM.
 */
LuxBuffer_t luxBuffer;
uint32_t frequency = 1000;

static SensorState sensor_state = SENSOR_BOOT_DELAY;
static uint32_t sensor_deadline_ms = 0;

static FSM_State fsm_state = FSM_WAIT_START;
static Frame_t frame;
static uint8_t data_index = 0;
static uint8_t crc_index = 0;
static uint8_t crc_ascii[3];
static uint8_t computed_crc = 0;
static volatile uint8_t frame_ready = 0;
static uint8_t escape_pending = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void USART_StartTxIfIdle(void);
static void USART_TxEnqueue(const uint8_t *buf, uint16_t len);
static uint8_t USART_RxPushFromISR(uint8_t byte);
static uint8_t USART_RxPopFromISR(uint8_t *byte);

static void FSM_Reset(void);
static void FSM_ProcessByte(uint8_t rx);

static void Sensor_Init(void);
static void Sensor_Process(void);

static uint8_t TimeReached(uint32_t now, uint32_t deadline);
static uint32_t I2C_FetchEvents(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t TimeReached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static uint32_t I2C_FetchEvents(void)
{
    uint32_t events;

    /* Atomowe pobranie i wyczyszczenie flag zdarzeń z ISR.
     * Bez sekcji krytycznej moglibyśmy zgubić lub nadpisać bity ustawione
     * równolegle przez callbacki I2C/DMA.
     */
    __disable_irq();
    events = i2c_events;
    i2c_events = 0;
    __enable_irq();

    return events;
}

static void USART_StartTxIfIdle(void)
{
    uint8_t start_tx = 0;
    uint8_t byte_to_send = 0;

    /* Sekcja krytyczna chroni indeksy bufora TX przed wyścigiem z ISR UART.
     * Dzięki temu nie dojdzie do uszkodzenia kolejki podczas równoległych zmian.
     */
    __disable_irq();
    if ((usart_tx_head != usart_tx_tail) && (huart2.gState == HAL_UART_STATE_READY)) {
        byte_to_send = usart_tx_buf[usart_tx_tail];
        usart_tx_tail = (uint16_t)((usart_tx_tail + 1u) % USART_TXBUF_LEN);
        start_tx = 1;
    }
    __enable_irq();

    if (start_tx) {
        uart_tx_byte_in_flight = byte_to_send;
        /* Gdy HAL odrzuci start transmisji, cofamy ogon kolejki TX,
         * aby nie utracić bajtu zdjętego wcześniej z bufora.
         */
        if (HAL_UART_Transmit_IT(&huart2, &uart_tx_byte_in_flight, 1u) != HAL_OK) {
            __disable_irq();
            usart_tx_tail = (uint16_t)((usart_tx_tail == 0u) ? (USART_TXBUF_LEN - 1u) : (usart_tx_tail - 1u));
            __enable_irq();
        }
    }
}

static void USART_TxEnqueue(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == NULL) || (len == 0u)) {
        return;
    }

    /*
     * Unikamy czekania aktywnego i blokowania głównej pętli.
     */
    __disable_irq();
    for (i = 0u; i < len; i++) {
        uint16_t next = (uint16_t)((usart_tx_head + 1u) % USART_TXBUF_LEN);
        if (next == usart_tx_tail) {
            break;
        }
        usart_tx_buf[usart_tx_head] = buf[i];
        usart_tx_head = next;
    }
    __enable_irq();

    USART_StartTxIfIdle();
}

void USART_SendBuffer(const uint8_t *buf, uint16_t len)
{
    USART_TxEnqueue(buf, len);
}

void USART_fsend(char *format, ...)
{
    char tmp[128];
    int n;
    va_list arglist;

    va_start(arglist, format);
    n = vsnprintf(tmp, sizeof(tmp), format, arglist);
    va_end(arglist);

    if (n <= 0) {
        return;
    }

    if ((size_t)n >= sizeof(tmp)) {
        n = (int)(sizeof(tmp) - 1u);
    }

    USART_TxEnqueue((const uint8_t *)tmp, (uint16_t)n);
}

static uint8_t USART_RxPushFromISR(uint8_t byte)
{
    uint16_t next = (uint16_t)((usart_rx_head + 1u) % USART_RXBUF_LEN);
    if (next == usart_rx_tail) {
        usart_rx_overflow++;
        return 0u;
    }

    usart_rx_buf[usart_rx_head] = byte;
    usart_rx_head = next;
    return 1u;
}

static uint8_t USART_RxPopFromISR(uint8_t *byte)
{
    if (usart_rx_tail == usart_rx_head) {
        return 0u;
    }

    *byte = usart_rx_buf[usart_rx_tail];
    usart_rx_tail = (uint16_t)((usart_rx_tail + 1u) % USART_RXBUF_LEN);
    return 1u;
}

static void FSM_Reset(void)
{
    fsm_state = FSM_WAIT_START;
    data_index = 0;
    crc_index = 0;
    computed_crc = 0;
    escape_pending = 0;
}

static void FSM_ProcessByte(uint8_t rx)
{
    /* Parser ramki działający bajt-po-bajcie.
     * Obsługa escape:
     * - '/:' oznacza znak ':',
     * - '//' oznacza znak '/'.
     * Escape dopuszczamy tylko tam, gdzie ma sens semantyczny (sender/receiver/data).
     * W polach LEN i CRC sekwencje escape są błędem składni ramki.
     */
    switch (fsm_state) {
    case FSM_WAIT_START:
        if (rx == ':') {
            computed_crc = rx;
            data_index = 0;
            crc_index = 0;
            frame.status = FRAME_NONE;
            escape_pending = 0;
            fsm_state = FSM_WAIT_SENDER;
        }
        break;

    case FSM_WAIT_SENDER:
        if (escape_pending) {
            if ((rx != ':') && (rx != '/')) {
                frame.status = FRAME_ERR_ESC;
                FSM_Reset();
                break;
            }
            escape_pending = 0;
        } else {
            if (rx == '/') {
                escape_pending = 1;
                break;
            }
            if (rx == ':') {
                frame.status = FRAME_ERR_ESC;
                FSM_Reset();
                break;
            }
        }
        frame.sender = rx;
        computed_crc = (uint8_t)(computed_crc + rx);
        fsm_state = FSM_WAIT_RECEIVER;
        break;

    case FSM_WAIT_RECEIVER:
        if (escape_pending) {
            if ((rx != ':') && (rx != '/')) {
                frame.status = FRAME_ERR_ESC;
                frame_ready = 1;
                FSM_Reset();
                break;
            }
            escape_pending = 0;
        } else {
            if (rx == '/') {
                escape_pending = 1;
                break;
            }
            if (rx == ':') {
                frame.status = FRAME_ERR_ESC;
                frame_ready = 1;
                FSM_Reset();
                break;
            }
        }

        frame.receiver = rx;
        computed_crc = (uint8_t)(computed_crc + rx);
        fsm_state = FSM_WAIT_LEN;
        break;

    case FSM_WAIT_LEN:
        if (escape_pending || (rx == '/') || (rx == ':')) {
            frame.status = FRAME_ERR_ESC;
            frame_ready = 1;
            FSM_Reset();
            break;
        }

        if ((rx < '0') || (rx > '9')) {
            frame.status = FRAME_ERR_LEN;
            frame_ready = 1;
            FSM_Reset();
            break;
        }

        frame.length = (uint8_t)(rx - '0');
        computed_crc = (uint8_t)(computed_crc + rx);
        data_index = 0;

        if (frame.length > MAX_DATA_LEN) {
            frame.status = FRAME_ERR_LEN;
            frame_ready = 1;
            FSM_Reset();
            break;
        }

        fsm_state = (frame.length > 0u) ? FSM_WAIT_DATA : FSM_WAIT_CRC;
        break;

    case FSM_WAIT_DATA:
        if (escape_pending) {
            if ((rx != ':') && (rx != '/')) {
                frame.status = FRAME_ERR_ESC;
                frame_ready = 1;
                FSM_Reset();
                break;
            }
            escape_pending = 0;
        } else {
            if (rx == '/') {
                escape_pending = 1;
                break;
            }
            if (rx == ':') {
                frame.status = FRAME_ERR_ESC;
                frame_ready = 1;
                FSM_Reset();
                break;
            }
        }

        frame.data[data_index++] = rx;
        computed_crc = (uint8_t)(computed_crc + rx);
        if (data_index >= frame.length) {
            fsm_state = FSM_WAIT_CRC;
        }
        break;

    case FSM_WAIT_CRC:
        if (escape_pending || (rx == '/') || (rx == ':')) {
            frame.status = FRAME_ERR_ESC;
            frame_ready = 1;
            FSM_Reset();
            break;
        }

        if ((rx < '0') || (rx > '9')) {
            frame.status = FRAME_ERR_CRC;
            frame_ready = 1;
            FSM_Reset();
            break;
        }

        crc_ascii[crc_index++] = rx;
        if (crc_index < 3u) {
            break;
        }

        frame.crc = (uint8_t)((crc_ascii[0] - '0') * 100u +
                              (crc_ascii[1] - '0') * 10u +
                              (crc_ascii[2] - '0'));

        if (frame.crc == computed_crc) {
            frame.status = FRAME_OK;
        } else {
            frame.status = FRAME_ERR_CRC;
        }

        frame_ready = 1;
        FSM_Reset();
        break;

    default:
        FSM_Reset();
        break;
    }
}

static void Sensor_Init(void)
{
    /* Czas ochronny po starcie zasilania.
     * BH1750 wymaga krótkiej stabilizacji zanim przyjmie pierwsze komendy.
     */
    sensor_state = SENSOR_BOOT_DELAY;
    sensor_deadline_ms = HAL_GetTick() + 200u;
}

static void Sensor_Process(void)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t events = I2C_FetchEvents();

    /* Nieblokująca maszyna stanów czujnika BH1750.
     * Przejścia stanu wynikają wyłącznie z:
     * - zdarzeń DMA/I2C zgłaszanych przez przerwania,
     * - timeoutów liczonych na podstawie HAL_GetTick().
     * Brak HAL_Delay i brak oczekiwania aktywnego.
     */
    switch (sensor_state) {
    case SENSOR_BOOT_DELAY:
        if (TimeReached(now, sensor_deadline_ms)) {
            sensor_state = SENSOR_SEND_POWER_ON;
        }
        break;

    case SENSOR_SEND_POWER_ON:
        if (BH1750_SendCommand_DMA(&hi2c1, BH1750_POWER_ON) == HAL_OK) {
            sensor_state = SENSOR_WAIT_POWER_ON;
        } else {
            sensor_deadline_ms = now + 10u;
            sensor_state = SENSOR_BOOT_DELAY;
        }
        break;

    case SENSOR_WAIT_POWER_ON:
        if ((events & I2C_EVT_ERR) != 0u) {
            sensor_state = SENSOR_SEND_POWER_ON;
        } else if ((events & I2C_EVT_TX_DONE) != 0u) {
            sensor_deadline_ms = now + 10u;
            sensor_state = SENSOR_SEND_RESET;
        }
        break;

    case SENSOR_SEND_RESET:
        if (!TimeReached(now, sensor_deadline_ms)) {
            break;
        }

        if (BH1750_SendCommand_DMA(&hi2c1, BH1750_RESET) == HAL_OK) {
            sensor_state = SENSOR_WAIT_RESET;
        } else {
            sensor_deadline_ms = now + 10u;
        }
        break;

    case SENSOR_WAIT_RESET:
        if ((events & I2C_EVT_ERR) != 0u) {
            sensor_state = SENSOR_SEND_RESET;
        } else if ((events & I2C_EVT_TX_DONE) != 0u) {
            sensor_deadline_ms = now + 10u;
            sensor_state = SENSOR_SEND_CONT_H_RES;
        }
        break;

    case SENSOR_SEND_CONT_H_RES:
        if (!TimeReached(now, sensor_deadline_ms)) {
            break;
        }

        if (BH1750_SendCommand_DMA(&hi2c1, BH1750_CONT_H_RES) == HAL_OK) {
            sensor_state = SENSOR_WAIT_CONT_H_RES;
        } else {
            sensor_deadline_ms = now + 10u;
        }
        break;

    case SENSOR_WAIT_CONT_H_RES:
        if ((events & I2C_EVT_ERR) != 0u) {
            sensor_state = SENSOR_SEND_CONT_H_RES;
        } else if ((events & I2C_EVT_TX_DONE) != 0u) {
            sensor_deadline_ms = now + 180u;
            sensor_state = SENSOR_FIRST_MEAS_DELAY;
        }
        break;

    case SENSOR_FIRST_MEAS_DELAY:
        if (TimeReached(now, sensor_deadline_ms)) {
            sensor_state = SENSOR_READ_REQUEST;
        }
        break;

    case SENSOR_READ_REQUEST:
        if (BH1750_ReadRaw_DMA(&hi2c1, bh1750_rx_buf, sizeof(bh1750_rx_buf)) == HAL_OK) {
            sensor_state = SENSOR_WAIT_READ;
        } else {
            sensor_deadline_ms = now + 10u;
            sensor_state = SENSOR_FIRST_MEAS_DELAY;
        }
        break;

    case SENSOR_WAIT_READ:
        if ((events & I2C_EVT_ERR) != 0u) {
            sensor_deadline_ms = now + 10u;
            sensor_state = SENSOR_FIRST_MEAS_DELAY;
        } else if ((events & I2C_EVT_RX_DONE) != 0u) {
            float lux;
            uint16_t lux_x10;

            BH1750_ConvertLux(bh1750_rx_buf, &lux);
            lux_x10 = (uint16_t)(lux * 10.0f);
            LuxBuffer_Push(&luxBuffer, lux_x10);

            sensor_deadline_ms = now + frequency;
            sensor_state = SENSOR_FIRST_MEAS_DELAY;
        }
        break;

    default:
        sensor_state = SENSOR_SEND_POWER_ON;
        break;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        USART_StartTxIfIdle();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t byte;

        /* Bajt z UART trafia najpierw do bufora RX.
         * Pozwala to buforować krótkie bursty oraz ogranicza ryzyko utraty danych.
         */
        (void)USART_RxPushFromISR(uart_rx_byte_in_flight);

        /* FSM nadal uruchamiamy wyłącznie w ISR RX,
         * ale wejściem jest teraz kolejka RX zamiast pojedynczej zmiennej.
         */
        while (USART_RxPopFromISR(&byte) != 0u) {
            FSM_ProcessByte(byte);
        }

        (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte_in_flight, 1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte_in_flight, 1u);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_events |= I2C_EVT_TX_DONE;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_events |= I2C_EVT_RX_DONE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_events |= I2C_EVT_ERR;
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
  LuxBuffer_Init(&luxBuffer);
  Sensor_Init();

  (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte_in_flight, 1u);

  {
      const char msg[] = "START USART2 IRQ\r\n";
      USART_SendBuffer((const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1u));
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (frame_ready != 0u) {
        Frame_t local_frame;

        __disable_irq();
        frame_ready = 0u;
        local_frame = frame;
        __enable_irq();

        Frame_Handle(&local_frame);
    }

    Sensor_Process();
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
#ifdef USE_FULL_ASSERT
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
