/*
 * frame_handler.c
 *
 *  Created on: Jan 7, 2026
 *      Author: pawel
 */
#include "stm32f3xx_hal.h"

#include "frame_handler.h"
#include "lux_service.h"
#include "lux_buffer.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define DEVICE 'p'

/* Bufor globalny z pomiarami */
extern LuxBuffer_t luxBuffer;

/* UART do odpowiedzi */
extern UART_HandleTypeDef huart2;

/* Maksymalna długość obsługiwanej ramki */
#define FRAME_MAX_LEN 64

#define CMD_ACK   0x06   // ACK
#define CMD_NACK  0x15   // NACK (ASCII NAK)

uint8_t error_code;


void Frame_Handle(const Frame_t *frame) // koordynator
{
	Frame_Validation(&frame);
}

void Frame_Validation(const Frame_t *frame)
{
    switch (frame->status)
    {
        case FRAME_OK:
            Handle_Ping();
            break;

        case FRAME_ERR_LEN:
        	Send_Error(&frame, FRAME_ERR_LEN);
            break;

        case FRAME_ERR_CRC:
        	Send_Error(&frame ,FRAME_ERR_CRC);
            break;
    }
}

static uint8_t calc_crc(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;

    for (uint16_t i = 0; i < len; i++) {
        crc += buf[i];
    }

    return crc;
}

void Send_Error(const Frame_t *frame, error_code)
{
    uint8_t tx_buf[7];
    uint16_t idx = 0;

    /* === SKŁADANIE RAMKI === */

    tx_buf[idx++] = ';';     		 // znak startu
    tx_buf[idx++] = DEVICE;     	 // kod urządzenia nadawcy
    tx_buf[idx++] = frame->sender;   // kod urządzenia odbiorcy
    tx_buf[idx++] = 2;               // długość
    tx_buf[idx++] = CMD_NACK;        // komenda: ERROR
    tx_buf[idx++] = (uint8_t)error_code;    // kod błędu

    /* === CRC === */
    tx_buf[idx++] = calc_crc(tx_buf, idx);

    /* === WYSYŁANIE === */
    HAL_UART_Transmit(&huart2, &tx_buf, 7, HAL_MAX_DELAY);
}

void Send_Response();

void FrameHandler_Process(uint8_t *data, uint16_t length)
{
    char cmd[FRAME_MAX_LEN];

    /* Ochrona przed zbyt długą ramką */
    if (length == 0 || length >= FRAME_MAX_LEN)
    {
        return;
    }

    /* Skopiowanie danych do bufora roboczego
       + dopisanie znaku końca stringa */
    memcpy(cmd, data, length);
    cmd[length] = '\0';

    /* ===== KOMENDA: LAST ===== */
    if (strcmp(cmd, "GETLAST") == 0)
    {
        /* Wywołanie logiki aplikacji */
        LuxService_PrintLastN(1);
        return;
    }

    /* ===== KOMENDA: GET N ===== */
    if (strncmp(cmd, "GETALL", 3) == 0)
    {
        /* Wywołanie logiki aplikacji */
        LuxService_PrintLastN(LUX_BUFFER_SIZE); // moge tak zrobic bo jest ogranicznik
        return;
    }

    /* ===== KOMENDA: GET N ===== */
    if (strncmp(cmd, "GET", 3) == 0)
    {
        /* Konwersja argumentu na liczbę */
        uint16_t n = (uint16_t)atoi(&cmd[3]);

        /* Wywołanie logiki aplikacji */
        LuxService_PrintLastN(n);
        return;
    }

    /* ===== KOMENDA: COUNT ===== */
    if (strcmp(cmd, "COUNT") == 0)
    {
        char buf[32];

        uint16_t count = LuxBuffer_Count(&luxBuffer);

        int len = snprintf(buf,
                           sizeof(buf),
                           "COUNT: %u\r\n",
                           count);

        HAL_UART_Transmit(&huart2,
                          (uint8_t*)buf,
                          len,
                          100);
        return;
    }

    /* ===== NIEZNANA KOMENDA ===== */
    {
        const char *err = "ERR: UNKNOWN CMD\r\n";
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)err,
                          strlen(err),
                          100);
    }
}


