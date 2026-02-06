/*
 * lux_service.c
 *
 *  Created on: Jan 7, 2026
 *      Author: pawel
 */

#include <stdio.h>

#include "main.h"

#include "lux_service.h"
#include "lux_buffer.h"
#include "stm32f3xx_hal.h"
#include "protocol.h"
#include "crc.h"

/* Bufor danych zdefiniowany globalnie */
extern LuxBuffer_t luxBuffer;

/* UART */
extern UART_HandleTypeDef huart2;

void LuxService_ReturnLastN(uint16_t count, uint8_t sender)
{
    /* Jeśli bufor pusty – nie ma co wypisywać */
    if (LuxBuffer_Count(&luxBuffer) == 0)
    {
        return;
    }

    /* Ograniczenie liczby wpisów */
    uint16_t available = LuxBuffer_Count(&luxBuffer);
    if (count > available)
    {
        count = available;
    }

    /* Indeks pierwszego wpisu (od najstarszego) */
    uint16_t start = available - count;

    /* Wypisywanie kolejnych wpisów */
    for (uint16_t i = 0; i < count; i++)
    {
        uint16_t lux_x10;

        /* Pobranie wpisu bezpośrednio z bufora */
        if (LuxBuffer_Get(&luxBuffer, start + i, &lux_x10))
        {
        	Print_Frame(lux_x10, sender);
        }
    }
}

void Print_Frame(uint16_t lux_x10, uint8_t sender) {
    uint8_t tx_buf[32];
    char payload[16];
    uint16_t idx = 0;
    int payload_len = 0;

    payload_len = snprintf(payload, sizeof(payload), "DATA%u", (unsigned int)lux_x10);
    if (payload_len <= 0 || payload_len > 9) {
        return;
    }

    /* === SKLADANIE RAMKI === */
    tx_buf[idx++] = ':';             // znak startu
    tx_buf[idx++] = DEVICE;          // kod urzadzenia nadawcy
    tx_buf[idx++] = (uint8_t)sender;   // kod urzadzenia odbiorcy
    tx_buf[idx++] = (uint8_t)('0' + payload_len); // dlugosc
    for (int i = 0; i < payload_len; i++) {
        tx_buf[idx++] = (uint8_t)payload[i];
    }
    /* === CRC (3 cyfry ASCII) === */
    {
        uint8_t crc = calc_crc(tx_buf, idx);
        tx_buf[idx++] = (uint8_t)('0' + (crc / 100));
        tx_buf[idx++] = (uint8_t)('0' + ((crc / 10) % 10));
        tx_buf[idx++] = (uint8_t)('0' + (crc % 10));
    }

    HAL_UART_Transmit(&huart2, tx_buf, idx, HAL_MAX_DELAY);
    //USART_SendBuffer(tx_buf, idx);
    return;
}

