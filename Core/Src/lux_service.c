/*
 * lux_service.c
 *
 *  Created on: Jan 7, 2026
 *      Author: pawel
 */
#include "lux_service.h"
#include "lux_buffer.h"
#include <stdio.h>

#include "stm32f3xx_hal.h"

/* Bufor danych zdefiniowany globalnie */
extern LuxBuffer_t luxBuffer;

/* UART */
extern UART_HandleTypeDef huart2;

void LuxService_PrintLastN(uint16_t count)
{
    char uart_buf[64];

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
            int len = snprintf(uart_buf,
                               sizeof(uart_buf),
                               "%u: %u.%u lux\r\n",
                               i,
                               lux_x10 / 10,
                               lux_x10 % 10);

            HAL_UART_Transmit(&huart2,
                              (uint8_t*)uart_buf,
                              len,
                              100);
        }
    }
}
