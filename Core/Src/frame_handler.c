/*
 * frame_handler.c
 *
 *  Created on: Jan 7, 2026
 *      Author: pawel
 */
#include "stm32f3xx_hal.h"

#include "main.h"
#include "frame_handler.h"
#include "lux_service.h"
#include "lux_buffer.h"
#include "protocol.h"
#include "crc.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Bufor globalny z pomiarami */
extern LuxBuffer_t luxBuffer;

/* UART do odpowiedzi */
extern UART_HandleTypeDef huart2;

extern uint32_t frequency;


/* Maksymalna długość obsługiwanej ramki */
#define FRAME_MAX_LEN 64

#define CMD_ACK   0x06   // ACK
#define CMD_NACK  0x15   // NACK (ASCII NAK)

uint8_t error_code;


void Frame_Handle(const Frame_t *frame) // koordynator
{
	Frame_Validation(frame);
}

void Frame_Validation(const Frame_t *frame)
{
	if (frame->status != FRAME_OK) {
    	Send_Error(frame, frame->status);
	}

	if (frame->receiver != DEVICE) {
    	Send_Error(frame, FRAME_ERR_ADD);
	}

	FrameHandler_Process(frame);
	return;
}

void Send_Error(const Frame_t *frame, frame_status_t error_code)
{
    uint8_t tx_buf[9];
    uint16_t idx = 0;
    const uint8_t payload_len = 2;

    /* === SKLADANIE RAMKI === */

    tx_buf[idx++] = ':';             // znak startu
    tx_buf[idx++] = DEVICE;          // kod urzadzenia nadawcy
    tx_buf[idx++] = (uint8_t)frame->sender;   // kod urzadzenia odbiorcy
    tx_buf[idx++] = (uint8_t)('0' + payload_len); // dlugosc
    tx_buf[idx++] = 'N';//(uint8_t)CMD_NACK;        // komenda: NAK
    tx_buf[idx++] = (uint8_t)('0' + error_code);      // kod bledu

    /* === CRC (3 cyfry ASCII) === */
    {
        uint8_t crc = calc_crc(tx_buf, idx);
        tx_buf[idx++] = (uint8_t)('0' + (crc / 100));
        tx_buf[idx++] = (uint8_t)('0' + ((crc / 10) % 10));
        tx_buf[idx++] = (uint8_t)('0' + (crc % 10));
    }

    /* === WYSYLANIE === */
    USART_SendBuffer(tx_buf, idx);
}

void Send_Response(const Frame_t *frame) {
    uint8_t tx_buf[9];
    uint16_t idx = 0;
    const uint8_t payload_len = 1;

    /* === SKLADANIE RAMKI === */

    tx_buf[idx++] = ':';             // znak startu
    tx_buf[idx++] = DEVICE;          // kod urzadzenia nadawcy
    tx_buf[idx++] = (uint8_t)frame->sender;   // kod urzadzenia odbiorcy
    tx_buf[idx++] = (uint8_t)('0' + payload_len); // dlugosc
    tx_buf[idx++] = 'A';//(uint8_t)CMD_NACK;        // komenda: NAK
    /* === CRC (3 cyfry ASCII) === */
    {
        uint8_t crc = calc_crc(tx_buf, idx);
        tx_buf[idx++] = (uint8_t)('0' + (crc / 100));
        tx_buf[idx++] = (uint8_t)('0' + ((crc / 10) % 10));
        tx_buf[idx++] = (uint8_t)('0' + (crc % 10));
    }

    /* === WYSYLANIE === */
    USART_SendBuffer(tx_buf, idx);
}

void Handle_Ping(const Frame_t *frame) {
    uint8_t tx_buf[9];
    uint16_t idx = 0;
    const uint8_t payload_len = 2;

    /* === SKLADANIE RAMKI === */

    tx_buf[idx++] = ':';             // znak startu
    tx_buf[idx++] = DEVICE;          // kod urzadzenia nadawcy
    tx_buf[idx++] = (uint8_t)frame->sender;   // kod urzadzenia odbiorcy
    tx_buf[idx++] = (uint8_t)('0' + payload_len); // dlugosc
    tx_buf[idx++] = 'A';//(uint8_t)CMD_NACK;        // komenda: NAK
    /* === CRC (3 cyfry ASCII) === */
    {
        uint8_t crc = calc_crc(tx_buf, idx);
        tx_buf[idx++] = (uint8_t)('0' + (crc / 100));
        tx_buf[idx++] = (uint8_t)('0' + ((crc / 10) % 10));
        tx_buf[idx++] = (uint8_t)('0' + (crc % 10));
    }

    /* === WYSYLANIE === */
    USART_SendBuffer(tx_buf, idx);
	return;
}

void FrameHandler_Process(const Frame_t *frame)
{
    char cmd[FRAME_MAX_LEN];

    /* Ochrona przed zbyt długą ramką */
    if (frame->length == 0 || frame->length >= FRAME_MAX_LEN)
    {
        return;
    }

    /* Skopiowanie danych do bufora roboczego
       + dopisanie znaku końca stringa */
    memcpy(cmd, frame->data, frame->length);
    cmd[frame->length] = '\0';

    /* ===== KOMENDA: LAST ===== */
    if (strcmp(cmd, "GETLAST") == 0)
    {
        /* Wywołanie logiki aplikacji */
        LuxService_ReturnLastN(1, frame->sender);
        return;
    }

    /* ===== KOMENDA: GET N ===== */
    if (strcmp(cmd, "GETALL") == 0)
    {
        /* Wywołanie logiki aplikacji */
        LuxService_ReturnLastN(LUX_BUFFER_SIZE, frame->sender); // moge tak zrobic bo jest ogranicznik
        return;
    }

    /* ===== KOMENDA: GET N ===== */
    if (strncmp(cmd, "GET", 3) == 0)
    {
        /* Konwersja argumentu na liczbę */
        uint16_t n = (uint16_t)atoi(&cmd[3]);

        /* Wywołanie logiki aplikacji */
        LuxService_ReturnLastN(n, frame->sender);
        return;
    }

    if (strncmp(cmd, "SETTM", 5) == 0)
    {
        uint32_t ms = (uint32_t)atoi(&cmd[5]);
        if (ms == 0) {
        	Send_Error(frame, FRAME_ERR_TIM);
        	return;
        }
        // ustawienie globalnej zmiennej z freertos.c
        extern uint32_t frequency;
        frequency = ms;

        Send_Response(frame);

        return;
    }

    /* ===== NIEZNANA KOMENDA ===== */
    Send_Error(frame, FRAME_ERR_CMD);
}


