/*
 * frame.h
 *
 *  Created on: 6 lut 2026
 *      Author: user
 */

#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include <stdint.h>

#define MAX_DATA_LEN 64

// Statusy
typedef enum {
	FRAME_OK,
    FRAME_NONE,     // brak ramki,
    FRAME_ERR_LEN,	// nieprawidłowa długość
    FRAME_ERR_CRC,	// błędne crc
	FRAME_ERR_ADD,	// adresat nie istnieje
	FRAME_ERR_CMD,	// komenda nie istnieje
	FRAME_ERR_TIM	// zbyt krótki interwał
} frame_status_t;

typedef struct {
    uint8_t sender;
    uint8_t receiver;
    uint8_t length;
    uint8_t data[MAX_DATA_LEN];
    uint8_t crc;
    frame_status_t status;
} Frame_t;

#endif /* INC_FRAME_H_ */
