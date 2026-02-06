/*
 * lux_service.h
 *
 *  Created on: Jan 7, 2026
 *      Author: pawel
 */

#ifndef INC_LUX_SERVICE_H_
#define INC_LUX_SERVICE_H_

#include <stdint.h>

/* Wypisuje ostatnie N pomiarów lux do terminala */
void LuxService_ReturnLastN(uint16_t count, uint8_t sender);

void Print_Frame(uint16_t lux_x10, uint8_t sender);

#endif /* INC_LUX_SERVICE_H_ */
