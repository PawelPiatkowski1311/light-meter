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
void LuxService_PrintLastN(uint16_t count);

#endif /* INC_LUX_SERVICE_H_ */
