/*
 * crc.h
 *
 *  Created on: 6 lut 2026
 *      Author: user
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include <stdint.h>

uint8_t calc_crc(const uint8_t *buf, uint16_t len);


#endif /* INC_CRC_H_ */
