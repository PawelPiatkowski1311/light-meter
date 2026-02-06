/*
 * crc.c
 *
 *  Created on: 6 lut 2026
 *      Author: user
 */
#include "crc.h"

uint8_t calc_crc(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;

    for (uint16_t i = 0; i < len; i++) {
        crc += buf[i];
    }

    return crc;
}
