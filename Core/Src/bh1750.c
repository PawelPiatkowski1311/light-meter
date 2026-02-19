/*
 * bh1750.c
 *
 *  Created on: Jan 6, 2026
 *      Author: pawel
 */


#include "bh1750.h"

/* Funkcje pomocnicze DMA utrzymują komunikację z czujnikiem w trybie nieblokującym:
 * start transferu następuje natychmiast, a zakończenie jest sygnalizowane z ISR.
 */
HAL_StatusTypeDef BH1750_SendCommand_DMA(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit_DMA(hi2c, BH1750_ADDR, &cmd, 1);
}

HAL_StatusTypeDef BH1750_ReadRaw_DMA(I2C_HandleTypeDef *hi2c, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Master_Receive_DMA(hi2c, BH1750_ADDR, buf, len);
}

void BH1750_ConvertLux(const uint8_t *buf, float *lux)
{
    /* BH1750 zwraca wynik w formacie big-endian (2 bajty).
     * Przeliczenie na luks według dokumentacji: lux = raw / 1.2.
     */
    uint16_t raw = (uint16_t)((buf[0] << 8) | buf[1]);
    *lux = raw / 1.2f;
}
