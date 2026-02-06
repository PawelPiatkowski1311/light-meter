/*
 * bh1750.h
 *
 *  Created on: Jan 6, 2026
 *      Author: pawel
 */

//#ifndef INC_BH1750_H_
//#define INC_BH1750_H_

/*------------------------------------------------------------------*/
#ifndef BH1750_H
#define BH1750_H

#include "stm32f3xx_hal.h"   // zmień jeśli masz inny STM32

/* Adres czujnika (ADDR = GND lub NC) */
#define BH1750_ADDR (0x23 << 1)

/* Komendy BH1750 */
#define BH1750_POWER_ON      0x01
#define BH1750_RESET         0x07
#define BH1750_CONT_H_RES    0x10   // Continuous High Resolution Mode

/* Inicjalizacja czujnika */
HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c);

/* Rozpoczęcie pomiaru */
HAL_StatusTypeDef BH1750_StartContHR(I2C_HandleTypeDef *hi2c); /// !?!?!?

/* Odczyt natężenia światła w lux */
HAL_StatusTypeDef BH1750_ReadLux(I2C_HandleTypeDef *hi2c, float *lux);

/* Wersje nieblokujace z DMA */
HAL_StatusTypeDef BH1750_SendCommand_DMA(I2C_HandleTypeDef *hi2c, uint8_t cmd);
HAL_StatusTypeDef BH1750_ReadRaw_DMA(I2C_HandleTypeDef *hi2c, uint8_t *buf, uint16_t len);
void BH1750_ConvertLux(const uint8_t *buf, float *lux);

#endif /* INC_BH1750_H_ */
