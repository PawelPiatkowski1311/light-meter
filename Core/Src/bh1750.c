/*
 * bh1750.c
 *
 *  Created on: Jan 6, 2026
 *      Author: pawel
 */


#include "bh1750.h"

/**
 * @brief Inicjalizacja czujnika BH1750
 */
HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd;

    /* Włączenie czujnika */
    cmd = BH1750_POWER_ON;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    //HAL_Delay(10); /// do zmiany!!!!!!!!!!!

    /* Reset danych pomiarowych */
    cmd = BH1750_RESET;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    //HAL_Delay(10);

    /* Tryb ciągły, wysoka rozdzielczość  ///// przemiesione
    cmd = BH1750_CONT_H_RES;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
    */
}

HAL_StatusTypeDef BH1750_StartContHR(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd;

    cmd = BH1750_CONT_H_RES;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Odczyt natężenia światła
 */
HAL_StatusTypeDef BH1750_ReadLux(I2C_HandleTypeDef *hi2c, float *lux)
{
    uint8_t buffer[2];
    uint16_t raw;

    /* BH1750 potrzebuje czasu na pomiar */
    //HAL_Delay(180); //// DO ZMIANY!!!!!!!!!!!!!!!!!!!!!


    /* Odczyt 2 bajtów danych */
    if (HAL_I2C_Master_Receive(hi2c, BH1750_ADDR, buffer, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    /* Połączenie bajtów */
    raw = (buffer[0] << 8) | buffer[1];

    /* Przeliczenie na lux (wg datasheet) */
    *lux = raw / 1.2f;

    return HAL_OK;
}

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
    uint16_t raw = (uint16_t)((buf[0] << 8) | buf[1]);
    *lux = raw / 1.2f;
}
