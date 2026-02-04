/*
 * lux_buffer.h
 *
 *  Created on: Jan 7, 2026
 *      Author: pawel
 */

#ifndef INC_LUX_BUFFER_H_
#define INC_LUX_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

/* Maksymalna liczba zapisów */
#define LUX_BUFFER_SIZE 900

/* Struktura bufora cyklicznego */
typedef struct
{
    uint16_t data[LUX_BUFFER_SIZE];  // lux * 10
    uint16_t head;                   // indeks zapisu
    uint16_t count;                  // liczba aktualnych wpisów
} LuxBuffer_t;

/* Inicjalizacja bufora */
void LuxBuffer_Init(LuxBuffer_t *buf);

/* Dodanie nowego pomiaru */
void LuxBuffer_Push(LuxBuffer_t *buf, uint16_t lux_x10);

/* Odczyt wpisu po indeksie (0 = najstarszy) */
bool LuxBuffer_Get(const LuxBuffer_t *buf,
                   uint16_t index,
                   uint16_t *lux_x10);

/* Pobranie NAJNOWSZEGO wpisu z bufora */
bool LuxBuffer_GetLatest(const LuxBuffer_t *buf,
                         uint16_t *lux_x10);

/* Pobranie ostatnich N wpisów do tablicy wyjściowej
   Zwraca liczbę faktycznie pobranych wpisów */
uint16_t LuxBuffer_GetLastN(const LuxBuffer_t *buf,
                            uint16_t count,
                            uint16_t *out);

/* Pobranie liczby zapisanych próbek */
uint16_t LuxBuffer_Count(const LuxBuffer_t *buf);

#endif /* INC_LUX_BUFFER_H_ */
