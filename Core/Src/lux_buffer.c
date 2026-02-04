#include "lux_buffer.h"

void LuxBuffer_Init(LuxBuffer_t *buf)
{
    /* Zerowanie indeksów */
    buf->head = 0;
    buf->count = 0;
}

void LuxBuffer_Push(LuxBuffer_t *buf, uint16_t lux_x10)
{
    /* Zapis nowej wartości */
    buf->data[buf->head] = lux_x10;

    /* Przesunięcie indeksu zapisu */
    buf->head++;

    if (buf->head >= LUX_BUFFER_SIZE)
    {
        buf->head = 0;  // zawinięcie bufora
    }

    /* Zwiększenie licznika wpisów (do maksimum) */
    if (buf->count < LUX_BUFFER_SIZE)
    {
        buf->count++;
    }
}

bool LuxBuffer_Get(const LuxBuffer_t *buf,
                   uint16_t index,
                   uint16_t *lux_x10)
{
    /* Sprawdzenie zakresu */
    if (index >= buf->count)
    {
        return false;
    }

    /* Obliczenie indeksu w tablicy
       0 = najstarszy wpis */
    uint16_t start =
        (buf->head + LUX_BUFFER_SIZE - buf->count) % LUX_BUFFER_SIZE;

    uint16_t pos =
        (start + index) % LUX_BUFFER_SIZE;

    *lux_x10 = buf->data[pos];

    return true;
}

bool LuxBuffer_GetLatest(const LuxBuffer_t *buf,
                         uint16_t *lux_x10)
{
    /* Jeśli bufor jest pusty – brak danych */
    if (buf->count == 0)
    {
        return false;
    }

    /* head wskazuje miejsce następnego zapisu,
       więc ostatni wpis jest o jeden wcześniej */
    int16_t last_index = (int16_t)buf->head - 1;

    /* Obsługa zawinięcia (jeśli head == 0) */
    if (last_index < 0)
    {
        last_index = LUX_BUFFER_SIZE - 1;
    }

    /* Odczyt najnowszej wartości */
    *lux_x10 = buf->data[last_index];

    return true;
}

uint16_t LuxBuffer_GetLastN(const LuxBuffer_t *buf,
                            uint16_t count,
                            uint16_t *out)
{
    /* Jeśli bufor pusty – nic nie pobieramy */
    if (buf->count == 0)
    {
        return 0;
    }

    /* Ograniczenie liczby żądanych wpisów
       do liczby faktycznie dostępnych */
    if (count > buf->count)
    {
        count = buf->count;
    }

    /* Indeks pierwszego wpisu do pobrania
       (liczony od najstarszego) */
    uint16_t start_index = buf->count - count;

    /* Pobranie kolejnych wpisów */
    for (uint16_t i = 0; i < count; i++)
    {
        LuxBuffer_Get(buf,
                      start_index + i,
                      &out[i]);
    }

    /* Zwrócenie liczby pobranych wpisów */
    return count;
}

uint16_t LuxBuffer_Count(const LuxBuffer_t *buf)
{
    return buf->count;
}
