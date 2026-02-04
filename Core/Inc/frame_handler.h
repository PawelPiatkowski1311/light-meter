/*
 * frame_handler.h
 *
 *  Created on: Jan 7, 2026
 *      Author: pawel
 */

#ifndef INC_FRAME_HANDLER_H_
#define INC_FRAME_HANDLER_H_

#include <stdint.h>

/* Główna funkcja obsługi ramki
   data   – wskaźnik na dane ramki
   length – długość ramki w bajtach */
void Frame_Handle(const Frame_t *frame);

void FrameHandler_Process(uint8_t *data, uint16_t length);

void Send_Error(const Frame_t *frame, uint8_t error_code);


#endif /* INC_FRAME_HANDLER_H_ */
