/*
 * Comunicaciones.h
 *
 *  Created on: Oct 11, 2023
 *      Author: jrlle
 */

#ifndef COMUNICACIONES_H_
#define COMUNICACIONES_H_
#include <stddef.h>
#include "stdint.h"
#include <string.h>

//|===================================================|
//|                 STRUCTURES & DEF                  |
//|===================================================|

/*
 * Pointer to a function for communication.
 */
typedef void(*comPtr)(uint8_t*myBuf, int length);

/*
 * Pointer to a function for payload.
 */
typedef void(*payPtr)(uint8_t *payloadBuf, uint8_t lenPx);

//|===================================================|
//|                 LIBRARY FUNCTIONS                 |
//|===================================================|

/*
 * Function to call in the callback interruption.
 * @param In mybuf: Pointer to reception buffer.
 * @param In Length: Length of the reception buffer.
 */
void Comunicaciones_Recieve(uint8_t *bufRx, uint8_t Rxiw, uint8_t *Rxir);

/*
 * Function to check for transmit data.
 * @param In TxBuf: Pointer to transmit buffer.
 * @param In TxBufLen: Length of the transmit buffer.
 */
void Comunicaciones_encode(uint8_t *TxBuf, uint8_t *Txiw, uint8_t *payBuf, uint8_t Pxiw, uint8_t *Pxir);

//|===================================================|
//|                 ATTACH FUNCTIONS                  |
//|===================================================|

/*
 * Attach function for communication.
 * Receives the data from the MCU interruption.
 * @param cComPtr: type of pointer.
 */
void Comunicaciones_Transmit(comPtr aComPtr);

/*
 * Attach function with payload content.
 * Contains the payload received.
 * @param aPayPtr: type of pointer.
 */
void Comunicaciones_Decode(payPtr aPayPtr);

#endif /* COMUNICACIONES_H_ */
