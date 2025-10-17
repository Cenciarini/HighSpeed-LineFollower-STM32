/*
 * Comunicaciones.c
 *
 *  Created on: Oct 11, 2023
 *      Author: jrlle
 */
#include "comunicaciones.h"
#include "stdint.h"
#include <string.h>
#include <stddef.h>

typedef enum{
	START,
	HEADER_1,
	HEADER_2,
	HEADER_3,
	NBYTES,
	NBYTES_D,
	TOKEN,
	PAYLOAD,
	ERROR_RX,
	OK
}_eProtocolo;

_eProtocolo estadoProtocolo;

static comPtr myComPtr = NULL;
static payPtr myPayPtr = NULL;
uint8_t payloadBuf[256];

void Comunicaciones_encode(uint8_t *TxBuf, uint8_t *Txiw, uint8_t *payBuf, uint8_t Pxiw, uint8_t *Pxir);

void Comunicaciones_Recieve(uint8_t *bufRx, uint8_t Rxiw, uint8_t *Rxir);

void Comunicaciones_Transmit(comPtr aComPtr);

void Comunicaciones_Decode(payPtr aPayPtr);

void Comunicaciones_Decode(payPtr aPayPtr){
	myPayPtr = aPayPtr;
}

void Comunicaciones_Transmit(comPtr aComPtr){
	myComPtr = aComPtr;
}

void Comunicaciones_Recieve(uint8_t *bufRx, uint8_t Rxiw, uint8_t *Rxir){
	int8_t nBytes=0;
	uint8_t Rxir_aux = 0, cksRx = 0, lenPx = 0;
	Rxir_aux = *Rxir;

	estadoProtocolo = START;

	while (Rxir_aux != Rxiw){
		switch (estadoProtocolo) {
			case START:
				if (bufRx[Rxir_aux++]=='U'){
					estadoProtocolo=HEADER_1;
				}else{
					estadoProtocolo=ERROR_RX;
				}
				break;
			case HEADER_1:
				if (bufRx[Rxir_aux++]=='N'){
					estadoProtocolo=HEADER_2;
				}else{
					estadoProtocolo=ERROR_RX;
				}
				break;
			case HEADER_2:
				if (bufRx[Rxir_aux++]=='E'){
					estadoProtocolo=HEADER_3;
				}else{
					estadoProtocolo=ERROR_RX;
				}
				break;
			case HEADER_3:
				if (bufRx[Rxir_aux++]=='R'){
					estadoProtocolo=NBYTES;
				}else{
					estadoProtocolo=ERROR_RX;
				}
				break;
			case NBYTES:
				nBytes=bufRx[Rxir_aux++];
				estadoProtocolo=TOKEN;
				break;
			case NBYTES_D:
				nBytes *= 10;
				nBytes += bufRx[Rxir_aux++];
				estadoProtocolo = TOKEN;
				break;
			case TOKEN:
				if (bufRx[Rxir_aux++]==':'){
					estadoProtocolo=PAYLOAD;
					cksRx ='U'^'N'^'E'^'R'^ nBytes ^':';
				}
				else{
					estadoProtocolo=NBYTES_D;
					if(nBytes > 10)
						estadoProtocolo = ERROR_RX;
				}
				break;
			case PAYLOAD://ID+paylodad+cks
				if (nBytes>1){
					payloadBuf[lenPx++]=bufRx[Rxir_aux];
					cksRx ^= bufRx[Rxir_aux++];
				}
				nBytes--;
				if(nBytes<=0){
					estadoProtocolo=START;
					if(cksRx != bufRx[Rxir_aux]){
						estadoProtocolo = ERROR_RX;
					}else{
						*Rxir = Rxiw;
						myPayPtr(payloadBuf,lenPx);
						return;
					}
				}
				break;
			case ERROR_RX:
				*Rxir = Rxiw;
				break;
			case OK:

			default:
				estadoProtocolo=START;
				break;
		}
	}
}


void Comunicaciones_encode(uint8_t *TxBuf, uint8_t *Txiw, uint8_t *payBuf, uint8_t Pxiw, uint8_t *Pxir){
	uint8_t Txiw_aux, Pxir_aux, PayLen = 0, cksTx = 0;
	Txiw_aux = *Txiw;
	Pxir_aux = *Pxir;

	TxBuf[Txiw_aux++]='U';
	TxBuf[Txiw_aux++]='N';
	TxBuf[Txiw_aux++]='E';
	TxBuf[Txiw_aux++]='R';
	if(Pxiw < Pxir_aux){
		PayLen = (256 - Pxir_aux) + Pxiw;
	}else{
		PayLen = Pxiw - Pxir_aux;
	}
	TxBuf[Txiw_aux++]= PayLen + 1;
	TxBuf[Txiw_aux++]=':';
	cksTx = 'U' ^ 'N' ^ 'E' ^ 'R' ^ (PayLen + 1) ^ ':';
	while(Pxir_aux != Pxiw){
		TxBuf[Txiw_aux++] = payBuf[Pxir_aux];
		cksTx ^= payBuf[Pxir_aux++];
	}
	TxBuf[Txiw_aux++]=cksTx;

	*Txiw = Txiw_aux;
	*Pxir = Pxir_aux;
}
