/*
 * l_ringBuffer.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_RINGBUFFER_H_
#define SRC_SPX_LIBS_L_RINGBUFFER_H_


#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"

typedef struct {
	uint8_t *buff;
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	uint16_t length;
	bool arriving;
} ringBuffer_s;

void rBufferCreateStatic ( ringBuffer_s *rB, uint8_t *storage_area, uint16_t size  );
bool rBufferPoke( ringBuffer_s *rB, char *cChar );
bool rBufferPokeFromISR( ringBuffer_s *rB, char *cChar );
bool rBufferPop( ringBuffer_s *rB, char *cChar );
bool rBufferPopFromISR( ringBuffer_s *rB, char *cChar );
void rBufferFlush( ringBuffer_s *rB );
uint16_t rBufferGetCount( ringBuffer_s *rB );
uint16_t rBufferGetFreeCount( ringBuffer_s *rB );
bool rBufferReachLowWaterMark( ringBuffer_s *rB);
bool rBufferReachHighWaterMark( ringBuffer_s *rB );

//--------------------------------------------------------------------------------------------
// Ring Buffers de Estructuras
typedef struct {
	void *buff;
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	uint16_t length;
	uint16_t elementsize;
} void_ringBuffer_s;

void ringBuffer_CreateStatic ( void_ringBuffer_s *rB, void *storage_area, uint16_t buffersize, uint16_t elementsize  );
bool ringBuffer_Poke( void_ringBuffer_s *rB, void *element );
bool ringBuffer_Pop( void_ringBuffer_s *rB, void *element );
bool ringBuffer_PopRead( void_ringBuffer_s *rB, void *element );
void ringBuffer_Flush( void_ringBuffer_s *rB );
uint16_t ringBuffer_GetCount( void_ringBuffer_s *rB );
uint16_t ringBuffer_GetFreeCount( void_ringBuffer_s *rB );


#endif /* SRC_SPX_LIBS_L_RINGBUFFER_H_ */
