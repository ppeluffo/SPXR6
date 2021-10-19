/*
 * l_float_ringBuffer.h
 *
 *  Created on: 18 oct. 2021
 *      Author: pablo
 */

#ifndef SPX_LIBS_INCLUDE_L_FLOAT_RINGBUFFER_H_
#define SPX_LIBS_INCLUDE_L_FLOAT_RINGBUFFER_H_


#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"

typedef void * ringBufferHandle_t;

typedef struct {
	float *buff;
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	uint16_t length;
} float_ringBuffer_s;

//------------------------------------------------------------------------------------

void rbf_CreateStatic ( float_ringBuffer_s *rB, float *storage_area, uint16_t size  );
bool rbf_Poke( float_ringBuffer_s *rB, float *fFloat );
bool rbf_Pop( float_ringBuffer_s *rB, float *fFloat );
void rbf_Flush( float_ringBuffer_s *rB );
uint16_t rbf_GetCount( float_ringBuffer_s *rB );
uint16_t rbf_GetFreeCount( float_ringBuffer_s *rB );


#endif /* SPX_LIBS_INCLUDE_L_FLOAT_RINGBUFFER_H_ */
