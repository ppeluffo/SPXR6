/*
 * l_float_ringBuffer.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */


#include "l_float_ringBuffer.h"

//------------------------------------------------------------------------------------
// RINGBUFFERS DE FLOATS ( Se usa en pilotos )
//------------------------------------------------------------------------------------
void rbf_CreateStatic ( float_ringBuffer_s *rB, float *storage_area, uint16_t size  )
{
	rB->buff = storage_area;
	rB->head = 0;	// start
	rB->tail = 0;	// end
	rB->count = 0;
	rB->length = size;

}
//------------------------------------------------------------------------------------
bool rbf_Poke( float_ringBuffer_s *rB, float *fFloat )
{

bool ret = false;

	taskENTER_CRITICAL();

	// Si el buffer esta vacio ajusto los punteros
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
	}

	if ( rB->count < rB->length ) {
		rB->buff[rB->head] = *fFloat;
		++rB->count;
		// Avanzo en modo circular
		rB->head = ( rB->head  + 1 ) % ( rB->length );
		ret = true;
    }

	taskEXIT_CRITICAL();
	return(ret);

}
//------------------------------------------------------------------------------------
bool rbf_Pop( float_ringBuffer_s *rB, float *fFloat )
{

bool ret = false;

	taskENTER_CRITICAL();

	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*fFloat = rB->buff[rB->tail];
	--rB->count;
	// Avanzo en modo circular
	rB->tail = ( rB->tail  + 1 ) % ( rB->length );
	ret = true;

	taskEXIT_CRITICAL();
	return(ret);
}
//------------------------------------------------------------------------------------
void rbf_Flush( float_ringBuffer_s *rB )
{

	rB->head = 0;
	rB->tail = 0;
	rB->count = 0;
	memset(rB->buff,'\0', rB->length );
}
//------------------------------------------------------------------------------------
uint16_t rbf_GetCount( float_ringBuffer_s *rB )
{

	return(rB->count);

}
//------------------------------------------------------------------------------------
uint16_t rbf_GetFreeCount( float_ringBuffer_s *rB )
{

	return(rB->length - rB->count);

}
//------------------------------------------------------------------------------------
