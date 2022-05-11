/*
 * dma_driver.h
 *
 *  Created on: 7 abr. 2022
 *      Author: pablo
 */

#ifndef ATMEL_LIBS_INCLUDE_DMA_DRIVER_H_
#define ATMEL_LIBS_INCLUDE_DMA_DRIVER_H_


#include "avr_compiler.h"


/*! \brief This function enable the DMA module.
 *
 *  \note Each individual DMA channel must be enabled separately
 *        using the DMA_EnableChannel() function.
 */
#define DMA_Enable()    ( DMA.CTRL |= DMA_ENABLE_bm )

/*! \brief This function disables the DMA module.
 *
 *  \note On-going transfers will be aborted.
 */
#define DMA_Disable()   ( DMA.CTRL &= ~DMA_ENABLE_bm )



/*! Prototyping of functions. */
void DMA_Reset( void );
void DMA_ConfigDoubleBuffering( DMA_DBUFMODE_t dbufMode );
void DMA_SetPriority( DMA_PRIMODE_t priMode );
uint8_t DMA_CH_IsOngoing( volatile DMA_CH_t * channel );
uint8_t DMA_IsOngoing( void );
uint8_t DMA_CH_IsPending( volatile DMA_CH_t * channel );
uint8_t DMA_IsPending( void );
uint8_t DMA_ReturnStatus_non_blocking( volatile DMA_CH_t * channel );
uint8_t DMA_ReturnStatus_blocking( volatile DMA_CH_t * channel );
void DMA_EnableChannel( volatile DMA_CH_t * channel );
void DMA_DisableChannel( volatile DMA_CH_t * channel );
void DMA_ResetChannel( volatile DMA_CH_t * channel );
void DMA_SetIntLevel( volatile DMA_CH_t * channel,
                      DMA_CH_TRNINTLVL_t transferInt,
                      DMA_CH_ERRINTLVL_t errorInt );
void DMA_SetupBlock( volatile DMA_CH_t * channel,
                     const void * srcAddr,
                     DMA_CH_SRCRELOAD_t srcReload,
                     DMA_CH_SRCDIR_t srcDirection,
                     void * destAddr,
                     DMA_CH_DESTRELOAD_t destReload,
                     DMA_CH_DESTDIR_t destDirection,
                     uint16_t blockSize,
                     DMA_CH_BURSTLEN_t burstMode,
                     uint8_t repeatCount,
                     bool useRepeat );
void DMA_EnableSingleShot( volatile DMA_CH_t * channel );
void DMA_DisableSingleShot( volatile DMA_CH_t * channel );
void DMA_SetTriggerSource( volatile DMA_CH_t * channel, uint8_t trigger );
void DMA_StartTransfer( volatile DMA_CH_t * channel );


#endif /* ATMEL_LIBS_INCLUDE_DMA_DRIVER_H_ */
