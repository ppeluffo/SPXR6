/*
 * drv_dma_spx.h
 *
 *  Created on: 7 abr. 2022
 *      Author: pablo
 */

#ifndef SPX_DRIVERS_INCLUDE_DRV_DMA_SPX_H_
#define SPX_DRIVERS_INCLUDE_DRV_DMA_SPX_H_

#include "stdint.h"
#include <avr/io.h>
#include "string.h"
#include "drv_uart_spx.h"
#include <avr/interrupt.h>
#include "dma_driver.h"

#define DMA_TERM_Channel &DMA.CH0
#define DMA_AUX_Channel &DMA.CH1

void configure_dma_channel_for_TERM(void);
void configure_dma_channel_for_AUX(void);
char *get_dma_aux_buffer(void);
uint8_t get_dma_dst_address0(void);

#endif /* SPX_DRIVERS_INCLUDE_DRV_DMA_SPX_H_ */
