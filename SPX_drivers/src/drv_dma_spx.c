/*
 * drv_dma_spx.c
 *
 *  Created on: 7 abr. 2022
 *      Author: pablo
 */


#include "drv_dma_spx.h"

#define DMA_BUFFER_TERM_SIZE 8
static char dma_buffer_TERM[DMA_BUFFER_TERM_SIZE];

#define DMA_BUFFER_AUX_SIZE 8
static char dma_buffer_AUX[DMA_BUFFER_AUX_SIZE];

uint8_t dma_dst_address0;


//----------------------------------------------------------------------------------------
/*
 * Configuro un canal del DMA para que transfiera los datos recibidos
 * por la UART que atiende a TERM hacia un buffer de 8 bytes.
 * Luego en la interrupcion de DMA de transaction end, copio los bytes
 * hacia el buffer circular uart_term.RXringBuffer.
 * En un sistema en que tenemos multiples fuentes de interrupcion, puede
 * ocurrir si todas tienen la misma prioridad que el tiempo de atencion
 * de la puerta serial no sea suficiente y halla un override de los datos
 * recibidos ( cuando la velocidad es alta )
 * Con esta arquitectura no nos preocupamos porque los datos serian guardados
 * en un pequeno buffer que luego cuando se genere la interrupcion del DMA los
 * podemos vaciar.
 *
 */
void configure_dma_channel_for_TERM(void)
{
	/*
	 * Procedimientos sacados de la AVR1304.
	 */

	/*
	memset(dma_buffer_TERM, '\0', DMA_BUFFER_TERM_SIZE );

	// Reseteo ( para inicializar todos los registros). Debe estar deshabilitado
	DMA.CTRL &= ~DMA_ENABLE_bm;
	DMA.CTRL |= DMA_RESET_bm;

	// Habilitamos el controlador de DMA
	DMA.CTRL |= DMA_ENABLE_bm;

	// Configuro el canal0 para SINGLE-SHOT, BURSTLEN=00 ( 1 byte )
	DMA.CH0.CTRLA |= DMA_CH_SINGLE_bm;

	// EL trigger source es USARTF0.RXE
	DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_USARTC0_RXC_gc;

	// ADDRCONTROL
	// La source address no importa. Tampoco si hago o no reload.
	// DSTRELOAD = NONE
	// DESTDIR = INC
	DMA.CH0.ADDRCTRL = DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_INC_gc;

	// DSTADDRESS es la del buffer. Lo debo incrementar en c/transaccion
	DMA.CH0.DESTADDR0 = &dma_buffer_TERM;

	// Habilito el canal
	DMA.CH0.CTRLA |= DMA_ENABLE_bm;
	*/

    DMA_SetupBlock(
                    DMA_TERM_Channel,
                    (void *) &USARTF0.DATA,
                    DMA_CH_SRCRELOAD_NONE_gc,
                    DMA_CH_SRCDIR_FIXED_gc,
					dma_buffer_TERM,
                    DMA_CH_DESTRELOAD_NONE_gc,
                    DMA_CH_DESTDIR_INC_gc,
					DMA_BUFFER_TERM_SIZE,
                    DMA_CH_BURSTLEN_1BYTE_gc,
                    0, // Perform once
                    false
                   );

    DMA_EnableSingleShot( DMA_TERM_Channel );

    // USART Trigger source, Receive complete
    DMA_SetTriggerSource( DMA_TERM_Channel, DMA_CH_TRIGSRC_USARTD0_RXC_gc);
}
//----------------------------------------------------------------------------------------
/*
ISR( DMA_CH0_vect )
{

	 // Se genera al terminar la transaccion.
	 //  Pueden haberse transferido uno o mas bytes al buffer.
	 //  Los copio al buffer circular de RX de TERM.


uint8_t i;
char c;

	// Solo si interrumpi por transaccion end
	if ( DMA.INTFLAGS & DMA_CH0TRNIF_bm ) {
		//
		for (i=0; i<DMA_BUFFER_TERM_SIZE; i++ ) {
			c =  dma_buffer_TERM[i];
			if ( c != '\0') {
				rBufferPokeFromISR( &uart_term.RXringBuffer, &c );
			}
		}

		// Borro la interrupcion
		DMA.INTFLAGS &= ~DMA_CH0TRNIF_bm;

	}
	// Prepara para la proxima transaccion.
	DMA.CH0.DESTADDR0 = (uint8_t)dma_buffer_TERM;


}
*/
//----------------------------------------------------------------------------------------
void configure_dma_channel_for_AUX(void)
{

    DMA_SetupBlock(
                    DMA_AUX_Channel,				// The channel to configure.
                    (void *) &USARTC0.DATA,			// Source memory address.
                    DMA_CH_SRCRELOAD_NONE_gc,		// Source address reload mode.
                    DMA_CH_SRCDIR_FIXED_gc,			// Source address direction (fixed, increment, or decrement).
					dma_buffer_AUX,					// Destination memory address.
                    DMA_CH_DESTRELOAD_NONE_gc,		// Destination address reload mode.
                    DMA_CH_DESTDIR_INC_gc,			// Destination address direction (fixed, increment, or decrement).
					1,								// Block size in number of bytes (0 = 64k).
                    DMA_CH_BURSTLEN_1BYTE_gc,		// Number of bytes per data transfer (1, 2, 4, or 8 bytes).
                    0x00,							// Number of blocks, 0x00 if you want to repeat at infinitum.
                    true							// True if reapeat should be used, false if not.
                   );

    DMA_EnableSingleShot( DMA_AUX_Channel );

    // USART Trigger source, Receive complete
    DMA_SetTriggerSource( DMA_AUX_Channel, DMA_CH_TRIGSRC_USARTC0_RXC_gc);

    DMA_SetIntLevel( DMA_AUX_Channel, DMA_CH_TRNINTLVL_LO_gc, DMA_CH_ERRINTLVL_LO_gc );
}
//----------------------------------------------------------------------------------------
char *get_dma_aux_buffer(void)
{
	return(dma_buffer_AUX);
}
//----------------------------------------------------------------------------------------
/*
ISR( DMA_CH1_vect )
{

	 // Se genera al terminar la transaccion.
	 //  Pueden haberse transferido uno o mas bytes al buffer.
	 //  Los copio al buffer circular de RX de TERM.

char cChar;

	cChar = dma_buffer_AUX[0];
	if ( (cChar != '\r') && ( cChar != '\n')) {
		rBufferPokeFromISR( &uart_aux1.RXringBuffer, &cChar );
	}
	DMA.CH1.CTRLB |= DMA_CH_TRNIF_bm;

}
*/
//----------------------------------------------------------------------------------------
uint8_t get_dma_dst_address0(void)
{
	return(dma_dst_address0);
}
//----------------------------------------------------------------------------------------

