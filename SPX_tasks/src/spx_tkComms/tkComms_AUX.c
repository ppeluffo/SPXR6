/*
 * tkComms_AUX.c
 *
 *  Created on: 20 ago. 2021
 *      Author: pablo
 */


#include "tkComms.h"
#include "ul_modbus.h"

//------------------------------------------------------------------------------------
void tkAuxRX(void * pvParameters)
{
	// Esta tarea lee y procesa las respuestas del GPRS. Lee c/caracter recibido y lo va
	// metiendo en un buffer circular propio del GPRS que permite luego su impresion,
	// analisis, etc.

( void ) pvParameters;
char c;
uint32_t ulNotifiedValue;

	// Espero la notificacion para arrancar
	while ( ((start_byte >> WDG_AUXRX) & 1 ) != 1 )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );


	xprintf_P( PSTR("starting tkAuxRX..\r\n"));

	aux_init();

	for( ;; )	{

		u_wdg_kick(WDG_AUXRX, 60);

		if ( systemVars.modbus_conf.slave_address != 0x00 ) {
			// Leo el UART de AUX1
			if ( frtos_read( fdAUX1, &c, 1 ) == 1 ) {
				aux_rxbuffer_put2(c);
			}

		} else {

			xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 10000 / portTICK_RATE_MS ) );

		}
	}
}
//------------------------------------------------------------------------------------
void aux_init(void)
{

	IO_config_AUX_PWR();
	IO_config_AUX_RTS();

	IO_clr_AUX_PWR();
	IO_clr_AUX_RTS();

	if ( systemVars.modbus_conf.slave_address != 0x00 ) {
		xprintf_P( PSTR("AUX1: power on..\r\n"));
		aux_prender();
	}

}
//------------------------------------------------------------------------------------
void aux_prender(void)
{
	// Prendo la fuente del aux.
	IO_set_AUX_PWR();
	vTaskDelay( 1000 / portTICK_RATE_MS );
}
//------------------------------------------------------------------------------------
void aux_apagar(void)
{
	IO_clr_AUX_PWR();
}
//------------------------------------------------------------------------------------
void aux_rts_on(void)
{
	IO_set_AUX_RTS();
}
//------------------------------------------------------------------------------------
void aux_rts_off(void)
{
	IO_clr_AUX_RTS();
}
//------------------------------------------------------------------------------------
void aux_rxbuffer_reset(void)
{
	// Vacia el buffer y lo inicializa

//	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
//		taskYIELD();

		memset( aux_rxbuffer.buffer, '\0', AUX_RXBUFFER_LEN);
		aux_rxbuffer.ptr = 0;

//	xSemaphoreGive( sem_RXBUFF );

}
//------------------------------------------------------------------------------------
void aux_txbuffer_reset(void)
{
	// Vacia el buffer de trasmision y lo inicializa
	// No necesita semaforo

	memset( aux_txbuffer.buffer, '\0', AUX_TXBUFFER_LEN);
	aux_txbuffer.ptr = 0;


}
//------------------------------------------------------------------------------------
bool aux_rxbuffer_full(void)
{
	return ( aux_rxbuffer.ptr == AUX_RXBUFFER_LEN );
}
//------------------------------------------------------------------------------------
bool aux_rxbuffer_empty(void)
{
	return (!aux_rxbuffer_full());
}
//------------------------------------------------------------------------------------
uint16_t aux_rxbuffer_usedspace(void)
{
uint16_t freespace;

	freespace = AUX_RXBUFFER_LEN - aux_rxbuffer.ptr;
	return (freespace);
}
//------------------------------------------------------------------------------------
void aux_rxbuffer_put( char data)
{
	// Avanza sobreescribiendo el ultimo si esta lleno

//	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
//		taskYIELD();

	aux_rxbuffer.buffer[ aux_rxbuffer.ptr ] = data;
	if ( aux_rxbuffer.ptr < AUX_RXBUFFER_LEN )
		aux_rxbuffer.ptr++;

//	xSemaphoreGive( sem_RXBUFF );
}
//------------------------------------------------------------------------------------
bool aux_rxbuffer_put2( char data )
{
	// Solo inserta si hay lugar

	 if(!aux_rxbuffer_full()) {
		 aux_rxbuffer_put(data);
		 return(true);
	 }

    return(false);
}
//------------------------------------------------------------------------------------
void aux_flush_RX_buffer(void)
{
	// El rx buffer de la uart es circular por lo tanto no los flusheo.
	frtos_ioctl( fdAUX1,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	aux_rxbuffer_reset();
}
//------------------------------------------------------------------------------------
void aux_flush_TX_buffer(void)
{
	frtos_ioctl( fdAUX1,ioctl_UART_CLEAR_TX_BUFFER, NULL);
}
//------------------------------------------------------------------------------------
bool aux_rxbuffer_copyto ( uint8_t *dst_buffer, uint8_t *size, int8_t max_size )
{
	/*
	 * Copia el rxbuffer del aux al dst_buffer.
	 * Pueden haber '\0' por lo que no uso strcpy.
	 * La cantidad de bytes copiados queda en *size
	 * No copio mas de max_size
	 */

int8_t i;

	for (i=0; i < aux_rxbuffer.ptr; i++ ) {
		dst_buffer[i] = aux_rxbuffer.buffer[i];
	}

	*size = aux_rxbuffer.ptr;

	return(true);

}
//------------------------------------------------------------------------------------
void aux_print_RX_buffer( bool ascii_mode )
{
	// Imprimo en hexadecimal el buffer.

char *p;

	// Imprimo todo el buffer local de RX. Sale por \0.
	xprintf_P( PSTR ("AUX: rxbuff>\r\n\0"));

	p = aux_rxbuffer.buffer;
	while(*p) {
		if ( ascii_mode ) {
			xprintf_P( PSTR("%c"),(*p));
		} else {
			xprintf_P( PSTR("[0x%02x]"),(*p));
		}
		*p++;
	}

	xprintf_P( PSTR ("\r\n[%d]\r\n\0"), aux_rxbuffer.ptr );

}
//------------------------------------------------------------------------------------
uint8_t aux_get_RX_buffer_ptr(void)
{
	return(aux_rxbuffer.ptr);
}
//------------------------------------------------------------------------------------

