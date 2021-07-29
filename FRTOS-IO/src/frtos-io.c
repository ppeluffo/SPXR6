/*
 * frtos-io.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "frtos-io.h"

//#define GPRS_IsTXDataRegisterEmpty() ((USARTE0.STATUS & USART_DREIF_bm) != 0)

#define USART_IsTXDataRegisterEmpty(_usart) (((_usart)->STATUS & USART_DREIF_bm) != 0)
#define USART_IsTXShiftRegisterEmpty(_usart) (((_usart)->STATUS & USART_TXCIF_bm) != 0)

#define USART_PutChar(_usart, _data) ((_usart)->DATA = _data)

#define GPRS_USART	USARTE0
//------------------------------------------------------------------------------------
int16_t frtos_open( file_descriptor_t fd, uint32_t flags)
{
	// Funcion general para abrir el puerto que invoca a una mas
	// especializada para c/periferico.
	// Es la que invoca la aplicacion.
	// Retorna -1 en error o un nro positivo ( fd )

int16_t xRet = -1;

	switch(fd) {
	case fdGPRS:
		xRet = frtos_uart_open( &xComGPRS, fd, iUART_GPRS, flags );
		break;
	case fdAUX1:
		xRet = frtos_uart_open( &xComAUX1, fd, iUART_AUX1, flags );
		break;
	case fdTERM:
		xRet = frtos_uart_open( &xComTERM, fd, iUART_TERM, flags );
		break;
	case fdI2C:
		xRet = frtos_i2c_open( &xBusI2C, fd, &I2C_xMutexBuffer, flags );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
int16_t frtos_ioctl( file_descriptor_t fd, uint32_t ulRequest, void *pvValue )
{

int16_t xRet = -1;

	switch(fd) {
	case fdGPRS:
		xRet = frtos_uart_ioctl( &xComGPRS, ulRequest, pvValue );
		break;
	case fdAUX1:
		xRet = frtos_uart_ioctl( &xComAUX1, ulRequest, pvValue );
		break;
	case fdTERM:
		xRet = frtos_uart_ioctl( &xComTERM, ulRequest, pvValue );
		break;
	case fdI2C:
		xRet = frtos_i2c_ioctl( &xBusI2C, ulRequest, pvValue );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
int16_t frtos_write( file_descriptor_t fd ,const char *pvBuffer, const uint16_t xBytes )
{

int16_t xRet = -1;

	switch(fd) {
	case fdGPRS:
		//xRet = frtos_uart_write( &xComGPRS, pvBuffer, xBytes );
		xRet = frtos_uart_write_poll( &xComGPRS, pvBuffer, xBytes );
		break;
	case fdAUX1:
		xRet = frtos_uart_write( &xComAUX1, pvBuffer, xBytes );
		break;
	case fdTERM:
		xRet = frtos_uart_write_poll( &xComTERM, pvBuffer, xBytes );
		break;
	case fdI2C:
		xRet = frtos_i2c_write( &xBusI2C, pvBuffer, xBytes );
		break;
	case fdFILE:
//		xRet = frtos_file_write( pvBuffer, xBytes );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
int16_t frtos_send( file_descriptor_t fd ,const char *pvBuffer, const uint16_t xBytes )
{
	// Trasmite el buffer sin considerar si tiene NULL 0x00 en el medio.
	// Transmite en forma transparente los xBytes.

int16_t xRet = -1;

	switch(fd) {
	case fdGPRS:
		break;
	case fdAUX1:
		xRet = frtos_uart_send( &xComAUX1, pvBuffer, xBytes );
		break;
	case fdTERM:
	case fdI2C:
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
void frtos_putchar( file_descriptor_t fd ,const char cChar )
{
	// Escribe en el puerto (serial) en modo transparente.

	switch(fd) {
	case fdGPRS:
		// Espero que este libre para transmitir

		// Transmito.
		USARTE0.DATA = cChar;
		while ( ( USARTE0.STATUS & USART_DREIF_bm) == 0)
			vTaskDelay( ( TickType_t)( 1 ) );
		break;
	case fdAUX1:
		while ( ( USARTC0.STATUS & USART_DREIF_bm) == 0)
			vTaskDelay( ( TickType_t)( 1 ) );
		USARTC0.DATA = cChar;
		break;
	case fdTERM:
		while ( ( USARTF0.STATUS & USART_DREIF_bm) == 0)
			vTaskDelay( ( TickType_t)( 1 ) );
		USARTF0.DATA = cChar;

		break;
	case fdI2C:
		break;
	default:
		break;
	}

}
//------------------------------------------------------------------------------------
int16_t frtos_read( file_descriptor_t fd , char *pvBuffer, uint16_t xBytes )
{

int16_t xRet = -1;

	switch(fd) {
	case fdGPRS:
		xRet = frtos_uart_read( &xComGPRS, pvBuffer, xBytes );
		break;
	case fdAUX1:
		xRet = frtos_uart_read( &xComAUX1, pvBuffer, xBytes );
		break;
	case fdTERM:
		xRet = frtos_uart_read( &xComTERM, pvBuffer, xBytes );
		break;
	case fdI2C:
		xRet = frtos_i2c_read( &xBusI2C, pvBuffer, xBytes );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
// FUNCIONES ESPECIFICAS DE UART's
//------------------------------------------------------------------------------------
int16_t frtos_uart_open( periferico_serial_port_t *xCom, file_descriptor_t fd, uart_id_t uart_id, uint32_t flags)
{

	xCom->fd = fd;
	xCom->xBlockTime = (10 / portTICK_RATE_MS );
	// Inicializo la uart del usb (iUART_USB) y la asocio al periferico
	xCom->uart = drv_uart_init( uart_id, flags );

	return(xCom->fd);

}
//------------------------------------------------------------------------------------
int16_t frtos_uart_write( periferico_serial_port_t *xCom, const char *pvBuffer, const uint16_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision del
	// puerto serial apuntado por xCom
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar = '\0';
char *p = NULL;
uint16_t bytes2tx = 0;
int16_t wBytes = 0;

	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;

	// Trasmito.
	// Espero que los buffers esten vacios. ( La uart se va limpiando al trasmitir )
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;
	while (*p && (bytes2tx-- > 0) ) {

		// Voy cargando la cola de a uno.
		cChar = *p;
		rBufferPoke( &xCom->uart->TXringBuffer, &cChar  );
		//rBufferPoke( &uart_usb.TXringBuffer, &cChar  );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo

		// Si la cola esta llena, empiezo a trasmitir y espero que se vacie.
		if (  rBufferReachHighWaterMark( &xCom->uart->TXringBuffer ) ) {
			// Habilito a trasmitir para que se vacie
			drv_uart_interruptOn( xCom->uart->uart_id );
			// Y espero que se haga mas lugar.
			while ( ! rBufferReachLowWaterMark( &xCom->uart->TXringBuffer ) )
				vTaskDelay( ( TickType_t)( 1 ) );
		}
	}

	// Luego inicio la trasmision invocando la interrupcion.
	drv_uart_interruptOn( xCom->uart->uart_id );

	// Espero que trasmita todo
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	return (wBytes);
}
//------------------------------------------------------------------------------------
int16_t frtos_uart_send( periferico_serial_port_t *xCom, const char *pvBuffer, const uint16_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision del
	// puerto serial apuntado por xCom
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.
	// NO CONSIDERA SI EL CARACTER ES NULL 0x00. Tansmite xBytes.

char cChar = '\0';
char *p = NULL;
uint16_t bytes2tx = 0;
int16_t wBytes = 0;

	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;

	// Trasmito.
	// Espero que los buffers esten vacios. ( La uart se va limpiando al trasmitir )
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;
	while ( bytes2tx-- > 0 ) {

		// Voy cargando la cola de a uno.
		cChar = *p;
		rBufferPoke( &xCom->uart->TXringBuffer, &cChar  );
		//rBufferPoke( &uart_usb.TXringBuffer, &cChar  );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo

		// Si la cola esta llena, empiezo a trasmitir y espero que se vacie.
		if (  rBufferReachHighWaterMark( &xCom->uart->TXringBuffer ) ) {
			// Habilito a trasmitir para que se vacie
			drv_uart_interruptOn( xCom->uart->uart_id );
			// Y espero que se haga mas lugar.
			while ( ! rBufferReachLowWaterMark( &xCom->uart->TXringBuffer ) )
				vTaskDelay( ( TickType_t)( 1 ) );
		}
	}

	// Luego inicio la trasmision invocando la interrupcion.
	drv_uart_interruptOn( xCom->uart->uart_id );

	// Espero que trasmita todo
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	return (wBytes);
}
//------------------------------------------------------------------------------------
int16_t frtos_uart_write_poll( periferico_serial_port_t *xCom, const char *pvBuffer, const uint16_t xBytes )
{
	// Transmite los datos del pvBuffer de a uno. No usa interrupcion de TX.
	//

char cChar = '\0';
char *p = NULL;
uint16_t bytes2tx = 0;
int16_t wBytes = 0;
int timeout;

	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;

	// Trasmito.
	// Espero que los buffers esten vacios. ( La uart se va limpiando al trasmitir )
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;

	while ( bytes2tx-- > 0 ) {
		// Voy cargando la cola de a uno.
		cChar = *p;
		timeout = 10;	// Espero 10 ticks maximo
		while( --timeout > 0) {

			if ( USART_IsTXDataRegisterEmpty(xCom->uart->usart) ) {
				USART_PutChar(xCom->uart->usart, cChar);
				p++;
				wBytes++;	// Cuento los bytes que voy trasmitiendo
				break;
			} else {
				// Espero
				vTaskDelay( ( TickType_t)( 1 ) );
			}

			if ( timeout == 0 ) {
				// Error de transmision: SALGO
				return(-1);
			}

		}
	}

	return (wBytes);
}
//------------------------------------------------------------------------------------
int16_t frtos_uart_ioctl( periferico_serial_port_t *xCom, uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;

	switch( ulRequest )
	{

		case ioctl_SET_TIMEOUT:
			xCom->xBlockTime = *((uint8_t *)pvValue);
			break;
		case ioctl_UART_CLEAR_RX_BUFFER:
			rBufferFlush(&xCom->uart->RXringBuffer);
			break;
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBufferFlush(&xCom->uart->TXringBuffer);
			break;
		case ioctl_UART_ENABLE_TX_INT:
			drv_uart_enable_tx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_TX_INT:
			drv_uart_disable_tx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_ENABLE_RX_INT:
			drv_uart_enable_rx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_RX_INT:
			drv_uart_disable_rx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_ENABLE_TX:
			drv_uart_enable_tx( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_TX:
			drv_uart_disable_tx( xCom->uart->uart_id );
			break;
		case ioctl_UART_ENABLE_RX:
			drv_uart_enable_rx( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_RX:
			drv_uart_disable_rx( xCom->uart->uart_id );
			break;
		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------------
int16_t frtos_uart_read( periferico_serial_port_t *xCom, char *pvBuffer, uint16_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int16_t xBytesReceived = 0U;
portTickType xTicksToWait = 0;
xTimeOutType xTimeOut;

	xTicksToWait = 10;
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{

		if( rBufferPop( &xCom->uart->RXringBuffer, &((char *)pvBuffer)[ xBytesReceived ] ) == true ) {
			xBytesReceived++;
			taskYIELD();
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( xTicksToWait ) );
		}

		// Time out has expired ?
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
		{
			break;
		}
	}

	return ( xBytesReceived );

}
//------------------------------------------------------------------------------------
int16_t frtos_write_modbus( const char *pvBuffer, uint16_t xBytes )
{
	// Hago control de flujo ya que el SP3485 debe operarse HALF-DUPLEX !!
	// Trasmite el buffer sin considerar si tiene NULL 0x00 en el medio.
	// Transmite en forma transparente los xBytes por poleo de modo que controlo exactamente
	// cuando termino de transmitir c/byte.
	// Solo opera sobre xComAUX1
	//

char cChar = '\0';
char *p = NULL;
uint16_t bytes2tx = 0;
int16_t wBytes = 0;
int timeout;

	// Deshabilito la recepcion
	//frtos_ioctl (fdAUX1, ioctl_UART_DISABLE_RX, NULL );

	// RTS ON. Habilita el sentido de trasmision del chip.
	IO_set_AUX_RTS();

	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;

	// Trasmito.
	// Espero que los buffers esten vacios. ( La uart se va limpiando al trasmitir )
	while  ( rBufferGetCount(  &xComAUX1.uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	p = (char *)pvBuffer;

	while ( bytes2tx-- > 0 ) {
		// Voy cargando la cola de a uno.
		cChar = *p;
		timeout = 10;	// Espero 10 ticks maximo
		while( --timeout > 0) {

			// Los datos se escriben solo cuando el DREIF es 1.
			if ( USART_IsTXDataRegisterEmpty(xComAUX1.uart->usart) ) {
				USART_PutChar(xComAUX1.uart->usart, cChar);
				p++;
				wBytes++;	// Cuento los bytes que voy trasmitiendo
				break;
			} else {
				// Espero
				vTaskDelay( ( TickType_t)( 1 ) );
			}

			if ( timeout == 0 ) {
				// Error de transmision: SALGO
				return(-1);
			}
		}
	}

	// Espero que se halla vaciado la UART. STATUS.TXCIF = 1
	// El ultimo byte debe haberse shift out para habilitar el RTS
	while ( ! USART_IsTXShiftRegisterEmpty(xComAUX1.uart->usart) )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Debo habilitar la recepcion con el RTS
	// Primero por las dudas borro el buffer de recepcion
	frtos_uart_ioctl( &xComAUX1,ioctl_UART_CLEAR_RX_BUFFER, NULL);

	// Este delay es importante en modbus porque permite el turn-round. !!!
	vTaskDelay( ( TickType_t)( 3 ) );

	// Habilito Recepcion ( Half Duplex )
	//frtos_ioctl (fdAUX1, ioctl_UART_ENABLE_RX, NULL );

	// RTS OFF: Habilita la recepcion del chip
	IO_clr_AUX_RTS();

	return (wBytes);
}
//------------------------------------------------------------------------------------
// FUNCIONES ESPECIFICAS DEL BUS I2C/TWI
//------------------------------------------------------------------------------------
int16_t frtos_i2c_open( periferico_i2c_port_t *xI2c, file_descriptor_t fd, StaticSemaphore_t *i2c_semph, uint32_t flags)
{
	// Asigno las funciones particulares ed write,read,ioctl
	xI2c->fd = fd;
	xI2c->xBusSemaphore = xSemaphoreCreateMutexStatic( i2c_semph );
	xI2c->xBlockTime = (10 / portTICK_RATE_MS );
	xI2c->i2c_error_code = I2C_OK;
	//
	// Abro e inicializo el puerto I2C solo la primera vez que soy invocado
	drv_I2C_init();

	return(1);
}
//------------------------------------------------------------------------------------
int16_t frtos_i2c_write( periferico_i2c_port_t *xI2c, const char *pvBuffer, const uint16_t xBytes )
{

int16_t xReturn = 0U;

	if ( ( xReturn = drv_I2C_master_write (xI2c->devAddress, xI2c->dataAddress, xI2c->dataAddressLength,  (char *)pvBuffer, xBytes) ) >= 0 ) {
		xI2c->i2c_error_code = I2C_OK;
	} else {
		// Error de escritura indicado por el driver.
		xI2c->i2c_error_code = I2C_WR_ERROR;
	}

	return(xReturn);

}
//------------------------------------------------------------------------------------
int16_t frtos_i2c_ioctl( periferico_i2c_port_t *xI2c, uint32_t ulRequest, void *pvValue )
{

int16_t xReturn = 0;
uint32_t *p = NULL;

	p = pvValue;

	switch( ulRequest )
	{
		case ioctl_OBTAIN_BUS_SEMPH:
			// Espero el semaforo en forma persistente.
			while ( xSemaphoreTake(xI2c->xBusSemaphore, ( TickType_t ) 5 ) != pdTRUE )
				taskYIELD();
			break;
			case ioctl_RELEASE_BUS_SEMPH:
				xSemaphoreGive( xI2c->xBusSemaphore );
				break;
			case ioctl_SET_TIMEOUT:
				xI2c->xBlockTime = *p;
				break;
			case ioctl_I2C_SET_DEVADDRESS:
				xI2c->devAddress = (int8_t)(*p);
				break;
			case ioctl_I2C_SET_DATAADDRESS:
				xI2c->dataAddress = (uint16_t)(*p);
				break;
			case ioctl_I2C_SET_DATAADDRESSLENGTH:
				xI2c->dataAddressLength = (int8_t)(*p);
				break;
			case ioctl_I2C_GET_LAST_ERROR:
				xReturn = xI2c->i2c_error_code;
				break;
			case ioctl_I2C_SCAN:
				//xReturn = drv_I2C_scan_device(xI2c->devAddress );
				break;
			default :
				xReturn = -1;
				break;
		}

	return xReturn;

}
//------------------------------------------------------------------------------------
int16_t frtos_i2c_read( periferico_i2c_port_t *xI2c, const char *pvBuffer, const uint16_t xBytes )
{

int16_t xReturn = 0U;

	if ( ( xReturn = drv_I2C_master_read (xI2c->devAddress, xI2c->dataAddress, xI2c->dataAddressLength,  (char *)pvBuffer, xBytes) ) >= 0 ) {
		xI2c->i2c_error_code = I2C_OK;
	} else {
		// Error de escritura indicado por el driver.
		xI2c->i2c_error_code = I2C_RD_ERROR;
	}

	return(xReturn);
}
//------------------------------------------------------------------------------------
