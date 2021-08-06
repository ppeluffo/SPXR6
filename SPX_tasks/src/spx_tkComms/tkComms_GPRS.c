/*
 * tkComms_GPRS.c
 *
 *  Created on: 5 mar. 2020
 *      Author: pablo
 *
 * MS: Mobile Station ( modem )
 * SGSN: Nodo de soporte ( registro / autentificacion )
 * GGSN: Nodo gateway(router). Interfase de la red celular a la red de datos IP
 *
 * ATTACH: Proceso por el cual el MS se conecta al SGSN en una red GPRS
 * PDP Activation: Proceso por el cual se establece una sesion entre el MS y la red destino.
 * Primero hay que attachearse y luego activarse !!!
 *
 * 1- Verificar el CPIN
 *    Nos indica que el dispositivo puede usarse
 *
 * 2- CREG?
 *    Nos indica que el dispositivo esta registrado en la red celular, que tiene senal.
 *    Si no se registra: 1-verificar la antena, 2-verificar la banda
 *
 * 3- Solicitar informacion a la red.
 *
 * 4- Ver la calidad de senal (CSQ)
 *
 * 5- Atachearse a la red GPRS
 *    Se usa el comando AT+CGATT
 *    Atachearse no significa que pueda establecer un enlace de datos ya que la sesión queda identificada
 *    por el PDP context(APN).
 *    El PDP establece la relacion entre el dipositivo y el GGSN por lo tanto debemos establecer un
 *    contexto PDP antes de enviar/recibir datos
 *
 * 6- Definir el contexto.
 *    Si uso un dial-up uso el comando AT+CGDCONT
 *    Si uso el stack TCP/IP uso el comando AT+CGSOCKCONT
 *    Indico cual es el contexto por defecto:
 *    Indico la autentificacion requerida
 *
 * 7- Debo activar el contexto y la red IP me va a devolver una direccion IP.
 *    Como estoy usando el stack TCP/IP, esto automaticamente ( activacion, abrir un socket local y pedir una IP )
 *    lo hace el comado NETOPEN.
 *
 * 8- Para abrir un socket remoto usamos TCPCONNECT
 */

#include "tkComms.h"

//------------------------------------------------------------------------------------
// MANEJO DEL BUFFER DE RECEPCION DE GPRS ( Circular o Lineal )
// Lo alimenta la rx interrupt del puerto gprs.
//------------------------------------------------------------------------------------
void gprs_rxbuffer_reset(void)
{
	// Vacia el buffer y lo inicializa

	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		taskYIELD();

		memset( gprs_rxbuffer.buffer, '\0', GPRS_RXBUFFER_LEN);
		gprs_rxbuffer.ptr = 0;

	xSemaphoreGive( sem_RXBUFF );

}
//------------------------------------------------------------------------------------
void gprs_txbuffer_reset(void)
{
	// Vacia el buffer de trasmision y lo inicializa
	// No necesita semaforo

	memset( gprs_txbuffer.buffer, '\0', GPRS_TXBUFFER_LEN);
	gprs_txbuffer.ptr = 0;


}
//------------------------------------------------------------------------------------
bool gprs_rxbuffer_full(void)
{
	return ( gprs_rxbuffer.ptr == GPRS_RXBUFFER_LEN );
}
//------------------------------------------------------------------------------------
bool gprs_rxbuffer_empty(void)
{
	return (!gprs_rxbuffer_full());
}
//------------------------------------------------------------------------------------
uint16_t gprs_rxbuffer_usedspace(void)
{
uint16_t freespace;

	freespace = GPRS_RXBUFFER_LEN - gprs_rxbuffer.ptr;
	return (freespace);
}
//------------------------------------------------------------------------------------
void gprs_rxbuffer_put( char data)
{
	// Avanza sobreescribiendo el ultimo si esta lleno

	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		taskYIELD();

		gprs_rxbuffer.buffer[ gprs_rxbuffer.ptr ] = data;
		if ( gprs_rxbuffer.ptr < GPRS_RXBUFFER_LEN )
			gprs_rxbuffer.ptr++;

	xSemaphoreGive( sem_RXBUFF );
}
//------------------------------------------------------------------------------------
bool gprs_rxbuffer_put2( char data )
{
	// Solo inserta si hay lugar

	 if(!gprs_rxbuffer_full()) {
		 gprs_rxbuffer_put(data);
		 return(true);
	 }

    return(false);
}
//------------------------------------------------------------------------------------
bool gprs_rxbuffer_get( char * data )
{
	// Retorna el ultimo y true.
	// Si esta vacio retorna false.
    return(false);
}
//------------------------------------------------------------------------------------
void gprs_flush_RX_buffer(void)
{
	// El rx buffer de la uart es circular por lo tanto no los flusheo.
	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	gprs_rxbuffer_reset();
}
//------------------------------------------------------------------------------------
void gprs_flush_TX_buffer(void)
{
	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_TX_BUFFER, NULL);
}
//------------------------------------------------------------------------------------
void gprs_print_RX_buffer( void )
{
	// NO USO SEMAFORO PARA IMPRIMIR !!!!!

	if ( ! DF_COMMS )
		return;

	xprintf_P( PSTR ("\r\nGPRS: rxbuff>\r\n\0"));

	// Imprimo todo el buffer local de RX. Sale por \0.
	// Uso esta funcion para imprimir un buffer largo, mayor al que utiliza xprintf_P. !!!
	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		taskYIELD();

		xnprint( gprs_rxbuffer.buffer, GPRS_RXBUFFER_LEN );
		xprintf_P( PSTR ("\r\n[%d]\r\n\0"), gprs_rxbuffer.ptr );

	xSemaphoreGive( sem_RXBUFF );
}
//------------------------------------------------------------------------------------
bool gprs_check_response( const uint16_t timeout, const char *rsp )
{
	// Espera una respuesta.
	// El timeout esta en intervalos de 100 milisegundos

bool retS = false;
int16_t local_timer = timeout;

	for(;;) {

		local_timer--;

		// Busco
		while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
			taskYIELD();
		if  ( strstr(gprs_rxbuffer.buffer, rsp) != NULL ) {
			retS = true;
		}
		xSemaphoreGive( sem_RXBUFF );

		// Respuesta correcta. Salgo enseguida
		if (retS) {
			return(retS);
		}

		// One time: salgo
		if (local_timer <= 0 ) {
			return(retS);
		}

		// Espero
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );


	}

	return(retS);
}
//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL
//------------------------------------------------------------------------------------
void gprs_init(void)
{
	// GPRS
	IO_config_GPRS_SW();
	IO_config_GPRS_PWR();
	IO_config_GPRS_RTS();
	IO_config_GPRS_CTS();
	IO_config_GPRS_DCD();
	IO_config_GPRS_RI();
	IO_config_GPRS_RX();
	IO_config_GPRS_TX();
	IO_config_GPRS_DTR();

	IO_set_GPRS_DTR();
	IO_set_GPRS_RTS();

	memset( xCOMMS_stateVars.gprs_ccid, '\0', IMEIBUFFSIZE );
	memset( xCOMMS_stateVars.gprs_ccid, '\0', IMEIBUFFSIZE );
}
//------------------------------------------------------------------------------------
void gprs_hw_pwr_on(uint8_t delay_factor)
{
	/*
	 * Prendo la fuente del modem y espero que se estabilize la fuente.
	 */

	xprintf_PD( DF_COMMS, PSTR("COMMS: gprs_hw_pwr_on.\r\n\0") );

	IO_clr_GPRS_SW();	// GPRS=0V, PWR_ON pullup 1.8V )
	IO_set_GPRS_PWR();	// Prendo la fuente ( alimento al modem ) HW

	vTaskDelay( (portTickType)( ( 2000 + 2000 * delay_factor) / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_apagar(void)
{
	/*
	 * Apaga el dispositivo quitando la energia del mismo
	 *
	 */

	xprintf_PD( DF_COMMS, PSTR("COMMS: gprs_apagar\r\n\0") );

	xprintf_PD( DF_COMMS, PSTR("COMMS: gprs CPOF\r\n"));
	FSM_sendATcmd(1, "AT+CPOF\r");
	vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );
	xprintf_PD( DF_COMMS, PSTR("COMMS: gprs CPOF OK.\r\n\0"));


	IO_clr_GPRS_SW();	// Es un FET que lo dejo cortado
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	IO_clr_GPRS_PWR();	// Apago la fuente.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_sw_pwr(void)
{
	/*
	 * Genera un pulso en la linea PWR_SW. Como tiene un FET la senal se invierte.
	 * En reposo debe la linea estar en 0 para que el fet flote y por un pull-up del modem
	 * la entrada PWR_SW esta en 1.
	 * El PWR_ON se pulsa a 0 saturando el fet.
	 */
	xprintf_PD( DF_COMMS, PSTR("COMMS: gprs_sw_pwr\r\n\0") );
	IO_set_GPRS_SW();	// GPRS_SW = 3V, PWR_ON = 0V.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	IO_clr_GPRS_SW();	// GPRS_SW = 0V, PWR_ON = pullup, 1.8V

}
//------------------------------------------------------------------------------------
int8_t FSM_sendATcmd( const uint8_t timeout, char *cmd )
{
	// Envia un comando:
	// Sale por: OK/ERROR/BufferFull/TIMEOUT

	// Las respuestas pueden ser nula ( no espero ) u otra.
	// El comando lo doy una sola vez !!!.
	// Si me da error u respuesta no esperada, no tiene sentido repetir el comando.
	// El timeout es en segundos pero chequeo c/100 ms.

int8_t exit_code = -1;
int16_t local_timer = (10 * timeout);

	//xprintf_P(PSTR("DEBUG: timeout=%d\r\n"), timeout);
	//xprintf_P(PSTR("DEBUG: cmd=%s\r\n"), cmd );
	gprs_flush_RX_buffer();

	// Doy el comando.
	//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: (%d) %s\r\n"), local_timer, cmd );
	gprs_flush_TX_buffer();
	// Espera antes de c/comando. ( ver recomendaciones de TELIT )
	vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );
	xfprintf_P( fdGPRS , PSTR("%s"), cmd);

	if ( timeout == 0 ) {
		// Salida inmediata:
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: OUT INMEDIATE.\r\n"));
		return(99);
	}

	// Espero respuesta
	while ( exit_code == -1 ) {

		// Espero de a 100 ms.
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		// Chequeo respuestas:
		while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
			taskYIELD();

			if  ( strstr(gprs_rxbuffer.buffer, "OK") != NULL ) {
				// Respuesta esperada: salgo
				exit_code = ATRSP_OK;

			} else if ( strstr(gprs_rxbuffer.buffer, "ERROR") != NULL ) {
				// Respuesta ERR: El comando respondio con error: salgo.
				exit_code = ATRSP_ERROR;

			} else if ( gprs_rxbuffer_full() ) {
				// Si el gprs buffer esta full es que recibio algo pero quedo trunco. ( CGDCONT )
				// Asumo que recibio una respuesta OK. !!!
				exit_code = ATRSP_OK;
			}

		xSemaphoreGive( sem_RXBUFF );

		// TIMEOUT
		local_timer -= 10;
		if (  local_timer <= 0 ) {
			exit_code = ATRSP_TIMEOUT;
		}
	}

	// Print debug causes
	switch( exit_code) {
	case ATRSP_OK:
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ATRSP_OK\r\n"));
		break;
	case ATRSP_ERROR:
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ERROR\r\n"));
		break;
	case ATRSP_TIMEOUT:
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: TIMEOUT\r\n"));
		break;
	default:
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ERROR RSP NO ESPERADA !!\r\n"));
		break;
	}

	gprs_print_RX_buffer();
	return (exit_code);

}
//------------------------------------------------------------------------------------
