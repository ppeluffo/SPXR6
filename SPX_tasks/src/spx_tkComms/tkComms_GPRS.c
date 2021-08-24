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

const char at_string_0[] PROGMEM = "rspNONE";
const char at_string_1[] PROGMEM = "rspOK";
const char at_string_2[] PROGMEM = "rspERROR";
const char at_string_3[] PROGMEM = "rspTO";
const char at_string_4[] PROGMEM = "rspOUTNOW";
const char at_string_5[] PROGMEM = "rspUNKNOWN";

const char * const AT_RESPONSES[] PROGMEM = { at_string_0, at_string_1, at_string_2, at_string_3, at_string_4, at_string_5 };

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
void gprs_set_MODO( uint8_t modo)
{

char cmd_str[32];

	// Setea con el comando CNMP los modos 2G,3G en que trabaja
	switch(modo) {
	case 2:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set modo AUTO\r\n"));
		break;
	case 13:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set modo 2G(GSM) only\r\n"));
		break;
	case 14:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set modo 3G(WCDMA) only\r\n"));
		break;
	default:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set modo ERROR !!.\r\n"));
		return;
	}

	memset(cmd_str,'\0', sizeof(cmd_str));
	snprintf_P( cmd_str, sizeof(cmd_str), PSTR("AT+CNMP=%d\r\0"),modo);
	FSM_sendATcmd( 2, cmd_str );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_set_PREF(uint8_t modo)
{
	// Setea el orden de preferencia para acceder a la red 2G o 3G

char cmd_str[32];

	switch(modo) {
	case 0:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set preference AUTO\r\n"));
		break;
	case 1:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set preferece 2G,3G\r\n"));
		break;
	case 2:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set preference 3G,2G\r\n"));
		break;
	default:
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs set preference ERROR !!.\r\n"));
		return;
	}

	memset(cmd_str,'\0', sizeof(cmd_str));
	snprintf_P( cmd_str, sizeof(cmd_str), PSTR("AT+CNAOP=%d\r\0"),modo);
	FSM_sendATcmd( 2, cmd_str );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_set_BANDS( char *s_bands)
{
	// Configura las bandas en que puede trabajar el modem.
	// Si el argumento es nulo, configura la banda por defecto.
	// ANTEL opera GSM(2G) en 900/1800 y 3G(UTMS/WCDMA) en 850/2100
	// 7	GSM_DCS_1800
	// 8	GSM_EGSM_900
	// 9	GSM_PGSM_900
	// 19	GSM_850
	// 26	WCDMA_850

char str_bands[20];

	// SOLO HABILITO LAS BANDAS DE ANTEL !!!!
	memset(str_bands,'\0', sizeof(str_bands));
	snprintf_P( str_bands, sizeof(str_bands), PSTR("AT+CNBP=0x0000000004080380\r"));
	FSM_sendATcmd( 5, str_bands );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_set_SAT(uint8_t modo)
{
/*
 * Seguimos viendo que luego de algún CPIN se cuelga el modem y ya aunque lo apague, luego al encenderlo
 * no responde al PIN.
 * En https://www.libelium.com/forum/viewtopic.php?t=21623 reportan algo parecido.
 * https://en.wikipedia.org/wiki/SIM_Application_Toolkit
 * Parece que el problema es que al enviar algun comando al SIM, este interactua con el STK (algun menu ) y lo bloquea.
 * Hasta no conocer bien como se hace lo dejamos sin usar.
 * " la tarjeta SIM es un ordenador diminuto con sistema operativo y programa propios.
 *   STK responde a comandos externos, por ejemplo, al presionar un botón del menú del operador,
 *   y hace que el teléfono ejecute ciertas acciones
 * "
 * https://www.techopedia.com/definition/30501/sim-toolkit-stk
 *
 * El mensaje +STIN: 25 es un mensaje no solicitado que emite el PIN STK.
 *
 * Esta rutina lo que hace es interrogar al SIM para ver si tiene la funcion SAT habilitada
 * y dar el comando de deshabilitarla
 *
 */

	switch(modo) {
	case 0:
		// Disable
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs SAT.(modo 0). Disable\r\n\0"));
		FSM_sendATcmd( 5, "AT+STK=0\r" );
		break;
	case 1:
		// Enable
		xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs SAT.(modo 1). Enable\r\n\0"));
		FSM_sendATcmd( 5, "AT+STK=1\r" );
		break;
	case 2:
		// Check. Query STK status ?
		xprintf_P(PSTR("GPRS: query STK status ?\r\n\0"));
		FSM_sendATcmd( 5, "AT+STK?\r" );
		break;
	default:
		return;
	}

	return;

}
//------------------------------------------------------------------------------------
void gprs_read_MODO(void)
{

int8_t cmd_rsp;

	xCOMMS_stateVars.gprs_mode = 0;
	cmd_rsp = FSM_sendATcmd( 5, "AT+CNMP?\r" );

	if (cmd_rsp	== ATRSP_OK ) {

		if ( gprs_check_response( 0, "CNMP: 2") ) {
			xCOMMS_stateVars.gprs_mode = 2;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs modo AUTO\r\n"));
			return;
		}

		if ( gprs_check_response( 0, "CNMP: 13") ) {
			xCOMMS_stateVars.gprs_mode = 13;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs modo 2G(GSM) only\r\n"));
			return;
		}

		if ( gprs_check_response( 0, "CNMP: 14") ) {
			xCOMMS_stateVars.gprs_mode = 14;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs modo 3G(WCDMA) only\r\n"));
			return;
		}
	}

	xprintf_PD( DF_COMMS,  PSTR("CMD CNMP Error.\r\n"));
}
//------------------------------------------------------------------------------------
void gprs_read_PREF(void)
{

int8_t cmd_rsp;

	xCOMMS_stateVars.gprs_pref = 0;
	cmd_rsp = FSM_sendATcmd( 5, "AT+CNAOP?\r" );

	if (cmd_rsp	== ATRSP_OK ) {

		if ( gprs_check_response( 0, "CNAOP: 0") ) {
			xCOMMS_stateVars.gprs_pref = 2;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs pref. AUTO\r\n"));
			return;
		}
		if ( gprs_check_response( 0, "CNAOP: 1") ) {
			xCOMMS_stateVars.gprs_pref = 1;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs pref. 2G,3G\r\n"));
			return;
		}
		if ( gprs_check_response( 0, "CNAOP: 2") ) {
			xCOMMS_stateVars.gprs_pref = 2;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs pref. 3G,2G\r\n"));
			return;
		}
	}

	xprintf_PD( DF_COMMS,  PSTR("CMD CNAOP Error.\r\n"));
}
//------------------------------------------------------------------------------------
void gprs_read_BANDS(void)
{
	// ANTEL opera GSM(2G) en 900/1800 y 3G(UTMS/WCDMA) en 850/2100
	// Al leer las bandas tenemos un string con 16 bytes y 64 bits.
	// C/bit en 1 indica una banda predida.
	// Las bandas que me interesan estan los los primeros 4 bytes por lo tanto uso
	// 32 bits. !!!

int8_t cmd_rsp;
char *ts = NULL;
char c = '\0';
char *ptr = NULL;
uint8_t i;

//
union {
	uint8_t u8[4];
	uint32_t u32;
} bands;

	xCOMMS_stateVars.gprs_mode = 0;
	memset( xCOMMS_stateVars.gprs_bands, '\0', sizeof(xCOMMS_stateVars.gprs_bands) );

	cmd_rsp = FSM_sendATcmd( 5, "AT+CNBP?\r" );
	if (cmd_rsp	== ATRSP_OK ) {

		if ( gprs_check_response( 0, "+CNBP: ") ) {
			ptr = xCOMMS_stateVars.gprs_bands;
			ts = strstr( gprs_rxbuffer.buffer, "+CNBP: ");
			ts++;
			while ( (c = *ts) != '\r') {
				*ptr++ = c;
				ts++;
			}
			*ptr = '\0';
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs bands=[%s]\r\n\0"), xCOMMS_stateVars.gprs_bands );

			// DECODIFICACION:
			bands.u32 = strtoul( &xCOMMS_stateVars.gprs_bands[12], &ptr, 16);
			i = 0;

			// 7 GSM_DCS_1800
			if ( bands.u8[0] & 0x80 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_DCS_1800.\r\n\0"),i++);
			}

			// 8 GSM_EGSM_900
			// 9 GSM_PGSM_900
			if ( bands.u8[1] & 0x01 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_EGSM_900.\r\n\0"),i++);
			}
			if ( bands.u8[1] & 0x02 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_PGSM_900.\r\n\0"),i++);
			}

			// 16 GSM_450
			// 17 GSM_480
			// 18 GSM_750
			// 19 GSM_850
			// 20 GSM_RGSM_900
			// 21 GSM_PCS_1900
			// 22 WCDMA_IMT_2000
			// 23 WCDMA_PCS_1900
			if ( bands.u8[2] & 0x01 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_450.\r\n\0"),i++);
			}
			if ( bands.u8[2] & 0x02 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_480.\r\n\0"),i++);
			}
			if ( bands.u8[2] & 0x04 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_750.\r\n\0"),i++);
			}
			if ( bands.u8[2] & 0x08 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_850.\r\n\0"),i++);
			}
			if ( bands.u8[2] & 0x10 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_RGSM_900.\r\n\0"),i++);
			}
			if ( bands.u8[2] & 0x20 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d GSM_PCS_1900.\r\n\0"),i++);
			}
			if ( bands.u8[2] & 0x40 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d WCDMA_IMT_2000.\r\n\0"),i++);
			}
			if ( bands.u8[2] & 0x80 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d WCDMA_PCS_1900.\r\n\0"),i++);
			}

			// 24 WCDMA_III_1700
			// 25 WCDMA_IV_1700
			// 26 WCDMA_850
			// 27 WCDMA_800
			if ( bands.u8[3] & 0x01 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d WCDMA_III_1700.\r\n\0"),i++);
			}
			if ( bands.u8[3] & 0x02 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d WCDMA_IV_1700.\r\n\0"),i++);
			}
			if ( bands.u8[3] & 0x04 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d WCDMA_850.\r\n\0"),i++);
			}
			if ( bands.u8[3] & 0x08 ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: band_%d WCDMA_800.\r\n\0"),i++);
			}
		}
	}
}
//------------------------------------------------------------------------------------
void AT_RSP2NAME(int8_t at_rsp_code )
{

}
//------------------------------------------------------------------------------------
int8_t FSM_sendATcmd( const uint8_t timeout, char *cmd )
{
	// Envia un comando:
	// Sale por: OK/ERROR/BufferFull/TIMEOUT

	// Las respuestas pueden ser nula ( no espero ) u otra.

	// ATRSP_NONE, ATRSP_OK, ATRSP_ERROR, ATRSP_TIMEOUT, ATRSP_OUT_INMEDIATE, ATRSP_UNKNOWN

	// El comando lo doy una sola vez !!!.
	// Si me da error u respuesta no esperada, no tiene sentido repetir el comando.
	// El timeout es en segundos pero chequeo c/50 ms.

int8_t exit_code = -1;
int16_t ticks = (20 * timeout);	// Cuantos ticks de 50ms corresponden al timeout.

	//xprintf_P(PSTR("DEBUG: timeout=%d\r\n"), timeout);
	//xprintf_P(PSTR("DEBUG: cmd=%s\r\n"), cmd );
	gprs_flush_RX_buffer();

	// Doy el comando.
	//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: (%d) %s\r\n"), local_timer, cmd );
	gprs_flush_TX_buffer();
	// Espera antes de c/comando. ( ver recomendaciones de TELIT )
	vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );
	xfprintf_P( fdGPRS , PSTR("%s"), cmd);

	// Respuesta inmediata: No importa el resultado.
	if ( timeout == 0 ) {
		// Salida inmediata:
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: OUT INMEDIATE.\r\n"));
		exit_code = ATRSP_OUT_INMEDIATE;
		return(exit_code);
	}

	// Espero respuesta
	while ( exit_code == -1 ) {

		// Espero 1 tick: Cada 50 ms chequeo el buffer de salida del modem por una respuesta.
		vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );

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
		if (  ticks-- <= 0 ) {
			exit_code = ATRSP_TIMEOUT;
		}
	}

	// Print debug causes
	switch( exit_code) {
	case ATRSP_OK:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ATRSP_OK\r\n"));
		break;
	case ATRSP_ERROR:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ERROR\r\n"));
		break;
	case ATRSP_TIMEOUT:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: TIMEOUT\r\n"));
		break;
	default:
		//xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ERROR RSP NO ESPERADA !!\r\n"));
		exit_code = ATRSP_UNKNOWN;
		break;
	}

	gprs_print_RX_buffer();
	return (exit_code);

}
//------------------------------------------------------------------------------------
