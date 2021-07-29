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

void gprs_CPOF( void );

//------------------------------------------------------------------------------------

#define GPRS_RXBUFFER_LEN	512
struct {
	char buffer[GPRS_RXBUFFER_LEN];
	uint16_t ptr;
	uint16_t head;
	uint16_t tail;
	uint16_t max; //of the buffer
	bool full;
} gprs_rxbuffer;

#define IMEIBUFFSIZE	24
#define CCIDBUFFSIZE	24

struct {
	char buff_gprs_imei[IMEIBUFFSIZE];
	char buff_gprs_ccid[CCIDBUFFSIZE];
} gprs_status;

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
int16_t gprs_check_response( uint16_t start, const char *rsp )
{
	// Modifico para solo compara en mayusculas

int16_t i;

	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		taskYIELD();

		i = gprs_findstr_lineal(start, rsp);

	xSemaphoreGive( sem_RXBUFF );
	return(i);
}
//------------------------------------------------------------------------------------
int16_t gprs_check_response_with_to( uint16_t start, const char *rsp, uint8_t timeout )
{
	// Espera una respuesta durante un tiempo dado.
	// Hay que tener cuidado que no expire el watchdog por eso lo kickeo aqui. !!!!

int16_t ret = -1;

	while ( timeout > 0 ) {
		timeout--;
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

		// Veo si tengo la respuesta correcta.
		ret = gprs_check_response (start, rsp);
		if ( ret >= 0)
			return(ret);
	}

	return(-1);
}
//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL
//------------------------------------------------------------------------------------
void gprs_atcmd_preamble(void)
{
	// Espera antes de c/comando. ( ver recomendaciones de TELIT )
	vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );
}
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

	memset(gprs_status.buff_gprs_ccid, '\0', IMEIBUFFSIZE );
	memset(gprs_status.buff_gprs_ccid, '\0', IMEIBUFFSIZE );
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
	gprs_CPOF();

	IO_clr_GPRS_SW();	// Es un FET que lo dejo cortado
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	IO_clr_GPRS_PWR();	// Apago la fuente.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_CPOF( void )
{
	// Apaga el modem en modo 'soft'

	xprintf_PD( DF_COMMS, PSTR("COMMS: gprs CPOF\r\n"));
	gprs_flush_RX_buffer();
	gprs_flush_TX_buffer();
	gprs_atcmd_preamble();
	xfprintf_P( fdGPRS,PSTR("AT+CPOF\r\0"));
	vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );
	xprintf_PD( DF_COMMS, PSTR("COMMS: gprs CPOF OK.\r\n\0"));
	return;
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

	gprs_flush_RX_buffer();
	gprs_flush_TX_buffer();
	gprs_atcmd_preamble();
	xfprintf_P( fdGPRS , PSTR("AT+CNMP=%d\r\0"),modo);
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void gprs_set_PREF(uint8_t modo)
{
	// Setea el orden de preferencia para acceder a la red 2G o 3G

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

	gprs_flush_RX_buffer();
	gprs_flush_TX_buffer();
	gprs_atcmd_preamble();
	xfprintf_P( fdGPRS , PSTR("AT+CNAOP=%d\r\0"),modo);
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

char bands[20];

//	if ( *s_bands == '\0') {

		// SOLO HABILITO LAS BANDAS DE ANTEL !!!!
		strncpy(bands, "0000000004080380", strlen(bands));
		gprs_flush_RX_buffer();
		gprs_flush_TX_buffer();
		gprs_atcmd_preamble();
		xfprintf_P( fdGPRS , PSTR("AT+CNBP=0x%s\r\0"), bands);
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
//	}


}
//------------------------------------------------------------------------------------
bool gprs_SAT_set(uint8_t modo)
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


	xprintf_PD( DF_COMMS,  PSTR("GPRS: gprs SAT.(modo=%d)\r\n\0"),modo);

	switch(modo) {
	case 0:
		// Disable
		gprs_flush_RX_buffer();
		gprs_flush_TX_buffer();
		gprs_atcmd_preamble();
		xfprintf_P( fdGPRS , PSTR("AT+STK=0\r\0"));
		vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );
		break;
	case 1:
		// Enable
		gprs_flush_RX_buffer();
		gprs_flush_TX_buffer();
		gprs_atcmd_preamble();
		xfprintf_P( fdGPRS , PSTR("AT+STK=1\r\0"));
		vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );
		break;
	case 2:
		// Check. Query STK status ?
		xprintf_P(PSTR("GPRS: query STK status ?\r\n\0"));
		gprs_flush_RX_buffer();
		gprs_flush_TX_buffer();
		gprs_atcmd_preamble();
		xfprintf_P( fdGPRS , PSTR("AT+STK?\r\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		gprs_print_RX_buffer();
		break;
	default:
		return(false);
	}

	return (true);

}
//------------------------------------------------------------------------------------
void gprs_modem_status(void)
{
	// MODO:
	switch(xCOMMS_stateVars.gprs_mode) {
	case 2:
		xprintf_PD( DF_COMMS,  PSTR("  modo: AUTO\r\n"));
		break;
	case 13:
		xprintf_PD( DF_COMMS,  PSTR("  modo: 2G(GSM) only\r\n"));
		break;
	case 14:
		xprintf_PD( DF_COMMS,  PSTR("  modo: 3G(WCDMA) only\r\n"));
		break;
	default:
		xprintf_PD( DF_COMMS,  PSTR("  modo: ??\r\n") );
	}

	// PREFERENCE
	switch(xCOMMS_stateVars.gprs_pref) {
	case 0:
		xprintf_PD( DF_COMMS,  PSTR("  pref: AUTO\r\n"));
		break;
	case 1:
		xprintf_PD( DF_COMMS,  PSTR("  pref: 2G,3G\r\n"));
		break;
	case 2:
		xprintf_PD( DF_COMMS,  PSTR("  pref: 3G,2G\r\n"));
		break;
	default:
		xprintf_PD( DF_COMMS,  PSTR("  pref: ??\r\n") );
		return;
	}

	// BANDS:
	xprintf_PD( DF_COMMS,  PSTR("  bands:[%s]\r\n\0"), xCOMMS_stateVars.gprs_bands );

}
//------------------------------------------------------------------------------------
void gprs_read_MODO(void)
{

bool retS = false;

	xCOMMS_stateVars.gprs_mode = 0;
//	retS = gprs_ATCMD( false, MAX_TRYES_CNMP , TIMEOUT_CNMP, (PGM_P *)AT_CNMP );
	if ( retS ) {
		if ( gprs_check_response(0, "CNMP: 2") > 0 ) {
			xCOMMS_stateVars.gprs_mode = 2;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs modo AUTO\r\n"));
			return;
		}
		if ( gprs_check_response(0, "CNMP: 13") > 0 ) {
			xCOMMS_stateVars.gprs_mode = 13;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs modo 2G(GSM) only\r\n"));
			return;
		}
		if ( gprs_check_response(0, "CNMP: 14") > 0 ) {
			xCOMMS_stateVars.gprs_mode = 14;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs modo 3G(WCDMA) only\r\n"));
			return;
		}
	}
}
//------------------------------------------------------------------------------------
void gprs_read_PREF(void)
{
bool retS = false;

	xCOMMS_stateVars.gprs_pref = 0;
//	retS = gprs_ATCMD( false, MAX_TRYES_CNAOP , TIMEOUT_CNAOP, (PGM_P *)AT_CNAOP );
	if ( retS ) {
		if ( gprs_check_response(0, "CNAOP: 0") > 0 ) {
			xCOMMS_stateVars.gprs_pref = 2;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs pref. AUTO\r\n"));
			return;
		}
		if ( gprs_check_response(0, "CNAOP: 1") > 0 ) {
			xCOMMS_stateVars.gprs_pref = 1;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs pref. 2G,3G\r\n"));
			return;
		}
		if ( gprs_check_response(0, "CNAOP: 2") > 0 ) {
			xCOMMS_stateVars.gprs_pref = 2;
			xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs pref. 3G,2G\r\n"));
			return;
		}
	}
}
//------------------------------------------------------------------------------------
void gprs_read_BANDS(void)
{
	// ANTEL opera GSM(2G) en 900/1800 y 3G(UTMS/WCDMA) en 850/2100
	// Al leer las bandas tenemos un string con 16 bytes y 64 bits.
	// C/bit en 1 indica una banda predida.
	// Las bandas que me interesan estan los los primeros 4 bytes por lo tanto uso
	// 32 bits. !!!

bool retS = false;
char *ts = NULL;
char c = '\0';
char *ptr = NULL;
uint8_t i;

//
union {
	uint8_t u8[4];
	uint32_t u32;
} bands;


	memset( xCOMMS_stateVars.gprs_bands, '\0', sizeof(xCOMMS_stateVars.gprs_bands) );
//	retS = gprs_ATCMD( false, MAX_TRYES_CNBP , TIMEOUT_CNBP, (PGM_P *)AT_CNBP );
	if ( retS ) {
		ptr = xCOMMS_stateVars.gprs_bands;
		ts = strchr( gprs_rxbuffer.buffer, ':');
		ts++;
		while ( (c= *ts) != '\r') {
			*ptr++ = c;
			ts++;
		}
		*ptr = '\0';
		xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs bands=[%s]\r\n\0"), xCOMMS_stateVars.gprs_bands );
		/*
		for (i=0; i <16; i++) {
			xprintf_PD(DF_COMMS, PSTR("DEBUG: pos=%d, val=0x%02x [%d]\r\n"), i, xCOMMS_stateVars.gprs_bands[i], xCOMMS_stateVars.gprs_bands[i] );
		}
		*/

		// DECODIFICACION:

		bands.u32 = strtoul( &xCOMMS_stateVars.gprs_bands[12], &ptr, 16);
		/*
		xprintf_P(PSTR("test b0=%d\r\n"), bands.u8[0]);
		xprintf_P(PSTR("test b1=%d\r\n"), bands.u8[1]);
		xprintf_P(PSTR("test b2=%d\r\n"), bands.u8[2]);
		xprintf_P(PSTR("test b3=%d\r\n"), bands.u8[3]);
		xprintf_P(PSTR("test u32=%lu\r\n"), bands.u32);
		xprintf_P(PSTR("test str=[%s]\r\n"), &xCOMMS_stateVars.gprs_bands[12]);
	*/

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
//------------------------------------------------------------------------------------
int8_t FSM_sendATcmd(int8_t timeout, char *cmd, char *rsp )
{
	// Envia un comando y espera una respuesta.
	// El comando puede ser nulo y entonces solo esperamos respuesta.(PBDONE)
	// Las respuestas pueden ser nula ( no espero ) u otra.
	// El comando lo doy una sola vez !!!.
	// Si me da error u respuesta no esperada, no tiene sentido repetir el comando.

int8_t exit_code = ATRSP_UNKNOWN;

	// Doy el comando.
	if ( cmd != '\0' ) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: %s\r\n"),cmd );
		gprs_flush_TX_buffer();
		gprs_flush_RX_buffer();
		gprs_atcmd_preamble();
		xfprintf_P( fdGPRS , PSTR("%s"), cmd);
	}

	while ( timeout-- > 0 ) {

		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: AWAIT %d\r\n"), timeout );

		// No espero respuesta: salgo
		if ( rsp == '\0') {
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: NONE\r\n"));
			exit_code = ATRSP_NONE;
			break;
		}

		// Chequeo respuestas:
		// Respuesta esperada: salgo
		if ( gprs_check_response(0, rsp ) >= 0 ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: EXPECTED\r\n"));
			exit_code = ATRSP_EXPECTED;
			break;
		}

		// Respuesta ERR: El comando respondio con error: salgo.
		if ( gprs_check_response(0, "ERROR" ) >= 0 ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: ERR\r\n"));
			exit_code = ATRSP_ERR;
			break;
		}

		// Respuesta OK: El comando termino pero respondio con otra cosa que lo esperado: salgo.
		if ( gprs_check_response(0, "OK" ) >= 0 ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: NOEXPECTED\r\n"));
			exit_code = ATRSP_NOTEXPECTED;
			break;
		}

		// Timeout
		if ( timeout == 0 ) {
			exit_code = ATRSP_TIMEOUT;
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSM_sendATcmd: TIMEOUT\r\n"));
			break;
		}

	}

	//
	gprs_print_RX_buffer();
	return (exit_code);

}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES SOBRE EL rxbuffer
//------------------------------------------------------------------------------------
int16_t gprs_findstr_lineal( uint16_t start, const char *rsp )
{
uint16_t i, j, k;
char c1, c2;

	i = start;
	while( 1 ) {
		if ( gprs_rxbuffer.buffer[i] == '\0')
		return(-1);
		//
		j = i;
		k = 0;
		while (1) {
			c1 = rsp[k];
			if ( c1 == '\0')
				return (i);

			c2 =  gprs_rxbuffer.buffer[j];
			if ( toupper(c2) != toupper(c1) )
				break;

			j++;
			k++;
		}
		// Avanzo ptr.
		i++;
		// Controlo la salida
		if ( i == GPRS_RXBUFFER_LEN )
			break;
	}

		return(-1);
}
//------------------------------------------------------------------------------------
void gprs_rxbuffer_copy_to( char *dst, uint16_t start, uint16_t size )
{

uint16_t i;

#ifdef GPRS_RX_LINEAL_BUFFER

	while ( xSemaphoreTake( sem_RXBUFF, MSTOTAKERXBUFFSEMPH ) != pdTRUE )
		taskYIELD();

		for (i = 0; i < size; i++)
			dst[i] = gprs_rxbuffer.buffer[i+start];

	xSemaphoreGive( sem_RXBUFF );

#else

uint16_t j;

		if (start == 0) {
			i = gprs_rxbuffer.tail;
		} else {
			i = start;
		}
		j = 0;
		while(j < size) {
			dst[j] = gprs_rxbuffer.buffer[i];
			j++;
			i = (i + 1) % gprs_rxbuffer.max;
		}

#endif


}
//------------------------------------------------------------------------------------
