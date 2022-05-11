/*
 * sms.c
 *
 *  Created on: 8 dic. 2021
 *      Author: pablo
 */


#include "tkComms_sms.h"
#include "spx.h"
#include "string.h"

// ------------------------------------------------------------------------------------
void SMS_test(void)
{
	SMS_rxcheckpoint();
	/*
	if ( ! SMS_rxcheckpoint() ) {
		gprs_flush_TX_buffer();
		snprintf_P( gprs_txbuffer.buffer, GPRS_TXBUFFER_LEN, PSTR("Sin mensajes\nGracias."));
		SMSCB.smsTxMsg = gprs_txbuffer.buffer;
		SMS_send();
	}
	*/

}

/* ------------------------------------------------------------------------------------
 * ENVIO DE SMS
 * ------------------------------------------------------------------------------------
 */
bool SMS_send( void )
{
	// Envio un mensaje sms.
	// Mando un comando para setear el modo de sms en texto
	// Envio el comando.
	// Puedo mandar cualquier caracter de control !!
	// Se usa para mensajes formateados

char localStr[32];
uint8_t ctlz = 0x1A;
bool retS = false;

	// CMGS: Envio SMS.
	// Primera parte del comando.
	memset(localStr,'\0', sizeof(localStr));
	snprintf_P( localStr, sizeof(localStr),PSTR("AT+CMGS=\"%s\"\r"), systemVars.sms_conf.sms_auth_numbers[SMSCB.sms_nbr_idx]);
	FSM_sendATcmd( 0, localStr );

	// Espero el prompt > para enviar el mensaje.
	if ( ! gprs_check_response ( 50, ">" ) ) {
		xprintf_P( PSTR("SMS: ERROR Sent Fail( prompt Timeout) !!\r\n" ));
		return(false);
	}

	// Recibi el prompt
	gprs_print_RX_buffer();

	// Envio el mensaje:
	gprs_flush_RX_buffer();
	gprs_flush_TX_buffer();
	// Espera antes de c/comando. ( ver recomendaciones de TELIT )
	vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );
	xfprintf_P( fdGPRS, PSTR("%s\r %c"), SMSCB.smsTxMsg, ctlz  );

	xprintf_PD( DF_SMS, PSTR("SMS: msgtxt=[%s]\r\n"), SMSCB.smsTxMsg );

	// Espero el OK
	if ( ! gprs_check_response ( 50, "OK" ) ) {
		xprintf_P( PSTR("SMS: ERROR Sent Fail(sent Timeout) !!\r\n" ));
		retS = false;
	} else {
		xprintf_PD( DF_SMS, PSTR("SMS: Sent OK !!\r\n" ));
		retS = true;
	}

	gprs_print_RX_buffer();

	return(retS);

}
/* ------------------------------------------------------------------------------------
 * LECTURA DE SMS
 * ------------------------------------------------------------------------------------
 */

bool SMS_rxcheckpoint(void)
{
	/*
	 * Leo todos los mensaejes que hay en la memoria, los proceso y los borro
	 * El comando AT+CMGL="ALL" lista todos los mensajes.
	 * Filtro por el primer +CMGL: index,
	 * Leo y borro con AT+CMGRD=index
	 * Cuando no tengo mas mensajes, el comando AT+CMGL no me devuelve nada mas.
	 */

uint8_t msg_index;
bool retS = false;

	xprintf_PD( DF_SMS, PSTR("SMS: RX checkpoint Start\r\n" ));

	SMSCB.smsRxMsg = NULL;
	SMSCB.smsTxMsg = NULL;
	SMSCB.sms_nbr_idx = -1;

	while ( SMS_received(&msg_index) ) {
		// Veo si hay mensajes pendientes
		SMSCB.smsRxMsg = SMS_read_and_delete_by_index(msg_index);
		SMS_process();
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		retS = true;
	}

	xprintf_PD( DF_SMS, PSTR("SMS: RX checkpoint End\r\n" ));
	return(retS);

}
//------------------------------------------------------------------------------------
char *SMS_read_and_delete_by_index( uint8_t msg_index )
{
	/* Leo y borro con AT+CMGRD=index
	 * Retorno un puntero al mensaje
	 * Filtro el mensaje y lo imprimo.
	 * +CMGRD: "REC READ","+59899394959","","19/10/30,16:11:24-12"
	 * Msg3
	 *
	 * OK
	 */

char localStr[32];
int8_t cmd_rsp;
char *lines[4];
int16_t i;
int j;
bool next_is_line = false;
char ch;

	// Doy el comando para ver si hay mensajes SMS almacendos.
	memset(localStr,'\0', sizeof(localStr));
	snprintf_P( localStr, sizeof(localStr),PSTR("AT+CMGRD=%d\r"), msg_index);
	cmd_rsp = FSM_sendATcmd( 50, localStr );

	if ( cmd_rsp == ATRSP_OK ) {
		// El comando respondio OK.
		if ( gprs_check_response( 0, "+CMGRD:") ) {

			// Separo por lines
			j=0;
			next_is_line = true;
			for (i=0; i < sizeof(gprs_rxbuffer.buffer); i++ ) {

				ch = gprs_rxbuffer.buffer[i];

				if ( (ch == '\r') || ( ch == '\n') ) {
					// Descarto.
					gprs_rxbuffer.buffer[i] = '\0';
					next_is_line = true;
					continue;
				}

				// Es un caracter comun
				if ( next_is_line == true ) {
					// Encontre el comienzo de una linea.
					next_is_line = false;
					lines[j] = &gprs_rxbuffer.buffer[i];
					j++;
					if (j == 4 ) {
						break;
					}
				}

			}

			// Imprimo cada linea
			for (i=0; i<4; i++ ) {
				if ( lines[i] != NULL ) {
					xprintf_PD( DF_SMS, PSTR("SMS: MSG %d: [%s]\r\n"), i, lines[i] );
				}
			}

		}
	}

	// El contenido del mensaje esta en la linea 1
	return(lines[1]);


}
//------------------------------------------------------------------------------------
bool SMS_received( uint8_t *first_msg_index )
{
	/*
	 * Funcion que la tarea gprsTX utiliza para saber si el modem ha
	 * recibido un SMS.
	 *
	 * Envio el comando AT+CMGL="ALL" y espero por una respuesta
	 * del tipo +CMGL:
	 * Si no la recibo no hay mensajes pendientes.
	 * +CMGL: 1,"REC READ","+59899394959","","19/10/30,15:39:26-12"
	 * Va 2
	 * +CMGL: 0,"REC UNREAD","+59899394959","","19/10/30,15:45:38-12"
	 * Otro
	 */

bool retS = false;
char localStr[32] = { 0 };
char *stringp = NULL;
char *tk_idx= NULL;
char *delim = ",:";
int8_t cmd_rsp;
char *ts = NULL;
bool autorization = false;
int i;

	// Doy el comando para ver si hay mensajes SMS almacendos.
	memset(localStr,'\0', sizeof(localStr));
	snprintf_P( localStr, sizeof(localStr), PSTR("AT+CMGL=\"ALL\"\r") );
	cmd_rsp = FSM_sendATcmd( 50, localStr );

	if ( cmd_rsp == ATRSP_OK ) {
		// El comando respondio OK.
		if ( gprs_check_response( 0, "+CMGL:") ) {
			// Hay al menos un mensaje pendiente. Decodifico su indice

			// Veo si proviene de un numero autorizado.
			for ( i=0; i < SMS_AUTH_NUMBER_MAX; i++ ) {
				if ( strstr(gprs_rxbuffer.buffer, systemVars.sms_conf.sms_auth_numbers[i] )  ) {
					SMSCB.sms_nbr_idx = i;
					autorization = true;
				}
			}

			if ( ! autorization ) {
				// Borro
				xprintf_P(PSTR("SMS ALERTA: Fuente no autorizada !!\r\n"));
				SMSCB.sms_nbr_idx = -1;
				memset(localStr,'\0', sizeof(localStr));
				snprintf_P( localStr, sizeof(localStr), PSTR("AT+CMGD=,4\r") );
				cmd_rsp = FSM_sendATcmd( 50, localStr );
				return(false);
			}

			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( gprs_rxbuffer.buffer, "+CMGL:");
			strncpy(localStr, ts, sizeof(localStr));
			stringp = localStr;
			tk_idx = strsep(&stringp,delim);		// +CMGL:
			tk_idx = strsep(&stringp,delim);		//  0
			*first_msg_index = atoi(tk_idx);
			xprintf_PD( DF_SMS, PSTR("SMS id = [%d]\r\n"), *first_msg_index );
			retS = true;
		}
	}

	return(retS);
}
//------------------------------------------------------------------------------------
void SMS_process( void )
{
	// Aqui es donde parseo el mensaje del SMS y actuo en consecuencia.


RtcTimeType_t rtc;
int16_t pos;

	xprintf_PD( DF_SMS, PSTR("SMS: PROCESS Start\r\n") );
	xprintf_PD( DF_SMS, PSTR("SMS: rxmsg=[%s]\r\n"), SMSCB.smsRxMsg);

	// Agrego fecha y hora al mensaje
	memset( &rtc, '\0', sizeof(RtcTimeType_t));
	RTC_read_dtime(&rtc);
	gprs_txbuffer_reset();
	pos = snprintf_P( gprs_txbuffer.buffer, GPRS_TXBUFFER_LEN, PSTR("Fecha: %02d/%02d/%02d %02d:%02d\n"), rtc.year, rtc.month, rtc.day, rtc.hour, rtc.min );

	// Mensajes fijos.
	if ( strcmp_P( strupr(SMSCB.smsRxMsg), PSTR("READ APN")) == 0 ) {
		snprintf_P( &gprs_txbuffer.buffer[pos], GPRS_TXBUFFER_LEN, PSTR("APN:[%s]\n"), comms_conf.apn );
		SMSCB.smsTxMsg = gprs_txbuffer.buffer;
		SMS_send();

	} else 	if ( strcmp_P( strupr(SMSCB.smsRxMsg), PSTR("READ IP")) == 0 ) {
		snprintf_P( &gprs_txbuffer.buffer[pos], GPRS_TXBUFFER_LEN, PSTR("SERVER IP:[%s]\n"), comms_conf.server_ip_address );
		SMSCB.smsTxMsg = gprs_txbuffer.buffer;
		SMS_send();

	} else 	if ( strcmp_P( strupr(SMSCB.smsRxMsg), PSTR("READ PORT")) == 0 ) {
		snprintf_P( &gprs_txbuffer.buffer[pos], GPRS_TXBUFFER_LEN, PSTR("SERVER PORT:[%s]\n"), comms_conf.server_tcp_port );
		SMSCB.smsTxMsg = gprs_txbuffer.buffer;
		SMS_send();

	} else 	if ( strcmp_P( strupr(SMSCB.smsRxMsg), PSTR("READ SCRIPT")) == 0 ) {
		snprintf_P( &gprs_txbuffer.buffer[pos], GPRS_TXBUFFER_LEN, PSTR("SERVER SCRIPT:[%s]\n"), comms_conf.serverScript );
		SMSCB.smsTxMsg = gprs_txbuffer.buffer;
		SMS_send();

	} else 	if ( strcmp_P( strupr(SMSCB.smsRxMsg), PSTR("RESET")) == 0 ) {
		snprintf_P( &gprs_txbuffer.buffer[pos], GPRS_TXBUFFER_LEN, PSTR("RESET\n") );
		xprintf_P(PSTR("SMS ALERT: RESET !!\r\n"));
		SMSCB.smsTxMsg = gprs_txbuffer.buffer;
		SMS_send();

	} else 	if ( strcmp_P( strupr(SMSCB.smsRxMsg), PSTR("FRAME")) == 0 ) {
		data_sprintf_actual_inputs( gprs_txbuffer.buffer );
		//xprintf_P(PSTR("DEBUG: [%s]\r\n"), gprs_txbuffer.buffer);
		SMSCB.smsTxMsg = gprs_txbuffer.buffer;
		SMS_send();

	} else {

		// Chequeo contra el diccionario de ordenes.
		SMS_process_dict_orders( &gprs_txbuffer.buffer[pos] );

	}

	xprintf_PD( DF_SMS, PSTR("SMS:PROCESS Stop\r\n") );
}
//------------------------------------------------------------------------------------
void SMS_process_dict_orders( char *rsp )
{
	// Si el mensaje tiene argumentos, envio el argumento a la direccion modbus asociada al comando
	// Si esta solo, hago un query a la direccion modbus asociada y respondo con dicha respuesta del comando

char localStr[32] = { 0 };
char *stringp = NULL;
char *arg0 = NULL;
char *arg1 = NULL;
char *arg2 = NULL;
char *delim = ",;:=>< ";
int8_t i;
float mb_rsp;

	xprintf_PD( DF_SMS, PSTR("SMS: process_dict_orders\r\n"));

	// Parseo el mensaje.
	memset(localStr,'\0',sizeof(localStr));
	strncpy(localStr, SMSCB.smsRxMsg, sizeof(localStr));
	stringp = localStr;
	arg0 = strsep(&stringp,delim);		// Comando
	arg1 = strsep(&stringp,delim);		// primer argumento
	arg2 = strsep(&stringp,delim);		// segundo argumento

	xprintf_PD( DF_SMS, PSTR("SMS: args: 0[%s],1[%s],2[%s]\r\n"), arg0, arg1, arg2);

	if ( arg0 == NULL) {	// Mensaje nulo.
		return;
	}

	for (i=1; i<SMS_ORDERS_MAX; i++) {

		// Comparo el texto del mensaje con c/entrada del diccionario
		if ( strcmp ( strupr(systemVars.sms_conf.sms_orders[i].smsText), "X" ) == 0 ) {
			// No hay orden configurada.
			snprintf_P( rsp, GPRS_TXBUFFER_LEN, PSTR("COMANDO INCORRECTO"));
			SMSCB.smsTxMsg = gprs_txbuffer.buffer;
			SMS_send();
			return;
		}

		if ( strcmp( strupr(arg0), systemVars.sms_conf.sms_orders[i].smsText ) == 0 ) {
			// Encontre una orden en el diccionario para ejecutar.
			// Vemos si el canal modbus esta configurado
			//xprintf_P(PSTR("DEBUG1 [%s]\r\n"), modbus_conf.channel[ systemVars.sms_conf.sms_orders[i].mb_channel ].name );
			if ( strcmp ( modbus_conf.channel[ systemVars.sms_conf.sms_orders[i].mb_channel ].name, "X" ) == 0 ) {
				// Canal no configurado
				snprintf_P( rsp, GPRS_TXBUFFER_LEN, PSTR("CANAL MBUS NO CONFIGURADO"));
				SMSCB.smsTxMsg = gprs_txbuffer.buffer;
				SMS_send();
				return;
			}

			// Comando valido y canal modbus correcto:
			if ( arg1 == NULL ) {
				xprintf_PD( DF_SMS, PSTR("SMS: QUERY #%d\r\n"),i);
				mb_rsp = modbus_read_channel( systemVars.sms_conf.sms_orders[i].mb_channel );
				snprintf_P( rsp, GPRS_TXBUFFER_LEN, PSTR("RSP:[%.03f]\n"), mb_rsp );

			} else {

				xprintf_PD( DF_SMS, PSTR("SMS: SET #%d [%s]\r\n"), i, arg1);
				mb_rsp = modbus_write_output_channel(systemVars.sms_conf.sms_orders[i].mb_channel, atof(arg1));
				snprintf_P( rsp, GPRS_TXBUFFER_LEN, PSTR("OK:[%.03f]\n"), mb_rsp );
			}

			SMSCB.smsTxMsg = gprs_txbuffer.buffer;
			SMS_send();
			return;
		}
	}
	// No hay ninguna orden como el texto enviado por el SMS.
	snprintf_P( rsp, GPRS_TXBUFFER_LEN, PSTR("COMANDO INCORRECTO"));
	SMSCB.smsTxMsg = gprs_txbuffer.buffer;
	SMS_send();
	return;
}

//------------------------------------------------------------------------------------
// CONFIGURACION Y STATUS
//------------------------------------------------------------------------------------
void sms_config_defaults(void)
{

uint8_t i;

	// Configuro los numeros autorizados por defecto
	strcpy( systemVars.sms_conf.sms_auth_numbers[0],"99000000");
	strcpy( systemVars.sms_conf.sms_auth_numbers[1],"99000001");
	strcpy( systemVars.sms_conf.sms_auth_numbers[2],"99000002");

	for (i=0; i<SMS_ORDERS_MAX; i++) {
		systemVars.sms_conf.sms_orders[i].mb_channel = -1;
		memset( systemVars.sms_conf.sms_orders[i].smsText, '\0', SMS_ORDERS_LENGTH );
		strncpy(systemVars.sms_conf.sms_orders[i].smsText, "X", SMS_ORDERS_LENGTH);
	}
	// Textos fijos.
	systemVars.sms_conf.sms_orders[0].mb_channel = -1;
	strncpy(systemVars.sms_conf.sms_orders[0].smsText, "FRAME", SMS_ORDERS_LENGTH);

}
//------------------------------------------------------------------------------------
bool sms_config_auth_number( int pos, char *s_number )
{

	if (pos < 3) {
		strcpy( systemVars.sms_conf.sms_auth_numbers[pos], s_number);
		return(true);
	}
	return(false);
}
//------------------------------------------------------------------------------------
bool sms_config_order_dict( int pos, char *s_mb_channel, char *s_order)
{
	// El primer mensaje es FIJO
	if ( pos == 0 ) {
		return(false);
	}

	// Solo se permiten SMS_MAX_ORDERS orders
	if (pos > SMS_ORDERS_MAX ) {
		return(false);
	}

	systemVars.sms_conf.sms_orders[pos].mb_channel = atoi(s_mb_channel);
	strncpy(systemVars.sms_conf.sms_orders[pos].smsText, s_order, SMS_ORDERS_LENGTH);
	return(true);
}
//------------------------------------------------------------------------------------
void sms_status(void)
{

uint8_t i;

	xprintf_P( PSTR(">SMS: \r\n"));
	xprintf_P( PSTR("   auth_nbrs:[%s] [%s] [%s]\r\n"), systemVars.sms_conf.sms_auth_numbers[0],
			systemVars.sms_conf.sms_auth_numbers[1],
			systemVars.sms_conf.sms_auth_numbers[2]);

	xprintf_P( PSTR("   orders:\r\n"));
	xprintf_P( PSTR("   "));
	for (i=0; i<5; i++ ) {
		xprintf_P( PSTR("[%d:%s,%d] "), i, systemVars.sms_conf.sms_orders[i].smsText, systemVars.sms_conf.sms_orders[i].mb_channel );
	}
	xprintf_P( PSTR("\r\n"));
	xprintf_P( PSTR("   "));
	for (i=5; i<SMS_ORDERS_MAX; i++ ) {
		xprintf_P( PSTR("[%d:%s,%d] "), i, systemVars.sms_conf.sms_orders[i].smsText, systemVars.sms_conf.sms_orders[i].mb_channel );
	}
	xprintf_P( PSTR("\r\n"));

}
//------------------------------------------------------------------------------------
uint8_t sms_hash(void)
{

uint16_t i;
uint8_t hash = 0;
char hash_buffer[32];
char *p;
uint8_t j = 0;

	// SMS AUTH NUMBERS
	for(i=0;i<SMS_AUTH_NUMBER_MAX;i++) {
		// Vacio el buffer temoral
		memset(hash_buffer,'\0', sizeof(hash_buffer));
		// Copio sobe el buffer una vista ascii ( imprimible ) de c/registro.
		snprintf_P( hash_buffer, sizeof(hash_buffer), PSTR("SMS_NBR_%d,%s;"), i, systemVars.sms_conf.sms_auth_numbers[i] );

		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		while (*p != '\0') {
			hash = u_hash(hash, *p++);
		}
		//xprintf_P(PSTR("DEBUG SMS HASH: [%s][%d]\r\n"), hash_buffer, hash);
	}

	// SMS DICT ORDERS
	for(i=0;i<SMS_ORDERS_MAX;i++) {
		// Vacio el buffer temoral
		memset(hash_buffer,'\0', sizeof(hash_buffer));
		// Copio sobe el buffer una vista ascii ( imprimible ) de c/registro.
		snprintf_P( &hash_buffer[j], sizeof(hash_buffer), PSTR("S_DICT%d:%d,%s;"), i, systemVars.sms_conf.sms_orders[i].mb_channel , systemVars.sms_conf.sms_orders[i].smsText );

		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		while (*p != '\0') {
			hash = u_hash(hash, *p++);
		}
		//xprintf_P(PSTR("DEBUG SMS HASH: [%s][%d]\r\n"), hash_buffer, hash);
	}

	return(hash);

}
//------------------------------------------------------------------------------------

