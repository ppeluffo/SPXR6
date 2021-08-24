/*
 * ul_modbus.c
 *
 *  Created on: 10 ago. 2021
 *      Author: pablo
 *
 *  Se definen hasta 20 canales de LECTURA.
 *
 */

#include "ul_modbus.h"
#include "tkComms.h"

#define DF_MBUS ( systemVars.debug == DEBUG_MODBUS )

const uint8_t auchCRCHi[] PROGMEM = {
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
		0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40
	} ;

const uint8_t auchCRCLo[] PROGMEM = {
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
		0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
		0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
		0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
		0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
		0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
		0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
		0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
		0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
		0x40
	};


mbus_CONTROL_BLOCK_t mbus_cb;
modbus_hold_t hold_reg;

//------------------------------------------------------------------------------------
bool modbus_read( float mbus_data[] )
{
	// Lee todos los canales modbus configurados y deja los valores en el correspondiente
	// luegar del array del parametro.
	// Los nombres de los canales deben ser #X y estar en order. Al primer canal X, sale. !!!

bool retS = false;
uint8_t ch;

	// Si no tengo canales configurados, salgo.
	if ( modbus_conf.slave_address == 0x00 ) {
		return(true);
	}

	for ( ch = 0; ch < MODBUS_CHANNELS; ch++) {
		// Si un canal no esta definido, salgo
		if ( strcmp ( modbus_conf.channel[ch].name, "X" ) == 0 ) {
			break;
		}
		//
		modbus_read_channel ( ch, &mbus_data[ch] );
	}

	return(retS);
}

//------------------------------------------------------------------------------------
// CONFIGURACION
//------------------------------------------------------------------------------------
void modbus_config_defaults(void)
{

uint8_t i;

	for ( i = 0; i < MODBUS_CHANNELS; i++ ) {
		strcpy( modbus_conf.channel[i].name, "X");
		modbus_conf.channel[i].address = 0x00;
		modbus_conf.channel[i].type = 'I';
		modbus_conf.channel[i].nro_regs = 1;
		modbus_conf.channel[i].divisor_p10 = 1.0;
		modbus_conf.channel[i].rcode = 0x03;
	}

	modbus_conf.slave_address = 0x00;
	modbus_conf.waiting_poll_time = 250;

}
//------------------------------------------------------------------------------------
void modbus_config_status(void)
{

uint8_t i;

	xprintf_P( PSTR(">Modbus: ( name,address,type,nro_regs,factor_p10,rcode )\r\n"));

	xprintf_P( PSTR("   sla_addr=0x%02x, wpTime=%d\r\n"), modbus_conf.slave_address, modbus_conf.waiting_poll_time );
	if ( modbus_conf.slave_address == 0x00 ) {
		return;
	}

	for ( i = 0; i < MODBUS_CHANNELS; i++ ) {
		if ( (i % 4) == 0 ) {
			xprintf_P(PSTR("   "));
		}

		if ( strcmp ( modbus_conf.channel[i].name, "X" ) == 0 ) {
			xprintf_P(PSTR("\r\n"));
			return;
		}

		xprintf_P( PSTR("MB%02d:[%s,0x%04X,%c,%d,0x%02d,%0.3f], "), i,
				modbus_conf.channel[i].name,
				modbus_conf.channel[i].address,
				modbus_conf.channel[i].type,
				modbus_conf.channel[i].nro_regs,
				modbus_conf.channel[i].rcode,
				modbus_conf.channel[i].divisor_p10	);

		if ( ((i+1) % 4) == 0 ) {
			xprintf_P(PSTR("\r\n"));
		}
	}

	xprintf_P(PSTR("\r\n"));

}
//------------------------------------------------------------------------------------
bool modbus_config_slave( char *s_slave_address )
{
	modbus_conf.slave_address = atoi(s_slave_address);
	return(true);
}
//------------------------------------------------------------------------------------
bool modbus_config_channel( char *s_ch, char *s_name, char *s_addr, char *s_nro_recds, char *s_rcode, char *s_type, char *s_divisor_p10 )
{
uint8_t i;

	i = atoi(s_ch);

	if (( s_name == NULL ) || ( s_addr == NULL) || (s_nro_recds == NULL) || ( s_rcode == NULL) || ( s_divisor_p10 == NULL)  ) {
		return(false);
	}

	snprintf_P( modbus_conf.channel[i].name, PARAMNAME_LENGTH, PSTR("%s\0"), s_name );
	modbus_conf.channel[i].address = atoi(s_addr);
	modbus_conf.channel[i].nro_regs = atoi(s_nro_recds);
	modbus_conf.channel[i].rcode = atoi(s_rcode);
	modbus_conf.channel[i].type = toupper(s_type[0]);
	modbus_conf.channel[i].divisor_p10 = atoi(s_divisor_p10);
	return(true);

}
//------------------------------------------------------------------------------------
bool modbus_config_waiting_poll_time( char *s_waiting_poll_time)
{
	// Configura el tiempo que espera una respuesta

	modbus_conf.waiting_poll_time = atoi(s_waiting_poll_time);
	return(true);
}
//------------------------------------------------------------------------------------
uint8_t modbus_hash(void)
{

uint8_t hash = 0;
char *p;
uint8_t i,j;
int16_t free_size = sizeof(hash_buffer);

	// Vacio el buffer temoral
	memset(hash_buffer,'\0', sizeof(hash_buffer));

	j = 0;
	j += snprintf_P( &hash_buffer[j], free_size, PSTR("MODBUS;SLA:%04d;"), modbus_conf.slave_address );
	//xprintf_P( PSTR("DEBUG_MBHASH = [%s]\r\n\0"), hash_buffer );
	free_size = (  sizeof(hash_buffer) - j );
	if ( free_size < 0 ) goto exit_error;

	p = hash_buffer;
	while (*p != '\0') {
		//checksum += *p++;
		hash = u_hash(hash, *p++);
	}

	for (i=0; i < MODBUS_CHANNELS; i++ ) {
		// Vacio el buffer temoral
		memset(hash_buffer,'\0', sizeof(hash_buffer));
		free_size = sizeof(hash_buffer);
		// Copio sobe el buffer una vista ascii ( imprimible ) de c/registro.
		j = snprintf_P( hash_buffer, free_size, PSTR("MB%02d:%s,%04d,%02d,%02d,%c;%.03f;"), i,
				modbus_conf.channel[i].name,
				modbus_conf.channel[i].address,
				modbus_conf.channel[i].nro_regs,
				modbus_conf.channel[i].rcode,
				modbus_conf.channel[i].type,
				modbus_conf.channel[i].divisor_p10
				);
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;
		//xprintf_P( PSTR("DEBUG_MBHASH = [%s]\r\n\0"), hash_buffer );
		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		while (*p != '\0') {
			hash = u_hash(hash, *p++);
		}

	}
	return(hash);

exit_error:

	xprintf_P( PSTR("MODBUS: Hash ERROR !!!\r\n\0"));
	return(0x00);

}
//------------------------------------------------------------------------------------
bool modbus_io( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb, modbus_hold_t *hreg )
{
	//
	pv_modbus_make_ADU (mbus_cb);
	pv_modbus_txmit_ADU( f_debug, mbus_cb );
	if ( ! pv_modbus_rcvd_ADU( f_debug, mbus_cb ) )
		return(false);

	if ( ! pv_modbus_decode_ADU ( f_debug, mbus_cb,hreg ) )
		return(false);

	return(true);
}
//------------------------------------------------------------------------------------
void modbus_read_channel ( uint8_t ch, float *result )
{
	// Poleo el canal.
	mbus_cb.sla_address = modbus_conf.slave_address;
	mbus_cb.function_code = modbus_conf.channel[ch].rcode;
	mbus_cb.address = modbus_conf.channel[ch].address;
	mbus_cb.nro_recds = modbus_conf.channel[ch].nro_regs;
	mbus_cb.type = modbus_conf.channel[ch].type;
	mbus_cb.divisor_p10 = modbus_conf.channel[ch].divisor_p10;
	//
	modbus_io( DF_MBUS, &mbus_cb, &hold_reg );
	//
	if ( mbus_cb.type == 'I') {
		*result = hold_reg.u32_value / mbus_cb.divisor_p10 ;
	} else {
		*result = hold_reg.float_value;
	}
	//
	vTaskDelay( ( TickType_t)( modbus_conf.waiting_poll_time / portTICK_RATE_MS ) );
}
//------------------------------------------------------------------------------------
// FUNCIONES DE TEST
//------------------------------------------------------------------------------------
void modbus_test_genpoll(char *arg_ptr[16] )
{
	// Recibe el argv con los datos en char hex para trasmitir el frame.
	// El formato es: { type(F|I} sla fcode addr length }
	// write modbus genpoll F 9 3 4118 2
	// TX: (len=8):[0x09][0x03][0x10][0x16][0x00][0x02][0x20][0x47]

	/*
	xprintf_P(PSTR("MODBUS GENPOLL START:\r\n"));

	memset( hreg.rep_str, '\0', 4 );
	memset ( &mbus_ctl, '\0', sizeof(mbus_ctl));

	mbus_ctl.type = toupper(arg_ptr[3][0]);
	mbus_ctl.sla_address = atoi(arg_ptr[4]);
	mbus_ctl.function_code = atoi(arg_ptr[5]);
	mbus_ctl.address = atoi(arg_ptr[6]);
	mbus_ctl.length = atoi(arg_ptr[7]);
	mbus_ctl.divisor_p10 = 1;

	pv_modbus_io( true, &mbus_ctl, &hreg);
	*/

	xprintf_P(PSTR("MODBUS GENPOLL END:\r\n"));

}
//------------------------------------------------------------------------------------
void modbus_test_chpoll(char *s_channel)
{
/*
uint8_t channel;
hold_reg_t hreg;

	xprintf_P(PSTR("MODBUS CHPOLL START:\r\n"));

	channel = atoi(s_channel);

	if ( channel >= MODBUS_CHANNELS ) {
		xprintf_P(PSTR("ERROR: Nro.canal < %d\r\n"), MODBUS_CHANNELS);
		return;
	}

	if ( ! strcmp ( systemVars.modbus_conf.mbchannel[channel].name, "X" ) ) {
		xprintf_P(PSTR("ERROR: Canal no definido (X)\r\n"));
		return;
	}

	modbus_poll_channel(true, channel, &hreg );
*/
	xprintf_P(PSTR("MODBUS CHPOLL END.\r\n"));
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
uint16_t pv_modbus_CRC16( uint8_t *msg, uint8_t msg_size )
{

uint8_t CRC_Hi = 0xFF;
uint8_t CRC_Lo = 0xFF;
uint8_t tmp;

uint8_t uindex;

	while (msg_size--) {
		uindex = CRC_Lo ^ *msg++;
		tmp = (PGM_P) pgm_read_byte_far(&(auchCRCHi[uindex]));
		CRC_Lo = CRC_Hi ^ tmp;
		tmp = (PGM_P)pgm_read_byte_far(&(auchCRCLo[uindex]));
		CRC_Hi = tmp;
	}

	return( CRC_Hi << 8 | CRC_Lo);

}
//------------------------------------------------------------------------------------
void pv_modbus_make_ADU( mbus_CONTROL_BLOCK_t *mbus_cb )
{
	// Prepara el ADU ( raw frame ) con los datos de la variable global mbus_cb.
	// Calcula el CRC de modo que el frame queda pronto a transmitirse por el serial

uint8_t size = 0;
uint16_t crc;

	memset( mbus_cb->tx_buffer, '\0', MBUS_TXMSG_LENGTH );
	mbus_cb->tx_buffer[0] = mbus_cb->sla_address;		// SLA
	mbus_cb->tx_buffer[1] = mbus_cb->function_code;		// FCODE
	mbus_cb->tx_buffer[2] = (uint8_t) (( mbus_cb->address & 0xFF00  ) >> 8);		// DST_ADDR_H
	mbus_cb->tx_buffer[3] = (uint8_t) ( mbus_cb->address & 0x00FF );				// DST_ADDR_L
	mbus_cb->tx_buffer[4] = (uint8_t) ((mbus_cb->nro_recds & 0xFF00 ) >> 8);		// NRO_REG_HI
	mbus_cb->tx_buffer[5] = (uint8_t) ( mbus_cb->nro_recds & 0x00FF );			// NRO_REG_LO

	// CRC
	size = 6;
	crc = pv_modbus_CRC16( mbus_cb->tx_buffer, size );

	mbus_cb->tx_buffer[6] = (uint8_t)( crc & 0x00FF );			// CRC Low
	mbus_cb->tx_buffer[7] = (uint8_t)( (crc & 0xFF00) >> 8 );	// CRC High
	mbus_cb->tx_size = 8;

}
//------------------------------------------------------------------------------------
void pv_modbus_txmit_ADU( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{
	// Transmite el frame modbus almcenado en mbus_cb.tx_buffer
	//
	// OJO: En MODBUS, los bytes pueden ser 0x00 y no deben ser interpretados como NULL
	// al trasmitir por lo tanto hay que usar el data_size
	// Debemos cumplir extrictos limites de tiempo por lo que primero logueo y luego
	// muestro el debug.

uint8_t i;

	// Transmite un mensaje MODBUS
	// Habilita el RTS para indicar que transmite
	// Apaga la recepcion. ( El IC 485 copia la tx y rx )

	xprintf_PD( f_debug, PSTR("MODBUS TX: start\r\n"));

	// Log
	if ( f_debug ) {
		xprintf_P( PSTR("MODBUS TX: (len=%d):"), mbus_cb->tx_size);
		for ( i = 0 ; i < mbus_cb->tx_size ; i++ ) {
			xprintf_P( PSTR("[0x%02X]"), mbus_cb->tx_buffer[i]);
		}
		xprintf_P( PSTR("\r\n"));
	}

	// Transmito
	// borro buffers y espero 3.5T (9600) = 3.5ms ( START )
	aux_flush_TX_buffer();
	aux_flush_RX_buffer();
	vTaskDelay( (portTickType)( 10 / portTICK_RATE_MS ) );
	i = xnprintf_MBUS( (const char *)mbus_cb->tx_buffer, mbus_cb->tx_size );

	xprintf_PD( f_debug, PSTR("MODBUS TX end(%d):\r\n"),i);
}
//------------------------------------------------------------------------------------
bool pv_modbus_rcvd_ADU( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{
	// Recibe del auxBuffer un frame modbus y lo almacena en mbus_data_s.rx_buffer
	//
	// https://stackoverflow.com/questions/13254432/modbus-is-there-a-maximum-time-a-device-can-take-to-respond
	// Esperamos la respuesta 1s y si no llego damos timeout

	// Lee el buffer de recepcion del puerto AUX (MODBUS )
	// Es por poleo de modo que una vez que transmiti un frame, debo esperar un tiempo que llegue la respuesta
	// y luego usar esta funcion para decodificar la lectura.

	// Para copiar buffers no puedo usar strcpy porque pueden haber datos en 0.
	// Copio byte a byte con limite

	// El punto clave esta en la recepcion por medio de las funciones de aux !!!

uint16_t crc_rcvd;
uint16_t crc_calc;
uint8_t i;
bool retS = false;

	xprintf_PD( f_debug, PSTR("MODBUS RX: start\r\n"));

	// Leo el aux_rx_buffer y lo dejo en el modbus_rx_buffer
	memset( mbus_cb->rx_buffer, '\0', MBUS_RXMSG_LENGTH );
	aux_rxbuffer_copyto( mbus_cb->rx_buffer, &mbus_cb->rx_size, MBUS_RXMSG_LENGTH );
	xprintf_PD( f_debug, PSTR("MODBUS RX: rx_size=%d\r\n"), mbus_cb->rx_size );

	// Paso 1: Log
	if (f_debug) {
		xprintf_P( PSTR("MODBUS RX: (len=%d):"), mbus_cb->rx_size);
		for ( i = 0 ; i < mbus_cb->rx_size; i++ ) {
			xprintf_P( PSTR("[0x%02X]"), mbus_cb->rx_buffer[i]);
		}
		xprintf_P( PSTR("\r\n"));
	}

	// Paso 2: Controlo el largo.
	if ( mbus_cb->rx_size < 3) {
		// Timeout error:
		xprintf_PD( f_debug, PSTR("MODBUS RX: TIMEOUT ERROR\r\n\0"));
		retS = false;
		goto quit;
	}

	// Pass3: Calculo y verifico el CRC
	crc_calc = pv_modbus_CRC16( mbus_cb->rx_buffer, (mbus_cb->rx_size - 2) );
	crc_rcvd = mbus_cb->rx_buffer[mbus_cb->rx_size - 2] + ( mbus_cb->rx_buffer[mbus_cb->rx_size - 1] << 8 );

	if ( crc_calc != crc_rcvd) {
		xprintf_PD( f_debug, PSTR("MODBUS RX: CRC ERROR: rx[0x%02x], calc[0x%02x]\r\n\0"), crc_rcvd, crc_calc);
		// De acuerdo al protocolo se ignora el frame recibido con errores CRC
		retS = false;
		goto quit;
	}

	retS = true;

quit:

	xprintf_PD( f_debug, PSTR("MODBUS RX end:\r\n") );
	return(retS);

}
//------------------------------------------------------------------------------------
bool pv_modbus_decode_ADU ( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb, modbus_hold_t *hreg )
{
	// En la decodificacion de los enteros no hay problema porque los flowmeters y los PLC
	// mandan el MSB primero.
	// En el caso de los float, solo los mandan los PLC y ahi hay un swap de los bytes por
	// lo que para decodificarlo necesito saber que los 4 bytes corresponden a un float!!
	//
	// MODBUS es BIG_ENDIAN: Cuando se manda un nro de varios bytes, el MSB se manda primero
	// MSB ..... LSB
	// Big Endian: El MSB se pone en el byte de la direccion mas baja:
	// Little Endian: El LSB se pone en el byte de la direccion mas baja.

	// Decodifica el frame recibido que se encuentra en mbus_data.rx_buffer
	// Debo copiar los bytes a hreg->bytes_raw para luego interpretarlo como int o float.
	// La interpretacion la da el codigo F o I y se hace con la funcion pv_modbus_print_value

	// Lo que hacemos es escribir y leer registros del PLC.
	// (0x03) Read Holding Registers
	// (0x04) Read Input Registers
	// (0x06) Write Single Register
	// MBUS_RX_FRAME: SLA FCODE ..............

uint8_t function_code;
bool retS = false;

	xprintf_PD( f_debug, PSTR("MODBUS DECODE: start\r\n"));

	memset( hreg->str_value, '\0', 4 );

	function_code = mbus_cb->rx_buffer[1];	// Solo 3 o 6.
	switch(function_code) {
	case 0x3:
		xprintf_PD(f_debug, PSTR("MODBUS DECODE: Read Holding Register (0x03)\r\n"));
		//pv_modbus_decode_function3( hreg, mbus_ctl->type );
		retS = true;
		break;

	case 0x6:
		xprintf_PD(f_debug, PSTR("MODBUS DECODE: Write Holding Register (0x06)\r\n"));
		//pv_modbus_decode_function6( hreg, mbus_ctl->type );
		retS = true;
		break;

	default:
		xprintf_PD(f_debug, PSTR("MODBUS DECODE: Codigo de funcion no soportado.\r\n"));
		// Indicamos el error con NaN
		strcpy( hreg->str_value, "NaN" );
		xprintf_PD( f_debug, PSTR("MODBUS DECODE: io_error\r\n"));
		retS = false;
		break;
	}

	xprintf_PD(f_debug, PSTR("MODBUS DECODE: b0[0x%02X] b1[0x%02X] b2[0x%02X] b3[0x%02X]\r\n"), hreg->raw_value[0], hreg->raw_value[1], hreg->raw_value[2], hreg->raw_value[3]);
	xprintf_PD(f_debug, PSTR("MODBUS DECODE: Fcode=%d\r\n"), function_code);

	xprintf_PD( f_debug, PSTR("MODBUS DECODE: end\r\n") );
	return(retS);

}
//------------------------------------------------------------------------------------
