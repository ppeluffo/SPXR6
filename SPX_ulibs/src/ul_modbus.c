/*
 * ul_modbus.c
 *
 *  Created on: 10 ago. 2021
 *      Author: pablo
 *
 *  Se definen hasta 20 canales de LECTURA.
 *
 *  La conversion se puede hacer en https://www.binaryconvert.com/.
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

void pv_decoder_f3_c0123(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_decoder_f3_c3210(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_decoder_f3_c1032(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_decoder_f3_c2301(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );

void pv_encoder_f6_c0123(mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_encoder_f6_c2301(mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_encoder_f6_c3210(mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_encoder_f6_c1032(mbus_CONTROL_BLOCK_t *mbus_cb );

void pv_encoder_f16_c0123(mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_encoder_f16_c2301(mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_encoder_f16_c3210(mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_encoder_f16_c1032(mbus_CONTROL_BLOCK_t *mbus_cb );

//------------------------------------------------------------------------------------
// READ
//------------------------------------------------------------------------------------
void modbus_read( float mbus_data[] )
{
	// Lee todos los canales modbus configurados y deja los valores en el correspondiente
	// luegar del array del parametro.
	// Los nombres de los canales deben ser #X y estar en order. Al primer canal X, sale. !!!

uint8_t ch;

	for ( ch = 0; ch < MODBUS_CHANNELS; ch++) {
		// Si un canal no esta definido, salgo
		if ( strcmp ( modbus_conf.channel[ch].name, "X" ) == 0 ) {
			break;
		}
		//
		// Si ya el canal tiene slave_address en 0, salgo. Asumo que el resto
		// no esta definido.
		if ( modbus_conf.channel[ch].slave_address == 0 ) {
			return;
		}

		mbus_data[ch] = modbus_read_channel ( ch );
	}

	return;
}
//------------------------------------------------------------------------------------
float modbus_read_channel ( uint8_t ch )
{
	/*
	 *  Poleo un canal.
	 *  La funcion modbus_io me devuelve el valor leido en un hreg.
	 *  Dependiendo del tipo es como lo interpreto.
	 *  SIEMPRE lo convierto a float para devolverlo !!!.
	 */

uint32_t init_ticks = sysTicks;
float pvalue = 0.0;

	xprintf_PD( DF_MBUS, PSTR("MODBUS: CHPOLL %02d START\r\n"),ch );

	// Para acceder al bus debo tomar el semaforo.
	while ( xSemaphoreTake( sem_MBUS, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	// Preparo el registro con los datos del canal
	// Son operaciones de READ que siempre devuelven floats.
	memcpy( &mbus_cb.channel,  &modbus_conf.channel[ch], sizeof(modbus_channel_t));
	//
	modbus_io( DF_MBUS, &mbus_cb );
	//
	// Siempre retorno un float

	// Si hubo un error, pongo un NaN y salgo
	if (mbus_cb.io_status == false) {
		pvalue = mbus_cb.udata.float_value;
		goto quit;
	}

	// La operacion de io es exitosa.
	if ( mbus_cb.channel.type == FLOAT) {
		pvalue = mbus_cb.udata.float_value;

	} else if (mbus_cb.channel.type == i16 ) {
		// En este caso importa el parametro divisor_p10 ya que convierte al entero en float
		pvalue = 1.0 * mbus_cb.udata.i16_value / pow(10, mbus_cb.channel.divisor_p10 );

	} else if (mbus_cb.channel.type == u16 ) {
		pvalue = 1.0 * mbus_cb.udata.u16_value / pow(10, mbus_cb.channel.divisor_p10 );

	} else if (mbus_cb.channel.type == u32 ) {
		pvalue = 1.0 * mbus_cb.udata.u32_value / pow(10, mbus_cb.channel.divisor_p10 );

	} else if (mbus_cb.channel.type == i32 ) {
		pvalue = 1.0 * mbus_cb.udata.i32_value / pow(10, mbus_cb.channel.divisor_p10 );

	}

quit:

	xSemaphoreGive( sem_MBUS );

	modbus_print( DF_MBUS, &mbus_cb );

	xprintf_PD(DF_MBUS, PSTR("MODBUS: CHPOLL END (%.3f s.)\r\n"), ELAPSED_TIME_SECS(init_ticks) );


	return( pvalue );

}
//------------------------------------------------------------------------------------
void modbus_write_output_register( char *s_slaaddr,char *s_regaddr,char *s_nro_regs,char *s_fcode, char *s_type,char *s_codec, char *s_value )
{
	xprintf_P(PSTR("MODBUS: OUTREG START\r\n"));

	if ( ! is_aux_prendido()) {
		aux_prender();
	}

	while ( xSemaphoreTake( sem_MBUS, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	mbus_cb.channel.slave_address = atoi(s_slaaddr);
	mbus_cb.channel.reg_address = atoi(s_regaddr);
	mbus_cb.channel.nro_regs = atoi(s_nro_regs);
	mbus_cb.channel.fcode = atoi(s_fcode);

	// TYPE
	if ( strcmp ( strupr(s_type), "U16" ) == 0 ) {
		mbus_cb.channel.type = u16;
	} else 	if ( strcmp ( strupr(s_type), "I16" ) == 0 ) {
		mbus_cb.channel.type = i16;
	} else 	if ( strcmp ( strupr(s_type), "I32" ) == 0 ) {
		mbus_cb.channel.type = i32;
	} else 	if ( strcmp ( strupr(s_type), "U32" ) == 0 ) {
		mbus_cb.channel.type = u32;
	} else 	if ( strcmp ( strupr(s_type), "FLOAT" ) == 0 ) {
		mbus_cb.channel.type = FLOAT;
	} else {
		return;
	}

	// CODEC
	if ( strcmp ( strupr(s_codec), "C0123" ) == 0 ) {
		mbus_cb.channel.codec = CODEC0123;
	} else 	if ( strcmp ( strupr(s_codec), "C1032" ) == 0 ) {
		mbus_cb.channel.codec = CODEC1032;
	} else 	if ( strcmp ( strupr(s_codec), "C3210" ) == 0 ) {
		mbus_cb.channel.codec = CODEC3210;
	} else 	if ( strcmp ( strupr(s_codec), "C2301" ) == 0 ) {
		mbus_cb.channel.codec = CODEC2301;
	} else {
		return;
	}

	mbus_cb.channel.divisor_p10 = 0;

	switch ( mbus_cb.channel.type) {
	case i16:
		mbus_cb.udata.i16_value = atoi(s_value);
		break;
	case u16:
		mbus_cb.udata.u16_value = atoi(s_value);
		break;
	case u32:
		mbus_cb.udata.u32_value = atol(s_value);
		break;
	case i32:
		mbus_cb.udata.i32_value = atol(s_value);
		break;
	case FLOAT:
		mbus_cb.udata.float_value = atof(s_value);
		break;
	default:
		return;
	}

	modbus_io( true, &mbus_cb );
	xSemaphoreGive( sem_MBUS );

	modbus_print( true, &mbus_cb );
	xprintf_P(PSTR("MODBUS: OUTREG END\r\n"));
}
//------------------------------------------------------------------------------------
void modbus_report_status(uint8_t bit_pos, uint8_t bit_value )
{
	/*
	 * Transmite el status al canal de control del PLC.
	 * Cada funcion que la invoque debe prender un bit diferente para que el PLC
	 * sepa cual fue.
	 * comms_link: bit 0.
	 *
	 * Cuando se invoca, se tranmite todo el status_word.
	 *
	 */

static uint16_t status_word = 0x00;

	xprintf_P(PSTR("MODBUS: REPORT STATUS START\r\n"));

	if (  modbus_conf.control_channel.slave_address == 0 ) {
		return;
	}

	if ( ! is_aux_prendido()) {
		aux_prender();
	}


	while ( xSemaphoreTake( sem_MBUS, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	mbus_cb.channel.slave_address = modbus_conf.control_channel.slave_address;
	mbus_cb.channel.reg_address = modbus_conf.control_channel.reg_address;
	mbus_cb.channel.nro_regs = 1;	// 1 registro = 2 bytes
	mbus_cb.channel.fcode = 6;		// Escribimos
	// TYPE
	mbus_cb.channel.type = u16;
	// CODEC
	mbus_cb.channel.codec = CODEC3210;
	mbus_cb.channel.divisor_p10 = 0;

	// Prendo o apago el bit de status.
	if ( bit_value == 0 ) {
		status_word &= ~(1 << bit_pos);
	} else {
		status_word |= (1 << bit_pos);
	}
	mbus_cb.udata.u16_value = status_word;

	// Transmito
	modbus_io( true, &mbus_cb );
	xSemaphoreGive( sem_MBUS );

	modbus_print( true, &mbus_cb );

	xprintf_P(PSTR("MODBUS: REPORT STATUS END\r\n"));
}
// CONFIGURACION
//------------------------------------------------------------------------------------
void modbus_config_defaults(void)
{

uint8_t i;

	for ( i = 0; i < MODBUS_CHANNELS; i++ ) {
		strcpy( modbus_conf.channel[i].name, "X");
		modbus_conf.channel[i].slave_address = 0;
		modbus_conf.channel[i].reg_address = 0;
		modbus_conf.channel[i].type = u16;
		modbus_conf.channel[i].codec = CODEC0123;
		modbus_conf.channel[i].nro_regs = 1;
		modbus_conf.channel[i].divisor_p10 = 0;
		modbus_conf.channel[i].fcode = 3;
	}

	modbus_conf.waiting_poll_time = 1;
	modbus_conf.control_channel.slave_address = 0;
	modbus_conf.control_channel.reg_address = 0;

}
//------------------------------------------------------------------------------------
void modbus_config_status(void)
{

uint8_t i;

	xprintf_P( PSTR(">Modbus: ( name,address,nro_regs,rcode,type,factor_p10 )\r\n"));

	xprintf_P( PSTR("   wpTime=%d\r\n"), modbus_conf.waiting_poll_time );
	xprintf_P( PSTR("   controlSlave=0x%02x, controlAddress=0x%02x\r\n"), modbus_conf.control_channel.slave_address,modbus_conf.control_channel.reg_address  );

	for ( i = 0; i < MODBUS_CHANNELS; i++ ) {

		if ( strcmp ( modbus_conf.channel[i].name, "X" ) == 0 ) {
			xprintf_P(PSTR("\r\n"));
			return;
		}

		if ( (i % 2) == 0 ) {
			xprintf_P(PSTR("   "));
		}

		xprintf_P( PSTR("MB%02d:[%s,0x%02X,0x%04X,"),
				i,
				modbus_conf.channel[i].name,
				modbus_conf.channel[i].slave_address,
				modbus_conf.channel[i].reg_address
		);
		xprintf_P( PSTR("0x%X,0x%X,"), modbus_conf.channel[i].nro_regs, modbus_conf.channel[i].fcode );

		switch(	modbus_conf.channel[i].type ) {
		case u16:
			xprintf_P(PSTR("U16,"));
			break;
		case i16:
			xprintf_P(PSTR("I16,"));
			break;
		case u32:
			xprintf_P(PSTR("U32,"));
			break;
		case i32:
			xprintf_P(PSTR("I32,"));
			break;
		case FLOAT:
			xprintf_P(PSTR("FLT,"));
			break;
		}

		switch(	modbus_conf.channel[i].codec ) {
		case CODEC0123:
			xprintf_P(PSTR("c0123,"));
			break;
		case CODEC1032:
			xprintf_P(PSTR("c1032,"));
			break;
		case CODEC3210:
			xprintf_P(PSTR("c3210,"));
			break;
		case CODEC2301:
			xprintf_P(PSTR("c2301,"));
			break;
		}

		xprintf_P( PSTR("%d],"), modbus_conf.channel[i].divisor_p10	);

		if ( ((i+1) % 2) == 0 ) {
			xprintf_P(PSTR("\r\n"));
		}
	}

	xprintf_P(PSTR("\r\n"));

}
//------------------------------------------------------------------------------------
bool modbus_config_channel( uint8_t channel,char *s_name,char *s_sla,char *s_addr,char *s_nro_recds,char *s_fcode,char *s_type,char *s_codec,char *s_divisor_p10 )
{
	// Todos los datos vienen en decimal !!!

	/*
	 *
	 xprintf_P(PSTR("DEBUG MBCH (%02d): name:[%s],saddr:[%s],nrecs:[%s],fcode:[%s],type:[%s],codec:[%s],pow10:[%s]\r\n"),
			channel,
			s_name,
			s_addr,
			s_nro_recds,
			s_fcode,
			s_type,
			s_codec,
			s_divisor_p10 );
	*/

	if (( s_name == NULL ) || ( s_addr == NULL) || (s_sla == NULL) || (s_nro_recds == NULL) || ( s_fcode == NULL) || ( s_divisor_p10 == NULL)  ) {
		return(false);
	}

	// Tipo
	if ( !strcmp_P( strupr(s_type), PSTR("U16"))) {
		modbus_conf.channel[channel].type = u16;

	} else 	if ( !strcmp_P( strupr(s_type), PSTR("I16"))) {
		modbus_conf.channel[channel].type = i16;

	} else 	if ( !strcmp_P( strupr(s_type), PSTR("U32"))) {
		modbus_conf.channel[channel].type = u32;

	} else 	if ( !strcmp_P( strupr(s_type), PSTR("I32"))) {
		modbus_conf.channel[channel].type = i32;

	} else 	if ( !strcmp_P( strupr(s_type), PSTR("FLOAT"))) {
		modbus_conf.channel[channel].type = FLOAT;

	} else {
		xprintf_P(PSTR("MODBUS CONFIG ERROR: type [%s]\r\n"), s_type );
		return(false);
	}

	// Codec
	if ( !strcmp_P( strupr(s_codec), PSTR("C0123"))) {
		modbus_conf.channel[channel].codec = CODEC0123;

	} else 	if ( !strcmp_P( strupr(s_codec), PSTR("C1032"))) {
		modbus_conf.channel[channel].codec = CODEC1032;

	} else 	if ( !strcmp_P( strupr(s_codec), PSTR("C3210"))) {
		modbus_conf.channel[channel].codec = CODEC3210;

	} else 	if ( !strcmp_P( strupr(s_codec), PSTR("C2301"))) {
		modbus_conf.channel[channel].codec = CODEC2301;

	} else {
		xprintf_P(PSTR("MODBUS CONFIG ERROR: codec [%s]\r\n"), s_codec );
		return(false);
	}

	snprintf_P( modbus_conf.channel[channel].name, PARAMNAME_LENGTH, PSTR("%s\0"), s_name );
	modbus_conf.channel[channel].slave_address = atoi(s_sla);
	modbus_conf.channel[channel].reg_address = atoi(s_addr);
	modbus_conf.channel[channel].nro_regs = atoi(s_nro_recds);
	modbus_conf.channel[channel].fcode = atoi(s_fcode);
	modbus_conf.channel[channel].divisor_p10 = atoi(s_divisor_p10);
	return(true);

}
//------------------------------------------------------------------------------------
bool modbus_config_waiting_poll_time( char *s_waiting_poll_time)
{
	// Configura el tiempo que espera una respuesta

	modbus_conf.waiting_poll_time = atoi(s_waiting_poll_time);
	//xprintf_P(PSTR("DEBUG MBWT: [%s][%d]\r\n"), s_waiting_poll_time, modbus_conf.waiting_poll_time );
	return(true);
}
//------------------------------------------------------------------------------------
bool modbus_config_chcontrol ( char *sla_addr, char *reg_addr)
{
	modbus_conf.control_channel.slave_address = atoi(sla_addr);
	modbus_conf.control_channel.reg_address = atoi(reg_addr);
	return(true);

}
//------------------------------------------------------------------------------------
// AUXILIARES
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
	j += snprintf_P( &hash_buffer[j], free_size, PSTR("MODBUS;MBWT:%03d"),modbus_conf.waiting_poll_time );
	//xprintf_P(PSTR("DEBUG:[%s]\r\n"), hash_buffer);

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
		// Copio sobre el buffer una vista ascii ( imprimible ) de c/registro.
		j = 0;
		j += snprintf_P( &hash_buffer[j], free_size, PSTR(";MB%02d:%s,%02d,%04d,%02d,%02d,"), i,
							modbus_conf.channel[i].name,
							modbus_conf.channel[i].slave_address,
							modbus_conf.channel[i].reg_address,
							modbus_conf.channel[i].nro_regs,
							modbus_conf.channel[i].fcode
							);

		// TYPE
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;
		switch( modbus_conf.channel[i].type ) {
		case u16:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("U16,"));
			break;
		case i16:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("I16,"));
			break;
		case u32:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("U32,"));
			break;
		case i32:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("I32,"));
			break;
		case FLOAT:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("FLOAT,"));
			break;
		}

		// CODEC
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;
		switch( modbus_conf.channel[i].codec ) {
		case CODEC0123:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("C0123,"));
			break;
		case CODEC1032:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("C1032,"));
			break;
		case CODEC3210:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("C3210,"));
			break;
		case CODEC2301:
			j += snprintf_P( &hash_buffer[j], free_size, PSTR("C2301,"));
			break;
		}

		j += snprintf_P( &hash_buffer[j], free_size, PSTR("%02d"), modbus_conf.channel[i].divisor_p10 );

		//xprintf_P(PSTR("DEBUG:[%s]\r\n"), hash_buffer);

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
// FUNCIONES DE TEST
//------------------------------------------------------------------------------------
void modbus_test_genpoll(char *arg_ptr[16] )
{

	// Recibe el argv con los datos en char hex para trasmitir el frame.
	// El formato es: slaaddr,regaddr,nro_regs,fcode,type,codec
	// write modbus genpoll F 9 3 4118 2 c3212
	// TX: (len=8):[0x09][0x03][0x10][0x16][0x00][0x02][0x20][0x47]


	xprintf_P(PSTR("MODBUS: GENPOLL START\r\n"));

	if ( arg_ptr[8] == NULL ) {
		xprintf_P(PSTR("Error de argumentos\r\n"));
		return;
	}

	while ( xSemaphoreTake( sem_MBUS, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	mbus_cb.channel.slave_address = atoi(arg_ptr[3]);
	mbus_cb.channel.reg_address = atoi(arg_ptr[4]);
	mbus_cb.channel.nro_regs = atoi(arg_ptr[5]);
	mbus_cb.channel.fcode = atoi(arg_ptr[6]);

	// Preparo el registro con los datos del canal
	// TYPE
	if ( strcmp ( strupr(arg_ptr[7]), "U16" ) == 0 ) {
		mbus_cb.channel.type = u16;
	} else 	if ( strcmp ( strupr(arg_ptr[7]), "I16" ) == 0 ) {
		mbus_cb.channel.type = i16;
	} else 	if ( strcmp ( strupr(arg_ptr[7]), "I32" ) == 0 ) {
		mbus_cb.channel.type = i32;
	} else 	if ( strcmp ( strupr(arg_ptr[7]), "U32" ) == 0 ) {
		mbus_cb.channel.type = u32;
	} else 	if ( strcmp ( strupr(arg_ptr[7]), "FLOAT" ) == 0 ) {
		mbus_cb.channel.type = FLOAT;
	} else {
		xprintf_P(PSTR("ERROR: tipo no soportado\r\n"));
		return;
	}

	// CODEC
	if ( strcmp ( strupr(arg_ptr[8]), "C0123" ) == 0 ) {
		mbus_cb.channel.codec = CODEC0123;
	} else 	if ( strcmp ( strupr(arg_ptr[8]), "C1032" ) == 0 ) {
		mbus_cb.channel.codec = CODEC1032;
	} else 	if ( strcmp ( strupr(arg_ptr[8]), "C3210" ) == 0 ) {
		mbus_cb.channel.codec = CODEC3210;
	} else 	if ( strcmp ( strupr(arg_ptr[8]), "C2301" ) == 0 ) {
		mbus_cb.channel.codec = CODEC2301;
	} else {
		xprintf_P(PSTR("ERROR: codec no soportado\r\n"));
		return;
	}

	mbus_cb.channel.divisor_p10 = 0;
	//
	modbus_io( true, &mbus_cb );
	//
	xSemaphoreGive( sem_MBUS );
	//
	modbus_print( true, &mbus_cb );
	xprintf_P(PSTR("MODBUS: GENPOLL END\r\n"));

}
//------------------------------------------------------------------------------------
void modbus_test_chpoll(char *s_channel)
{
	// Hace un poleo de un canal modbus definido en el datalogger

uint8_t ch;

	ch = atoi(s_channel);

	if ( ch >= MODBUS_CHANNELS ) {
		xprintf_P(PSTR("ERROR: Nro.canal < %d\r\n"), MODBUS_CHANNELS);
		return;
	}

	if ( strcmp ( modbus_conf.channel[ch].name, "X" ) == 0 ) {
		xprintf_P(PSTR("ERROR: Canal no definido (X)\r\n"));
		return;
	}

	modbus_read_channel(ch);

}
//------------------------------------------------------------------------------------
void modbus_test_float( char *s_nbr )
{
	// Funcion de testing que lee un string representando a un float
	// y lo imprime como float y como los 4 bytes

union {
	float fvalue;
	uint8_t bytes_raw[4];
} rec;

	xprintf_P(PSTR("MODBUS FLOAT:\r\n"));

	rec.fvalue = atof(s_nbr);
	xprintf_P(PSTR("Str: %s\r\n"), s_nbr);
	xprintf_P(PSTR("Float=%f\r\n"), rec.fvalue);
	xprintf_P(PSTR("Bytes=b3[0x%02x] b2[0x%02x] b1[0x%02x] b0[0x%02x]\r\n"), rec.bytes_raw[3], rec.bytes_raw[2], rec.bytes_raw[1], rec.bytes_raw[0]);

}
//------------------------------------------------------------------------------------
void modbus_test_long( char *s_nbr )
{
	// Funcion de testing que lee un string representando a un integer
	// y lo imprime como int y como los 4 bytes

union {
	uint32_t ivalue;
	uint8_t bytes_raw[4];
} rec;

	xprintf_P(PSTR("MODBUS LONG:\r\n"));

	rec.ivalue = atol(s_nbr);
	xprintf_P(PSTR("Str: %s\r\n"), s_nbr);
	xprintf_P(PSTR("long=%lu\r\n"), rec.ivalue);
	xprintf_P(PSTR("Bytes=b3[0x%02x] b2[0x%02x] b1[0x%02x] b0[0x%02x]\r\n"), rec.bytes_raw[3], rec.bytes_raw[2], rec.bytes_raw[1], rec.bytes_raw[0]);

}
//-----------------------------------------------------------------------------------
void modbus_test_int( char *s_nbr )
{
	// Funcion de testing que lee un string representando a un integer
	// y lo imprime como int y como los 4 bytes

union {
	uint16_t ivalue;
	uint8_t bytes_raw[4];
} rec;

	xprintf_P(PSTR("MODBUS INT:\r\n"));

	rec.ivalue = atoi(s_nbr);
	xprintf_P(PSTR("Str: %s\r\n"), s_nbr);
	xprintf_P(PSTR("Int=%d\r\n"), rec.ivalue);
	xprintf_P(PSTR("Bytes=b3[0x%02x] b2[0x%02x] b1[0x%02x] b0[0x%02x]\r\n"), rec.bytes_raw[3], rec.bytes_raw[2], rec.bytes_raw[1], rec.bytes_raw[0]);

}
//-----------------------------------------------------------------------------------
// PROCESO
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
void modbus_io( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{
	//
	mbus_cb->io_status = false;
	//
	pv_modbus_make_ADU (mbus_cb);
	pv_modbus_txmit_ADU( f_debug, mbus_cb );

	// Espero la respuesta
	vTaskDelay( (portTickType)( systemVars.modbus_conf.waiting_poll_time / portTICK_RATE_MS ) );

	pv_modbus_rcvd_ADU( f_debug, mbus_cb );
	pv_modbus_decode_ADU ( f_debug, mbus_cb );

	return;
}
//------------------------------------------------------------------------------------
void pv_modbus_make_ADU( mbus_CONTROL_BLOCK_t *mbus_cb )
{
	/*
	 * Prepara el ADU ( raw frame ) a trasnmitir con los datos de la variable global mbus_cb.
	 * Calcula el CRC de modo que el frame queda pronto a transmitirse por el serial
	 * Solo utilizamos los mensajes tipo 3,6,10
	 */

uint8_t size = 0;
uint16_t crc;

	// Header:
	memset( mbus_cb->tx_buffer, '\0', MBUS_TXMSG_LENGTH );
	mbus_cb->tx_buffer[0] = mbus_cb->channel.slave_address;								// SLA
	mbus_cb->tx_buffer[1] = mbus_cb->channel.fcode;										// FCODE
	mbus_cb->tx_buffer[2] = (uint8_t) (( mbus_cb->channel.reg_address & 0xFF00  ) >> 8);// DST_ADDR_H
	mbus_cb->tx_buffer[3] = (uint8_t) ( mbus_cb->channel.reg_address & 0x00FF );		// DST_ADDR_L

	// Payload:
	if ( mbus_cb->channel.fcode == 0x03 ) {
		// READ HOLDING REGISTER
		// Hago el ADU de un frame tipo 0x03 ( Read Holding Register )
		mbus_cb->tx_buffer[4] = (uint8_t) ((mbus_cb->channel.nro_regs & 0xFF00 ) >> 8);	// NRO_REG_HI
		mbus_cb->tx_buffer[5] = (uint8_t) ( mbus_cb->channel.nro_regs & 0x00FF );		// NRO_REG_LO
		size = 6;

	} else if ( mbus_cb->channel.fcode == 0x06 ) {
		// WRITE SINGLE REGISTER
		// Hago el ADU de un frame tipo 0x06 ( Write Single Register )
		// El valor a escribir esta en mbus_cb.udata.
		// Escribo solo 2 bytes con esta funcion !!! (i16,u16)
		switch( mbus_cb->channel.codec ) {
		case CODEC0123:
			pv_encoder_f6_c0123( mbus_cb );
			break;
		case CODEC1032:
			pv_encoder_f6_c1032( mbus_cb );
			break;
		case CODEC3210:
			pv_encoder_f6_c3210( mbus_cb );
			break;
		case CODEC2301:
			pv_encoder_f6_c2301( mbus_cb );
			break;
		default:
			return;
		}
		size = 6;

	} else if ( mbus_cb->channel.fcode == 16 ) {
		// WRITE MULTIPLE REGISTER
		// Hago el ADU de un frame tipo 0x10 (16) ( Write Multitple Register )
		// El valor a escribir esta en mbus_cb.udata(float)
		// Escribo 4 bytes.
		// El simulador recibe los bytes en orden.
		// Quantity of Registers: 2 bytes, ( 1 registro o 1 variable de largo 4  )
		// Escribo solo 2 bytes con esta funcion !!! (i16,u16)
		switch( mbus_cb->channel.codec ) {
		case CODEC0123:
			pv_encoder_f16_c0123( mbus_cb );
			break;
		case CODEC1032:
			pv_encoder_f16_c1032( mbus_cb );
			break;
		case CODEC3210:
			pv_encoder_f16_c3210( mbus_cb );
			break;
		case CODEC2301:
			pv_encoder_f16_c2301( mbus_cb );
			break;
		default:
			return;
		}
		size = 11;

	} else {
		// Codigo no implementado
		return;
	}

	// CRC
	crc = pv_modbus_CRC16( mbus_cb->tx_buffer, size );
	mbus_cb->tx_buffer[size++] = (uint8_t)( crc & 0x00FF );			// CRC Low
	mbus_cb->tx_buffer[size++] = (uint8_t)( (crc & 0xFF00) >> 8 );	// CRC High
	mbus_cb->tx_size = size;
	return;

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

	//xprintf_PD( f_debug, PSTR("MODBUS: TX start\r\n"));

	// Log
	if ( f_debug ) {
		xprintf_P( PSTR("MODBUS: TX (len=%d)"), mbus_cb->tx_size);
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

	// La funcion xnprintf_MBUS maneja el control de flujo.
	i = xnprintf_MBUS( (const char *)mbus_cb->tx_buffer, mbus_cb->tx_size );

	//xprintf_PD( f_debug, PSTR("MODBUS: TX end(%d):\r\n"),i);
}
//------------------------------------------------------------------------------------
void pv_modbus_rcvd_ADU( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
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

	/*
	 *  Para ser efectivo en cuanto espero, el procedimiento es el siguiente:
	 *  Espero 100 ms iniciales
	 *  Leo el aux_buffer.ptr y lo guardo.
	 *  Cada 100 ms leo de nuevo el ptr. Si cambio es que estan viniendo datos.
	 *  Si no cambio, salgo.
	 *
	 */

uint16_t crc_rcvd;
uint16_t crc_calc;
uint8_t i;
int8_t lBytes, rBytes;
int8_t timeout_counts;

	//xprintf_PD( f_debug, PSTR("MODBUS: RX start\r\n"));

	rBytes = -1;
	lBytes = -1;
	timeout_counts = 20;	// 20 x 50ms: espero 1 sec

	while ( timeout_counts-- > 0 ) {

		vTaskDelay( (portTickType)( 50 / portTICK_RATE_MS ) );

		// Vemos si vinieron mas caracteres.
		rBytes = aux_get_RX_buffer_ptr();
		if ( rBytes != lBytes ) {
			lBytes = rBytes;
		} else {
			// No se movio. pero ya recibi algo: Salgo
			if (lBytes > 0) {
				break;
			}
		}
	}

	// Leo el aux_rx_buffer y lo dejo en el modbus_rx_buffer
	memset( mbus_cb->rx_buffer, '\0', MBUS_RXMSG_LENGTH );
	aux_rxbuffer_copyto( mbus_cb->rx_buffer, &mbus_cb->rx_size, MBUS_RXMSG_LENGTH );
	xprintf_PD( f_debug, PSTR("MODBUS: RX rx_size=%d\r\n"), mbus_cb->rx_size );

	// Paso 1: Log
	if (f_debug) {
		xprintf_P( PSTR("MODBUS: RX (len=%d):"), mbus_cb->rx_size);
		for ( i = 0 ; i < mbus_cb->rx_size; i++ ) {
			xprintf_P( PSTR("[0x%02X]"), mbus_cb->rx_buffer[i]);
		}
		xprintf_P( PSTR("\r\n"));
	}

	// Paso 2: Controlo el largo.
	if ( mbus_cb->rx_size < 3) {
		// Timeout error:
		xprintf_PD( f_debug, PSTR("MODBUS: RX TIMEOUT ERROR\r\n\0"));
		mbus_cb->io_status = false;
		goto quit;
	}

	// Pass3: Calculo y verifico el CRC
	crc_calc = pv_modbus_CRC16( mbus_cb->rx_buffer, (mbus_cb->rx_size - 2) );
	crc_rcvd = mbus_cb->rx_buffer[mbus_cb->rx_size - 2] + ( mbus_cb->rx_buffer[mbus_cb->rx_size - 1] << 8 );

	if ( crc_calc != crc_rcvd) {
		xprintf_PD( f_debug, PSTR("MODBUS: RX CRC ERROR: rx[0x%02x], calc[0x%02x]\r\n\0"), crc_rcvd, crc_calc);
		// De acuerdo al protocolo se ignora el frame recibido con errores CRC
		mbus_cb->io_status = false;
		goto quit;
	}

	mbus_cb->io_status = true;

quit:

	//xprintf_PD( f_debug, PSTR("MODBUS: RX end\r\n") );
	return;

}
//------------------------------------------------------------------------------------
void pv_modbus_decode_ADU ( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{

	/*
	 * Decodifica c/tipo de respuesta en base al function_code y tipo.
	 * Invoca a los decoders particulares.
	 *
	 */

	//xprintf_PD( f_debug, PSTR("MODBUS: DECODE start\r\n"));

	// Proceso los errores escribiendo un NaN
	if ( mbus_cb->io_status == false ) {
		mbus_cb->udata.raw_value[0] = 0xFF;
		mbus_cb->udata.raw_value[1] = 0xFF;
		mbus_cb->udata.raw_value[2] = 0xFF;
		mbus_cb->udata.raw_value[3] = 0xFF;
		goto quit;
	}

	if ( mbus_cb->channel.fcode == 0x03 ) {

		switch( mbus_cb->channel.codec ) {
		case CODEC0123:
			pv_decoder_f3_c0123(f_debug, mbus_cb );
			break;
		case CODEC1032:
			pv_decoder_f3_c1032(f_debug, mbus_cb );
			break;
		case CODEC3210:
			pv_decoder_f3_c3210(f_debug, mbus_cb );
			break;
		case CODEC2301:
			pv_decoder_f3_c2301(f_debug, mbus_cb );
			break;
		default:
			return;
		}


	} else if ( mbus_cb->channel.fcode == 0x06 ) {
		// No se analiza la respuesta ya que es echo
		return;

	} else if ( mbus_cb->channel.fcode == 0x10 ) {
		// No se analiza la respuesta ya que es echo
		return;
	} else {
		// Caso no implementado
		return;
	}

quit:

	xprintf_PD(f_debug, PSTR("MODBUS: DECODE (MSB)b3[0x%02X] b2[0x%02X] b1[0x%02X] b0[0x%02X](LSB)\r\n"),
			mbus_cb->udata.raw_value[3], mbus_cb->udata.raw_value[2], mbus_cb->udata.raw_value[1], mbus_cb->udata.raw_value[0]);

	//xprintf_PD( f_debug, PSTR("MODBUS: DECODE end\r\n") );
	return;

}
//------------------------------------------------------------------------------------
// PRINT
//------------------------------------------------------------------------------------
void modbus_print( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{

	// Imprime en la terminal el valor leido en una operacion de modbus.
	// Se usa en las funciones de mbustest.

float pvalue;

	if ( f_debug ) {

		if ( mbus_cb->io_status == false ) {
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE NaN\r\n"));
			return;
		}

		// La operacion de io es exitosa.
		if ( mbus_cb->channel.type == FLOAT) {
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %.03f (F)\r\n"), mbus_cb->udata.float_value);

		} else if (mbus_cb->channel.type == i16 ) {
			// En este caso importa el parametro divisor_p10 ya que convierte al entero en float
			pvalue = 1.0 * mbus_cb->udata.i16_value / pow(10, mbus_cb->channel.divisor_p10 );
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %d (i16), %.03f\r\n"), mbus_cb->udata.i16_value, pvalue );

		} else if (mbus_cb->channel.type == u16 ) {
			pvalue = 1.0 * mbus_cb->udata.u16_value / pow(10, mbus_cb->channel.divisor_p10 );
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %u (u16), %.03f\r\n"), mbus_cb->udata.u16_value, pvalue );

		} else if (mbus_cb->channel.type == u32 ) {
			pvalue = 1.0 * ( mbus_cb->udata.u32_value ) / pow(10, mbus_cb->channel.divisor_p10 );
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %lu (u32), %.03f\r\n"), mbus_cb->udata.u32_value, pvalue );

		} else if (mbus_cb->channel.type == i32 ) {
			pvalue = 1.0 * ( mbus_cb->udata.i32_value ) / pow(10, mbus_cb->channel.divisor_p10 );
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %ld (i32), %.03f\r\n"), mbus_cb->udata.i32_value, pvalue );
		}


	}
}
//------------------------------------------------------------------------------------
void modbus_data_print( file_descriptor_t fd, float mbus_data[] )
{
	// Muestra el valor de los canales leidos en la terminal
	// Todos los canales son float
	// Es usado por la tkData

uint8_t ch;
union {
	float value;
	uint8_t raw[4];	// Almaceno NaN (FF FF FF FF ) cuando hay un error.
} data;

	for ( ch = 0; ch < MODBUS_CHANNELS; ch++) {
		// Si un canal no esta definido, salgo
		if ( strcmp ( modbus_conf.channel[ch].name, "X" ) == 0 ) {
			break;
		}

		if ( modbus_conf.channel[ch].slave_address == 0x00) {
			return;
		}

		//
		data.value = mbus_data[ch];
		if ( ( data.raw[0] == 0xFF) && ( data.raw[1] == 0xFF) && ( data.raw[2] == 0xFF) && ( data.raw[3] == 0xFF)) {
			xfprintf_P(fd, PSTR("%s:NaN;"), modbus_conf.channel[ch].name );
		} else {
			xfprintf_P(fd, PSTR("%s:%.02f;"), modbus_conf.channel[ch].name, mbus_data[ch] );
		}
	}
}
//------------------------------------------------------------------------------------
char *modbus_sprintf( char *sbuffer, float src[] )
{
	// Imprime los canales configurados ( no X ) en un fd ( tty_gprs,tty_xbee,tty_term) en
	// forma formateada.
	// Los lee de una estructura array pasada como src
	// Es usado por tkData en data_sprintf_inputs que es cuando tkDatos imprime los datos en
	// en gprs_txbuffer.

uint8_t i = 0;
int16_t pos = 0;
char *p;
union {
	float value;
	uint8_t raw[4];	// Almaceno NaN (FF FF FF FF ) cuando hay un error.
} data;

	p = sbuffer;
	for ( i = 0; i < MODBUS_CHANNELS; i++) {
		// Si un canal no esta definido, salgo
		if ( strcmp ( modbus_conf.channel[i].name, "X" ) == 0 ) {
			break;
		}

		//
		data.value = src[i];
		if ( ( data.raw[0] == 0xFF) && ( data.raw[1] == 0xFF) && ( data.raw[2] == 0xFF) && ( data.raw[3] == 0xFF)) {
			pos = sprintf_P( p, PSTR("%s:NaN;"), modbus_conf.channel[i].name );
		} else {
			pos = sprintf_P( p, PSTR("%s:%.02f;"), modbus_conf.channel[i].name, src[i] );
		}
		p += pos;

	}

	return(p);

}
//------------------------------------------------------------------------------------
// DECODERS
/*
 *
 * https://betterexplained.com/articles/understanding-big-and-little-endian-byte-order/
 *
 * En la decodificacion de los enteros no hay problema porque los flowmeters y los PLC
 * mandan el MSB primero.
 * En el caso de los float, solo los mandan los PLC y ahi hay un swap de los bytes por
 * lo que para decodificarlo necesito saber que los 4 bytes corresponden a un float!!
 *
 * MODBUS es BIG_ENDIAN: Cuando se manda un nro de varios bytes, el MSB se manda primero
 * MSB ..... LSB
 * Big Endian: El MSB se pone en el byte de la direccion mas baja:
 * Little Endian: El LSB se pone en el byte de la direccion mas baja.
 *
 * El decoder indica el orden de los bytes recibidos.
 * Al recibirlos van a una memoria del datalogger que codifica el LSB en la posicion mas baja (b0)
 * El codec debe indicar en que orden se reciben los bytes.
 * Ej: c3210 indica que primero se recibe el byte3, byte2, byte1 y byte0
 *
 */
//------------------------------------------------------------------------------------
void pv_decoder_f3_c3210(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{
	/*
	 * c3210 significa decodificar los bytes en el orden natural.
	 * En una variable de memoria del datalogger, un valor de varios bytes se almacena
	 * con el LSB en la posicion 0.
	 * Cuando llegan los bytes por Modbus, el ultimo byte del payload es el LSB
	 * En el string del payload, la posicion ptr0 -> byte3, ptr1->byte2, ptr2->byte1, ptr0->byte0
	 */
uint8_t payload_ptr = 3;	// Posicion en el rxbuffer donde empieza el payload

	xprintf_PD(f_debug, PSTR("MODBUS: codec_f3_c3210\r\n"));
	if ( ( mbus_cb->channel.type == u16 ) || ( mbus_cb->channel.type == i16) ) {
		// El payload tiene 2 bytes.
		mbus_cb->udata.raw_value[3] = 0x00;
		mbus_cb->udata.raw_value[2] = 0x00;
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 0];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 1];
	} else {
		// El payload tiene 4 bytes.
		mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr + 0];
		mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 1];
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 2];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 3];
	}
	return;
}
//------------------------------------------------------------------------------------
void pv_decoder_f3_c0123(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{
	/*
	 * c4321 significa decodificar los bytes en orden inverso.
	 * La referencia es la memoria del datalogger que para interpretar un valor, requiere
	 * que el LSB se almacene en la posicion mas baja (b0)
	 * En esta codificacion, lo que hago es poner los datos que me llegan del modbus
	 * en un orden inverso al orden natural ( c0123 )
	 * En el string del payload, la posicion ptr0 -> byte0, ptr1->byte1, ptr2->byte2, ptr0->byte3
	 *
	 */

uint8_t payload_ptr = 3;	// Posicion en el rxbuffer donde empieza el payload

	xprintf_PD(f_debug, PSTR("MODBUS: codec_f3_c0123\r\n"));
	if ( ( mbus_cb->channel.type == u16 ) || ( mbus_cb->channel.type == i16) ) {
		mbus_cb->udata.raw_value[3] = 0x00;
		mbus_cb->udata.raw_value[2] = 0x00;
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 1];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 0];

	} else {
		mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr + 3];
		mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 2];
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 1];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 0];
	}
	return;

}
//------------------------------------------------------------------------------------
void pv_decoder_f3_c1032(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{

uint8_t payload_ptr = 3;	// Posicion en el rxbuffer donde empieza el payload

	xprintf_PD(f_debug, PSTR("MODBUS: codec_f3_c1032\r\n"));
	if ( ( mbus_cb->channel.type == u16 ) || ( mbus_cb->channel.type == i16) ) {
		mbus_cb->udata.raw_value[3] = 0x00;
		mbus_cb->udata.raw_value[2] = 0x00;
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 0];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 1];
	} else {
		mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr + 2];
		mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 3];
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 0];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 1];
	}
	return;
}
//------------------------------------------------------------------------------------
void pv_decoder_f3_c2301(bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb )
{

uint8_t payload_ptr = 3;	// Posicion en el rxbuffer donde empieza el payload

	xprintf_PD(f_debug, PSTR("MODBUS: codec_f3_c2301\r\n"));
	if ( ( mbus_cb->channel.type == u16 ) || ( mbus_cb->channel.type == i16) ) {
		mbus_cb->udata.raw_value[3] = 0x00;
		mbus_cb->udata.raw_value[2] = 0x00;
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 1];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 0];

	} else {
		mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr + 1];
		mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 0];
		mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 3];
		mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 2];
	}
	return;
}
//------------------------------------------------------------------------------------
/*
 * ENCODERS
 * Dado una variable int,long,float en la memoria del datalogger (b0=LSB), debo
 * ver como reordeno los bytes para transmitirlos al modbus.
 * En la funcion 0x06 solo se escriben 2 bytes.
 */
//------------------------------------------------------------------------------------
void pv_encoder_f6_c0123(mbus_CONTROL_BLOCK_t *mbus_cb )
{

	mbus_cb->tx_buffer[4] = mbus_cb->udata.raw_value[0];	// DATA_VAL_HI
	mbus_cb->tx_buffer[5] = mbus_cb->udata.raw_value[1];	// DATA_VAL_LO
	return;
}
//------------------------------------------------------------------------------------
void pv_encoder_f6_c2301(mbus_CONTROL_BLOCK_t *mbus_cb )
{

	mbus_cb->tx_buffer[4] = mbus_cb->udata.raw_value[0];	// DATA_VAL_HI
	mbus_cb->tx_buffer[5] = mbus_cb->udata.raw_value[1];	// DATA_VAL_LO
	return;
}
//------------------------------------------------------------------------------------
void pv_encoder_f6_c3210(mbus_CONTROL_BLOCK_t *mbus_cb )
{

	mbus_cb->tx_buffer[4] = mbus_cb->udata.raw_value[1];	// DATA_VAL_HI
	mbus_cb->tx_buffer[5] = mbus_cb->udata.raw_value[0];	// DATA_VAL_LO
	return;
}
//------------------------------------------------------------------------------------
void pv_encoder_f6_c1032(mbus_CONTROL_BLOCK_t *mbus_cb )
{

	mbus_cb->tx_buffer[4] = mbus_cb->udata.raw_value[1];	// DATA_VAL_HI
	mbus_cb->tx_buffer[5] = mbus_cb->udata.raw_value[0];	// DATA_VAL_LO
	return;
}
//------------------------------------------------------------------------------------
void pv_encoder_f16_c0123(mbus_CONTROL_BLOCK_t *mbus_cb )
{

uint8_t payload_ptr = 7;

	mbus_cb->tx_buffer[4] = 0;
	mbus_cb->tx_buffer[5] = 2;
	//
	// Byte count ,1 byte ( 1 float = 4 bytes , 2 x N => N=2)
	mbus_cb->tx_buffer[6] = 4;
	// LA POSICION ES FIJA EN WRITE: LA ACEPTADA POR LOS PLC KINCO
	// El PCL recibe los bytes en diferente orden !!!.
	mbus_cb->tx_buffer[ payload_ptr + 0 ] = mbus_cb->udata.raw_value[0];	// DATA
	mbus_cb->tx_buffer[ payload_ptr + 1 ] = mbus_cb->udata.raw_value[1];
	mbus_cb->tx_buffer[ payload_ptr + 2 ] = mbus_cb->udata.raw_value[2];
	mbus_cb->tx_buffer[ payload_ptr + 3 ] = mbus_cb->udata.raw_value[3];

	return;
}
//------------------------------------------------------------------------------------
void pv_encoder_f16_c2301(mbus_CONTROL_BLOCK_t *mbus_cb )
{

uint8_t payload_ptr = 7;

	mbus_cb->tx_buffer[4] = 0;
	mbus_cb->tx_buffer[5] = 2;
	//
	// Byte count ,1 byte ( 1 float = 4 bytes , 2 x N => N=2)
	mbus_cb->tx_buffer[6] = 4;
	mbus_cb->tx_buffer[ payload_ptr + 0 ] = mbus_cb->udata.raw_value[2];	// DATA
	mbus_cb->tx_buffer[ payload_ptr + 1 ] = mbus_cb->udata.raw_value[3];
	mbus_cb->tx_buffer[ payload_ptr + 2 ] = mbus_cb->udata.raw_value[0];
	mbus_cb->tx_buffer[ payload_ptr + 3 ] = mbus_cb->udata.raw_value[1];

	return;
}
//------------------------------------------------------------------------------------
void pv_encoder_f16_c3210(mbus_CONTROL_BLOCK_t *mbus_cb )
{

uint8_t payload_ptr = 7;

	mbus_cb->tx_buffer[4] = 0;
	mbus_cb->tx_buffer[5] = 2;
	//
	// Byte count ,1 byte ( 1 float = 4 bytes , 2 x N => N=2)
	mbus_cb->tx_buffer[6] = 4;
	mbus_cb->tx_buffer[ payload_ptr + 0 ] = mbus_cb->udata.raw_value[3];	// DATA
	mbus_cb->tx_buffer[ payload_ptr + 1 ] = mbus_cb->udata.raw_value[2];
	mbus_cb->tx_buffer[ payload_ptr + 2 ] = mbus_cb->udata.raw_value[1];
	mbus_cb->tx_buffer[ payload_ptr + 3 ] = mbus_cb->udata.raw_value[0];

	return;
}
//------------------------------------------------------------------------------------
void pv_encoder_f16_c1032(mbus_CONTROL_BLOCK_t *mbus_cb )
{

uint8_t payload_ptr = 7;

	mbus_cb->tx_buffer[4] = 0;
	mbus_cb->tx_buffer[5] = 2;
	//
	// Byte count ,1 byte ( 1 float = 4 bytes , 2 x N => N=2)
	mbus_cb->tx_buffer[6] = 4;
	mbus_cb->tx_buffer[ payload_ptr + 0 ] = mbus_cb->udata.raw_value[1];	// DATA
	mbus_cb->tx_buffer[ payload_ptr + 1 ] = mbus_cb->udata.raw_value[0];
	mbus_cb->tx_buffer[ payload_ptr + 2 ] = mbus_cb->udata.raw_value[3];
	mbus_cb->tx_buffer[ payload_ptr + 3 ] = mbus_cb->udata.raw_value[2];

	return;
}
//------------------------------------------------------------------------------------
