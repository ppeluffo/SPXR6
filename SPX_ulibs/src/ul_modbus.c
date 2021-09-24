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

void pv_modbus_make_ADU_code3( mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_make_ADU_code6( mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_make_ADU_code16_mbsim( mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_make_ADU_code16_kinco( mbus_CONTROL_BLOCK_t *mbus_cb );

void pv_modbus_codec_mbsim(bool f_debug, int8_t codec, mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_codec_kinco(bool f_debug, int8_t codec, mbus_CONTROL_BLOCK_t *mbus_cb );

//------------------------------------------------------------------------------------
// READ
//------------------------------------------------------------------------------------
void modbus_read( float mbus_data[] )
{
	// Lee todos los canales modbus configurados y deja los valores en el correspondiente
	// luegar del array del parametro.
	// Los nombres de los canales deben ser #X y estar en order. Al primer canal X, sale. !!!

uint8_t ch;

	// Si no tengo canales configurados, salgo.
	if ( modbus_conf.slave_address == 0x00 ) {
		return;
	}

	for ( ch = 0; ch < MODBUS_CHANNELS; ch++) {
		// Si un canal no esta definido, salgo
		if ( strcmp ( modbus_conf.channel[ch].name, "X" ) == 0 ) {
			break;
		}
		//
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
	mbus_cb.sla_address = modbus_conf.slave_address;
	mbus_cb.function_code = modbus_conf.channel[ch].rcode;
	mbus_cb.address = modbus_conf.channel[ch].address;
	mbus_cb.nro_recds = modbus_conf.channel[ch].nro_regs;
	mbus_cb.type = modbus_conf.channel[ch].type;
	mbus_cb.divisor_p10 = modbus_conf.channel[ch].divisor_p10;
	//
	modbus_io( DF_MBUS, &mbus_cb );
	//
	// Siempre retorno un float
	// La operacion de io es exitosa.
	if ( mbus_cb.type == FLOAT) {
		pvalue = mbus_cb.udata.float_value;

	} else if (mbus_cb.type == i16 ) {
		// En este caso importa el parametro divisor_p10 ya que convierte al entero en float
		pvalue = 1.0 * mbus_cb.udata.i16_value / pow(10, mbus_cb.divisor_p10 );

	} else if (mbus_cb.type == u16 ) {
		pvalue = 1.0 * mbus_cb.udata.u16_value / pow(10, mbus_cb.divisor_p10 );

	} else if (mbus_cb.type == u32 ) {
		pvalue = 1.0 * mbus_cb.udata.u32_value / pow(10, mbus_cb.divisor_p10 );

	} else if (mbus_cb.type == i32 ) {
		pvalue = 1.0 * mbus_cb.udata.i32_value / pow(10, mbus_cb.divisor_p10 );

	}

	xSemaphoreGive( sem_MBUS );

	modbus_print( DF_MBUS, &mbus_cb );

	xprintf_PD(DF_MBUS, PSTR("MODBUS: CHPOLL END (%.3f s.)\r\n"), ELAPSED_TIME_SECS(init_ticks) );


	return( pvalue );

}
//------------------------------------------------------------------------------------
// CONFIGURACION
//------------------------------------------------------------------------------------
void modbus_config_defaults(void)
{

uint8_t i;

	for ( i = 0; i < MODBUS_CHANNELS; i++ ) {
		strcpy( modbus_conf.channel[i].name, "X");
		modbus_conf.channel[i].address = 0;
		modbus_conf.channel[i].type = u16;
		modbus_conf.channel[i].nro_regs = 1;
		modbus_conf.channel[i].divisor_p10 = 0;
		modbus_conf.channel[i].rcode = 3;
	}

	modbus_conf.slave_address = 0x00;
	modbus_conf.waiting_poll_time = 1;
	modbus_conf.data_format = KINCO;

}
//------------------------------------------------------------------------------------
void modbus_config_status(void)
{

uint8_t i;

	xprintf_P( PSTR(">Modbus: ( name,address,nro_regs,rcode,type,factor_p10 )\r\n"));

	xprintf_P( PSTR("   sla_addr=0x%02x, wpTime=%d\r\n"), modbus_conf.slave_address, modbus_conf.waiting_poll_time );

	if ( modbus_conf.slave_address == 0x00 ) {
		return;
	}

	switch (modbus_conf.data_format ) {
	case KINCO:
		xprintf_P( PSTR("   format=kinco\r\n"));
		break;
	case SHINCO:
		xprintf_P( PSTR("   format=shinco\r\n"));
		break;
	case TAO:
		xprintf_P( PSTR("   format=tao\r\n"));
		break;
	case MBSIM:
		xprintf_P( PSTR("   format=mbsim\r\n"));
		break;
	default:
		xprintf_P( PSTR("   format=ERROR\r\n"));
		break;
	}

	for ( i = 0; i < MODBUS_CHANNELS; i++ ) {

		if ( strcmp ( modbus_conf.channel[i].name, "X" ) == 0 ) {
			xprintf_P(PSTR("\r\n"));
			return;
		}

		if ( (i % 2) == 0 ) {
			xprintf_P(PSTR("   "));
		}

		xprintf_P( PSTR("MB%02d:[%s,0x%04X,"), i, modbus_conf.channel[i].name, modbus_conf.channel[i].address );
		xprintf_P( PSTR("0x%X,0x%X,"), modbus_conf.channel[i].nro_regs, modbus_conf.channel[i].rcode );

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

		xprintf_P( PSTR("%d], "), modbus_conf.channel[i].divisor_p10	);

		if ( ((i+1) % 2) == 0 ) {
			xprintf_P(PSTR("\r\n"));
		}
	}

	xprintf_P(PSTR("\r\n"));

}
//------------------------------------------------------------------------------------
bool modbus_config_slave( char *s_slave_address )
{
	modbus_conf.slave_address = atoi(s_slave_address);
	//xprintf_P(PSTR("DEBUG MBSLA: [%s][%d]\r\n"), s_slave_address, modbus_conf.slave_address );
	return(true);
}
//------------------------------------------------------------------------------------
bool modbus_config_channel( uint8_t channel, char *s_name, char *s_addr, char *s_nro_recds, char *s_rcode, char *s_type, char *s_divisor_p10 )
{
	// Todos los datos vienen en decimal !!!

	//xprintf_P(PSTR("DEBUG MBCH%02d: name:[%s],addr:[%s],nrecs:[%s],rcode:[%s],type:[%s],pow10:[%s]\r\n"), channel, s_name, s_addr, s_nro_recds, s_rcode, s_type, s_divisor_p10 );

	if (( s_name == NULL ) || ( s_addr == NULL) || (s_nro_recds == NULL) || ( s_rcode == NULL) || ( s_divisor_p10 == NULL)  ) {
		return(false);
	}

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
		return(false);
	}

	snprintf_P( modbus_conf.channel[channel].name, PARAMNAME_LENGTH, PSTR("%s\0"), s_name );
	modbus_conf.channel[channel].address = atoi(s_addr);
	modbus_conf.channel[channel].nro_regs = atoi(s_nro_recds);
	modbus_conf.channel[channel].rcode = atoi(s_rcode);
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
bool modbus_config_data_format( char *s_format)
{
	// Configura como interpreto los bytes recibidos del slave
	if ( !strcmp_P( strupr(s_format), PSTR("KINCO"))) {
		modbus_conf.data_format = KINCO;
	} else if ( !strcmp_P( strupr(s_format), PSTR("SHINCO"))) {
		modbus_conf.data_format = SHINCO;
	} else if ( !strcmp_P( strupr(s_format), PSTR("TAO"))) {
		modbus_conf.data_format = TAO;
	} else if ( !strcmp_P( strupr(s_format), PSTR("MBSIM"))) {
		modbus_conf.data_format = MBSIM;
	}
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
	j += snprintf_P( &hash_buffer[j], free_size, PSTR("MODBUS;SLA:%04d;MBWT:%03d;"), modbus_conf.slave_address,modbus_conf.waiting_poll_time );
	switch( modbus_conf.data_format) {
	case KINCO:
		j += snprintf_P( &hash_buffer[j], free_size, PSTR("FORMAT:KINCO;"));
		break;
	case SHINCO:
		j += snprintf_P( &hash_buffer[j], free_size, PSTR("FORMAT:SHINCO;"));
		break;
	case TAO:
		j += snprintf_P( &hash_buffer[j], free_size, PSTR("FORMAT:TAO;"));
		break;
	default:
		return(0x00);
	}

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
		switch( modbus_conf.channel[i].type ) {
		case u16:
			j = snprintf_P( hash_buffer, free_size, PSTR("MB%02d:%s,%04d,%02d,%02d,U16,%02d;"), i,
					modbus_conf.channel[i].name,
					modbus_conf.channel[i].address,
					modbus_conf.channel[i].nro_regs,
					modbus_conf.channel[i].rcode,
					modbus_conf.channel[i].divisor_p10
					);
			break;
		case i16:
			j = snprintf_P( hash_buffer, free_size, PSTR("MB%02d:%s,%04d,%02d,%02d,I16,%02d;"), i,
					modbus_conf.channel[i].name,
					modbus_conf.channel[i].address,
					modbus_conf.channel[i].nro_regs,
					modbus_conf.channel[i].rcode,
					modbus_conf.channel[i].divisor_p10
					);
			break;
		case u32:
			j = snprintf_P( hash_buffer, free_size, PSTR("MB%02d:%s,%04d,%02d,%02d,U32,%02d;"), i,
					modbus_conf.channel[i].name,
					modbus_conf.channel[i].address,
					modbus_conf.channel[i].nro_regs,
					modbus_conf.channel[i].rcode,
					modbus_conf.channel[i].divisor_p10
					);
			break;
		case i32:
			j = snprintf_P( hash_buffer, free_size, PSTR("MB%02d:%s,%04d,%02d,%02d,I32,%02d;"), i,
					modbus_conf.channel[i].name,
					modbus_conf.channel[i].address,
					modbus_conf.channel[i].nro_regs,
					modbus_conf.channel[i].rcode,
					modbus_conf.channel[i].divisor_p10
					);
			break;
		case FLOAT:
			j = snprintf_P( hash_buffer, free_size, PSTR("MB%02d:%s,%04d,%02d,%02d,FLOAT,%02d;"), i,
					modbus_conf.channel[i].name,
					modbus_conf.channel[i].address,
					modbus_conf.channel[i].nro_regs,
					modbus_conf.channel[i].rcode,
					modbus_conf.channel[i].divisor_p10
					);
			break;

		}

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
// FUNCIONES DE TEST
//------------------------------------------------------------------------------------
void modbus_test_genpoll(char *arg_ptr[16] )
{

	// Recibe el argv con los datos en char hex para trasmitir el frame.
	// El formato es: {type(F|I} sla fcode addr nro_recds
	// write modbus genpoll F 9 3 4118 2
	// TX: (len=8):[0x09][0x03][0x10][0x16][0x00][0x02][0x20][0x47]


	xprintf_P(PSTR("MODBUS: GENPOLL START\r\n"));

	while ( xSemaphoreTake( sem_MBUS, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	// Preparo el registro con los datos del canal
	if ( strcmp ( strupr(arg_ptr[3]), "FLOAT" ) == 0 ) {
		mbus_cb.type = FLOAT;
	} else 	if ( strcmp ( strupr(arg_ptr[3]), "I16" ) == 0 ) {
		mbus_cb.type = i16;
	} else 	if ( strcmp ( strupr(arg_ptr[3]), "U16" ) == 0 ) {
		mbus_cb.type = u16;
	} else 	if ( strcmp ( strupr(arg_ptr[3]), "I32" ) == 0 ) {
		mbus_cb.type = i32;
	} else 	if ( strcmp ( strupr(arg_ptr[3]), "U32" ) == 0 ) {
		mbus_cb.type = u32;
	} else {
		return;
	}

	mbus_cb.sla_address = atoi(arg_ptr[4]);
	mbus_cb.function_code = atoi(arg_ptr[5]);
	mbus_cb.address = atoi(arg_ptr[6]);
	mbus_cb.nro_recds = atoi(arg_ptr[7]);
	mbus_cb.divisor_p10 = 0;
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
void modbus_write_output_register( char *s_address, char *s_type, char *s_value )
{
	// Si type = 'I' escribo un U16 con codigo 6 en un holding register
	// Si type = 'F' escribo un FLOAT con codigo 10 en un holding register

	xprintf_P(PSTR("MODBUS: OUTREG START\r\n"));

	while ( xSemaphoreTake( sem_MBUS, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	// Preparo el registro con los datos del canal
	mbus_cb.sla_address = modbus_conf.slave_address;
	mbus_cb.divisor_p10 = 1;

	// ADDRESS
	mbus_cb.address = atoi(s_address);
	// FCODE
	if ( *s_type == 'F') {
		// Escribo un FLOAT ( 4 bytes )
		mbus_cb.function_code = 0x10;
		mbus_cb.type = FLOAT;
		mbus_cb.nro_recds = 2;
		mbus_cb.udata.float_value = atof(s_value);

	} else if ( *s_type == 'I') {
		// Escribo un U16 ( 2 bytes )
		mbus_cb.function_code = 0x06;
		mbus_cb.type = u16;
		mbus_cb.nro_recds = 1;
		mbus_cb.udata.u16_value = atoi(s_value);

	} else {
		xprintf_P("ERROR type no reconocido.\r\n");
		xSemaphoreGive( sem_MBUS );
		return;
	}
	//
	modbus_io( true, &mbus_cb );
	xSemaphoreGive( sem_MBUS );

	modbus_print( true, &mbus_cb );
	xprintf_P(PSTR("MODBUS: OUTREG END\r\n"));

}
//------------------------------------------------------------------------------------
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
	// Prepara el ADU ( raw frame ) con los datos de la variable global mbus_cb.
	// Calcula el CRC de modo que el frame queda pronto a transmitirse por el serial

	if ( modbus_conf.data_format == MBSIM ) {
		pv_modbus_codec_mbsim (false, ENCODER, mbus_cb );

	} else if ( modbus_conf.data_format == KINCO ) {
		pv_modbus_codec_kinco (false, ENCODER, mbus_cb );

	}
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

	if ( modbus_conf.data_format == MBSIM ) {
		pv_modbus_codec_mbsim (f_debug, DECODER, mbus_cb );

	} else 	if ( modbus_conf.data_format == KINCO ) {
		pv_modbus_codec_kinco (f_debug, DECODER, mbus_cb );
	}

quit:

	xprintf_PD(f_debug, PSTR("MODBUS: DECODE b0[0x%02X] b1[0x%02X] b2[0x%02X] b3[0x%02X]\r\n"),
			mbus_cb->udata.raw_value[0], mbus_cb->udata.raw_value[1], mbus_cb->udata.raw_value[2], mbus_cb->udata.raw_value[3]);

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
		if ( mbus_cb->type == FLOAT) {
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %.03f (F)\r\n"), mbus_cb->udata.float_value);

		} else if (mbus_cb->type == i16 ) {
			// En este caso importa el parametro divisor_p10 ya que convierte al entero en float
			pvalue = 1.0 * mbus_cb->udata.i16_value / pow(10, mbus_cb->divisor_p10 );
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %d (i16), %.03f\r\n"), mbus_cb->udata.i16_value, pvalue );

		} else if (mbus_cb->type == u16 ) {
			pvalue = 1.0 * mbus_cb->udata.u16_value / pow(10, mbus_cb->divisor_p10 );
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %u (u16), %.03f\r\n"), mbus_cb->udata.u16_value, pvalue );

		} else if (mbus_cb->type == u32 ) {
			pvalue = 1.0 * ( mbus_cb->udata.u32_value ) / pow(10, mbus_cb->divisor_p10 );
			xprintf_PD( f_debug, PSTR("MODBUS: VALUE %lu (u32), %.03f\r\n"), mbus_cb->udata.u32_value, pvalue );

		} else if (mbus_cb->type == i32 ) {
			pvalue = 1.0 * ( mbus_cb->udata.i32_value ) / pow(10, mbus_cb->divisor_p10 );
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

	// Si no tengo canales configurados, salgo.
	if ( modbus_conf.slave_address == 0x00 ) {
		return;
	}

	for ( ch = 0; ch < MODBUS_CHANNELS; ch++) {
		// Si un canal no esta definido, salgo
		if ( strcmp ( modbus_conf.channel[ch].name, "X" ) == 0 ) {
			break;
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
char * modbus_sprintf( char *sbuffer, float src[] )
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
 */
//------------------------------------------------------------------------------------
void pv_modbus_codec_kinco(bool f_debug, int8_t codec, mbus_CONTROL_BLOCK_t *mbus_cb )
{
uint8_t size = 0;
uint16_t crc;
uint8_t payload_ptr;	// Posicion en el rxbuffer donde empieza el payload

	if (codec == DECODER ) {

		payload_ptr = 3;

		if ( mbus_cb->function_code == 0x03 ) {
			switch (mbus_cb->type ) {
			case i16:
				xprintf_PD(f_debug, PSTR("MODBUS: kDECODER F3 int16\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr];
				mbus_cb->udata.raw_value[2] = 0x00;
				mbus_cb->udata.raw_value[3] = 0x00;
				break;
			case u16:
				xprintf_PD(f_debug, PSTR("MODBUS: kDECODER F3 uint16\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr];
				mbus_cb->udata.raw_value[2] = 0x00;
				mbus_cb->udata.raw_value[3] = 0x00;
				break;
			case i32:
				break;
			case u32:
				break;
			case FLOAT:
				/*
				 * 3.15 => 0x4049999A
				 * 0x 40 49 99 9A
				 *     3  2  1  0
				 *  payload_ptr ==> 40
				 */
				xprintf_PD(f_debug, PSTR("MODBUS: kDECODER F3 float\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 3];
				mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr + 2];
				break;
			default:
				xprintf_P("MODBUS: DECODER R3 ERROR\r\n");
			}

		} else if ( mbus_cb->function_code == 0x06 ) {
			// No se analiza
			return;

		} else if ( mbus_cb->function_code == 0x10 ) {
			// No se analiza
			return;
		} else {
			// Caso no implementado
			return;
		}
		return;
	}


	if ( codec == ENCODER ) {

		// Header
		memset( mbus_cb->tx_buffer, '\0', MBUS_TXMSG_LENGTH );
		mbus_cb->tx_buffer[0] = mbus_cb->sla_address;								// SLA
		mbus_cb->tx_buffer[1] = mbus_cb->function_code;								// FCODE
		mbus_cb->tx_buffer[2] = (uint8_t) (( mbus_cb->address & 0xFF00  ) >> 8);	// DST_ADDR_H
		mbus_cb->tx_buffer[3] = (uint8_t) ( mbus_cb->address & 0x00FF );			// DST_ADDR_L

		// Payload
		if ( mbus_cb->function_code == 0x03 ) {
			// Hago el ADU de un frame tipo 0x03 ( Read Holding Register )
			mbus_cb->tx_buffer[4] = (uint8_t) ((mbus_cb->nro_recds & 0xFF00 ) >> 8);	// NRO_REG_HI
			mbus_cb->tx_buffer[5] = (uint8_t) ( mbus_cb->nro_recds & 0x00FF );			// NRO_REG_LO
			size = 6;

		} else if ( mbus_cb->function_code == 0x06 ) {

			// Hago el ADU de un frame tipo 0x06 ( Write Single Register )
			// El valor a escribir esta en mbus_cb.udata.
			// Escribo solo 2 bytes con esta funcion.
			mbus_cb->tx_buffer[4] = mbus_cb->udata.raw_value[1];	// DATA_VAL_HI
			mbus_cb->tx_buffer[5] = mbus_cb->udata.raw_value[0];	// DATA_VAL_LO
			size = 6;

		} else if ( mbus_cb->function_code == 0x10 ) {

			// Hago el ADU de un frame tipo 0x10 (16) ( Write Multitple Register )
			// El valor a escribir esta en mbus_cb.udata(float)
			// Escribo 4 bytes.
			// El simulador recibe los bytes en orden.
			// Quantity of Registers: 2 bytes, ( 1 registro o 1 variable de largo 4  )
			mbus_cb->tx_buffer[4] = 0;
			mbus_cb->tx_buffer[5] = 2;
			//
			// Byte count ,1 byte ( 1 float = 4 bytes , 2 x N => N=2)
			mbus_cb->tx_buffer[6] = 4;

			// El PCL recibe los bytes en diferente orden !!!.
			mbus_cb->tx_buffer[7] = mbus_cb->udata.raw_value[1];	// DATA
			mbus_cb->tx_buffer[8] = mbus_cb->udata.raw_value[0];
			mbus_cb->tx_buffer[9] = mbus_cb->udata.raw_value[3];
			mbus_cb->tx_buffer[10] = mbus_cb->udata.raw_value[2];
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
}
//------------------------------------------------------------------------------------
void pv_modbus_codec_mbsim(bool f_debug, int8_t codec, mbus_CONTROL_BLOCK_t *mbus_cb )
{

uint8_t size = 0;
uint16_t crc;
uint8_t payload_ptr;	// Posicion en el rxbuffer donde empieza el payload

	if (codec == DECODER ) {

		payload_ptr = 3;

		if ( mbus_cb->function_code == 0x03 ) {
			switch (mbus_cb->type ) {
			case i16:
				/* Decodifico un i16 de acuerdo a lo que manda el simulador pyModSimulator.py
				 *
				 *  1134 = 0x046E
				 * -1134 = 0xFB92
				 *  0x 00 00 04 6E
				 *      3  2  1  0
				 *  payload_ptr ==> 04
				 *  0x 00 00 FB 92
				 *      3  2  1  0
				 *  payload_ptr ==> FB
				 */
				xprintf_PD(f_debug, PSTR("MODBUS: simDECODER F3 int16\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr];
				mbus_cb->udata.raw_value[2] = 0x00;
				mbus_cb->udata.raw_value[3] = 0x00;
				break;
			case u16:
				// 45678 => 0x 00 00 B2 63
				xprintf_PD(f_debug, PSTR("MODBUS: simDECODER F3 uint16\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr];
				mbus_cb->udata.raw_value[2] = 0x00;
				mbus_cb->udata.raw_value[3] = 0x00;
				break;
			case i32:
				/* Decodifico un i32 ( 4 bytes ) de acuerdo a lo que manda el simulador pyModSimulator.py
				 *
				 *  84675 = 0x00014AC3
				 *  0x 00 01 4A C3
				 *      3  2  1  0
				 *  payload_ptr ==> 00
				 */
				xprintf_PD(f_debug, PSTR("MODBUS: simDECODER F3 int32\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 3];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 2];
				mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr];

				break;
			case u32:
				xprintf_PD(f_debug, PSTR("MODBUS: simDECODER F3 uint32\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 3];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 2];
				mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr];
				break;
			case FLOAT:
				/*
				 * 8492.21 => 0x460BB8D7
				 * 0x 46 0B B8 D7
				 *     3  2  1  0
				 *  payload_ptr ==> 46
				 */
				xprintf_PD(f_debug, PSTR("MODBUS: simDECODER F3 float\r\n"));
				mbus_cb->udata.raw_value[0] = mbus_cb->rx_buffer[payload_ptr + 3];
				mbus_cb->udata.raw_value[1] = mbus_cb->rx_buffer[payload_ptr + 2];
				mbus_cb->udata.raw_value[2] = mbus_cb->rx_buffer[payload_ptr + 1];
				mbus_cb->udata.raw_value[3] = mbus_cb->rx_buffer[payload_ptr];
				break;
			default:
				xprintf_P("MODBUS: DECODER R3 ERROR\r\n");
			}

		} else if ( mbus_cb->function_code == 0x06 ) {
			// No se analiza
			return;

		} else if ( mbus_cb->function_code == 0x10 ) {
			// No se analiza
			return;
		} else {
			// Caso no implementado
			return;
		}
		return;
	}


	if ( codec == ENCODER ) {

		// Header
		memset( mbus_cb->tx_buffer, '\0', MBUS_TXMSG_LENGTH );
		mbus_cb->tx_buffer[0] = mbus_cb->sla_address;								// SLA
		mbus_cb->tx_buffer[1] = mbus_cb->function_code;								// FCODE
		mbus_cb->tx_buffer[2] = (uint8_t) (( mbus_cb->address & 0xFF00  ) >> 8);	// DST_ADDR_H
		mbus_cb->tx_buffer[3] = (uint8_t) ( mbus_cb->address & 0x00FF );			// DST_ADDR_L

		// Payload
		if ( mbus_cb->function_code == 0x03 ) {
			// Hago el ADU de un frame tipo 0x03 ( Read Holding Register )
			mbus_cb->tx_buffer[4] = (uint8_t) ((mbus_cb->nro_recds & 0xFF00 ) >> 8);	// NRO_REG_HI
			mbus_cb->tx_buffer[5] = (uint8_t) ( mbus_cb->nro_recds & 0x00FF );			// NRO_REG_LO
			size = 6;

		} else if ( mbus_cb->function_code == 0x06 ) {

			// Hago el ADU de un frame tipo 0x06 ( Write Single Register )
			// El valor a escribir esta en mbus_cb.udata.
			// Escribo solo 2 bytes con esta funcion.
			mbus_cb->tx_buffer[4] = mbus_cb->udata.raw_value[1];	// DATA_VAL_HI
			mbus_cb->tx_buffer[5] = mbus_cb->udata.raw_value[0];	// DATA_VAL_LO
			size = 6;

		} else if ( mbus_cb->function_code == 0x10 ) {

			// Hago el ADU de un frame tipo 0x10 (16) ( Write Multitple Register )
			// El valor a escribir esta en mbus_cb.udata(float)
			// Escribo 4 bytes.
			// El simulador recibe los bytes en orden.
			// Quantity of Registers: 2 bytes, ( 1 registro o 1 variable de largo 4  )
			mbus_cb->tx_buffer[4] = 0;
			mbus_cb->tx_buffer[5] = 2;
			//
			// Byte count ,1 byte ( 1 float = 4 bytes , 2 x N => N=2)
			mbus_cb->tx_buffer[6] = 4;

			mbus_cb->tx_buffer[7] = mbus_cb->udata.raw_value[3];	// DATA
			mbus_cb->tx_buffer[8] = mbus_cb->udata.raw_value[2];
			mbus_cb->tx_buffer[9] = mbus_cb->udata.raw_value[1];
			mbus_cb->tx_buffer[10] = mbus_cb->udata.raw_value[0];
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
}
//------------------------------------------------------------------------------------
