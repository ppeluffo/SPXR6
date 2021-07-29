/*
 * l_i2c.c
 *
 *  Created on: 26 de mar. de 2018
 *      Author: pablo
 */

#include "l_i2c.h"
#include "l_bytes.h"

const char str_i2c_dev_0[] PROGMEM = "ERR";
const char str_i2c_dev_1[] PROGMEM = "EE";
const char str_i2c_dev_2[] PROGMEM = "RTC";
const char str_i2c_dev_3[] PROGMEM = "MCP";
const char str_i2c_dev_4[] PROGMEM = "INA_A";
const char str_i2c_dev_5[] PROGMEM = "INA_B";
const char str_i2c_dev_6[] PROGMEM = "ADT7410";
const char str_i2c_dev_7[] PROGMEM = "I2CMUX";

const char * const I2C_names[] PROGMEM = { str_i2c_dev_0, str_i2c_dev_1, str_i2c_dev_2, str_i2c_dev_3, str_i2c_dev_4, str_i2c_dev_5, str_i2c_dev_6, str_i2c_dev_7 };

uint8_t pv_i2_addr_2_idx( uint8_t i2c_bus_address );

char buffer[10] = { 0 };

//------------------------------------------------------------------------------------
bool I2C_scan_device( uint8_t i2c_bus_address )
{
	// Utiliza las funciones SCAN del driver para testear su presencia.
	// No genera mensajes de error en consola ya que se usa al inicio para detectar que
	// placa hay conectada.

uint8_t bus_address = 0;
bool retS = true;

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);

	// 1) Indicamos el periferico i2c en el cual queremos leer ( variable de 8 bits !!! )
	bus_address = i2c_bus_address;
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DEVADDRESS, &bus_address);
	retS = frtos_ioctl(fdI2C,ioctl_I2C_SCAN, false);

	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(retS);

}
//------------------------------------------------------------------------------------
void I2C_reinit_devices(void)
{
	// En caso de una falla en el bus I2C ( por lo general por ruidos al activar bombas )
	// debo reiniciar los dispositivos.

	// Espero 100 ms que se elimine la fuente de ruido.
	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	//INA_config_avg128( INA_A );
	//INA_config_avg128( INA_B );
	//RTC_init();

	xprintf_P(PSTR("WARNING: I2C_reinit_devices.\r\n\0") );
}
//------------------------------------------------------------------------------------
uint8_t pv_i2_addr_2_idx( uint8_t i2c_bus_address )
{
	switch( i2c_bus_address ) {
	case BUSADDR_EEPROM_M2402:
		return(1);
		break;
	case BUSADDR_RTC_M79410:
		return(2);
		break;
	case BUSADDR_MCP23018:
		return(3);
		break;
	case BUSADDR_INA_A:
		return(4);
		break;
	case BUSADDR_INA_B:
		return(5);
		break;
	case BUSADDR_INA_C:
		return(6);
		break;
	case BUSADDR_BPS120:
		return(7);
		break;
	case BUSADDR_ADT7410:
		return(8);
		break;
	default:
		return(0);
	}

	return(0);

}
//------------------------------------------------------------------------------------
// NEW VERSIONS
//------------------------------------------------------------------------------------
void I2C_get_semaphore(void)
{
	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);
}
//------------------------------------------------------------------------------------
void I2C_release_semaphore(void)
{
	frtos_ioctl(fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);
}
//------------------------------------------------------------------------------------
int16_t I2C_write ( uint8_t i2c_bus_address, uint16_t data_address, uint8_t data_address_length, char *data, uint8_t length )
{
	// En esta version pasamos la direccion interna del chip donde escribir y el largo.
	// Si chip_address_length es  0, no escribimos la direccion interna, pasamos directo a los datos.
	// Si legth es 0 entonces es un dummy_write de un ciclo read.
	// NO USA SEMAFOROS
	// Lo debe hacer la funcion invocante !!!

//	xprintf_P(PSTR("DEBUG:I2C_write: i2c_bus_address=0x%02x, data_address=0x%02x, data_address_length=0x%02x, length=0x%02x\r\n"), i2c_bus_address, data_address, data_address_length, length);

int16_t xReturn = 0U;
uint8_t i2c_error_code = 0;

	// 1) Indicamos el periferico i2c en el cual queremos escribir ( variable de 8 bits !!! )
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DEVADDRESS, &i2c_bus_address );

	// 2) Indicamos al direccion interna del chip donde comenzar a escribir
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DATAADDRESS, &data_address );
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DATAADDRESSLENGTH, &data_address_length );

	// 3) Por ultimo escribimos. No controlo fronteras.
	xReturn = frtos_write(fdI2C, data, length);

	// 4) Controlo errores
	i2c_error_code = frtos_ioctl(fdI2C, ioctl_I2C_GET_LAST_ERROR, NULL );

	if (i2c_error_code != I2C_OK ) {
		memset(buffer,'\0', 10);
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(I2C_names[pv_i2_addr_2_idx( i2c_bus_address )])));
		xprintf_P(PSTR("ERROR: I2C WR. i2c_addr=0x0%X, err_code=0x0%X, perferico=%s.\r\n\0"), i2c_bus_address, i2c_error_code, buffer );
	}

	return(xReturn);
}
//------------------------------------------------------------------------------------
int16_t I2C_read ( uint8_t i2c_bus_address, uint16_t data_address, uint8_t data_address_length, char *data, uint8_t length )
{

	// Implementa solo la parte de lectura del ciclo.
	// La primera parte que es solo escribir la direccion de donde leer la hacemos
	// con I2C_write_R1. ( Dummy Write )

int16_t xReturn = 0U;
uint8_t bus_address = 0;
uint8_t i2c_error_code = 0;

//	xprintf_P(PSTR("DEBUG:I2C_read: i2c_bus_address=0x%02x, length=0x%02x, length=0x%02x\r\n"), i2c_bus_address, length);

	// 1) Indicamos el periferico i2c en el cual queremos leer ( variable de 8 bits !!! )
	bus_address = i2c_bus_address;
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DEVADDRESS, &bus_address);

	// 2) Indicamos al direccion interna del chip donde comenzar a escribir
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DATAADDRESS, &data_address );
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DATAADDRESSLENGTH, &data_address_length );


	// 3) Leemos. No controlo fronteras.
	xReturn = frtos_read(fdI2C, data, length);

	// 4) Controlo errores.
	i2c_error_code = frtos_ioctl(fdI2C, ioctl_I2C_GET_LAST_ERROR, NULL );
	if (i2c_error_code != I2C_OK ) {
		memset(buffer,'\0', sizeof(buffer));
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(I2C_names[pv_i2_addr_2_idx( i2c_bus_address )])));
		xprintf_P(PSTR("ERROR: I2C RD. i2c_addr=0x0%X, err_code=0x0%X, buffer=%s.\r\n\0"), i2c_bus_address, i2c_error_code, buffer );
		xReturn = -1;
	}

	if (xReturn != length ) {
		xprintf_P(PSTR("ERROR: I2C RD. i2c_addr=0x0%X, (xbytes=%d) != (xReturn=%d).\r\n\0"), i2c_bus_address, length, xReturn  );
		xReturn = -1;
	}

	return(xReturn);

}
//------------------------------------------------------------------------------------
