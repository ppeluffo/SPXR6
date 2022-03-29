/*
 * drv_i2c_spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_DRIVERS_DRV_I2C_SPX_H_
#define SRC_SPX_DRIVERS_DRV_I2C_SPX_H_


#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/twi.h>
#include "FreeRTOS.h"
#include "task.h"

#include "l_iopines.h"

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 	0
#define NACK 	1

#define DEBUG_I2C false

#define I2C_WRITE 0
#define I2C_READ 1

/*! Transaction result enumeration. */
typedef enum TWIM_RESULT_enum {
	TWIM_RESULT_UNKNOWN          = (0x00<<0),
	TWIM_RESULT_OK               = (0x01<<0),
	TWIM_RESULT_BUFFER_OVERFLOW  = (0x02<<0),
	TWIM_RESULT_ARBITRATION_LOST = (0x03<<0),
	TWIM_RESULT_BUS_ERROR        = (0x04<<0),
	TWIM_RESULT_NACK_RECEIVED    = (0x05<<0),
	TWIM_RESULT_FAIL             = (0x06<<0),
	TWIM_RESULT_TIMEOUT          = (0x07<<0),
} TWIM_RESULT_t;

int16_t drv_I2C_master_write ( const uint8_t devAddress, const uint16_t dataAddress, const uint8_t dataAddressLength, char *pvBuffer, size_t xBytes );
int16_t drv_I2C_master_read ( const uint8_t devAddress, const uint16_t dataAddress, const uint8_t dataAddressLength, char *pvBuffer, size_t xBytes );
void drv_I2C_init(void);

#endif /* SRC_SPX_DRIVERS_DRV_I2C_SPX_H_ */
