/*
 * ul_ainputs.h
 *
 *  Created on: 14 jul. 2021
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_AINPUTS_H_
#define SPX_ULIBS_INCLUDE_UL_AINPUTS_H_

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#include "FreeRTOS.h"

#include "l_printf.h"
#include "l_ina3221.h"

#include "ul_utils.h"

xSemaphoreHandle sem_AINPUTS;
StaticSemaphore_t AINPUTS_xMutexBuffer;
#define MSTOTAKEAINPUTSSEMPH ((  TickType_t ) 10 )

#define ANALOG_CHANNELS		5

// Configuracion de canales analogicos
typedef struct {
	uint8_t imin[ANALOG_CHANNELS];	// Coeficientes de conversion de I->magnitud (presion)
	uint8_t imax[ANALOG_CHANNELS];
	float mmin[ANALOG_CHANNELS];
	float mmax[ANALOG_CHANNELS];
	char name[ANALOG_CHANNELS][PARAMNAME_LENGTH];
	float offset[ANALOG_CHANNELS];
	uint8_t pwr_settle_time;
} ainputs_conf_t;

ainputs_conf_t ainputs_conf;

bool debug_ainputs;

void ainputs_init(void);
void ainputs_awake(void);
void ainputs_sleep(void);
bool ainputs_config_channel( uint8_t channel,char *s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax,char *s_offset );
void ainputs_config_defaults(void);
void ainputs_config_timepwrsensor ( char *s_timepwrsensor );
bool ainputs_read( float ain[], float *battery );
void ainputs_print(file_descriptor_t fd, float src[] );
void ainputs_battery_print( file_descriptor_t fd, float battery );
uint8_t ainputs_hash(void);
void ainputs_test_channel( uint8_t io_channel);
void ainputs_set_debug(void);
void ainputs_clr_debug(void);
void ainputs_print_channel_status(void);
uint8_t ainputs_get_timePwrSensor(void);


#endif /* SPX_ULIBS_INCLUDE_UL_AINPUTS_H_ */
