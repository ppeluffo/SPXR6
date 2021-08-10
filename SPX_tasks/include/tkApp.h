/*
 * tkApp.h
 *
 *  Created on: 7 ago. 2021
 *      Author: pablo
 */

#ifndef SPX_TASKS_INCLUDE_TKAPP_H_
#define SPX_TASKS_INCLUDE_TKAPP_H_

#include "spx.h"

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "inttypes.h"

#include "FreeRTOS.h"

#include "ul_consignas.h"
#include "ul_pilotos.h"

typedef enum { APP_OFF = 0, APP_CONSIGNA, APP_PILOTO } t_applicacion;

bool aplicacion_config( char *modo );
void aplicacion_config_status(void);
void aplicacion_config_defaults(void);
uint8_t aplicacion_hash(void);


#endif /* SPX_TASKS_INCLUDE_TKAPP_H_ */
