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
#include "ul_genpulsos.h"
#include "ul_oceanus.h"

typedef enum { APP_OFF = 0, APP_CONSIGNA, APP_PILOTO, APP_GENPULSOS, APP_OCEANUS } t_applicacion;

#define DF_APP ( (systemVars.debug == DEBUG_APP ) || (systemVars.debug == DEBUG_ALL ))

bool aplicacion_config( char *modo );
void aplicacion_config_status(void);
void aplicacion_config_defaults(void);
uint8_t aplicacion_hash(void);


#endif /* SPX_TASKS_INCLUDE_TKAPP_H_ */
