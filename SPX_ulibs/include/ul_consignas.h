/*
 * ul_consignas.h
 *
 *  Created on: 9 ago. 2021
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_CONSIGNAS_H_
#define SPX_ULIBS_INCLUDE_UL_CONSIGNAS_H_

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "l_printf.h"
#include "ul_utils.h"

#include "l_drv8814.h"
#include "l_rtc79410.h"


typedef enum { CONSIGNA_DIURNA = 0, CONSIGNA_NOCTURNA } t_consigna;

void consigna_init(void);
bool consigna_config( char *tipo, char *hhmm );
void consigna_config_status(void);
void consigna_config_defaults(void);
void consigna_set_diurna(void);
void consigna_set_nocturna(void);
bool consigna_init_service(void);
void consigna_app_service( uint8_t app_wdt );
uint8_t consigna_hash(void);

typedef struct {
	st_time_t consigna_diurna;
	st_time_t consigna_nocturna;
} consigna_conf_t;

consigna_conf_t consigna_conf;

int8_t consigna_aplicada;


#endif /* SPX_ULIBS_INCLUDE_UL_CONSIGNAS_H_ */
