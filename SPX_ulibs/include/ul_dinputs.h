/*
 * ul_dinputs.h
 *
 *  Created on: 14 jul. 2021
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_DINPUTS_H_
#define SPX_ULIBS_INCLUDE_UL_DINPUTS_H_

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"

#include "l_printf.h"
#include "ul_utils.h"

#define DINPUTS_CHANNELS	2

void dinputs_init( void );
void dinputs_config_defaults(void);
bool dinputs_config_channel( uint8_t channel, char *s_aname );
void dinputs_clear(void);
bool dinputs_read(uint8_t dst[]);
void dinputs_print(file_descriptor_t fd, uint8_t src[] );
uint8_t dinputs_hash(void);
void dinputs_test_read(void);
void dinputs_print_status(void);

// Configuracion de canales digitales
typedef struct {
	char name[DINPUTS_CHANNELS][PARAMNAME_LENGTH];
} dinputs_conf_t;

dinputs_conf_t dinputs_conf;


#endif /* SPX_ULIBS_INCLUDE_UL_DINPUTS_H_ */
