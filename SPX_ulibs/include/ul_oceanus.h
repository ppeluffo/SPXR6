/*
 * ul_ambiental.h
 *
 *  Created on: 11 may. 2022
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_OCEANUS_H_
#define SPX_ULIBS_INCLUDE_UL_OCEANUS_H_


#include "spx.h"

#define OCEANUS_CHANNELS		5

void tkApp_oceanus( uint8_t app_wdt );
void oceanus_init(void);
bool oceanus_read( float oceanus_data[] );

float oceanusValues[OCEANUS_CHANNELS];

#define OCEANUS_RXMSG_LENGTH	64


#endif /* SPX_ULIBS_INCLUDE_UL_OCEANUS_H_ */
