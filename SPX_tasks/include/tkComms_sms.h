/*
 * sms.h
 *
 *  Created on: 8 dic. 2021
 *      Author: pablo
 */

#ifndef SPX_TASKS_INCLUDE_TKCOMMS_SMS_H_
#define SPX_TASKS_INCLUDE_TKCOMMS_SMS_H_


#include "tkComms.h"

void SMS_write_test(void);
void SMS_read_test(void);
void SMS_full_test(void);
void SMS_test(void);
void SMS_frame_test(void);
uint8_t sms_hash(void);

bool SMS_rxcheckpoint(void);
char *SMS_read_and_delete_by_index( uint8_t msg_index );
bool SMS_received( uint8_t *first_msg_index );
void SMS_process( void );
void SMS_process_dict_orders(char *rsp );

void sms_config_defaults(void);
bool sms_config_auth_number( int pos, char *s_number );
bool sms_config_order_dict( int pos, char *s_mb_channel, char *s_order);
void sms_status(void);

bool SMS_send( void );

#define DF_SMS ( (systemVars.debug == DEBUG_SMS ) || (systemVars.debug == DEBUG_ALL ))

struct {
	char *smsRxMsg;
	char *smsTxMsg;
	int16_t sms_nbr_idx;

} SMSCB;

#endif /* SPX_TASKS_INCLUDE_TKCOMMS_SMS_H_ */
