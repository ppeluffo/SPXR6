/*
 * main.c
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 *
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e -U flash:w:spxR4.hex
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e
 *
 *  REFERENCIA: /usr/lib/avr/include/avr/iox256a3b.h
 *
 *  El watchdog debe estar siempre prendido por fuses.
 *  FF 0A FD __ F5 D4
 *
 *  PROGRAMACION FUSES:
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Uflash:w:spx.hex:a -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -Uflash:w:/home/pablo/Spymovil/workspace/spxR5/Release/spxR5.hex:a
 *  /usr/bin/avrdude -p x256a3b -P /dev/ttyUSB0 -b 9600 -c avr109 -v -v -e -V -Uflash:w:/home/pablo/Spymovil/workspace/spxR5/Release/spxR5.hex
 *
 *  Para ver el uso de memoria usamos
 *  avr-nm -n spxR5.elf | more
 *  avr-nm -Crtd --size-sort spxR5.elf | grep -i ' [dbv] '
 *
 *  Archivos modificados en el server para version 4.0.0a
 *  spy.py
 *  spy.conf
 *  spy_raw_frame.py
 *  spy_raw_frame_test.py
 *  SPY_init_conf_global.py
 *  SPY_init_conf_analog.py
 *  SPY_init_conf_app.py
 *  SPY_init_conf_base.py
 *  SPY_init_conf_counter.py
 *  SPY_init_conf_digital.py
 *  spy_utils.py
 *
 *  R4.0.0e @ 2021-09-24:
 *  - Modificaciones al modbus:
 *  Cada registro debe contener la dirección del dispositivo remoto.
 *  De este modo podemos polear a varios dispositivos en el mismo bus.
 *
 *
 *  R4.0.0d @ 2021-09-24:
 *  - Cuando tiene muchos datos para transmitir, se restea por wdg de comms.
 *    Se soluciona limpiando el wdg luego que transmite c/ventana.
 *  - Los reintentos de cerrar el socket no funcionan por lo que solo se demora mas.
 *    Pasamos los reintentos ( MAX_TRYES_SOCKET_CLOSE ) a 1.
 *  - Aumentamos los slots de pilotos a 12
 *  - Agregamos que remotamente se pueda fijar una presion momentaneamente.
 *  - Al arrancar el modem le doy ATE0 para evitar el echo.
 *
 *
 *
 *  R4.0.0c @ 2021-08-31:
 *  - Modbus: Polea e imprime todos los canales.
 *    NO TRANSMITE los canales modbus !!
 *    NO SE CONFIGURA online modbus !!
 *    REVISAR CONFIGRAR CANALES MODBUS LA ADDRESS !!!
 *  - Pilotos:
 *    Sale por wdg al ajustar. Corregido
 *    En el ajuste, considera pB y no pREF para el dP: Corregido.
 *
 *  R4.0.0b @ 2021-08-24:
 *  - Como los motores de los pilotos pueden cambiar los pulsoxrev, agrego 2 parametros a pilotos_conf
 *    que son pulsoXrev y pwidth.
 *    Estos se configuran por cmdMode y online.
 *  - Agrego setear en GPRS el modo,sat,band,pref.
 *  - Por un bug no se configuraba el modo hw de los contadores. Listo.
 *
 *  R4.0.0a @ 2021-08-16:
 *  - En modo apagado prendo ATE1 para seguir en el log los comandos enviados
 *    En modo online (entry) lo apago ATE0 para no llenar el rxbuffer con lineas de comandos
 *  - El CSQ no se estaba actualizando y mandaba 0. Actualizo la variable xCOMMS_stateVars.csq.
 *  - En modo online espera y modo apagado espera, monitoreo la senal SPX_REDIAL
 *  - Cuando apago el modem, actualizo xCOMMS_stateVars.csq a 0.
 *  - El config default pone aplicacion en 0 y modbus en off.
 *
 */

#include "spx.h"

//------------------------------------------------------------------------------------
int main( void )
{
	// Leo la causa del reset para trasmitirla en el init.
	wdg_resetCause = RST.STATUS;
	RST.STATUS = wdg_resetCause;
	//RST_PORF_bm | RST_EXTRF_bm | RST_BORF_bm | RST_WDRF_bm | RST_PDIRF_bm | RST_SRF_bm | RST_SDRF_bm;

	// Clock principal del sistema
	u_configure_systemMainClock();
	u_configure_RTC32();
	sysTicks = 0;

	// Configuramos y habilitamos el watchdog a 8s.
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );
	if ( WDT_IsWindowModeEnabled() )
		WDT_DisableWindowMode();

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	initMCU();

	frtos_open(fdTERM, 9600 );
	frtos_open(fdGPRS, 115200);
	frtos_open(fdI2C, 100 );
	frtos_open(fdAUX1, 9600);

	// Creo los semaforos
	sem_SYSVars = xSemaphoreCreateMutexStatic( &SYSVARS_xMutexBuffer );
	sem_WDGS = xSemaphoreCreateMutexStatic( &WDGS_xMutexBuffer );
	sem_AINPUTS = xSemaphoreCreateMutexStatic( &AINPUTS_xMutexBuffer );
	sem_RXBUFF = xSemaphoreCreateMutexStatic( &RXBUFF_xMutexBuffer );
	sem_MBUS = xSemaphoreCreateMutexStatic( &MBUS_xMutexBuffer );

	xprintf_init();
	FAT_init();
	consigna_init();

	piloto_setup_outofrtos();
	counters_setup_outofrtos();

	// Creamos las tareas
	start_byte = 0x00;

	xHandle_tkCtl = xTaskCreateStatic(tkCtl, "CTL", tkCtl_STACK_SIZE, (void *)1, tkCtl_TASK_PRIORITY, xTask_Ctl_Buffer, &xTask_Ctl_Buffer_Ptr );
	xHandle_tkCmd = xTaskCreateStatic(tkCmd, "CMD", tkCmd_STACK_SIZE, (void *)1, tkCmd_TASK_PRIORITY, xTask_Cmd_Buffer, &xTask_Cmd_Buffer_Ptr );
	xHandle_tkData = xTaskCreateStatic(tkData, "IN", tkData_STACK_SIZE, (void *)1, tkData_TASK_PRIORITY, xTask_Data_Buffer, &xTask_Data_Buffer_Ptr );
	xHandle_tkApp = xTaskCreateStatic(tkApp, "APP", tkApp_STACK_SIZE, (void *)1, tkApp_TASK_PRIORITY, xTask_App_Buffer, &xTask_App_Buffer_Ptr );
	xHandle_tkComms = xTaskCreateStatic(tkComms, "COMMS", tkComms_STACK_SIZE, (void *)1, tkComms_TASK_PRIORITY, xTask_Comms_Buffer, &xTask_Comms_Buffer_Ptr );
	xHandle_tkGprsRX = xTaskCreateStatic(tkGprsRX, "RX", tkGprsRX_STACK_SIZE, (void *)1, tkGprsRX_TASK_PRIORITY, xTask_GprsRX_Buffer, &xTask_GprsRX_Buffer_Ptr );
	xHandle_tkAuxRX = xTaskCreateStatic(tkAuxRX, "AUX", tkAuxRX_STACK_SIZE, (void *)1, tkAuxRX_TASK_PRIORITY, xTask_AuxRX_Buffer, &xTask_AuxRX_Buffer_Ptr );

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);

}
//------------------------------------------------------------------------------------
void vApplicationIdleHook( void )
{
	// Como trabajo en modo tickless no entro mas en modo sleep aqui.
//	if ( sleepFlag == true ) {
//		sleep_mode();
//	}
}
//------------------------------------------------------------------------------------
void vApplicationTickHook( void )
{
	sysTicks++;

}
//------------------------------------------------------------------------------------
// Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP().
//------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	// Es invocada si en algun context switch se detecta un stack corrompido !!
	// Cuando el sistema este estable la removemos.
	// En FreeRTOSConfig.h debemos habilitar
	// #define configCHECK_FOR_STACK_OVERFLOW          2

	xprintf_P( PSTR("PANIC:%s !!\r\n\0"),pcTaskName);

}
//------------------------------------------------------------------------------------
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
//------------------------------------------------------------------------------------
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
//------------------------------------------------------------------------------------

