#ifndef DEBUG_H_
#define DEBUG_H_

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC32XX.h>
#include <ti/drivers/UART.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

#include "board.h"

UART_Handle UART0;
UART_Params uartParams;

#define UART_DEBUGGING


void dbgGPIOInit();
void dbgUARTInit();
void dbgUARTVal(char* outVal);
void dbgOutputLoc(unsigned int outLoc);
void errorRoutine(unsigned int error_hex);

#endif /* DEBUG_H_ */
