/**
 * debug.c
 *
 * Created on 9/10/19
 * Author: Megan Salvatore
 */
#include "debug.h"

/**
 * Init function for UART
 */

void dbgUARTInit(){
//    UART_init();

    /* Create a UART with the parameters below. */
    UART_Params_init(&uartParams);
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.baudRate = 115200;
    uartParams.readEcho = UART_ECHO_OFF;

    /* Open UART with header pin selected in sysconfig i.e. UART0 */
    UART0 = UART_open(UART_USB, &uartParams);
    if (UART0 == NULL) {
        /* Error creating UART */
        while (1);
    }

    /* Block read since UART is write-only for this application */
    UART_control(UART0, UART_CMD_RXDISABLE, NULL);
}

void dbgGPIOInit(){
//    GPIO_init();
//
//    GPIO_setConfig(DEBUG_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(DEBUG_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(DEBUG_2, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(DEBUG_3, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(DEBUG_4, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(DEBUG_STATUS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
}

/**
    This value will be written to the UART.
    You'll need to select an appropriate header pin for the UART
    and configure it via sysconfig.
**/
void dbgUARTVal(char* outVal)
{
    /* Write to UART0 */
    UART_write(UART0, (void *) outVal, strlen(outVal));
}

/**
    The value of outLoc must be <= 127. This routine must
    verify this and call your "fail" routine below if it's not in range.
    You'll need to select 8 appropriate header pins for the GPIO and configure
    them via sysconfig.
**/
void dbgOutputLoc(unsigned int outLoc)
{
//    if (outLoc > 31) // call fail routine -> out of range
//    {
//        errorRoutine(outLoc);
//    }
//    else //print to GPIO
//    {
//        // configured GPIO pins in sysconfig
//        GPIO_write(DEBUG_0, (outLoc >> 0) & 0x1);
//        GPIO_write(DEBUG_1, (outLoc >> 1) & 0x1);
//        GPIO_write(DEBUG_2, (outLoc >> 2) & 0x1);
//        GPIO_write(DEBUG_3, (outLoc >> 3) & 0x1);
//        GPIO_write(DEBUG_4, (outLoc >> 4) & 0x1);
//        GPIO_toggle(DEBUG_STATUS);
//    }

}

/**
 * Stop other code processes, and light up LED to show the stop.
 * The error should print out the constant location, plus the method it is in
 */
void errorRoutine(unsigned int outLoc)
{
    dbgOutputLoc(outLoc);

    // suspend all threads and disable interrupts
//    vTaskSuspendAll();
//    stop_timer();


}
