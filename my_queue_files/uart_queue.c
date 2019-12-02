/*
 * uart_queue.c
 *
 *  Created on: Oct 9, 2019
 *      Author: lukeb
 */



#include "my_queue_files/uart_queue.h"


/**
 * Wrapper for createQueue
 */
int createUartQueue(){
    uartQueue = xQueueCreate(UART_QUEUE_LENGTH, UART_QUEUE_WIDTH);

    if(uartQueue != NULL)
        return CREATE_QUEUE_SUCCESS;
    else
        errorRoutine(0);

    return CREATE_QUEUE_FAILURE;
}

/**
 * Sends signed value
 * Returns FreeRTOS return value
 */
int sendMsgToUart(char* debug_stmt){

    // Debug before sending within ISR/callback
    char msg[UART_QUEUE_WIDTH];
    memset(msg,'\0',UART_QUEUE_WIDTH);
    int msg_len = strlen(debug_stmt);
    if (msg_len > UART_QUEUE_WIDTH)
        strncpy(msg, debug_stmt, UART_QUEUE_WIDTH);
    else
        strncpy(msg, debug_stmt, msg_len);

    BaseType_t ret_val = xQueueSend(uartQueue, (const void*) msg, portMAX_DELAY );

    // Debug after sending within ISR/callback

    if (ret_val == errQUEUE_FULL)
        return QUEUE_FULL;
    else
        return SENT_SUCCESS;

}

/**
 * Blocking read from Queue. Fills message buffer
 */
int readMsqFromUartQueue(char* debug_stmt){

    // Debug before receiving from queue in ISR

    // Block until message, check if valid
    if (xQueueReceive(uartQueue, (void*) debug_stmt, portMAX_DELAY) == pdTRUE) {
        return READ_SUCCESS;
    }
    else
        errorRoutine(0);

    return READ_FAILURE; // shouldn't be reached
}




