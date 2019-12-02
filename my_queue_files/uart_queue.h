/*
 * uart_queue.h
 *
 *  Created on: Oct 9, 2019
 *      Author: lukeb
 */

#ifndef MY_QUEUE_FILES_UART_QUEUE_H_
#define MY_QUEUE_FILES_UART_QUEUE_H_


#include <FreeRTOS.h>
#include <queue.h>
#include "debug.h"

// queue parameters
#define UART_QUEUE_LENGTH 16
#define UART_QUEUE_WIDTH 128 // number of characters allowed

// Return value definitions
#define CREATE_QUEUE_FAILURE -1
#define CREATE_QUEUE_SUCCESS 0

#define QUEUE_FULL -1
#define SENT_SUCCESS 0

#define READ_FAILURE -1
#define READ_SUCCESS 0

QueueHandle_t uartQueue;

// Public queue functions
int createUartQueue();
int sendMsgToUart(char* debug_stmt);
int readMsqFromUartQueue(char* debug_stmt);



#endif /* MY_QUEUE_FILES_UART_QUEUE_H_ */
