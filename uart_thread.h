/*
 * uart_thread.h
 *
 *  Created on: Oct 9, 2019
 *      Author: lukeb
 */

#ifndef UART_THREAD_H_
#define UART_THREAD_H_


#include "debug.h"
#include "my_queue_files/uart_queue.h"

void *uartThread(void *arg0);

#endif /* UART_THREAD_H_ */
