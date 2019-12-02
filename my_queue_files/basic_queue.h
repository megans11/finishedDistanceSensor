/*
 * basicQueue.h
 *
 *  Created on: Sep 26, 2019
 *      Author: Megan Salvatore
 *
 * This file contains wrappers for all of the freertos calls to prevent
 * duplication of code. All specific functions will be built on this generic
 */

#ifndef BASIC_QUEUE_H_
#define BASIC_QUEUE_H_

#include <FreeRTOS.h>
#include <queue.h>
#include "../my_debug_files/distance_sensor_debug.h"
#include "debug.h"

// message format
#define MSG_TYPE 0
#define MSG_DATA 1
typedef uint32_t message_type[2];

// queue parameters
#define QUEUE_LENGTH 64
#define QUEUE_WIDTH sizeof(message_type)

// Return value definitions
#define CREATE_QUEUE_FAILURE -1
#define CREATE_QUEUE_SUCCESS 0

#define QUEUE_FULL -1
#define SENT_SUCCESS 0

#define READ_FAILURE -1
#define READ_SUCCESS 0



// Public queue functions
int createQueue(QueueHandle_t* qh);
int sendMsgToQueue(QueueHandle_t qh, message_type* msg);
int sendMsgToQueue_ISR(QueueHandle_t qh, message_type* msg);
int readMsqFromQueue(QueueHandle_t qh, message_type *msg_buffer);


#endif /* MY_QUEUE_FILES_BASIC_QUEUE_H_ */
