/*
 * distance_task_queue.h
 *
 */

#ifndef DISTANCE_TASK_QUEUE_H_
#define DISTANCE_TASK_QUEUE_H_

#include <FreeRTOS.h>
#include <queue.h>
#include "../my_debug_files/distance_sensor_debug.h"

#include "debug.h"
#include "my_queue_files/basic_queue.h"
#include <time.h>

// message struct types
#define MSG_TYPE_TIMER 0
#define MSG_TYPE_RISE 1
#define MSG_TYPE_FALL 2
#define MSG_TYPE_STOP 3



// Collection of all queue handles in project
QueueHandle_t distanceTaskQueue;

// Public queue functions
int create_DistanceTaskQueue();
int captureRiseDistanceTaskQueue(struct timespec currentTime);
int captureFallDistanceTaskQueue(uint32_t interval);
int timerExpired_DistanceTaskQueue(unsigned long int time_elapsed);
int stop_DistanceTaskQueue();
int readMsg_DistanceTaskQueue(uint32_t *amount);


#endif /* DISTANCE_TASK_QUEUE_H_ */
