/*
 * find_distance_task_thread.h
 *
 */

#ifndef FIND_DISTANCE_TASK_THREAD_H_
#define FIND_DISTANCE_TASK_THREAD_H_

/* For usleep() */
//#include <unistd.h>
//#include <stdint.h>
//#include <stddef.h>

// Include local files
#include "Board.h"

#include "debug.h"
#include "my_debug_files/distance_sensor_debug.h"
#include "my_driver_files/distance_sensor_driver.h"
#include "my_queue_files/distance_task_queue.h"
#include "my_queue_files/mqtt_queue.h"


void *findDistanceTaskThread(void *arg0);

#endif /* FIND_DISTANCE_TASK_THREAD_H_ */
