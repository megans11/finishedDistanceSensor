/*
 * main_thread.h
 *
 */

#ifndef MAIN_THREAD_H_
#define MAIN_THREAD_H_

/* Includes */
#include "debug.h"
#include "my_debug_files/distance_sensor_debug.h"
#include "my_callback_files/distance_sensor_timer.h"
#include "my_driver_files/distance_sensor_driver.h"
#include "my_queue_files/basic_queue.h"
#include "my_queue_files/distance_task_queue.h"
#include "find_distance_task_thread.h"
//#include "test_thread.h"

// Include local files
#include "Board.h"

//static int echo_rise;
//static int echo_fall;

void *mainThread(void *arg0);

#endif /* MAIN_THREAD_H_ */
