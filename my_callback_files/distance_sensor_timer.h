/*
 *  Created on: Sep 29, 2019
 *      Author: Megan Salvatore
 */

#ifndef DISTANCE_SENSOR_TIMER_H
#define DISTANCE_SENSOR_TIMER_H

#include <ti/drivers/Timer.h>
#include <ti/drivers/Capture.h>
#include "Board.h"
#include "debug.h"
#include <time.h>
#include "../my_debug_files/distance_sensor_debug.h"
#include "../my_queue_files/distance_task_queue.h"
//#include "main_thread.h"


#define DISTANCE_SENSOR_TIMER_PERIOD 10 // 10 microseconds
#define TIMER_FAILURE -1
#define TIMER_SUCCESS 0

Timer_Handle distance_sensor_timer;
Timer_Params distance_sensor_params;
Capture_Params distanceCaptureParams;
Capture_Handle distanceCapture;

int init_distanceSensorTimer();
void distanceSensorTimerCallback(Timer_Handle myHandle);
//void distanceCaptureCallback(Capture_Handle handle);
void distanceCaptureCallback(Capture_Handle handle, uint32_t interval);

#endif /* DISTANCE_SENSOR_TIMER_H_ */
