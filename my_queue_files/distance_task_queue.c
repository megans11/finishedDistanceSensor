/*
 * distanceTaskQueue.c
 *
 */

#include <my_queue_files/distance_task_queue.h>
#include <my_driver_files/distance_sensor_driver.h>


/**
 * Wrapper for createQueue to create predefined queue1
 */
int create_DistanceTaskQueue(){
    return createQueue(&distanceTaskQueue);
}

int timerExpired_DistanceTaskQueue(unsigned long int time_elapsed) {

    message_type msg;
    msg[MSG_TYPE] = MSG_TYPE_TIMER;
    msg[MSG_DATA] = time_elapsed;

    return sendMsgToQueue_ISR(distanceTaskQueue, &msg);
}

int captureRiseDistanceTaskQueue(struct timespec currentTime)
{

    long long riseSeconds = currentTime.tv_sec;
    long riseNanoseconds = currentTime.tv_nsec;
    long long riseTime = 0;

//    char buffer[17];
//    snprintf(buffer, sizeof(buffer), "distance %ld", distance);
//    snprintf(buffer, sizeof(buffer), "distance %ld", distance);


//    char buffer2[60];
//    snprintf(buffer2, sizeof buffer2, "nano %ld\n", riseNanoseconds);
//    UART_write(distanceSensorUART, buffer2, sizeof(buffer2));

    riseTime = (riseSeconds * 1000000000) + riseNanoseconds;

    message_type msg;
    msg[MSG_TYPE] = MSG_TYPE_RISE;
//    msg[MSG_DATA] = time_elapsed;
    msg[MSG_DATA] = riseTime;

    return sendMsgToQueue_ISR(distanceTaskQueue, &msg);
}

int captureFallDistanceTaskQueue(uint32_t interval)
{

    char buffer[45];
    snprintf(buffer, sizeof(buffer), "interval %lld", (long long)interval);
    dbgUARTVal(buffer);


    message_type msg;
    msg[MSG_TYPE] = MSG_TYPE_FALL;
    msg[MSG_DATA] = interval;

    return sendMsgToQueue_ISR(distanceTaskQueue, &msg);
}


/*
 * Message send from timer callback.
 */
int stop_DistanceTaskQueue() {

    message_type msg;
    msg[MSG_TYPE] = MSG_TYPE_STOP;
    msg[MSG_DATA] = 0;

    return sendMsgToQueue_ISR(distanceTaskQueue, &msg);
}

/**
 * Blocking read from Queue 1. Fills correct buffer, return type specifies type received.
 */
int readMsg_DistanceTaskQueue(uint32_t *amount){
    message_type msg_buffer;

    // Debug before receiving from queue in ISR
    int read_status = readMsqFromQueue(distanceTaskQueue, &msg_buffer);

    // Block until message, check if valid
    if (read_status == READ_SUCCESS) {

        // Fill correct buffer
        switch (msg_buffer[MSG_TYPE]){
        case MSG_TYPE_TIMER:
            *amount = msg_buffer[MSG_DATA];
            return MSG_TYPE_TIMER;
        case MSG_TYPE_RISE:
            *amount = msg_buffer[MSG_DATA];
            return MSG_TYPE_RISE;
        case MSG_TYPE_FALL:
            *amount = msg_buffer[MSG_DATA];
            return MSG_TYPE_FALL;
        case MSG_TYPE_STOP:
            *amount = NULL;
            return MSG_TYPE_STOP;
        default:
            errorRoutine(READ_QUEUE_FAILED);
        }
    }

    //if no returns were called earlier, then read fail
    return READ_FAILURE;
}
