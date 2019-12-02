/*
 * basic_queue.c
 *
 *  Created on: Sep 26, 2019
 *      Author: Megan
 */

#include <my_queue_files/basic_queue.h>


/**
 * Wrapper for createQueue
 */
int createQueue(QueueHandle_t* qh){
    *qh = xQueueCreate(QUEUE_LENGTH, QUEUE_WIDTH);

    if(qh != NULL)
        return CREATE_QUEUE_SUCCESS;
    else
        errorRoutine(READ_QUEUE_FAILED);

    return CREATE_QUEUE_FAILURE;
}

/**
 * Sends signed value - use when called from ISR
 * Returns FreeRTOS return value
 */
int sendMsgToQueue_ISR(QueueHandle_t qh, message_type* msg){

    // No higher task is woken in this ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Debug before sending within ISR/callback
    dbgOutputLoc(SENDING_TO_QUEUE_FROM_ISR);

    BaseType_t ret_val = xQueueSendFromISR(qh, (const void*) msg, &xHigherPriorityTaskWoken);

    // Debug after sending within ISR/callback
    dbgOutputLoc(SENT_TO_QUEUE_FROM_ISR);

    if (ret_val == errQUEUE_FULL)
        return QUEUE_FULL;
    else
        return SENT_SUCCESS;

}

/**
 * Sends signed value
 * Returns FreeRTOS return value
 */
int sendMsgToQueue(QueueHandle_t qh, message_type* msg){

    // Debug before sending within ISR/callback
//    dbgOutputLoc(SENDING_TO_QUEUE);

    BaseType_t ret_val = xQueueSend(qh, (const void*) msg, portMAX_DELAY );

    // Debug after sending within ISR/callback
//    dbgOutputLoc(SENT_TO_QUEUE);

    if (ret_val == errQUEUE_FULL)
        return QUEUE_FULL;
    else
        return SENT_SUCCESS;

}

/**
 * Blocking read from Queue. Fills message buffer
 */
int readMsqFromQueue(QueueHandle_t qh, message_type *msg_buffer){

    // Debug before receiving from queue in ISR
    dbgOutputLoc(WAITING_QUEUE_RECEIVE);

    // Block until message, check if valid
    if (xQueueReceive(qh, (void*) msg_buffer, portMAX_DELAY) == pdTRUE) {
        dbgOutputLoc(RECEIVED_FROM_QUEUE);
        return READ_SUCCESS;
    }
    else
        errorRoutine(READ_QUEUE_FAILED);

    return READ_FAILURE; // shouldn't be reached
}
