/*
 * sensor_queue.c
 *
 *  Created on: Sep 8, 2019
 *      Author: Luke Beckwith
 */
#include <my_queue_files/mqtt_queue.h>


/**
 * Wrapper for createQueue to create predefined queue1
 */
int create_MqttQueue(){
    mqttQueue = xQueueCreate(MQTT_QUEUE_LENGTH, MQTT_QUEUE_WIDTH);

    if(mqttQueue != NULL)
        return CREATE_QUEUE_SUCCESS;
    else
        errorRoutine(0);

    return CREATE_QUEUE_FAILURE;
}

/**
 * Sends signed value - use when called from ISR
 * Returns FreeRTOS return value
 */
int sendMsg_MqttQueue(char* topic, char * type, char* action){

    static int msg_count = 0;

    // Debug before sending within ISR/callback
    mqtt_msg_struct msg;

//    memset(msg.payload,'\0',sizeof(msg.payload));

//    strncpy(msg.topic, topic, strlen(topic));

    memset(msg.topic,'\0',sizeof(msg.topic));
    strncpy(msg.topic, topic, strlen(topic));
    sprintf(msg.payload, "{\"type\": \"%s\", \"action\": \"%s\", \"count\": %d}", type, action, msg_count);
//    msg.type = type;
    msg.msg_type = PUBLISH_MESSAGE;
    msg.count = msg_count;

    BaseType_t ret_val = xQueueSend(mqttQueue, (const void*) &msg, pdFALSE);

    msg_count++;
    // Debug after sending within ISR/callback

    if (ret_val == errQUEUE_FULL)
        return QUEUE_FULL;
    else
        return SENT_SUCCESS;

}

int receivedMsg_MqttQueue(char* payload){

    // Debug before sending within ISR/callback
    mqtt_msg_struct msg;

    strncpy(msg.payload, payload, strlen(payload));
    msg.msg_type = RECEIVED_MESSAGE;

    BaseType_t ret_val = xQueueSendFromISR(mqttQueue, (const void*) &msg, pdFALSE);

    // Debug after sending within ISR/callback

    if (ret_val == errQUEUE_FULL)
        return QUEUE_FULL;
    else
        return SENT_SUCCESS;

}


int sendCmdMsg_MqttQueue(int type){

    // Debug before sending within ISR/callback
    mqtt_msg_struct msg;
    msg.msg_type = type;

    BaseType_t ret_val = xQueueSendFromISR(mqttQueue, (const void*) &msg, pdFALSE);

    // Debug after sending within ISR/callback

    if (ret_val == errQUEUE_FULL)
        return QUEUE_FULL;
    else
        return SENT_SUCCESS;

}

/**
 * Blocking read from Queue 1. Fills correct buffer, return type specifies type received.
 */
int readMsg_MqttQueue(mqtt_msg_struct *msg_buffer){
    // Debug before receiving from queue in ISR

    int read_status = xQueueReceive(mqttQueue, (void*) msg_buffer, portMAX_DELAY);

    // Block until message, check if valid
    if (read_status == pdTRUE) {
        //
       return READ_SUCCESS;

    }
    return READ_FAILURE;
}

