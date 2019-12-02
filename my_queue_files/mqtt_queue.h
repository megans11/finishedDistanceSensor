/*
 * sensor_queue.h
 *
 *  Created on: Sep 8, 2019
 *      Author: Luke_
 */


// TODO: update for message control

#ifndef MESSAGE_QUEUE_H_
#define MESSAGE_QUEUE_H_

#include <FreeRTOS.h>
#include <queue.h>
#include "debug.h"

// message format
typedef struct {
   int  msg_type;
//   int type;
   int count;
   char topic[32];
   char payload[128];
} mqtt_msg_struct;

//#define DEBUG_MODE

// queue parameters
#define MQTT_QUEUE_LENGTH 16
#define MQTT_QUEUE_WIDTH sizeof(mqtt_msg_struct) // number of characters allowed

// Return value definitions
#define CREATE_QUEUE_FAILURE -1
#define CREATE_QUEUE_SUCCESS 0

#define QUEUE_FULL  -1
#define SENT_SUCCESS 0

#define READ_FAILURE -1
#define READ_SUCCESS  0

// message struct types
#define PUBLISH_MESSAGE           0
#define RECEIVED_MESSAGE          1
#define CLIENT_DISCONNECTION      3
#define RESET_PUSH_BUTTON_PRESSED 4
#define THREAD_TERMINATE_REQUEST  5

// Collection of all queue handles in project
QueueHandle_t mqttQueue;

// Public queue functions
int create_MqttQueue();
int sendMsg_MqttQueue(char* topic, char * type, char* action);
int receivedMsg_MqttQueue(char* payload);
int sendCmdMsg_MqttQueue(int type);
int readMsg_MqttQueue(mqtt_msg_struct *msg_buffer);


#endif /* SENSOR_QUEUE_H_ */
