/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
//
//! \addtogroup mqtt_server
//! @{
//
//*****************************************************************************
/* Standard includes                                                         */
#include <stdlib.h>

/* Kernel (Non OS/Free-RTOS/TI-RTOS) includes                                */
#include "pthread.h"
#include "mqueue.h"

/* Application includes                                                      */
#include "client_cbs.h"
#include "my_queue_files/mqtt_queue.h"
#include "my_queue_files/uart_queue.h"

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
#define APP_PRINT               Report

#define OS_WAIT_FOREVER         (0xFFFFFFFF)
#define OS_NO_WAIT              (0)
#define OS_OK                   (0)

#define MQTTClientCbs_ConnackRC(data) (data & 0xff) 
/**< CONNACK: Return Code (LSB) */

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

/* Message Queue                                                              */
extern mqd_t g_PBQueue;
extern char *topic[];
struct client_info client_info_table[MAX_CONNECTION];

//*****************************************************************************
//                 Queue external function
//*****************************************************************************
extern int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement);

//****************************************************************************
//                      CLIENT CALLBACKS
//****************************************************************************

//*****************************************************************************
//
//! Callback in case of various event (for clients connection with remote
//! broker)
//!
//! \param[in]  event       - is a event occurred
//! \param[in]  metaData    - is the pointer for the message buffer
//!                           (for this event)
//! \param[in]  metaDateLen - is the length of the message buffer
//! \param[in]  data        - is the pointer to the buffer for data
//!                           (for this event)
//! \param[in]  dataLen     - is the length of the buffer data
//!
//! return none
//
//*****************************************************************************

void MqttClientCallback(int32_t event,
                        void * metaData,
                        uint32_t metaDateLen,
                        void *data,
                        uint32_t dataLen)
{
    memset(payload_buff, '\0', BUFF_SIZE);

    switch((MQTTClient_EventCB)event)
    {
    case MQTTClient_OPERATION_CB_EVENT:
    {
        switch(((MQTTClient_OperationMetaDataCB *)metaData)->messageType)
        {
        case MQTTCLIENT_OPERATION_CONNACK:
        {
            uint16_t *ConnACK = (uint16_t*) data;

#ifdef UART_DEBUGGING
            sendMsgToUart("CONNACK\r\n\0");
#endif
            /* Check if Conn Ack return value is Success (0) or       */
            /* Error - Negative value                                 */
            if(0 == (MQTTClientCbs_ConnackRC(*ConnACK)))
            {
#ifdef UART_DEBUGGING
                sendMsgToUart("Connection Success\r\n\0");
#endif
            }
            else
            {
#ifdef UART_DEBUGGING
                sendMsgToUart("Connection Error\r\n\0");
#endif
            }
            break;
        }

        case MQTTCLIENT_OPERATION_EVT_PUBACK:
        {
#ifdef UART_DEBUGGING
            sendMsgToUart("PubAck\r\n\0");
#endif
            break;
        }

        case MQTTCLIENT_OPERATION_SUBACK:
        {
#ifdef UART_DEBUGGING
            sendMsgToUart("SubAck\r\n\0");
#endif
            break;
        }

        case MQTTCLIENT_OPERATION_UNSUBACK:
        {
#ifdef UART_DEBUGGING
            sendMsgToUart("UnsubAck\r\n\0");
#endif
            break;
        }

        default:
            break;
        }
        break;
    }
    case MQTTClient_RECV_CB_EVENT:
    {
        // get payload
        memcpy((void*) payload_buff, (const void*) data, dataLen);

        // Send payload to main task
        if(receivedMsg_MqttQueue(payload_buff) == QUEUE_FULL)
        {
#ifdef UART_DEBUGGING
            sendMsgToUart("Queue is full\r\n\0");
#endif
        }
        break;
    }
    case MQTTClient_DISCONNECT_CB_EVENT:
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("Bridge disconnect\r\n\0");
#endif
        break;
    }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
