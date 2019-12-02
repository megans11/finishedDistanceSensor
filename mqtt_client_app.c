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

/*****************************************************************************

   Application Name     -   MQTT Client
   Application Overview -   The device is running a MQTT client which is
                           connected to the online broker. Three LEDs on the
                           device can be controlled from a web client by
                           publishing msg on appropriate topics. Similarly,
                           message can be published on pre-configured topics
                           by pressing the switch buttons on the device.

   Application Details  - Refer to 'MQTT Client' README.html

*****************************************************************************/
//*****************************************************************************
//
//! \addtogroup mqtt_server
//! @{
//
//*****************************************************************************
/* Standard includes                                                         */
#include <stdlib.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <unistd.h>

/* TI-Driver includes                                                        */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Simplelink includes                                                       */
#include <ti/drivers/net/wifi/simplelink.h>

/* SlNetSock includes                                                        */
#include <ti/drivers/net/wifi/slnetifwifi.h>

/* MQTT Library includes                                                     */
#include <ti/net/mqtt/mqttclient.h>

/* Common interface includes                                                 */
#include "network_if.h"

/* Application includes                                                      */
#include "Board.h"
#include "client_cbs.h"
#include "debug.h"
#include "my_queue_files/mqtt_queue.h"
#include "my_queue_files/uart_queue.h"
//#include "arm_control_queue.h"
//#include "servo_control_queue.h"
#include "jsonParse.h"
#include "my_callback_files/distance_sensor_timer.h"

//#include "timer.h"
//
//#include "arm_control_queue.h"



//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
/* enables secured client                                                    */
//#define SECURE_CLIENT

/* enables client authentication by the server                               */
//#define CLNT_USR_PWD

#define CLIENT_INIT_STATE        (0x01)
#define MQTT_INIT_STATE          (0x04)

#define APPLICATION_VERSION      "1.1.1"
#define APPLICATION_NAME         "MQTT client"

#define SLNET_IF_WIFI_PRIO       (5)

/* Operate Lib in MQTT 3.1 mode.                                             */
#define MQTT_3_1_1               false
#define MQTT_3_1                 true

#define WILL_TOPIC               "Client"
#define WILL_MSG                 "Client Stopped"
#define WILL_QOS                 MQTT_QOS_2
#define WILL_RETAIN              false

/* Defining Broker IP address and port Number                                */
//#define SERVER_ADDRESS           "messagesight.demos.ibm.com"
#define SERVER_ADDRESS           "192.168.2.5"
#define SERVER_IP_ADDRESS        "192.168.2.5"
#define PORT_NUMBER              1883
#define SECURED_PORT_NUMBER      8883
#define LOOPBACK_PORT            1882

/* Clean session flag                                                        */
#define CLEAN_SESSION            true

/* Retain Flag. Used in publish message.                                     */
#define RETAIN_ENABLE            1

/* Defining Number of subscription topics                                    */
#define SUBSCRIPTION_TOPIC_COUNT 1

/* Defining Subscription Topic Values                                        */
#define SUBSCRIPTION_TOPIC0      "distance"


/* Defining Publish Topic Values                                             */
#define PUBLISH_TOPIC0           "distance"
#define PUBLISH_TOPIC0_DATA      "distance data"
#define PUBLISH_TOPIC1           "debug"
#define PUBLISH_TOPIC1_DATA      "debug data"

/* Spawn task priority and Task and Thread Stack Size                        */
#define TASKSTACKSIZE            2048
#define RXTASKSIZE               4096
#define MQTTTHREADSIZE           2048
#define SPAWN_TASK_PRIORITY      2
#define THREADSIZE               2048

/* secured client requires time configuration, in order to verify server     */
/* certificate validity (date).                                              */

/* Day of month (DD format) range 1-31                                       */
#define DAY                      1
/* Month (MM format) in the range of 1-12                                    */
#define MONTH                    5
/* Year (YYYY format)                                                        */
#define YEAR                     2017
/* Hours in the range of 0-23                                                */
#define HOUR                     12
/* Minutes in the range of 0-59                                              */
#define MINUTES                  33
/* Seconds in the range of 0-59                                              */
#define SEC                      21

/* Number of files used for secure connection                                */
#define CLIENT_NUM_SECURE_FILES  1

/* Expiration value for the timer that is being used to toggle the Led.      */
#define TIMER_EXPIRATION_VALUE   100 * 1000000

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
void * MqttClient(void *pvParameters);
void Mqtt_ClientStop(uint8_t disconnect);
void Mqtt_ServerStop();
void Mqtt_Stop();
void Mqtt_start();
int32_t Mqtt_IF_Connect();
int32_t MqttServer_start();
int32_t MqttClient_start();


int32_t MQTT_connect();
int16_t MQTT_subscribe();
int16_t MQTT_publish(char * pubTopic, char * pubData);

extern void *findDistanceTaskThread(void *arg0);
//extern void *sensorMainThread(void *arg0);
//extern void *armControlThread(void *arg0);
//extern void *servoPositionThread(void *arg0);
//**************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

/* Connection state: (0) - connected, (negative) - disconnected              */
int32_t gApConnectionState = -1;
uint32_t gInitState = 0;
uint32_t memPtrCounterfree = 0;
static MQTTClient_Handle gMqttClient;
MQTTClient_Params MqttClientExmple_params;
unsigned short g_usTimerInts;

/* Receive task handle                                                       */
pthread_t g_rx_task_hndl = (pthread_t) NULL;

/* AP Security Parameters                                                    */
SlWlanSecParams_t SecurityParams = { 0 };

/* Client ID                                                                 */
/* If ClientId isn't set, the MAC address of the device will be copied into  */
/* the ClientID parameter.                                                   */
char ClientId[13] = {'\0'};


/* Subscription topics and qos values                                        */
char *topic[SUBSCRIPTION_TOPIC_COUNT] =
{ SUBSCRIPTION_TOPIC0};

unsigned char qos[SUBSCRIPTION_TOPIC_COUNT] =
{ MQTT_QOS_2};


/* Message Queue                                                             */
pthread_t mqttThread = (pthread_t) NULL;



/* Printing new line                                                         */
char lineBreak[] = "\n\r";

//*****************************************************************************
//                 Banner VARIABLES
//*****************************************************************************
#ifdef  SECURE_CLIENT

char *Mqtt_Client_secure_files[CLIENT_NUM_SECURE_FILES] = {"ca-cert.pem"};

/*Initialization structure to be used with sl_ExtMqtt_Init API. In order to  */
/*use secured socket method, the flag MQTTCLIENT_NETCONN_SEC, cipher,        */
/*n_files and secure_files must be configured.                               */
/*certificates also must be programmed  ("ca-cert.pem").                     */
/*The first parameter is a bit mask which configures server address type and */
/*security mode.                                                             */
/*Server address type: IPv4, IPv6 and URL must be declared with The          */
/*corresponding flag.                                                        */
/*Security mode: The flag MQTTCLIENT_NETCONN_SEC enables the security (TLS)  */
/*which includes domain name verification and certificate catalog            */
/*verification, those verifications can be disabled by adding to the bit mask*/
/*MQTTCLIENT_NETCONN_SKIP_DOMAIN_NAME_VERIFICATION and                       */
/*MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION flags             */
/*Example: MQTTCLIENT_NETCONN_IP6 | MQTTCLIENT_NETCONN_SEC |                 */
/*MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION                   */
/*For this bit mask, the IPv6 address type will be in use, the security      */
/*feature will be enable and the certificate catalog verification will be    */
/*skipped.                                                                   */
/*Note: The domain name verification requires URL Server address type        */
/*      otherwise, this verification will be disabled.                       */
MQTTClient_ConnParams Mqtt_ClientCtx =
{
    MQTTCLIENT_NETCONN_IP4 | MQTTCLIENT_NETCONN_SEC,
    SERVER_IP_ADDRESS,  //SERVER_ADDRESS,
    SECURED_PORT_NUMBER, //  PORT_NUMBER
    SLNETSOCK_SEC_METHOD_SSLv3_TLSV1_2,
    SLNETSOCK_SEC_CIPHER_FULL_LIST,
    CLIENT_NUM_SECURE_FILES,
    Mqtt_Client_secure_files
};

void setTime()
{
    SlDateTime_t dateTime = {0};
    dateTime.tm_day = (uint32_t)DAY;
    dateTime.tm_mon = (uint32_t)MONTH;
    dateTime.tm_year = (uint32_t)YEAR;
    dateTime.tm_hour = (uint32_t)HOUR;
    dateTime.tm_min = (uint32_t)MINUTES;
    dateTime.tm_sec = (uint32_t)SEC;
    sl_DeviceSet(SL_DEVICE_GENERAL, SL_DEVICE_GENERAL_DATE_TIME,
                 sizeof(SlDateTime_t), (uint8_t *)(&dateTime));
}

#else
MQTTClient_ConnParams Mqtt_ClientCtx =
{
    MQTTCLIENT_NETCONN_IP4,
    SERVER_IP_ADDRESS,
    PORT_NUMBER, 0, 0, 0,
    NULL
};
#endif

/* Initialize the will_param structure to the default will parameters        */
MQTTClient_Will will_param =
{
    WILL_TOPIC,
    WILL_MSG,
    WILL_QOS,
    WILL_RETAIN
};

void * MqttClientThread(void * pvParameters)
{
    mqtt_msg_struct queueElemRecv;

    MQTTClient_run((MQTTClient_Handle)pvParameters);

    /*write message indicating disconnect Broker message.                   */
    if(sendCmdMsg_MqttQueue(CLIENT_DISCONNECTION) == QUEUE_FULL)
    {
        readMsg_MqttQueue(&queueElemRecv);
        sendCmdMsg_MqttQueue(CLIENT_DISCONNECTION);
    }

    pthread_exit(0);

    return(NULL);
}

//*****************************************************************************
//
//! Task implementing MQTT Server plus client bridge
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt client ans server libraries and set up MQTT
//!       with the remote broker.
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void * MqttClient(void *pvParameters)
{
    sendMsgToUart("Beginning of MqttClient\r\n\0");

    mqtt_msg_struct msg_buffer;
    long lRetVal = -1;

    /*Initializing Client and Subscribing to the Broker.                     */
    if(gApConnectionState >= 0)
    {
        lRetVal = MqttClient_start();
        if(lRetVal == -1)
        {
#ifdef UART_DEBUGGING
        sendMsgToUart("MQTT lib init fail\r\n\0");
#endif
            pthread_exit(0);
            return(NULL);
        }
    }
    
    //Counters for publish and receive
    /*handling the signals from various callbacks including the push button  */
    /*prompting the client to publish a msg on PUB_TOPIC OR msg received by  */
    /*the server on enrolled topic(for which the on-board client ha enrolled)*/
    /*from a local client(will be published to the remote broker by the      */
    /*client) OR msg received by the client from the remote broker (need to  */
    /*be sent to the server to see if any local client has subscribed on the */
    /*same topic).
     */
    char myType[12];
    char myAction[12];
    char myBoard[12];
    int myCount;
    char msg[64];

    pthread_t Fthread;
   pthread_attr_t FAttrs;
   struct sched_param FpriParam;

   /* Set priority and stack size attributes */
   pthread_attr_init(&FAttrs);
   FpriParam.sched_priority = 1;

   int detachState = PTHREAD_CREATE_DETACHED;
   int retc = pthread_attr_setdetachstate(&FAttrs, detachState);
   if(retc != 0)
   {
       /* pthread_attr_setdetachstate() failed */
       while(1)
       {
           ;
       }
   }

   pthread_attr_setschedparam(&FAttrs, &FpriParam);

   retc |= pthread_attr_setstacksize(&FAttrs, 2048);
   if(retc != 0)
   {
       /* pthread_attr_setstacksize() failed */
       while(1)
       {
           ;
       }
   }

   retc = pthread_create(&Fthread, &FAttrs, findDistanceTaskThread, NULL);
   if(retc != 0)
   {
       /* pthread_create() failed */
       while(1)
       {
           ;
       }
   }


#ifdef UART_DEBUGGING
    sendMsgToUart(" init thread started\r\n\0");
#endif


    for(;;)
    {
        /*waiting for signals                                                */
        lRetVal = readMsg_MqttQueue(&msg_buffer);

        sendMsgToUart("got the messages\r\n\0");

        if (lRetVal == READ_SUCCESS) {
            switch(msg_buffer.msg_type)
            {
            case PUBLISH_MESSAGE:
                sendMsgToUart("in publish\r\n\0");

                /*send publish message                                       */
                lRetVal = MQTT_publish(msg_buffer.topic, msg_buffer.payload);
                break;

                /*msg received by client from remote broker (on a topic      */
                /*subscribed by local client)                                */
                case RECEIVED_MESSAGE:
                    parseAction(msg_buffer.payload, myType, myAction, myBoard, &myCount);
#ifdef UART_DEBUGGING
                    sprintf(msg, "Type: %s, Action: %s, Board: %s, count: %d\r\n\0",myType,myAction, myBoard, myCount);
                    sendMsgToUart(msg);

#endif

                    break;


                /*On-board client disconnected from remote broker, only      */
                /*local MQTT network will work                               */
                case CLIENT_DISCONNECTION:
#ifdef UART_DEBUGGING
                    sendMsgToUart("CLIENT DISCONNECT\r\n\0");
#endif
                    break;

                case THREAD_TERMINATE_REQUEST:
                    pthread_exit(0);
                    return(NULL);

                default:
                    break;
            }
        }
    }
}

//*****************************************************************************
//
//! This function connect the MQTT device to an AP with the SSID which was
//! configured in SSID_NAME definition which can be found in Network_if.h file,
//! if the device can't connect to to this AP a request from the user for other
//! SSID will appear.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
int32_t Mqtt_IF_Connect()
{
    int32_t lRetVal;
    char SSID_Remote_Name[32];
    int8_t Str_Length;

    memset(SSID_Remote_Name, '\0', sizeof(SSID_Remote_Name));
    Str_Length = strlen(SSID_NAME);

    if(Str_Length)
    {
        /*Copy the Default SSID to the local variable                        */
        strncpy(SSID_Remote_Name, SSID_NAME, Str_Length);
    }


    /*Reset The state of the machine                                         */
    Network_IF_ResetMCUStateMachine();

    /*Start the driver                                                       */
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
        return(-1);
    }

    /*Initialize AP security params                                          */
    SecurityParams.Key = (signed char *) SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /*Connect to the Access Point                                            */
    lRetVal = Network_IF_ConnectAP(SSID_Remote_Name, SecurityParams);
    if(lRetVal < 0)
    {
        return(-1);
    }

    return(0);
}

//*****************************************************************************
//!
//! MQTT Start - Initialize and create all the items required to run the MQTT
//! protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void Mqtt_start()
{
    int32_t threadArg = 100;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int32_t retc = 0;

    /*sync object for inter thread communication                             */

    if (create_MqttQueue() == CREATE_QUEUE_FAILURE) {
        gInitState &= ~MQTT_INIT_STATE;
#ifdef UART_DEBUGGING
        sendMsgToUart("queue create fail\r\n\0");
#endif
        return;
    }

    /*Set priority and stack size attributes                                 */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 3;
    retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, MQTTTHREADSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);

    if(retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
#ifdef UART_DEBUGGING
        sendMsgToUart("mqtt thread create failed\r\n\0");
#endif
        return;
    }

    retc = pthread_create(&mqttThread, &pAttrs, MqttClient, (void *) &threadArg);
    if(retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
#ifdef UART_DEBUGGING
        sendMsgToUart("mqtt thread create failed\r\n\0");
#endif
        return;
    }

    // signal that initalization was done
    gInitState &= ~MQTT_INIT_STATE;
}

//*****************************************************************************
//!
//! MQTT Stop - Close the client instance and free all the items required to
//! run the MQTT protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_Stop()
{
    mqtt_msg_struct queueElemRecv;

    if(gApConnectionState >= 0)
    {
        Mqtt_ClientStop(1);
    }

    /*write message indicating publish message                               */
    if(sendCmdMsg_MqttQueue(THREAD_TERMINATE_REQUEST))
    {
        readMsg_MqttQueue(&queueElemRecv);
        sendCmdMsg_MqttQueue(THREAD_TERMINATE_REQUEST);
    }

    sleep(2);

    sl_Stop(SL_STOP_TIMEOUT);
#ifdef UART_DEBUGGING
        sendMsgToUart("Client stop complete\r\n\0");
#endif

}

int32_t MqttClient_start()
{
    int32_t lRetVal = -1;
    int32_t iCount = 0;

    int32_t threadArg = 100;
    pthread_attr_t pAttrs;
    struct sched_param priParam;

    MqttClientExmple_params.clientId = ClientId;
    MqttClientExmple_params.connParams = &Mqtt_ClientCtx;
    MqttClientExmple_params.mqttMode31 = MQTT_3_1;
    MqttClientExmple_params.blockingSend = true;

    gInitState |= CLIENT_INIT_STATE;

    /*Initialize MQTT client lib                                             */
    gMqttClient = MQTTClient_create(MqttClientCallback,
                                    &MqttClientExmple_params);
    if(gMqttClient == NULL)
    {
        /*lib initialization failed                                          */
        gInitState &= ~CLIENT_INIT_STATE;
        return(-1);
    }

    /*Open Client Receive Thread start the receive task. Set priority and    */
    /*stack size attributes                                                  */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 4;
    lRetVal = pthread_attr_setschedparam(&pAttrs, &priParam);
    lRetVal |= pthread_attr_setstacksize(&pAttrs, RXTASKSIZE);
    lRetVal |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    lRetVal |=
        pthread_create(&g_rx_task_hndl, &pAttrs, MqttClientThread,
                       (void *) &threadArg);
    if(lRetVal != 0)
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("Client thr fail\r\n\0");
#endif
        gInitState &= ~CLIENT_INIT_STATE;
        return(-1);
    }
#ifdef SECURE_CLIENT
    setTime();
#endif

    /*setting will parameters                                                */
    MQTTClient_set(gMqttClient, MQTTClient_WILL_PARAM, &will_param,
                   sizeof(will_param));

#ifdef CLNT_USR_PWD
    /*Set user name for client connection                                    */
    MQTTClient_set(gMqttClient, MQTTClient_USER_NAME, (void *)ClientUsername,
                   strlen(
                       (char*)ClientUsername));

    /*Set password                                                           */
    MQTTClient_set(gMqttClient, MQTTClient_PASSWORD, (void *)ClientPassword,
                   strlen(
                       (char*)ClientPassword));
#endif
    /*Initiate MQTT Connect                                                  */
    if(gApConnectionState >= 0)
    {
#if CLEAN_SESSION == false
        bool clean = CLEAN_SESSION;
        MQTTClient_set(gMqttClient, MQTTClient_CLEAN_CONNECT, (void *)&clean,
                       sizeof(bool));
#endif
        /*The return code of MQTTClient_connect is the ConnACK value that
           returns from the server */
        lRetVal = MQTTClient_connect(gMqttClient);

        /*negative lRetVal means error,
           0 means connection successful without session stored by the server,
           greater than 0 means successful connection with session stored by
           the server */
        if(0 > lRetVal)
        {
            /*lib initialization failed                                      */
#ifdef UART_DEBUGGING
            sendMsgToUart("Broker conn fail\r\n\0");
#endif
        }
        /*Subscribe to topics when session is not stored by the server       */
        if((0 <= lRetVal) && (0 == lRetVal))
        {
            uint8_t subIndex;
            MQTTClient_SubscribeParams subscriptionInfo[
                SUBSCRIPTION_TOPIC_COUNT];

            for(subIndex = 0; subIndex < SUBSCRIPTION_TOPIC_COUNT; subIndex++)
            {
                subscriptionInfo[subIndex].topic = topic[subIndex];
                subscriptionInfo[subIndex].qos = qos[subIndex];
            }

            if(MQTTClient_subscribe(gMqttClient, subscriptionInfo,
                                    SUBSCRIPTION_TOPIC_COUNT) < 0)
            {
#ifdef UART_DEBUGGING
                sendMsgToUart("sub fail\r\n\0");
#endif
                MQTTClient_disconnect(gMqttClient);
            }
            else
            {
                for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
                {
#ifdef UART_DEBUGGING
                    sendMsgToUart("sub success\r\n\0");
#endif
                }
            }
        }
    }

    gInitState &= ~CLIENT_INIT_STATE;

    sleep(1);

    return(0);
}

//*****************************************************************************
//!
//! MQTT Client stop - Unsubscribe from the subscription topics and exit the
//! MQTT client lib.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_ClientStop(uint8_t disconnect)
{
    uint32_t iCount;

    MQTTClient_UnsubscribeParams subscriptionInfo[SUBSCRIPTION_TOPIC_COUNT];

    for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
        subscriptionInfo[iCount].topic = topic[iCount];
    }

    MQTTClient_unsubscribe(gMqttClient, subscriptionInfo,
                           SUBSCRIPTION_TOPIC_COUNT);
    for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("unsub\r\n\0");
#endif
    }

    /*exiting the Client library                                             */
    MQTTClient_delete(gMqttClient);
}


//*****************************************************************************
//!
//! Set the ClientId with its own mac address
//! This routine converts the mac address which is given
//! by an integer type variable in hexadecimal base to ASCII
//! representation, and copies it into the ClientId parameter.
//!
//! \param  macAddress  -   Points to string Hex.
//!
//! \return void.
//!
//*****************************************************************************
int32_t SetClientIdNamefromMacAddress()
{
    int32_t ret = 0;
    uint8_t Client_Mac_Name[2];
    uint8_t Index;
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint8_t macAddress[SL_MAC_ADDR_LEN];

    /*Get the device Mac address */
    ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);

    /*When ClientID isn't set, use the mac address as ClientID               */
    if(ClientId[0] == '\0')
    {
        /*6 bytes is the length of the mac address                           */
        for(Index = 0; Index < SL_MAC_ADDR_LEN; Index++)
        {
            /*Each mac address byte contains two hexadecimal characters      */
            /*Copy the 4 MSB - the most significant character                */
            Client_Mac_Name[0] = (macAddress[Index] >> 4) & 0xf;
            /*Copy the 4 LSB - the least significant character               */
            Client_Mac_Name[1] = macAddress[Index] & 0xf;

            if(Client_Mac_Name[0] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index] = Client_Mac_Name[0] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index] = Client_Mac_Name[0] + '0';
            }
            if(Client_Mac_Name[1] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + '0';
            }
        }
    }
    return(ret);
}

//*****************************************************************************
//!
//! Utility function which Display the app banner
//!
//! \param[in] appName     -  holds the application name.
//! \param[in] appVersion  -  holds the application version.
//!
//! \return none.
//!
//*****************************************************************************



void mainThread(void * args)
{
    dbgUARTVal("in main thread");
//    sleep(2);

//
//    #ifdef UART_DEBUGGING
//        sendMsgToUart("mainThread started\r\n\0");
//    #endif

//    sleep(3);

//    createUartQueue();
//    dbgUARTInit();

    pthread_t spawn_thread = (pthread_t) NULL;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;
    int32_t retc = 0;

    /*Initialize SlNetSock layer with CC31xx/CC32xx interface */
    SlNetIf_init(0);
    SlNetIf_add(SLNETIF_ID_1, "CC32xx",
                (const SlNetIf_Config_t *)&SlNetIfConfigWifi,
                SLNET_IF_WIFI_PRIO);

    SlNetSock_init(0);
    SlNetUtil_init(0);

    SPI_init();
    GPIO_init();

    /*Create the sl_Task                                                     */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASKSTACKSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs_spawn, PTHREAD_CREATE_DETACHED);

    retc = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if(retc != 0)
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("simplelink failed\r\n\0");
#endif
        return;
    }

    retc = sl_Start(0, 0, 0);

//    sleep(3);

    if(retc < 0)
    {
        /*Handle Error */
#ifdef UART_DEBUGGING
        sendMsgToUart("sl Start failed\r\n\0");
#endif
        return;
    }

    /*Set the ClientId with its own mac address */
    retc |= SetClientIdNamefromMacAddress();


    retc = sl_Stop(SL_STOP_TIMEOUT);
    if(retc < 0)
    {
        /*Handle Error */
#ifdef UART_DEBUGGING
        sendMsgToUart("\n sl_Stop failed\r\n\0");
#endif
        return;
    }

    if(retc < 0)
    {
        /*Handle Error */
#ifdef UART_DEBUGGING
        sendMsgToUart("cant retrieve dev info\r\n\0");
#endif
        return;
    }

    topic[0] = SUBSCRIPTION_TOPIC0;
    gInitState = 0;

    /*Connect to AP                                                      */
    gApConnectionState = Mqtt_IF_Connect();

    gInitState |= MQTT_INIT_STATE;
    /*Run MQTT Main Thread (it will open the Client and Server)          */
    Mqtt_start();

    /*Wait for init to be completed!!!                                   */
    while(gInitState != 0)
    {
        sleep(1);
    }

#ifdef UART_DEBUGGING
    sendMsgToUart("mainThread finished\r\n\0");
#endif

    return;
}

/**
 * New version of the MQTT connecting method.
 */
int32_t MQTT_connect()
{
    int32_t lRetVal;
    char SSID_Remote_Name[32];
    int8_t Str_Length;

    memset(SSID_Remote_Name, '\0', sizeof(SSID_Remote_Name));
    Str_Length = strlen(SSID_NAME);

    if(Str_Length)
    {
        /*Copy the Default SSID to the local variable                        */
        strncpy(SSID_Remote_Name, SSID_NAME, Str_Length);
    }

    /*Reset The state of the machine                                         */
    Network_IF_ResetMCUStateMachine();

    /*Start the driver                                                       */
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("SimpleLink Failed\r\n\0");
#endif
        return(-1);
    }

    /*Initialize AP security params                                          */
    SecurityParams.Key = (signed char *) SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /*Connect to the Access Point                                            */
    lRetVal = Network_IF_ConnectAP(SSID_Remote_Name, SecurityParams);
    if(lRetVal < 0)
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("Connection AP failed\r\n\0");
#endif
        return(-1);
    }

        sleep(1);

    return(0);
}

/**
 * New version of subscribing
 */
int16_t MQTT_subscribe()
{
    sendMsgToUart("Beginning of subscribe\r\n\0");
    /*Subscribe to topics when session is not stored by the server       */
    uint8_t subIndex;
    MQTTClient_SubscribeParams subscriptionInfo[
        SUBSCRIPTION_TOPIC_COUNT];
    uint32_t iCount;

    for(subIndex = 0; subIndex < SUBSCRIPTION_TOPIC_COUNT; subIndex++)
    {
        sendMsgToUart("in the loop\r\n\0");
        subscriptionInfo[subIndex].topic = topic[subIndex];
        subscriptionInfo[subIndex].qos = qos[subIndex];
    }

    if(MQTTClient_subscribe(gMqttClient, subscriptionInfo,
                            SUBSCRIPTION_TOPIC_COUNT) < 0)
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("Subscribed failed\r\n\0");
#endif
        MQTTClient_disconnect(gMqttClient);
        return 0;
    }
    else
    {
        for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
        {
#ifdef UART_DEBUGGING
        sendMsgToUart("Subscribed\r\n\0");
#endif

        }
        return -1;
    }
}

/**
 * New version of publishing
 */
int16_t MQTT_publish(char * pubTopic, char * pubData)
{

    int16_t lRetVal = MQTTClient_publish(gMqttClient, pubTopic, strlen(pubTopic),
                                      pubData, strlen( pubData),
                                      MQTT_QOS_2 |((RETAIN_ENABLE) ? MQTT_PUBLISH_RETAIN : 0));
    return lRetVal;
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
