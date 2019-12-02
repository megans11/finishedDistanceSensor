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
// network_if.c
//
// Networking interface functions for CC32xx device
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup common_interface
//! @{
//
//*****************************************************************************
/* Standard includes                                                         */
#include <string.h>
#include <stdlib.h>

/* Kernel (Non OS/Free-RTOS/TI-RTOS) includes                                */
#include <pthread.h>
#include <mqueue.h>
#include <unistd.h>

/* Simplelink includes                                                       */
#include <ti/drivers/net/wifi/simplelink.h>

/* Common interface includes                                                 */
#include "network_if.h"

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************

/* Network App specific status/error codes which are                         */
/* used only in this file.                                                   */
typedef enum
{
    /* Choosing this number to avoid overlap with host-driver's error codes. */
    DEVICE_NOT_IN_STATION_MODE = -0x7F0,
    DEVICE_NOT_IN_AP_MODE = DEVICE_NOT_IN_STATION_MODE - 1,
    DEVICE_NOT_IN_P2P_MODE = DEVICE_NOT_IN_AP_MODE - 1,

    STATUS_CODE_MAX = -0xBB8
} e_NetAppStatusCodes;

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

/* Station IP address                                                        */
unsigned long g_ulStaIp = 0;
/* Network Gateway IP address                                                */
unsigned long g_ulGatewayIP = 0;
/* Connection SSID                                                           */
unsigned char g_ucConnectionSSID[SL_WLAN_SSID_MAX_LENGTH + 1];
/* Connection BSSID                                                          */
unsigned char g_ucConnectionBSSID[SL_WLAN_BSSID_LENGTH ];
/* SimpleLink Status                                                         */
volatile unsigned long g_ulStatus = 0;
/* Connection time delay index                                               */
volatile unsigned short g_usConnectIndex;

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************
void SimpleLinkHttpServerEventHandler(
    SlNetAppHttpServerEvent_t *pSlHttpServerEvent,
    SlNetAppHttpServerResponse_t *
    pSlHttpServerResponse)
{
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse)
{
}

void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
}

//*****************************************************************************
//!
//! On Successful completion of Wlan Connect, This function triggers connection
//! status to be set.
//!
//! \param[in]  pSlWlanEvent    - pointer indicating Event type
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    SlWlanEventDisconnect_t* pEventData = NULL;

    switch(pSlWlanEvent->Id)
    {
    case SL_WLAN_EVENT_CONNECT:
        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

        memcpy(g_ucConnectionSSID, pSlWlanEvent->Data.Connect.SsidName,
               pSlWlanEvent->Data.Connect.SsidLen);
        memcpy(g_ucConnectionBSSID, pSlWlanEvent->Data.Connect.Bssid,
               SL_WLAN_BSSID_LENGTH);

        break;

    case SL_WLAN_EVENT_DISCONNECT:
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);

        pEventData = &pSlWlanEvent->Data.Disconnect;

        /* If the user has initiated 'Disconnect' request, 'reason_code' */
        /* is SL_WLAN_DISCONNECT_USER_INITIATED                          */
        if(SL_WLAN_DISCONNECT_USER_INITIATED == pEventData->ReasonCode)
        {
#ifdef UART_DEBUGGING
            sendMsgToUart("dev disc on request\r\n\0");
#endif
        }
        else
        {
#ifdef UART_DEBUGGING
            sendMsgToUart("dev disc on err\r\n\0");
#endif
        }
        break;

    case SL_WLAN_EVENT_STA_ADDED:
        /* when device is in AP mode and any client connects to it.      */
        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
        break;

    case SL_WLAN_EVENT_STA_REMOVED:
        /* when device is in AP mode and any client disconnects from it. */
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
        break;

    default:
#ifdef UART_DEBUGGING
        sendMsgToUart("WLAN unexpected event\r\n\0");
#endif
        break;
    }
}

//*****************************************************************************
//
//! The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return     None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    switch(slFatalErrorEvent->Id)
    {
    case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
#ifdef UART_DEBUGGING
        sendMsgToUart("SL_FATAL_DEVICE_ABORT\r\n\0");
#endif
        break;

    case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
#ifdef UART_DEBUGGING
        sendMsgToUart("SL_FATAL_DRIVER_ABORT\r\n\0");
#endif
        break;

    case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
#ifdef UART_DEBUGGING
        sendMsgToUart("SL_FATAL_NO_CMD_ACK\r\n\0");
#endif
        break;

    case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
#ifdef UART_DEBUGGING
        sendMsgToUart("SL_FATAL_SYNC_LOSS\r\n\0");
#endif
        break;

    case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
#ifdef UART_DEBUGGING
        sendMsgToUart("SL_FATAL_CMD_TIMEOUTT\r\n\0");
#endif
        break;

    default:
#ifdef UART_DEBUGGING
        sendMsgToUart("UNSPECIFIED ERR\r\n\0");
#endif
        break;
    }
}

//*****************************************************************************
//
//! This function handles network events such as IP acquisition, IP leased, IP
//! released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Id)
    {
    case SL_NETAPP_EVENT_IPV4_ACQUIRED:
    case SL_NETAPP_EVENT_IPV6_ACQUIRED:
        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);

        break;

    case SL_NETAPP_EVENT_DHCPV4_LEASED:
        SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
        g_ulStaIp = (pNetAppEvent)->Data.IpLeased.IpAddress;

        break;

    case SL_NETAPP_EVENT_DHCPV4_RELEASED:
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

        break;

    default:
#ifdef UART_DEBUGGING
        sendMsgToUart("SL_netapp unexpected\r\n\0");
#endif
        break;
    }
}


//*****************************************************************************
//
//! This function handles General Events
//!
//! \param[in]  pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /* Most of the general errors are not FATAL. are to be handled           */
    /* appropriately by the application.                                     */
#ifdef UART_DEBUGGING
        sendMsgToUart("General event\r\n\0");
#endif
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]  pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    /* This application doesn't work w/ socket - Events are not expected     */
    switch(pSock->Event)
    {
    case SL_SOCKET_TX_FAILED_EVENT:
        switch(pSock->SocketAsyncEvent.SockTxFailData.Status)
        {
        case SL_ERROR_BSD_ECLOSE:
#ifdef UART_DEBUGGING
            sendMsgToUart("Closed socket ERR\r\n\0");
#endif
            break;
        default:
#ifdef UART_DEBUGGING
                    sendMsgToUart("Tx failed\r\n\0");
#endif
            break;
        }
        break;
    case SL_SOCKET_ASYNC_EVENT:
    {

        switch(pSock->SocketAsyncEvent.SockAsyncData.Type)
        {
        case SL_SSL_NOTIFICATION_CONNECTED_SECURED:
            break;
        case SL_SSL_NOTIFICATION_HANDSHAKE_FAILED:
            break;
        case SL_OTHER_SIDE_CLOSE_SSL_DATA_NOT_ENCRYPTED:
            break;
        case SL_SSL_NOTIFICATION_WRONG_ROOT_CA:
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! This function initializes the application variables
//!
//! \param  None
//!
//! \return 0 on success, negative error-code on error
//
//*****************************************************************************
void InitializeAppVariables(void)
{
    g_ulStatus = 0;
    g_ulStaIp = 0;
    g_ulGatewayIP = 0;

    memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
}

//*****************************************************************************
//
//! The function initializes a CC32xx device and triggers it to start operation
//!
//! \param[in]  uiMode  - device mode in which device will be configured
//!
//! \return 0 : success, -ve : failure
//
//*****************************************************************************
long Network_IF_InitDriver(uint32_t uiMode)
{
    long lRetVal = -1;

    /* Reset CC32xx Network State Machine                                    */
    InitializeAppVariables();

    /* Following function configure the device to default state by cleaning  */
    /* the persistent settings stored in NVMEM (viz. connection profiles     */
    /* & policies, power policy etc) Applications may choose to skip this    */
    /* step if the developer is sure that the device is in its default state */
    /* at start of application. Note that all profiles and persistent        */
    /* settings that were done on the device will be lost.                   */
    lRetVal = sl_Start(NULL, NULL, NULL);

    if(lRetVal < 0)
    {
        LOOP_FOREVER();
    }

    switch(lRetVal)
    {
    case ROLE_STA:
#ifdef UART_DEBUGGING
        sendMsgToUart("station mode\r\n\0");
#endif
        break;
    case ROLE_AP:
#ifdef UART_DEBUGGING
        sendMsgToUart("access-point mode ERR\r\n\0");
#endif
        break;
    case ROLE_P2P:
#ifdef UART_DEBUGGING
        sendMsgToUart("P2P mode\r\n\0");
#endif
        break;
    default:
#ifdef UART_DEBUGGING
        sendMsgToUart("unknown mode\r\n\0");
#endif
        break;
    }

    if(uiMode != lRetVal)
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("switching on request\r\n\0");
#endif

        /* Switch to AP role and restart                                     */
        lRetVal = sl_WlanSetMode(uiMode);

        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        lRetVal = sl_Start(0, 0, 0);

        if(lRetVal == uiMode)
        {
            switch(lRetVal)
            {
            case ROLE_STA:
                break;
            case ROLE_AP:
                /* If the device is in AP mode, we need to wait for this */
                /* event before doing anything.                          */
                while(!IS_IP_ACQUIRED(g_ulStatus))
                {
                    usleep(1000);
                }
                break;
            case ROLE_P2P:
                break;
            default:
                break;
            }
        }
        else
        {
#ifdef UART_DEBUGGING
            sendMsgToUart("could not configure network mode\r\n\0");
#endif
            LOOP_FOREVER();
        }
    }
    else
    {
        if(lRetVal == ROLE_AP)
        {
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
                usleep(1000);
            }
        }
    }
    return(0);
}

//*****************************************************************************
//
//! The function de-initializes a CC32xx device
//!
//! \param  None
//!
//! \return On success, zero is returned. On error, other
//
//*****************************************************************************
long Network_IF_DeInitDriver(void)
{
    long lRetVal = -1;

#ifdef UART_DEBUGGING
        sendMsgToUart("SL disconnect\r\n\0");
#endif

    /* Disconnect from the AP                                                */
    lRetVal = Network_IF_DisconnectFromAP();

    /* Stop the simplelink host                                              */
    sl_Stop(SL_STOP_TIMEOUT);

    /* Reset the state to uninitialized                                      */
    Network_IF_ResetMCUStateMachine();
    return(lRetVal);
}

//*****************************************************************************
//
//! Connect to an Access Point using the specified SSID
//!
//! \param[in]  pcSsid          - is a string of the AP's SSID
//! \param[in]  SecurityParams  - is Security parameter for AP
//!
//! \return On success, zero is returned. On error, -ve value is returned
//
//*****************************************************************************
long Network_IF_ConnectAP(char *pcSsid,
                          SlWlanSecParams_t SecurityParams)
{
    int lRetVal;
    unsigned long ulIP = 0;
    unsigned long ulSubMask = 0;
    unsigned long ulDefGateway = 0;
    unsigned long ulDns = 0;

    g_usConnectIndex = 0;

    /* Disconnect from the AP                                                */
    Network_IF_DisconnectFromAP();

    CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
    CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);

    /* Continue only if SSID is not empty                                    */
    if(pcSsid != NULL)
    {
        /* This triggers the CC32xx to connect to a specific AP.             */
        lRetVal =
            sl_WlanConnect((signed char *) pcSsid, strlen((const char *) pcSsid),
                           NULL, &SecurityParams, NULL);

        /* Wait for ~10 sec to check if connection to desire AP succeeds     */
        while(g_usConnectIndex < 10)
        {
            sleep(1);

            if(IS_CONNECTED(g_ulStatus) && IS_IP_ACQUIRED(g_ulStatus))
            {
                break;
            }
            g_usConnectIndex++;
        }
    }
    else
    {
#ifdef UART_DEBUGGING
        sendMsgToUart("empty ssid\r\n\0");
#endif
        return(-1);
    }

    if(!IS_CONNECTED(g_ulStatus) || !IS_IP_ACQUIRED(g_ulStatus)) {
#ifdef UART_DEBUGGING
        sendMsgToUart("connection failed\r\n\0");
#endif
        return(-1);
    }

    /* Get IP address                                                        */
    lRetVal = Network_IF_IpConfigGet(&ulIP, &ulSubMask, &ulDefGateway, &ulDns);

    return(0);
}

//*****************************************************************************
//
//! Disconnects from an Access Point
//!
//! \param  none
//!
//! \return 0 disconnected done, other already disconnected
//
//*****************************************************************************
long Network_IF_DisconnectFromAP(void)
{
    long lRetVal = 0;

    if(IS_CONNECTED(g_ulStatus))
    {
        lRetVal = sl_WlanDisconnect();
        if(0 == lRetVal)
        {
            while(IS_CONNECTED(g_ulStatus))
            {
                usleep(1000);
            }
            return(lRetVal);
        }
        else
        {
            return(lRetVal);
        }
    }
    else
    {
        return(lRetVal);
    }
}

//*****************************************************************************
//
//! Get the IP Address of the device.
//!
//! \param[in]  pulIP               - IP Address of Device
//! \param[in]  pulSubnetMask       - Subnetmask of Device
//! \param[in]  pulDefaultGateway   - Default Gateway value
//! \param[in]  pulDNSServer        - DNS Server
//!
//! \return On success, zero is returned. On error, -1 is returned
//
//*****************************************************************************
long Network_IF_IpConfigGet(unsigned long *pulIP,
                            unsigned long *pulSubnetMask,
                            unsigned long *pulDefaultGateway,
                            unsigned long *pulDNSServer)
{
    unsigned short usDHCP = 0;
    long lRetVal = -1;
    unsigned short len = sizeof(SlNetCfgIpV4Args_t);
    SlNetCfgIpV4Args_t ipV4 = { 0 };

    /* get network configuration                                             */
    lRetVal =
        sl_NetCfgGet(SL_NETCFG_IPV4_STA_ADDR_MODE, &usDHCP, &len,
                     (unsigned char *) &ipV4);

    *pulIP = ipV4.Ip;
    *pulSubnetMask = ipV4.IpMask;
    *pulDefaultGateway = ipV4.IpGateway;
    *pulDefaultGateway = ipV4.IpDnsServer;

    return(lRetVal);
}

//*****************************************************************************
//
//!  This function obtains the server IP address using a DNS lookup
//!
//! \param[in]  pcHostName        The server hostname
//! \param[out] pDestinationIP    This parameter is filled with host IP address
//!
//! \return On success, +ve value is returned. On error, -ve value is returned
//
//*****************************************************************************
long Network_IF_GetHostIP(char* pcHostName,
                          unsigned long * pDestinationIP)
{
    long lStatus = 0;

    lStatus =
        sl_NetAppDnsGetHostByName((signed char *) pcHostName, strlen(
                                      pcHostName), pDestinationIP, SL_AF_INET);
    return(lStatus);
}

//*****************************************************************************
//
//! Reset state from the state machine
//!
//! \param  None
//!
//! \return none
//!
//*****************************************************************************
void Network_IF_ResetMCUStateMachine()
{
    g_ulStatus = 0;
}

//*****************************************************************************
//
//! Return the current state bits
//!
//! \param  None
//!
//! \return none
//!
//
//*****************************************************************************
unsigned long Network_IF_CurrentMCUState()
{
    return(g_ulStatus);
}

//*****************************************************************************
//
//! Sets a state from the state machine
//!
//! \param[in]  cStat   - Status of State Machine defined in e_StatusBits
//!
//! \return none
//!
//*****************************************************************************
void Network_IF_SetMCUMachineState(char cStat)
{
    SET_STATUS_BIT(g_ulStatus, cStat);
}

//*****************************************************************************
//
//! Unsets a state from the state machine
//!
//! \param[in]  cStat   - Status of State Machine defined in e_StatusBits
//!
//! \return none
//!
//*****************************************************************************
void Network_IF_UnsetMCUMachineState(char cStat)
{
    CLR_STATUS_BIT(g_ulStatus, cStat);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
