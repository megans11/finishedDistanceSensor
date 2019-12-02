/*
 *  ======== find_color_task_thread.c ========
 */

#include <find_distance_task_thread.h>


/*
 *  ======== findColorTaskThread ========
 */
void *findDistanceTaskThread(void *arg0)
{

    dbgUARTVal("in distance task thread");

    sleep(2);

    // Command variables
    int msg_type;
    uint32_t msg_buffer;
    static uint32_t distance = 0;

    while (1) {

        msg_type = readMsg_DistanceTaskQueue(&msg_buffer);

        if (msg_type == MSG_TYPE_FALL)
        {
            distance = msg_buffer;
            distance = distance * 10;

            char buffer[17];
            snprintf(buffer, sizeof(buffer), "distance %ld", distance);

//            sendMsgToUart(buffer);

            sendMsg_MqttQueue("distance", "distance", buffer);


        }

        //print to UART

    }
}
