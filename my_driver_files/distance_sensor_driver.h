/*
 * distance_sensor_driver.h
 *
 */

#ifndef DISTANCE_SENSOR_DRIVER_H_
#define DISTANCE_SENSOR_DRIVER_H_

#include <ti/drivers/UART.h>
#include <ti/drivers/Capture.h>
#include <ti/drivers/GPIO.h>
#include "board.h"

// Function return values
#define I2C_INIT_FAIL -1
#define I2C_INIT_SUCCESS 0
#define UART_INIT_FAIL -1
#define UART_INIT_SUCCESS 0
#define GET_DISTANCE_FAIL -1
#define GET_DISTANCE_SUCCESS 0

// as defined in motor controller documentation
#define INIT_UART_MSG 170


UART_Params uartDistanceSensorParams;
UART_Handle distanceSensorUART;
//Capture_Params distanceCaptureParams;
//Capture_Handle distanceCapture;

char UARTBuffer[20];
char echoPrompt[256];

int init_distance_sensor_GPIO();
int init_distance_sensor_capture();
int init_distance_sensor_UART();


#endif /* COLOR_SENSOR_DRIVER_H_ */
