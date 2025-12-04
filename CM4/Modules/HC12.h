/*
 * HC12.h
 *
 *  Created on: __Dec__ 3, 2025
 *      Author: __davox__
 */

#ifndef HC12_H_
#define HC12_H_

#include "main.h"

/* Packet Sizes */
#define HC12_ALERT_PKG_SIZE         3   //[ID][STATUS][TX_TYPE]
#define HC12_ENERGY_PKG_SIZE        15  //[ID][STATUS][TX_TYPE][VOLTAGE(4)][%(4)][RATE(4)]
#define HC12_GPS_PKG_SIZE           11  //[ID][STATUS][TX_TYPE][LAT(4)][LONG(4)]
#define HC12_TRIANG_PKG_SIZE        15  //[ID][STATUS][13×RSSI]
#define HC12_ACK_PKG_SIZE           3   //[ID][0xAA][NODE_ROLE]
#define HC12_ERROR_PKG_SIZE         7   //[ID][STATUS][ESP32][HC12][GPS][CHRG][MIC]
#define HC12_MASTER_CONNECT_SIZE    2   //[ID][STATUS]
#define HC12_MAX_SIZE               32

/* AT Commands */
#define HC12_CMD_BAUDRATE   "AT+B9600\r\n"
#define HC12_CMD_CHANNEL    "AT+C001\r\n"
#define HC12_CMD_POWER      "AT+P8\r\n"
#define HC12_CMD_MODE       "AT+FU3\r\n"

//HIVEMASTER_COMMANDS
#define CENTRAL_CONNECTION_ACK 0xAA
#define SLEEP_CMD 0xA0
#define DETECTION_ACK_CMD 0xB0
#define DRONE_LOST_ACK_CMD 0xC0
#define ENERGY_ACK_CMD 0xC1
#define GPS_ACK_CMD 0xC2
#define DRONE_LOST_AUX_CMD 0xD0
#define WKP_CMD 0xE0

/* Function Prototypes */
void HC12_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *set_port, uint16_t set_pin);

void HC12_SendATCommand(UART_HandleTypeDef *huart, const char* cmd);

void HC12_StartRxIdle(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

/* ============ CENTRAL TX FUNCTIONS ============ */

void HC12_SendConnectionACK(UART_HandleTypeDef *huart, uint8_t node_id, uint8_t role);

void HC12_SendDetectionACK(UART_HandleTypeDef *huart, uint8_t node_id);

void HC12_SendDroneLostACK(UART_HandleTypeDef *huart, uint8_t node_id);

void HC12_SendEnergyACK(UART_HandleTypeDef *huart, uint8_t node_id);

void HC12_SendGpsACK(UART_HandleTypeDef *huart, uint8_t node_id);

void HC12_SendSleepCmd(UART_HandleTypeDef *huart, uint8_t node_id);

void HC12_SendWakeCmd(UART_HandleTypeDef *huart, uint8_t node_id);

#endif /* HC12_H_ */
