/*
 * HC12.c
 *
 *  Created on: Dec 3, 2025
 *      Author: davox
 */

/*
 * HC12.c
 * HC-12 UART Wireless Module Implementation
 */

#include "HC12.h"
#include <string.h>

void HC12_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *set_port, uint16_t set_pin) {
    HAL_GPIO_WritePin(set_port, set_pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    HC12_SendATCommand(huart, HC12_CMD_BAUDRATE);
    HAL_Delay(50);
    HC12_SendATCommand(huart, HC12_CMD_CHANNEL);
    HAL_Delay(50);
    HC12_SendATCommand(huart, HC12_CMD_POWER);
    HAL_Delay(50);
    HC12_SendATCommand(huart, HC12_CMD_MODE);
    HAL_Delay(50);

    HAL_GPIO_WritePin(set_port, set_pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

void HC12_SendATCommand(UART_HandleTypeDef *huart, const char* cmd) {
    HAL_UART_Transmit(huart, (uint8_t*)cmd, strlen(cmd), 100);
}

void HC12_StartRxIdle(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

#include "main.h"
#include <string.h>

/* ============ CENTRAL TX WRAPPER ============ */

void HC12_SendConnectionACK(UART_HandleTypeDef *huart, uint8_t node_id, uint8_t role) {
    uint8_t pkg[3];
    pkg[0] = node_id;
    pkg[1] = CENTRAL_CONNECTION_ACK;
    pkg[2] = role;
    HAL_UART_Transmit(huart, pkg, 3, 100);
}

void HC12_SendDetectionACK(UART_HandleTypeDef *huart, uint8_t node_id) {
    uint8_t pkg[2];
    pkg[0] = node_id;
    pkg[1] = DETECTION_ACK_CMD;
    HAL_UART_Transmit(huart, pkg, 2, 100);
}

void HC12_SendDroneLostACK(UART_HandleTypeDef *huart, uint8_t node_id) {
    uint8_t pkg[2];
    pkg[0] = node_id;
    pkg[1] = DRONE_LOST_ACK_CMD;
    HAL_UART_Transmit(huart, pkg, 2, 100);
}

void HC12_SendEnergyACK(UART_HandleTypeDef *huart, uint8_t node_id) {
    uint8_t pkg[2];
    pkg[0] = node_id;
    pkg[1] = ENERGY_ACK_CMD;
    HAL_UART_Transmit(huart, pkg, 2, 100);
}

void HC12_SendGpsACK(UART_HandleTypeDef *huart, uint8_t node_id) {
    uint8_t pkg[2];
    pkg[0] = node_id;
    pkg[1] = GPS_ACK_CMD;
    HAL_UART_Transmit(huart, pkg, 2, 100);
}

void HC12_SendSleepCmd(UART_HandleTypeDef *huart, uint8_t node_id) {
    uint8_t pkg[2];
    pkg[0] = node_id;
    pkg[1] = SLEEP_CMD;
    HAL_UART_Transmit(huart, pkg, 2, 100);
}

void HC12_SendWakeCmd(UART_HandleTypeDef *huart, uint8_t node_id) {
    uint8_t pkg[2];
    pkg[0] = node_id;
    pkg[1] = WKP_CMD;
    HAL_UART_Transmit(huart, pkg, 2, 100);
}







