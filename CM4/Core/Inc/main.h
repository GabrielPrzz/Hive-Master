/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_VERDE_Pin GPIO_PIN_0
#define LED_VERDE_GPIO_Port GPIOB
#define SPI1_RST_Pin GPIO_PIN_13
#define SPI1_RST_GPIO_Port GPIOE
#define LED_ROJO_Pin GPIO_PIN_14
#define LED_ROJO_GPIO_Port GPIOB
#define SPI1_SS_LoRa_Pin GPIO_PIN_14
#define SPI1_SS_LoRa_GPIO_Port GPIOD
#define LoRa_IRQ_Pin GPIO_PIN_14
#define LoRa_IRQ_GPIO_Port GPIOG
#define LED_AMARILLO_Pin GPIO_PIN_1
#define LED_AMARILLO_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define RSSI_BUFFER_SIZE 		13
#define MAX_HONEYCOMBS			3		//Se modifica con base a el tamaño del sistema, en este caso 3 balizas

#define LORA_MAX_SIZE 			64
#define LORA_MASTER_CONNECTION_PKG_SIZE 2
#define LORA_ACK_PKG_SIZE		3	//|ID|OxAA|role|
#define LORA_ERROR_PKG_SIZE		7
#define LORA_ENERGY_PKG_SIZE	15
#define LORA_ALERT_PKG_SIZE		3
#define LORA_GPS_PKG_SIZE		11
#define LORA_TRIANG_PKG_SIZE 	15

//CONNECTION_PKG ID|INITIALIZATION|														-RESPONSE
//ERROR PKG 	 ID|NODE_ERROR|ESP32_STATE|LORA_STATE|GPS_STATE|UNIT_STATE|MICRO_STATE|
//ENERGY PKG	 ID|SCAN|ENERGY|VOLTAGE|PERCENTAGE|RATE_OF_DISCHARGE|
//GPS PKG		 ID|SCAN|GPS|LAT0|LAT1|LAT2|LAT3|LON0|LON1|LON2|LON3|
//DETECTION PKG  ID|DETECTION|ALERT|													-RESPONSE
//SLEEPING PKG   ID|SLEEPING|ALERT|
//DRONE_LOST PKG ID|DRONE_LOST|ALERT|													-RESPONSE
//TRIANG PKG	 ID|TRIANGULATION|RSSI[0]|RSSI[1]|...|RSSI[12]|
typedef enum {
	INVALID_PKG,
    CONNECTION_PKG,
    ERROR_PKG,
    ENERGY_PKG,
    GPS_PKG,
    DETECTION_PKG,
    SLEEPING_PKG,
    DRONE_LOST_PKG,
    TRIANG_PKG
} msg_type;

typedef enum {
	INITIALIZATION,
	NODE_ERROR,							//Alguno de los modulos, ajeno al LoRa fallo o no esta funcionando correctamente
	SCAN,								//Solo escanendo el ambiente pero sin detectar nada todavia, funcionamiento normal
	DETECTION,							//ALERTA previa a la triangulación, central debe confirmar recepcion de la alerta
	TRIANGULATION,						//DRON DETECTADO y se estan mandando datos para la triangulacion
	DRONE_LOST,
	SLEEPING,
} BALIZA_STATE;

typedef enum {							//Señales que avisan que tipo de dato mandamos
	ENERGY,
	GPS,
	ALERT,
} TX_TYPE;

typedef enum {							//Señales que avisan que tipo de dato mandamos
	detector,
	aux,
} system_role;

typedef struct {
    int8_t rssi[RSSI_BUFFER_SIZE]; 		//Este rssi sirve para detectar dron
} scan_t;

typedef struct {
	float_t Voltage;
	float_t Percentage;
	float_t DischargeRate;
} energy_t;

typedef struct {
	float_t latitude;
	float_t longitude;
} gps_t;

typedef struct {
    TX_TYPE transmission_type;
    energy_t energy_data;
    gps_t location_data;
} lora_package;


typedef struct { 						//States para verificación en las Bálizas
	uint8_t LoRa_State;					//1 para errores, 0 para OK
	uint8_t Esp32_State;
	uint8_t GPS_State;
	uint8_t Charger_State;
	uint8_t Microphone_State;
} MODULES;

typedef struct {						//STRUCT DE ALMACENAMIENTO, NO DE CONTROL
	uint8_t baliza_id;
	BALIZA_STATE status;
	MODULES devices_status;				//Variable que almacena el status de los devices de dicho nodo

	lora_package honey_data;			//Variable utilizada para almacenar la info recibida por este nodo
	system_role node_role;
} HoneyComb_s;

typedef struct {
	uint8_t connected_honeycombs;
	HoneyComb_s honeycombs[MAX_HONEYCOMBS];
	uint8_t rx_buffer[LORA_MAX_SIZE];
	uint8_t tx_buffer[LORA_MAX_SIZE];

	uint8_t pending_tx;
} Hive_Master;

typedef struct {
    uint8_t msg_type;
    uint8_t node_id;
    uint8_t payload[LORA_ENERGY_PKG_SIZE]; //Se pone el size de pkg mas grande
    uint8_t payload_len;
} rx_message_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
