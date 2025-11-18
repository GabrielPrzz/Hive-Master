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
#define HISTORY_SIZE 			4
#define LORA_MAX_SIZE 			64
#define MAX_HONEYCOMBS			1		//Se modifica con base a el tamaño del sistema, en este caso 3 balizas

typedef enum {
	INITIALIZATION,
	NODE_ERROR,							//Alguno de los modulos, ajeno al LoRa fallo o no esta funcionando correctamente
	SCAN,								//Solo escanendo el ambiente pero sin detectar nada todavia, funcionamiento normal
	DETECTION,							//ALERTA previa a la triangulación, central debe confirmar recepcion de la alerta
	TRIANGULATION,						//DRON DETECTADO y se estan mandando datos para la triangulacion
	SLEEP_INCOMING,						//SLEEP INCOMING ALERTA
	SLEEPING,
} BALIZA_STATE;

typedef enum {							//Señales que avisan que tipo de dato mandamos
	ENERGY,
	GPS,
	ALERT,
} TX_TYPE;

typedef struct {
    int8_t rssi[RSSI_BUFFER_SIZE]; 		//Este rssi sirve para detectar dron
} scan_t;

typedef struct {
	float_t Voltage;
	float_t Percentage;
	float_t DischargeRate;
} energy_t;

typedef struct {						//Falta modificar
    uint8_t latitude;
} gps_t;

typedef struct {
    TX_TYPE transmission_type;
    energy_t energy_data;
    scan_t rssi_buffer[HISTORY_SIZE]; 	//Buffer que almacena HISTORY SIZE arreglos de las lecturas en los 13 canales
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
	uint8_t pending_ack;				//Indica si hay una transmisión pendiente HACIA ese slave, en init y en triang
} HoneyComb_s;

typedef struct {
	uint8_t connected_honeycombs;
	HoneyComb_s honeycombs[MAX_HONEYCOMBS];
	uint8_t rx_buffer[LORA_MAX_SIZE];
	uint8_t tx_buffer[LORA_MAX_SIZE];

	uint8_t pending_tx;
} Hive_Master;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
