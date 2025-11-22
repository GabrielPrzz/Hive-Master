/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
//#include "arm_math.h"
#include "LoRa.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
//#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
LoRa myLoRa;
Hive_Master hiveMaster;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void print_debug(const char *msg);
uint8_t master_add_node(Hive_Master* hive_master, uint8_t honey_comb_id, uint8_t status);								//Funcion que agrega nodo a la lista
void master_remove_node(Hive_Master* hive_master, uint8_t node_index);									//Funcion que remueve nodo
uint8_t master_update_honeycomb(Hive_Master* hive_master, uint8_t rx_len);								//Funcion que actualiza honey data y status recibido
void master_send_ack(LoRa* _Lora, Hive_Master* hive_master, uint8_t selected_node);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Stop_IT(&htim3);
	myLoRa = newLoRa(); 								//Inicializa el modulo LoRa con las configuraciones cargadas

	if(LoRa_connection(&myLoRa, &hspi1)) {
		//Funcion que prende buzzer
		print_debug("ERROR: LoRa Failed\r\n");
	} else {
		LoRa_startReceiving(&myLoRa);
		HAL_TIM_Base_Start_IT(&htim3);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Recepción constante esperando comando de conexión, recibe ID y status scan para validar existencia
	  //Regresa acknowledge
	  //Comienza recepción continua

	  for(uint8_t i=0; i < hiveMaster.connected_honeycombs; i++) {
		  if(hiveMaster.honeycombs[i].pending_ack) {
			  master_send_ack(&myLoRa, &hiveMaster, i);
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 58832;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_VERDE_Pin|LED_ROJO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_LoRa_GPIO_Port, SPI1_SS_LoRa_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_VERDE_Pin LED_ROJO_Pin */
  GPIO_InitStruct.Pin = LED_VERDE_Pin|LED_ROJO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_RST_Pin LED_AMARILLO_Pin */
  GPIO_InitStruct.Pin = SPI1_RST_Pin|LED_AMARILLO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_LoRa_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_LoRa_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_SS_LoRa_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_IRQ_Pin */
  GPIO_InitStruct.Pin = LoRa_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LoRa_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void print_debug(const char *msg) { //Funcion de debuggeo, posterior eliminación
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

uint8_t master_add_node(Hive_Master* hive_master, uint8_t honey_comb_id, uint8_t status) {
    for (uint8_t i = 0; i < hive_master->connected_honeycombs; i++) {
        if(honey_comb_id == hive_master->honeycombs[i].baliza_id) {
            print_debug("ERROR: ID duplicado\r\n");
            return 0;
        }
    }

	hive_master->honeycombs[hive_master->connected_honeycombs].baliza_id = honey_comb_id; //Se le asigna el ID a el siguiente honeycomb
	hive_master->honeycombs[hive_master->connected_honeycombs].status = status;
	hive_master->honeycombs[hive_master->connected_honeycombs].pending_ack = 1;			  //Le asignamos bandera para que en el siguiente tx se de ack
	hive_master->connected_honeycombs++;												  //Aumentamos los nodos conectados
	print_debug("Nueva baliza agregada \r\n");
	return 1;
}

void master_remove_node(Hive_Master* hive_master, uint8_t node_index) {
    char msg[50];
    sprintf(msg, "Removing node ID=%d (ERROR)\r\n", hive_master->honeycombs[node_index].baliza_id);
    print_debug(msg);

    //Desplazar elementos
    for(uint8_t i = node_index; i < hive_master->connected_honeycombs - 1; i++) {
        hive_master->honeycombs[i] = hive_master->honeycombs[i+1];
    }

    memset(&hive_master->honeycombs[hive_master->connected_honeycombs - 1], 0, sizeof(HoneyComb_s));
    hive_master->connected_honeycombs--;
}

uint8_t master_update_honeycomb(Hive_Master* hive_master, uint8_t rx_len) {
	uint8_t actual_id = 0;
	//Identificamos primero el nodo a modificar

	if(rx_len == LORA_ACK_PKG_SIZE) {

		//Verificamos si el ID que llego ya existe
		for (uint8_t i = 0; i < hive_master->connected_honeycombs; i++) {
			if(hive_master->rx_buffer[0] == hive_master->honeycombs[i].baliza_id) {

				//Si ya existe pero esta en error, es una reconexion, eliminamos y ponemos esta nueva conexion
				if(hive_master->honeycombs[i].status == NODE_ERROR && (hive_master->rx_buffer[1] == INITIALIZATION)) {
					char msg[40];
					sprintf(msg, "Reconexión de báliza con ID: %c \r\n", hive_master->rx_buffer[0]);
					print_debug(msg);
					master_remove_node(hive_master, i); //Removemos
					//Agregamos reconexion limpia
					return master_add_node(hive_master, hive_master->rx_buffer[0], hive_master->rx_buffer[1]);
				}
				else {
					print_debug("ERROR: ID YA PRESENTE O RECONEXION CON ERROR\r\n");
					return 0;
				}
			}
		}

		//Agregamos si no existe y si hay espacio
		if(hive_master->connected_honeycombs == MAX_HONEYCOMBS) { //Checamos si podemos agregar
			print_debug("Hive lleno\r\n");
		}
		return master_add_node(hive_master, hive_master->rx_buffer[0], hive_master->rx_buffer[1]);
	}
	else {
		for (uint8_t i = 0; i < hive_master->connected_honeycombs; i++) {
			actual_id = hive_master->honeycombs[i].baliza_id;
			if(hive_master->rx_buffer[0] == actual_id) {	//Si lo encontramos modificamos
				//Identificamos que tipo de pkg es:
				if(rx_len == LORA_ERROR_PKG_SIZE && (hive_master->rx_buffer[1] == NODE_ERROR)) {
					//|ID|STATUS|ESP32_STATE|LORA_STATE|GPS_STATE|UNIT_STATE|MICRO_STATE|
					hive_master->honeycombs[i].status = hive_master->rx_buffer[1];						//Cargamos el state
					hive_master->honeycombs[i].devices_status.Esp32_State =	hive_master->rx_buffer[2];	//Cargamos la data
					hive_master->honeycombs[i].devices_status.LoRa_State = hive_master->rx_buffer[3];
					hive_master->honeycombs[i].devices_status.GPS_State = hive_master->rx_buffer[4];
					hive_master->honeycombs[i].devices_status.Charger_State = hive_master->rx_buffer[5];
					hive_master->honeycombs[i].devices_status.Microphone_State = hive_master->rx_buffer[6];
					return 1;
				}
				else if (rx_len == LORA_ENERGY_PKG_SIZE && (hive_master->rx_buffer[2] == ENERGY)) {
					//Cargamos y actualizamos el struct de dicha baliza
					hive_master->honeycombs[i].status = hive_master->rx_buffer[1];						//Cargamos el state
				}
				else if (rx_len == LORA_ALERT_PKG_SIZE && (hive_master->rx_buffer[2] == ALERT)) {
					hive_master->honeycombs[i].status = hive_master->rx_buffer[1];						//Cargamos el state

					if(hive_master->honeycombs[i].status == DETECTION) {
						hive_master->honeycombs[i].pending_ack = 1;								//En la siguiente transmision se le avisa que sabemos pasara a triang
					} else if (hive_master->honeycombs[i].status == SLEEP_INCOMING) {
						//wake a la siguiente y cambiar state a sleep
					}
				}
				else if (rx_len == LORA_GPS_PKG_SIZE && (hive_master->rx_buffer[2] == GPS)) {
					hive_master->honeycombs[i].status = hive_master->rx_buffer[1];						//Cargamos el state
					//Cargamos y actualizamos el struct de dicha baliza
				}
				else if (rx_len == LORA_TRIANG_PKG_SIZE && (hive_master->rx_buffer[1] == TRIANGULATION)) {
					//Extraemos data y triangulamos con lo recibido
					hive_master->honeycombs[i].status = hive_master->rx_buffer[1];						//Cargamos el state
				}
				else {
					print_debug("No valid packet received\r\n");
					return 0;
				}
			return 1;
			}
		}
	}
	print_debug("ERROR: No se encontró honeycomb con dicho ID \r\n");
	return 0;
}

void master_send_ack(LoRa* _Lora, Hive_Master* hive_master, uint8_t selected_node) {
	uint8_t aux_buffer[LORA_ACK_PKG_SIZE]; //ID-0xAA

	aux_buffer[0] = hive_master->honeycombs[selected_node].baliza_id;
	aux_buffer[1] = 0xAA;
	LoRa_transmit(_Lora, aux_buffer, LORA_ACK_PKG_SIZE, 500);
	HAL_Delay(100);
	char msg[20];
	sprintf(msg, "ACK enviado a %c \r\n", hive_master->honeycombs[selected_node].baliza_id);
	print_debug(msg);
	hive_master->honeycombs[selected_node].pending_ack = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	uint8_t rx_len = 0;
	if (htim == &htim3) {
		rx_len = LoRa_receive(&myLoRa, hiveMaster.rx_buffer, LORA_MAX_SIZE);

		if(rx_len > 0) {
			char msg[100];
			sprintf(msg, "RX: %d bytes - ", rx_len);
			print_debug(msg);

			for(uint8_t i=0; i < rx_len; i++) {
				if(i==0) { //No alteres ID
					sprintf(msg, "%c", hiveMaster.rx_buffer[i]);
					print_debug(msg);
				}
				else {
					sprintf(msg, "%c", 48+hiveMaster.rx_buffer[i]);
					print_debug(msg);
				}
			}
			print_debug("\r\n");

			master_update_honeycomb(&hiveMaster, rx_len);
		} else {
			print_debug("RX: No data\r\n");
		}
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
