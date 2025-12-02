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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "LoRa.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VALID_NODE_IDS {65, 66, 67}

//CENTRAL PKG SIZES
//Respones
#define CENTRAL_CONNECTION_PKG_SIZE 3
#define DETECTION_ACK_PKG_SIZE 	2
#define DRONE_LOST_ACK_PKG_SIZE 2
//CMD
#define SLEEP_CMD_PKG_SIZE 2
#define TRIANGULACTION_CMD_PKG_SIZE 2
#define WKP_CMD_PKG_SIZE 2

//HIVEMASTER_COMMANDS
#define SLEEP_CMD 0xA0
#define DETECTION_ACK_CMD 0xB0
#define DRONE_LOST_ACK_CMD 0xC0
#define ENERGY_ACK_CMD 0xC1
#define GPS_ACK_CMD 0xC2
#define DRONE_LOST_AUX_CMD 0xD0
#define WKP_CMD 0xE0

//Retries
#define MAX_RETRIES 3

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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rxMsgQueue */
osMessageQueueId_t rxMsgQueueHandle;
const osMessageQueueAttr_t rxMsgQueue_attributes = {
  .name = "rxMsgQueue"
};
/* Definitions for txCmdQueue */
osMessageQueueId_t txCmdQueueHandle;
const osMessageQueueAttr_t txCmdQueue_attributes = {
  .name = "txCmdQueue"
};
/* Definitions for hiveMasterMutex */
osMutexId_t hiveMasterMutexHandle;
const osMutexAttr_t hiveMasterMutex_attributes = {
  .name = "hiveMasterMutex"
};
/* Definitions for loraMutex */
osMutexId_t loraMutexHandle;
const osMutexAttr_t loraMutex_attributes = {
  .name = "loraMutex"
};
/* Definitions for printUartMutex */
osMutexId_t printUartMutexHandle;
const osMutexAttr_t printUartMutex_attributes = {
  .name = "printUartMutex"
};
/* Definitions for centralRetxMutex */
osMutexId_t centralRetxMutexHandle;
const osMutexAttr_t centralRetxMutex_attributes = {
  .name = "centralRetxMutex"
};
/* Definitions for dashboardSem */
osSemaphoreId_t dashboardSemHandle;
const osSemaphoreAttr_t dashboardSem_attributes = {
  .name = "dashboardSem"
};
/* Definitions for rxValidPacketSem */
osSemaphoreId_t rxValidPacketSemHandle;
const osSemaphoreAttr_t rxValidPacketSem_attributes = {
  .name = "rxValidPacketSem"
};
/* Definitions for cycleAlarmSem */
osSemaphoreId_t cycleAlarmSemHandle;
const osSemaphoreAttr_t cycleAlarmSem_attributes = {
  .name = "cycleAlarmSem"
};
/* Definitions for loraRxSem */
osSemaphoreId_t loraRxSemHandle;
const osSemaphoreAttr_t loraRxSem_attributes = {
  .name = "loraRxSem"
};
/* USER CODE BEGIN PV */
/* RTOS */

osThreadId_t Constant_Rx_TaskHandle;
const osThreadAttr_t Constant_Rx_Task_attributes = {
  .name = "Constant_Rx_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t LoRa_Tx_TaskHandle;
const osThreadAttr_t LoRa_Tx_Task_attributes = {
  .name = "LoRa_Tx_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Node_Update_TaskHandle;
const osThreadAttr_t Node_Update_Task_attributes = {
  .name = "Node_Update_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t Node_Cycle_TaskHandle;
const osThreadAttr_t Node_Cycle_Task_attributes = {
  .name = "Node_Cycle_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Update_Dashboard_TaskHandle;
const osThreadAttr_t Update_Dashboard_Task_attributes = {
  .name = "Update_Dashboard_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/*PRIVATE VARIABLES*/

LoRa myLoRa;
Hive_Master hiveMaster;
uint16_t cycle_counter = 0;
CentralRetxTracker central_retx = {0xFF, 0xFF, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* RTOS */
void Constant_Rx_Task(void *argument);
void LoRa_Tx_Task(void *argument);
void Node_Update_Task(void *argument);
void Node_Cycle_Task(void *argument);
void Update_Dashboard_Task(void *argument);


/* RTOS */

/* FUNCTIONS */
uint8_t validate_message(uint8_t* buffer, uint8_t rx_len);
void print_debug_F(const char *msg);
void print_debug(const char *msg);
void master_print_all_nodes(Hive_Master* hive_master);
void debug_validate(uint8_t *buffer, uint8_t rx_len);
/* FUNCTIONS */

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
  print_debug_F("Init Sequence\r\n");
  HAL_TIM_Base_Stop_IT(&htim3);
  uint8_t LoRa_Status;
	myLoRa = newLoRa(); 	//Inicializa el modulo LoRa con las configuraciones cargadas
	LoRa_Status = LoRa_connection(&myLoRa, &hspi1);
	while (LoRa_Status) {
		//Funcion que prende buzzer
		print_debug_F("CRITICAL ERROR: LoRa Failed\r\n");
		HAL_Delay(500);
		//Reintentamos...
		LoRa_Status = LoRa_connection(&myLoRa, &hspi1);
	}
	print_debug_F("Init SUCCESFUL\r\n");
	HAL_TIM_Base_Start_IT(&htim3);

	//Si no esta bien el LoRa, no tiene caso que inicie nada...
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of hiveMasterMutex */
  hiveMasterMutexHandle = osMutexNew(&hiveMasterMutex_attributes);

  /* creation of loraMutex */
  loraMutexHandle = osMutexNew(&loraMutex_attributes);

  /* creation of printUartMutex */
  printUartMutexHandle = osMutexNew(&printUartMutex_attributes);

  /* creation of centralRetxMutex */
  centralRetxMutexHandle = osMutexNew(&centralRetxMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of dashboardSem */
  dashboardSemHandle = osSemaphoreNew(1, 0, &dashboardSem_attributes);

  /* creation of rxValidPacketSem */
  rxValidPacketSemHandle = osSemaphoreNew(1, 0, &rxValidPacketSem_attributes);

  /* creation of cycleAlarmSem */
  cycleAlarmSemHandle = osSemaphoreNew(1, 0, &cycleAlarmSem_attributes);

  /* creation of loraRxSem */
  loraRxSemHandle = osSemaphoreNew(1, 0, &loraRxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rxMsgQueue */
  rxMsgQueueHandle = osMessageQueueNew (16, sizeof(rx_message_t), &rxMsgQueue_attributes);

  /* creation of txCmdQueue */
  txCmdQueueHandle = osMessageQueueNew (16, sizeof(TxCommand), &txCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  Constant_Rx_TaskHandle = osThreadNew(Constant_Rx_Task, NULL, &Constant_Rx_Task_attributes);
  LoRa_Tx_TaskHandle = osThreadNew(LoRa_Tx_Task, NULL, &LoRa_Tx_Task_attributes);
  Node_Update_TaskHandle = osThreadNew(Node_Update_Task, NULL, &Node_Update_Task_attributes);
  Node_Cycle_TaskHandle = osThreadNew(Node_Cycle_Task, NULL, &Node_Cycle_Task_attributes);
  Update_Dashboard_TaskHandle = osThreadNew(Update_Dashboard_Task, NULL, &Update_Dashboard_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  htim3.Init.Prescaler = 255;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62499;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* RTOS TASKS */

void Constant_Rx_Task(void *argument) {

	msg_type type; //Mandamos numero para el switch case que hace update
	uint8_t rx_len;
	rx_message_t aux_rx_message = {0};

	for(;;) {
		osDelay(20); //Cada 50 ms escuchamos entorno
		rx_len = 0;
		LoRa_gotoMode(&myLoRa, RXSINGLE_MODE);
		if(osSemaphoreAcquire(loraRxSemHandle, 500) == osOK) {
			osMutexAcquire(loraMutexHandle, osWaitForever);
			rx_len = LoRa_receive_no_mode_change(&myLoRa, hiveMaster.rx_buffer, LORA_ENERGY_PKG_SIZE); //Se pone el size del pkg mas grande
			LoRa_gotoMode(&myLoRa, SLEEP_MODE);
			osMutexRelease(loraMutexHandle);

			if(rx_len > 0) {
				debug_validate(hiveMaster.rx_buffer, rx_len);
				type = validate_message(hiveMaster.rx_buffer, rx_len);
				if(type != INVALID_PKG) {
					aux_rx_message.node_id = hiveMaster.rx_buffer[0];
					aux_rx_message.msg_type = type;
					memcpy(aux_rx_message.payload, &hiveMaster.rx_buffer[1], rx_len - 1); //Copia de status en adelante
					aux_rx_message.payload_len = rx_len - 1;
					osSemaphoreRelease(rxValidPacketSemHandle);
					osMessageQueuePut(rxMsgQueueHandle, &aux_rx_message, 0, 0);
				} else {
					print_debug("Paquete invalido recibido\r\n"); //Me falta ponerle Mutex
				}
			}
		}
	}
}

void LoRa_Tx_Task(void *argument) {
	TxCommand tx_cmd;
	uint8_t cmd = 0xFF;
	uint8_t node_index = 0xFF;
	uint8_t baliza_id = 0;
	char debug_msg[80];

	for(;;){
		osDelay(50);
		//Obtener nuevo comando solo si no hay pendiente
		osMutexAcquire(centralRetxMutexHandle, osWaitForever);
		if (central_retx.pending_cmd == 0xFF && osMessageQueueGet(txCmdQueueHandle, &tx_cmd, NULL, 10) == osOK) {
			central_retx.pending_cmd = tx_cmd.cmd;
			central_retx.pending_node_index = tx_cmd.node_index;
			central_retx.retry_count = 0;
		}
		osMutexRelease(centralRetxMutexHandle);

		//Reintentar actual
		osMutexAcquire(centralRetxMutexHandle, osWaitForever);
		if (central_retx.pending_cmd != 0xFF && central_retx.retry_count < MAX_RETRIES) {
			cmd = central_retx.pending_cmd;
			node_index = central_retx.pending_node_index;
			central_retx.retry_count++;
			osMutexRelease(centralRetxMutexHandle);

			osMutexAcquire(loraMutexHandle, osWaitForever);
			osMutexAcquire(hiveMasterMutexHandle, osWaitForever);

			baliza_id = hiveMaster.honeycombs[node_index].baliza_id;

			switch (cmd) {
			  case 0:
					hiveMaster.tx_buffer[0] = baliza_id;
					hiveMaster.tx_buffer[1] = 0xAA;
					hiveMaster.tx_buffer[2] = hiveMaster.honeycombs[node_index].node_role;
					LoRa_transmit(&myLoRa, hiveMaster.tx_buffer, CENTRAL_CONNECTION_PKG_SIZE, 500);
					sprintf(debug_msg, "CONNECTION_ACK to %d role=%d (retry %d)\r\n",
						baliza_id, hiveMaster.honeycombs[node_index].node_role, central_retx.retry_count);
			  break;
		  	  case 1:
		  		  hiveMaster.tx_buffer[0] = baliza_id;
		  		  hiveMaster.tx_buffer[1] = DETECTION_ACK_CMD;
		  		  LoRa_transmit(&myLoRa, hiveMaster.tx_buffer, DETECTION_ACK_PKG_SIZE, 500);
				  sprintf(debug_msg, "DETECTION_ACK to %d (retry %d)\r\n", baliza_id, central_retx.retry_count);
			  break;
		  	  case 2:
		  		  hiveMaster.tx_buffer[0] = baliza_id;
		  		  hiveMaster.tx_buffer[1] = DRONE_LOST_ACK_CMD;
		  		  LoRa_transmit(&myLoRa, hiveMaster.tx_buffer, DRONE_LOST_ACK_PKG_SIZE, 500);
				  sprintf(debug_msg, "DRONE_LOST_ACK to %d (retry %d)\r\n", baliza_id, central_retx.retry_count);
			  break;
		  	  case 3:
		  		  hiveMaster.tx_buffer[0] = baliza_id;
		  		  hiveMaster.tx_buffer[1] = SLEEP_CMD;
		  		  LoRa_transmit(&myLoRa, hiveMaster.tx_buffer, SLEEP_CMD_PKG_SIZE, 500);
				  sprintf(debug_msg, "SLEEP_CMD to %d (retry %d)\r\n", baliza_id, central_retx.retry_count);
			  break;
		  	  case 4:
		  		  hiveMaster.tx_buffer[0] = baliza_id;
		  		  hiveMaster.tx_buffer[1] = WKP_CMD;
				  LoRa_transmit(&myLoRa, hiveMaster.tx_buffer, WKP_CMD_PKG_SIZE, 500);
				  sprintf(debug_msg, "WKP_CMD to %d (retry %d)\r\n", baliza_id, central_retx.retry_count);
			  break;
		  	  case 5:
		  		  hiveMaster.tx_buffer[0] = baliza_id;
		  		  hiveMaster.tx_buffer[1] = ENERGY_ACK_CMD;
		  		  LoRa_transmit(&myLoRa, hiveMaster.tx_buffer, 2, 500);
		  		  sprintf(debug_msg, "ENERGY_ACK to %d (retry %d)\r\n", baliza_id, central_retx.retry_count);
		  	  break;
		  	  case 6:
		  		  hiveMaster.tx_buffer[0] = baliza_id;
		  		  hiveMaster.tx_buffer[1] = GPS_ACK_CMD;
		  		  LoRa_transmit(&myLoRa, hiveMaster.tx_buffer, 2, 500);
		  		  sprintf(debug_msg, "GPS_ACK to %d (retry %d)\r\n", baliza_id, central_retx.retry_count);
			  break;
			}

			osMutexRelease(hiveMasterMutexHandle);
			osMutexRelease(loraMutexHandle);
			print_debug(debug_msg);
			osDelay(500);
		}
		else if (central_retx.pending_cmd != 0xFF && central_retx.retry_count >= MAX_RETRIES) {
			sprintf(debug_msg, "TX Completed: cmd %d to node %d (3 retries)\r\n",
					central_retx.pending_cmd, hiveMaster.honeycombs[central_retx.pending_node_index].baliza_id);
			osMutexRelease(centralRetxMutexHandle);
			print_debug(debug_msg);

			osMutexAcquire(centralRetxMutexHandle, osWaitForever);
			central_retx.pending_cmd = 0xFF;
			central_retx.pending_node_index = 0xFF;
			central_retx.retry_count = 0;
			osMutexRelease(centralRetxMutexHandle);
		}
		else {
			osMutexRelease(centralRetxMutexHandle);
		}
	}
}

void Node_Update_Task(void *argument) {
	rx_message_t aux_rx_message = {0};
	uint8_t honey_comb_index = 0xFF;
	uint8_t existing_node = 0; //checa reconexiones
	uint8_t first_error = 1; //Solo se usa para mensaje de error
	TxCommand tx_cmd; //0 CONNECTION_PKG ACK, 1 DETECTION ACK, 2 DRONE_LOST_ACK, 3 SLEEP_CMD A AUXILIARES
	TxCommand tx_cmd_sleep;
	char debug_msg[50];

	for(;;) {
		//Solo actualizamos cuando llega un paquete valido a procesar
		if (osSemaphoreAcquire(rxValidPacketSemHandle, osWaitForever) == osOK) {
			osMessageQueueGet(rxMsgQueueHandle, &aux_rx_message, 0, 0);
			osMutexAcquire(hiveMasterMutexHandle, osWaitForever);
			if(hiveMaster.connected_honeycombs == 0) {
				honey_comb_index = 0;
			} else {
				for(uint8_t i = 0; i < hiveMaster.connected_honeycombs; i++ ) {
					if(aux_rx_message.node_id == hiveMaster.honeycombs[i].baliza_id) {
						honey_comb_index = i;
					}
				}
			}

			if(aux_rx_message.msg_type == CONNECTION_PKG) { 	//RESPONSE
				//Verificar si el nodo ya existe
				honey_comb_index = 0;
				existing_node = 0;

				for(uint8_t i = 0; i < hiveMaster.connected_honeycombs; i++) {
					if(aux_rx_message.node_id == hiveMaster.honeycombs[i].baliza_id) {
						existing_node = 1;
						honey_comb_index = i;
					}
				}

				//Si nodo existe pero está en error, RECONECTAR
				if(existing_node && hiveMaster.honeycombs[honey_comb_index].status == NODE_ERROR) {
					hiveMaster.honeycombs[honey_comb_index].devices_status.Esp32_State = 0;
					hiveMaster.honeycombs[honey_comb_index].devices_status.LoRa_State = 0;
					hiveMaster.honeycombs[honey_comb_index].devices_status.GPS_State = 0;
					hiveMaster.honeycombs[honey_comb_index].devices_status.Charger_State = 0;
					hiveMaster.honeycombs[honey_comb_index].devices_status.Microphone_State = 0;
					hiveMaster.honeycombs[honey_comb_index].status = aux_rx_message.payload[0];

					//Connection ack
					tx_cmd.cmd = 0;
					tx_cmd.node_index =  honey_comb_index;
					osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);

					sprintf(debug_msg, "UPDATE: Node%d RECONNECTED [OK]\r\n", aux_rx_message.node_id);
					print_debug(debug_msg);
				}
				else if(existing_node && hiveMaster.honeycombs[honey_comb_index].status != NODE_ERROR) { 				//Si nodo existe y está bien, DESCARTAR DUPLICADO
					sprintf(debug_msg, "UPDATE: Node%d DUPLICATE (discarded)\r\n", aux_rx_message.node_id);
					print_debug(debug_msg);
					//No enviar ACK, simplemente descartar
				}
				else if(!existing_node) { //Si NO existe, AGREGAR NUEVO
					if(hiveMaster.connected_honeycombs < MAX_HONEYCOMBS) {
						honey_comb_index = hiveMaster.connected_honeycombs;
						hiveMaster.honeycombs[honey_comb_index].baliza_id = aux_rx_message.node_id;
						hiveMaster.honeycombs[honey_comb_index].status = aux_rx_message.payload[0];

						// BUSCAR SI HAY DETECTOR
						uint8_t has_detector = 0;
						for(uint8_t i = 0; i < hiveMaster.connected_honeycombs; i++) {
							if(hiveMaster.honeycombs[i].node_role == detector) {
								has_detector = 1;
								break;
							}
						}

						// ASIGNAR ROL
						if(!has_detector) {
							hiveMaster.honeycombs[honey_comb_index].node_role = detector;
							sprintf(debug_msg, "UPDATE: Node%d ADDED as DETECTOR (total: %d)\r\n",
								aux_rx_message.node_id, hiveMaster.connected_honeycombs + 1);
						} else {
							hiveMaster.honeycombs[honey_comb_index].node_role = aux;
							sprintf(debug_msg, "UPDATE: Node%d ADDED as AUX (total: %d)\r\n",
								aux_rx_message.node_id, hiveMaster.connected_honeycombs + 1);
						}

						tx_cmd.cmd = 0;
						tx_cmd.node_index = honey_comb_index;
						osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);

						print_debug(debug_msg);
						hiveMaster.connected_honeycombs++;
					} else {
						print_debug("UPDATE: Hive FULL, cannot add more nodes\r\n");
					}
				}
			} else if (aux_rx_message.msg_type == ERROR_PKG) {
				hiveMaster.honeycombs[honey_comb_index].status = aux_rx_message.payload[0];
				hiveMaster.honeycombs[honey_comb_index].devices_status.Esp32_State = aux_rx_message.payload[1];
				hiveMaster.honeycombs[honey_comb_index].devices_status.LoRa_State = aux_rx_message.payload[2];
				hiveMaster.honeycombs[honey_comb_index].devices_status.GPS_State = aux_rx_message.payload[3];
				hiveMaster.honeycombs[honey_comb_index].devices_status.Charger_State = aux_rx_message.payload[4];
				hiveMaster.honeycombs[honey_comb_index].devices_status.Microphone_State = aux_rx_message.payload[5];

				sprintf(debug_msg, "UPDATE: Node%d ERROR [", aux_rx_message.node_id);
				print_debug(debug_msg);

				//MENSAJE mas explicito de debug
				first_error = 1;

				if(hiveMaster.honeycombs[honey_comb_index].devices_status.Esp32_State) {
					if(first_error) {
						print_debug("ESP32");
					} else {
						print_debug(", ESP32");
					}
					first_error = 0;
				}
				if(hiveMaster.honeycombs[honey_comb_index].devices_status.LoRa_State) {
					if(first_error) {
						print_debug("LoRa");
					} else {
						print_debug(", LoRa");
					}
					first_error = 0;
				}
				if(hiveMaster.honeycombs[honey_comb_index].devices_status.GPS_State) {
					if(first_error) {
						print_debug("GPS");
					} else {
						print_debug(", GPS");
					}
					first_error = 0;
				}
				if(hiveMaster.honeycombs[honey_comb_index].devices_status.Charger_State) {
					if(first_error) {
						print_debug("Charger");
					} else {
						print_debug(", Charger");
					}
					first_error = 0;
				}
				if(hiveMaster.honeycombs[honey_comb_index].devices_status.Microphone_State) {
					if(first_error) {
						print_debug("Microphone");
					} else {
						print_debug(", Microphone");
					}
					first_error = 0;
				}

				// Si no hay errores, mostrar "No errors"
				if(first_error) {
					print_debug("No errors");
				}

			print_debug("]\r\n"); //Cierre de update y mensaje
			} else if (aux_rx_message.msg_type == ENERGY_PKG) {
				hiveMaster.honeycombs[honey_comb_index].status = aux_rx_message.payload[0];
				hiveMaster.honeycombs[honey_comb_index].honey_data.transmission_type = aux_rx_message.payload[1];
				//FLOATS
				memcpy(&hiveMaster.honeycombs[honey_comb_index].honey_data.energy_data.Voltage, &aux_rx_message.payload[2], 4);
				memcpy(&hiveMaster.honeycombs[honey_comb_index].honey_data.energy_data.Percentage, &aux_rx_message.payload[6], 4);
				memcpy(&hiveMaster.honeycombs[honey_comb_index].honey_data.energy_data.DischargeRate, &aux_rx_message.payload[10], 4);

				sprintf(debug_msg, "UPDATE: Node%d ENERGY [V=%.2f P=%.1f]\r\n",
				aux_rx_message.node_id,
				hiveMaster.honeycombs[honey_comb_index].honey_data.energy_data.Voltage,
				hiveMaster.honeycombs[honey_comb_index].honey_data.energy_data.Percentage);
			    tx_cmd.cmd = 5;
			    tx_cmd.node_index = honey_comb_index;
			    osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);
				print_debug(debug_msg);
			} else if (aux_rx_message.msg_type == GPS_PKG) {
				hiveMaster.honeycombs[honey_comb_index].status = aux_rx_message.payload[0];
				hiveMaster.honeycombs[honey_comb_index].honey_data.transmission_type = aux_rx_message.payload[1];
				//FLOATS
				memcpy(&hiveMaster.honeycombs[honey_comb_index].honey_data.location_data.latitude, &aux_rx_message.payload[2], 4);
				memcpy(&hiveMaster.honeycombs[honey_comb_index].honey_data.location_data.longitude, &aux_rx_message.payload[6], 4);

				sprintf(debug_msg, "UPDATE: Node%d GPS [LAT=%.4f LON=%.4f]\r\n",
				aux_rx_message.node_id,
				hiveMaster.honeycombs[honey_comb_index].honey_data.location_data.latitude,
				hiveMaster.honeycombs[honey_comb_index].honey_data.location_data.longitude);
			    tx_cmd.cmd = 6;
			    tx_cmd.node_index = honey_comb_index;
			    osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);
				print_debug(debug_msg);
			} else if (aux_rx_message.msg_type == DETECTION_PKG) { 		//RESPONSE
				hiveMaster.honeycombs[honey_comb_index].status = aux_rx_message.payload[0];
				hiveMaster.honeycombs[honey_comb_index].honey_data.transmission_type = aux_rx_message.payload[1];

				//Drone detected ack
				tx_cmd.cmd = 1;
				tx_cmd.node_index = honey_comb_index;
				osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);

				print_debug("UPDATE: DRONE DETECTED [ACK QUEUED]\r\n");
			} else if (aux_rx_message.msg_type == SLEEPING_PKG) {
				//CONFIRMAMOS SLEEP
				hiveMaster.honeycombs[honey_comb_index].status = SLEEPING;
				hiveMaster.honeycombs[honey_comb_index].node_role = aux;

				sprintf(debug_msg, "UPDATE: Node%d SLEEPING [OK]\r\n", aux_rx_message.node_id);
				print_debug(debug_msg);
			} else if (aux_rx_message.msg_type == DRONE_LOST_PKG) { 	//RESPONSE
				//Primero respondemos a ese nodo
				tx_cmd.cmd = 2;
				tx_cmd.node_index = honey_comb_index;
				osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);

				for (uint8_t i = 0; i < hiveMaster.connected_honeycombs; i++ ) {
				    if(i != honey_comb_index) {
				        tx_cmd_sleep.cmd = 3;
				        tx_cmd_sleep.node_index = i;
				        osMessageQueuePut(txCmdQueueHandle, &tx_cmd_sleep, 0, 0);
				    }
				}
				print_debug("UPDATE: DRONE LOST [SLEEP QUEUED TO AUXILIARIES]\r\n");

			} else if (aux_rx_message.msg_type == TRIANG_PKG) {
				//AVISAR A SEMAFORO HSEM PARA QUE TRIANGULES
				print_debug("UPDATE: TRIANG PKG RECEIVED [HSEM SIGNAL PENDING]\r\n");
			}

			//master_print_all_nodes(&hiveMaster);
			osMutexRelease(hiveMasterMutexHandle);
		}
		osDelay(50);
	}
}

void Node_Cycle_Task(void *argument) {
    uint8_t nextDetectorIndex;
    TxCommand tx_cmd;
    uint8_t currentDetectorIndex = 0xFF;
    uint8_t in_triangulation = 0;
    char debug_msg[80];

    for(;;) {
        if(osSemaphoreAcquire(cycleAlarmSemHandle, osWaitForever) == osOK) {
            osMutexAcquire(hiveMasterMutexHandle, osWaitForever);

            //Verificar triangulación en progreso
            in_triangulation = 0;
            for(uint8_t i = 0; i < hiveMaster.connected_honeycombs; i++) {
                if(hiveMaster.honeycombs[i].status == TRIANGULATION) {
                    in_triangulation = 1;
                }
            }

            //Solo rotar si NO hay triangulación
            if(!in_triangulation && hiveMaster.connected_honeycombs > 1) {

                for(uint8_t i = 0; i < hiveMaster.connected_honeycombs; i++) {
                    if(hiveMaster.honeycombs[i].node_role == detector) {
                        currentDetectorIndex = i;
                    }
                }

                if(currentDetectorIndex != 0xFF) {
                	//Sleep cmd
                	tx_cmd.cmd = 3;
                	tx_cmd.node_index = currentDetectorIndex;
                	osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);
                    hiveMaster.honeycombs[currentDetectorIndex].node_role = aux;
                    sprintf(debug_msg, "CYCLE: Node%d SLEEP\r\n",
                    hiveMaster.honeycombs[currentDetectorIndex].baliza_id);
                }

                nextDetectorIndex = (currentDetectorIndex + 1) % hiveMaster.connected_honeycombs;
            	tx_cmd.cmd = 4;
            	tx_cmd.node_index = nextDetectorIndex;
            	osMessageQueuePut(txCmdQueueHandle, &tx_cmd, 0, 0);
                hiveMaster.honeycombs[nextDetectorIndex].node_role = detector;
                sprintf(debug_msg, "CYCLE: Node%d WAKE\r\n",
                hiveMaster.honeycombs[nextDetectorIndex].baliza_id);

                osMutexRelease(hiveMasterMutexHandle);
                print_debug(debug_msg);
            } else {
                osMutexRelease(hiveMasterMutexHandle);
            }
        }
    }
}

void Update_Dashboard_Task(void *argument) {

	for(;;){
		if (osSemaphoreAcquire(dashboardSemHandle, osWaitForever) ) {
			print_debug("UPDATE: update dashboard\r\n");
		}
	}
}

/* RTOS TASKS */

uint8_t validate_message(uint8_t *buffer, uint8_t rx_len) {
    uint8_t node_id = buffer[0];
    uint8_t status_byte = buffer[1];
    uint8_t tx_type = buffer[2];
    uint8_t valid_ids[] = VALID_NODE_IDS;
    uint8_t id_valid = 0;

    //Validamos primero ID, byte 0
    for(uint8_t i = 0; i < MAX_HONEYCOMBS; i++) {
        if(node_id == valid_ids[i]) {
            id_valid = 1;
            break;
        }
    }

    if(!id_valid) {
    	return INVALID_PKG; //Mensaje descartado por no ser de la red o ser basura
    }

    if(rx_len == LORA_MASTER_CONNECTION_PKG_SIZE && (status_byte == INITIALIZATION || status_byte == NODE_ERROR)) {
    	return CONNECTION_PKG;
    } else if (rx_len == LORA_ERROR_PKG_SIZE && (status_byte == NODE_ERROR)) {
    	return ERROR_PKG;
    } else if (rx_len == LORA_ENERGY_PKG_SIZE && (tx_type == ENERGY)) {
    	return ENERGY_PKG;
    } else if (rx_len == LORA_GPS_PKG_SIZE && (tx_type == GPS)) {
    	return GPS_PKG;
    } else if (rx_len == LORA_ALERT_PKG_SIZE && (status_byte == DETECTION && tx_type == ALERT)) {
    	return DETECTION_PKG;
    } else if (rx_len == LORA_ALERT_PKG_SIZE && (status_byte == SLEEPING && tx_type == ALERT)) {
    	return SLEEPING_PKG;
    } else if (rx_len == LORA_ALERT_PKG_SIZE && (status_byte == DRONE_LOST && tx_type == ALERT)) {
    	return DRONE_LOST_PKG;
    } else if (rx_len == LORA_TRIANG_PKG_SIZE && (status_byte == TRIANGULATION)) {
    	return TRIANG_PKG;
    }

    return INVALID_PKG; //No se encontro estuctura de ningun pkg valido
}

void print_debug_F(const char *msg) { //Funcion de debuggeo, posterior eliminación
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

void print_debug(const char *msg) { //Funcion de debuggeo, posterior eliminación
	osMutexAcquire(printUartMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    osMutexRelease(printUartMutexHandle);
}

void master_print_all_nodes(Hive_Master* hive_master) {
    char msg[200];
    HoneyComb_s* nodo;

    sprintf(msg, "\n=== HIVE STATUS: %d nodes ===\r\n", hive_master->connected_honeycombs);
    print_debug(msg);

    for(uint8_t i = 0; i < hive_master->connected_honeycombs; i++) {
        nodo = &hive_master->honeycombs[i];

        //Info nodo
        sprintf(msg, "\n[node %d] ID=%c Status=%d\r\n",
                i, nodo->baliza_id, nodo->status);
        print_debug(msg);

        //Devices status
        sprintf(msg, "Devices: ESP32=%d LoRa=%d GPS=%d Charger=%d Mic=%d\r\n",
                nodo->devices_status.Esp32_State,
                nodo->devices_status.LoRa_State,
                nodo->devices_status.GPS_State,
                nodo->devices_status.Charger_State,
                nodo->devices_status.Microphone_State);
        print_debug(msg);

        //Energia
        sprintf(msg,"Voltage=%.2f%% Percentage=%.2f%% Discharge=%.2f\r\n",
        		nodo->honey_data.energy_data.Voltage,
                nodo->honey_data.energy_data.Percentage,
                nodo->honey_data.energy_data.DischargeRate);
        print_debug(msg);

        //Ubicacion
        sprintf(msg,"Latitude=%.2f%% Longitude=%.2f\r\n",
                nodo->honey_data.location_data.latitude,
                nodo->honey_data.location_data.longitude);
        print_debug(msg);
    }
    print_debug("=== END HIVE ===\r\n\n");
}

void debug_validate(uint8_t *buffer, uint8_t rx_len) {
	osMutexAcquire(printUartMutexHandle, osWaitForever);
	char msg[100];
	sprintf(msg, "DEBUG: ID=%d Status=%d Type=%d RxLen=%d\r\n",
		buffer[0], buffer[1], buffer[2], rx_len);
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	uint8_t valid_ids[] = VALID_NODE_IDS;
	for(uint8_t i = 0; i < 3; i++) {
		sprintf(msg, "ValidID[%d]=%d Match=%d\r\n", i, valid_ids[i], buffer[0]==valid_ids[i]);
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
	}
	osMutexRelease(printUartMutexHandle);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == LoRa_IRQ_Pin) {
		osSemaphoreRelease(loraRxSemHandle);
		print_debug_F("LORA_RX callback\r\n");
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM15 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  if(htim == &htim3) {	//500ms update dashboard
		cycle_counter++;

		//240 * 500ms = 120s = 2 minutos
		if(cycle_counter >= 240) {
			cycle_counter = 0;
			osSemaphoreRelease(cycleAlarmSemHandle);
		}

		// Dashboard cada 500ms (ya estaba)
		osSemaphoreRelease(dashboardSemHandle);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM15)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
