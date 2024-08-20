/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"

#include "rtc.h"
#include "tim.h"

#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

typedef enum {
	TAG,
	BEACONA,
	BEACONB,
	BEACONC,
} Device_t;

typedef struct {
	Device_t device;
	uint8_t id;
} TrackRequest_t;

#define TRACK_REQUEST_SIZE 2

typedef struct {
	Device_t device;
	uint8_t id;
	int16_t recv_rssi;
} TrackResponse_t;

#define TRACK_RESPONSE_SIZE 4

typedef enum {
	NODE_STATE_INIT,
	NODE_STATE_TRACK_REQ_PHASE,
	NODE_STATE_TRACK_RES_PHASE,
	NODE_STATE_WAIT,
} NodeState_t;

static NodeState_t node_state = NODE_STATE_INIT;
static NodeState_t next_state = NODE_STATE_INIT;

static UTIL_TIMER_Object_t timerTimeout;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000

#define TIMEOUT_PERIOD_MS			  400

/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN                200
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333
/* LED blink Period*/
#define LED_PERIOD_MS                 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
/*Ping Pong FSM states */
static States_t State = RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/* Led Timers objects*/
/* device state. Master: true, Slave: false*/
bool isMaster = true;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
   take for the devices to sync when started simultaneously*/
static int32_t random_delay;

static TrackResponse_t last_beacon_a_trk_res;
static TrackResponse_t last_beacon_b_trk_res;
static TrackResponse_t last_beacon_c_trk_res;

uint8_t last_id = 0;
int16_t last_rssi = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Updates the `node_state` global with the new state value configured in `next_state`.
 */
static void UpdateState();

/**
 * @brief Sets the `node_state` global to `new_state`.
 * @param new_state New `NodeState_t` value for `node_state`.
 */
static void SetState(NodeState_t new_state);

/**
 * @brief Add `TrackingProcess` to UTIL_SEQ task-queue. (Queue `TrackingProcess` for execution.)
 */
static void QueueTrackingTask();

/**
 * @brief Function to call when an error occurs
 * @param errorMsg Short string which explains the error.
 */
static void ErrorHandler(char* errorMsg);

/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  Function executed on when led timer elapses
  * @param  context ptr of LED context
  */
static void OnledEvent(void *context);

/**
 * @brief   Function executed when timeout timer elapses
 * @param context ptr of Timeout context
 */
static void OnTimeoutEvent(void *context);

/**
  * @brief Tracking state machine implementation
  */
//static void Tracking_Process(void);

/**
 * @brief Create byte buffer with data from `TrackRequest_t`.
 * @param buffer destination position of byte buffer
 * @param packet data to encode
 * @warn Caller must ensure that `buffer` can hold at least 2 elements.
 */
void encode_track_request(uint8_t *buffer, TrackRequest_t packet);

/**
 * @brief Create `TrackRequest_t` with data encoded in byte buffer.
 * @param packet pointer to `TrackRequest_t` where decoded `TrackRequest_t` should be stored.
 * @param buffer Data encoded as byte buffer.
 * @warn Caller must ensure that `buffer` has at least 2 elements.
 */
void decode_track_request(TrackRequest_t *packet, uint8_t *buffer);

/**
 * @brief Create byte buffer with data from `TrackResponse_t`.
 * @param buffer destination position of byte buffer
 * @param packet data to encode
 * @warn Caller must ensure that `buffer` can hold at least 4 elements.
 */
void encode_track_response(uint8_t *buffer, TrackResponse_t packet);

/**
 * @brief Create `TrackResponse_t` with data encoded in byte buffer.
 * @param packet pointer to `TrackResponse_t` where decoded `TrackResponse_t` should be stored.
 * @param buffer Data encoded as byte buffer.
 * @warn Caller must ensure that `buffer` has at least 4 elements.
 */
void decode_track_response(TrackResponse_t *packet, uint8_t *buffer);

/**
 * @brief Estimate the current position from 3 TrackRepsonses from different Beacons.
 * @param a `TrackResponse_t` from first Beacon.
 * @param b `TrackResponse_t` from second Beacon.
 * @param c `TrackResponse_t` from third Beacon.
 */
void estimate_position(TrackResponse_t a, TrackResponse_t b, TrackResponse_t c);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
	/* Set verbose-level for debug printing
	 * VLEVEL_H = All error and debug messages.
	 * VLEVEL_M = Data output (RSSI readings) and critical messages.
	 * VLEVEL_L = Only critical messages.
	 */
	UTIL_ADV_TRACE_SetVerboseLevel(VLEVEL_M);

  APP_LOG(TS_OFF, VLEVEL_M, "\n\rLoRa Locator\n\r");
  /* Get SubGHY_Phy APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* Led Timers*/
  if (UTIL_TIMER_Create(&timerTimeout, TIMEOUT_PERIOD_MS, UTIL_TIMER_ONESHOT, OnTimeoutEvent, NULL) != UTIL_TIMER_OK) {
	  APP_LOG(TS_ON, VLEVEL_H, "Could not create timeout timer.\n\r");
  }
//  UTIL_TIMER_Create(&timerLed, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledEvent, NULL);
//  UTIL_TIMER_Start(&timerLed);
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  /*calculate random delay for synchronization*/
  random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/

  /* Radio Set frequency */
  Radio.SetChannel(RF_FREQUENCY);

  /* Radio configuration */
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);

  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                    0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                    0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                    0, 0, false, true);

  Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);

#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  /*fills tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  APP_LOG(TS_ON, VLEVEL_H, "rand=%d\n\r", random_delay);
  /*starts reception*/
  Radio.Rx(RX_TIMEOUT_VALUE + random_delay);

  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, Tracking_Process);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_H, "OnTxDone\n\r");
  /* Update the State of the FSM*/
  State = TX;
  /* State change point for `node_state` FSM */
  UpdateState();
  /* Run PingPong process in background*/
  QueueTrackingTask();
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  APP_LOG(TS_ON, VLEVEL_H, "OnRxDone\n\r");
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_ON, VLEVEL_H, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
  /* Record payload Signal to noise ratio in Lora*/
  SnrValue = LoraSnr_FskCfo;
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
#if ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_ON, VLEVEL_H, "RssiValue=%d dBm, Cfo=%dkHz\n\r", rssi, LoraSnr_FskCfo);
  SnrValue = 0; /*not applicable in GFSK*/
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
  /* Update the State of the FSM*/
  State = RX;
  /* Clear BufferRx*/
  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    memcpy(BufferRx, payload, RxBufferSize);
  }

#if IS_BEACON_DEVICE == 1
  if (node_state == NODE_STATE_TRACK_REQ_PHASE) {
	  if (size >= 2) {
		  TrackRequest_t packet;
		  decode_track_request(&packet, payload);
		  last_id = packet.id;
		  last_rssi = rssi;
	  } else {
		  ErrorHandler("Received packet which was too short.\n\r");
	  }
  } else {
	  ErrorHandler("Received packet in wrong `node_state`.\n\r");
  }
#elif IS_TAG_DEVICE == 1
  if (node_state == NODE_STATE_TRACK_RES_PHASE) {
	  if (size >= 4) {
		  TrackResponse_t packet;
		  decode_track_response(&packet, payload);

		  switch (packet.device) {
		  case BEACONA: {
			  last_beacon_a_trk_res = packet;
		  } break;
		  case BEACONB: {
			  last_beacon_b_trk_res = packet;
		  } break;
		  case BEACONC: {
			  last_beacon_c_trk_res = packet;
		  } break;
		  }
		  APP_LOG(TS_ON, VLEVEL_M, "Recv Packet, %u, %d\n\r", packet.device, packet.recv_rssi);
	  } else {
		  ErrorHandler("Received packet which was too short.\n\r");
	  }
  } else {
	  ErrorHandler("Received packet in wrong `node_state`.\n\r");
  }
#endif

  /* Record Received Signal Strength*/
  RssiValue = rssi;
  /* Record payload content*/

  /* State change point for `node_state` FSM */
  UpdateState();

  /* Run PingPong process in background*/
  QueueTrackingTask();
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  /* Update the State of the FSM*/
  State = TX_TIMEOUT;
  ErrorHandler("TX Timeout\n\r");
  /* Run PingPong process in background*/
  QueueTrackingTask();
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  /* Update the State of the FSM*/
  State = RX_TIMEOUT;
  ErrorHandler("RX Timeout\n\r");
  /* Run PingPong process in background*/
  QueueTrackingTask();
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  /* Update the State of the FSM*/
  State = RX_ERROR;
  ErrorHandler("Error while receiving LoRa-packet.\n\r");
  /* Run PingPong process in background*/
  QueueTrackingTask();
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
void encode_track_request(uint8_t *buffer, TrackRequest_t packet) {
	if (buffer == NULL) {
		return;
	}

	buffer[0] = (uint8_t) packet.device;
	buffer[1] = packet.id;
}

void decode_track_request(TrackRequest_t *packet, uint8_t *buffer) {
	if (buffer == NULL || packet == NULL) {
		packet = NULL;
		return;
	}
	if (buffer[0] > BEACONC) {
		packet = NULL;
		return;
	}

	packet->device = buffer[0];
	packet->id = buffer[1];
}

void encode_track_response(uint8_t *buffer, TrackResponse_t packet) {
	if (buffer == NULL) {
		return;
	}

	buffer[0] = (uint8_t) packet.device;
	buffer[1] = packet.id;
	buffer[2] = (uint8_t) (packet.recv_rssi >> 8) & 0xFF;
	buffer[3] = (uint8_t) packet.recv_rssi & 0xFF;
}

void decode_track_response(TrackResponse_t *packet, uint8_t *buffer) {
	if (buffer == NULL || packet == NULL) {
		packet = NULL;
		return;
	}
	if (buffer[0] > BEACONC) {
		packet = NULL;
		return;
	}

	packet->device = buffer[0];
	packet->id = buffer[1];
	packet->recv_rssi = buffer[2] << 8;
	packet->recv_rssi |= buffer[3];
}

#define RSSI0 50
#define PLE 32.0

float calculate_distance(int16_t rssi) {
	return pow(10, (rssi + RSSI0) / PLE);
}

void estimate_position(TrackResponse_t a, TrackResponse_t b, TrackResponse_t c) {
	float distance_a = calculate_distance(a.recv_rssi);
	float distance_b = calculate_distance(b.recv_rssi);
	float distance_c = calculate_distance(c.recv_rssi);
	return;
}

static uint8_t track_request_id = 1;

void Tracking_Process(void)
{

#if IS_BEACON_DEVICE == 1
		switch(node_state) {
		case NODE_STATE_INIT: {
			APP_LOG(TS_ON, VLEVEL_H, "INIT_PHASE\n\r");
			// Beacon has nothing to do in INIT phase
			SetState(NODE_STATE_TRACK_REQ_PHASE);
			QueueTrackingTask();
		}break;
		case NODE_STATE_TRACK_REQ_PHASE: {
			// Beacon is ready for receiving TrackRequests
			APP_LOG(TS_ON, VLEVEL_H, "REQ_PHASE\n\r");
			next_state = NODE_STATE_TRACK_RES_PHASE;
			Radio.Rx(RX_TIMEOUT_VALUE);
		}break;
		case NODE_STATE_TRACK_RES_PHASE: {
			APP_LOG(TS_ON, VLEVEL_H, "RES_PHASE\n\r");
			// Beacon answers TrackRequest with a TrackResponse
			TrackResponse_t packet;
			switch (BEACON_ID) {
			case 'A': {
				packet.device = BEACONA;
			} break;
			case 'B': {
				packet.device = BEACONB;
			} break;
			case 'C': {
				packet.device = BEACONC;
			} break;
			}
			packet.id = last_id;
			packet.recv_rssi = last_rssi;

			encode_track_response(BufferTx, packet);

			next_state = NODE_STATE_WAIT;

			Radio.Send(BufferTx, PAYLOAD_LEN);
		}break;
		case NODE_STATE_WAIT: {
			APP_LOG(TS_ON, VLEVEL_H, "WAIT_PHASE\n\r");
			SetState(NODE_STATE_INIT);
			QueueTrackingTask();
		}break;
		}
#elif IS_TAG_DEVICE == 1
		switch(node_state) {
		case NODE_STATE_INIT: {
			// nothing to do -> switch to next state;
			APP_LOG(TS_ON, VLEVEL_H, "INIT_PHASE\n\r");
			SetState(NODE_STATE_TRACK_REQ_PHASE);
			QueueTrackingTask();
		}break;
		case NODE_STATE_TRACK_REQ_PHASE: {
			// Tag sends TrackRequest in REQ_PHASE
			APP_LOG(TS_ON, VLEVEL_H, "REQ_PHASE\n\r");
			TrackRequest_t track_request;
			track_request.device = TAG;
			track_request.id = track_request_id;
			track_request_id++;

			encode_track_request(BufferTx, track_request);

			APP_LOG(TS_ON, VLEVEL_H, "Radio.Send()\n\r");

			next_state = NODE_STATE_TRACK_RES_PHASE;

			UTIL_TIMER_Start(&timerTimeout);
			Radio.Send(BufferTx, PAYLOAD_LEN);
		}break;
		case NODE_STATE_TRACK_RES_PHASE: {
			// Tag receives TrackResponses from all three beacons
			//APP_LOG(TS_ON, VLEVEL_H, "RES_PHASE\n\r");
			if (last_beacon_a_trk_res.id >= 1 && last_beacon_a_trk_res.id == last_beacon_b_trk_res.id && last_beacon_a_trk_res.id == last_beacon_c_trk_res.id) {
				estimate_position(last_beacon_a_trk_res, last_beacon_b_trk_res, last_beacon_c_trk_res);
				SetState(NODE_STATE_WAIT);

			} else {
				Radio.Rx(RX_TIMEOUT_VALUE);
			}
		}break;
		case NODE_STATE_WAIT: {
			APP_LOG(TS_ON, VLEVEL_H, "WAIT_PHASE\n\r");
			SetState(NODE_STATE_INIT);
			QueueTrackingTask();
		}break;
		}
#endif
}

//static void OnledEvent(void *context)
//{
//  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN */
//  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
//  UTIL_TIMER_Start(&timerLed);
//}

static void OnTimeoutEvent(void *context)
{
	ErrorHandler("TX Timeout\n\r");
	QueueTrackingTask();
}

static void ErrorHandler(char* errorMsg) {
	APP_LOG(TS_ON, VLEVEL_H, "ERROR: %s", errorMsg);
	SetState(NODE_STATE_INIT);
}

static void UpdateState() {
	node_state = next_state;
}

static void SetState(NodeState_t new_state) {
	next_state = new_state;
	UpdateState();
}

static void QueueTrackingTask() {
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

/* USER CODE END PrFD */
