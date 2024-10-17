/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    lora_locator_app.c
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
#include "lora_locator_app.h"
#include "lora_locator_conf.h"
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
    TAG = 'T',
    BEACONA = 'A',
    BEACONB = 'B',
    BEACONC = 'C',
} Device_t;

/**
 * @brief Discriminator between possible packet types. Value for one packet type
 * must not change between software versions, otherwise backwards compatibility
 * would break.
 */
typedef enum {
    PACKET_TYPE_PING = 1,
    PACKET_TYPE_ACK = 2,
    PACKET_TYPE_ANCHOR_RESPONSE = 3,
} PacketType_t;

typedef struct {
    PacketType_t packet_type;
    uint8_t device_id;
    uint8_t packet_id;
} Ping_t;

#define PING_SIZE 3

/**
 * @note Even though there is a discriminator defined for AnchorResponse_t
 * packet type it is not used, because the packet type can already determined
 * by the packet size.
 */
typedef struct {
    Device_t anchor_id;
    uint8_t packet_id;
    int16_t recv_rssi;
} AnchorResponse_t;

#define ANCHOR_RESPONSE_SIZE 4

typedef struct {
    PacketType_t packet_type;
    Device_t receiver_id;
    uint8_t packet_id;
} Ack_t;

#define ACK_SIZE 3

typedef enum {
    NODE_STATE_INIT,
    NODE_STATE_INTERVAL_START,
    NODE_STATE_RX_END,
    NODE_STATE_TX_END,
} NodeState_t;

typedef enum {
    RESULT_OK,
    RESULT_ERROR,
} ResultState_t;

typedef struct {
    int16_t rssi;
    uint16_t payload_size;
    int8_t snr;
} RxMeta_t;

typedef struct {
    ResultState_t state;
    char* msg;
    PacketType_t packet_type;
    void* packet;
    RxMeta_t metadata;
} RxResult_t;

typedef struct {
    ResultState_t state;
    char* msg;
    PacketType_t packet_type;
    void* packet;
} TxResult_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_MS               200
#define PING_TIMEOUT_MS             200
#define ACK_TIMEOUT_MS              400
#define TX_TIMEOUT_MS               200
// 1200ms Timeout because End node sends Ping_t every 1000ms, so if a sending Endnode exists a Ping_t must be received in interval of 1200ms
#define ACQUIRE_PING_TIMEOUT_MS     1200

#define INTERVAL_PERIOD_MS          1000
/* Listen Period*/
#define LISTEN_PERIOD_MS            200

#define MAX_ANCHOR_RESPONSE_RETRIES   3

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

#if (IS_ANCHOR_NODE == 1 && IS_END_NODE == 0)
#define SWITCH_DEVICE_TYPE(ANCHOR_CODE, END_NODE_CODE) #ANCHOR_CODE;
#elif (IS_ANCHOR_NODE == 0 && IS_END_NODE == 1)
#define SWITCH_DEVICE_TYPE(ANCHOR_CODE, END_NODE_CODE) #END_NODE_CODE;
#else
#endif

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
/* App Tx Buffer*/
static uint8_t tx_buffer[MAX_APP_BUFFER_SIZE];

/** @brief State variable that records from which context the node entered
 * the core function `LoraLocator_Process`.
 */
static NodeState_t node_state = NODE_STATE_INIT;

/** @brief Result of a RX operation. */
static RxResult_t rx_result;
/** @brief Result of a TX operation. */
static TxResult_t tx_result;

/** @brief Last received `AnchorResponse_t`. */
static AnchorResponse_t rx_anchor_response;
/** @brief Last transmitted `AnchorResponse_t`. */
static AnchorResponse_t tx_anchor_response;
/** @brief Track how often `AnchorResponse_t` transmission was retried. */
static uint8_t anchor_response_retries = 0;

/** @brief Last received `Ping_t`. */
static Ping_t rx_ping;
/** @brief RSSI value of last received `Ping_t`. */
static int16_t rx_ping_rssi;
/** @brief Last transmitted `Ping_t`. */
static Ping_t tx_ping;

/** @brief Last received `Ack_t`. */
static Ack_t rx_ack;
/** @brief Last transmitted `Ack_t`. */
static Ack_t tx_ack;

/** @brief Timer that triggers `LoraLocator_Process` periodically to either transmit a `Ping_t` (end node) or listen for a `Ping_t` (anchor node). */
static UTIL_TIMER_Object_t interval_timer;
/** @brief Timer that limits the time spent per interval listening for incoming transmissions. Transceiver is put in sleep mode after `listen_timer` timeout. */
static UTIL_TIMER_Object_t listen_timer;

/** @brief Random delay to make sure 2 devices will sync
 *  the closest the random delays are, the longer it will
 *  take for the devices to sync when started simultaneously
 */
static int32_t random_delay;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

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
 * @brief   Function executed when interval timer elapses
 * @param context ptr of Timeout context
 */
static void OnIntervalEvent(void *context);

/**
 * @brief  Function executed on when listen timer elapses
 * @param  context ptr of LED context
 */
static void OnListenEndEvent(void *context);

/**
 * @brief Create byte buffer with data from `EndNodeRequest_t`.
 * @param buffer destination position of byte buffer
 * @param packet data to encode
 * @warn Caller must ensure that `buffer` can hold at least 2 elements.
 */
void encode_Ping(uint8_t *buffer, const Ping_t* packet);

/**
 * @brief Create `EndNodeRequest_t` with data encoded in byte buffer.
 * @param packet pointer to `EndNodeRequest_t` where decoded `EndNodeRequest_t` should be stored.
 * @param buffer Data encoded as byte buffer.
 * @warn Caller must ensure that `buffer` has at least 2 elements.
 */
void decode_Ping(Ping_t *packet, const uint8_t *buffer);

/**
 * @brief Create byte buffer with data from `AnchorResponse_t`.
 * @param buffer destination position of byte buffer
 * @param packet data to encode
 * @warn Caller must ensure that `buffer` can hold at least 4 elements.
 */
void encode_AnchorResponse(uint8_t* buffer, const AnchorResponse_t* packet);

/**
 * @brief Create `AnchorResponse_t` with data encoded in byte buffer.
 * @param packet pointer to `AnchorResponse_t` where decoded `AnchorResponse_t` should be stored.
 * @param buffer Data encoded as byte buffer.
 * @warn Caller must ensure that `buffer` has at least 4 elements.
 */
void decode_AnchorResponse(AnchorResponse_t* packet, const uint8_t* buffer);

/**
 * @brief Create byte buffer with data from `Ack_t`.
 * @param buffer destination position of byte buffer
 * @param packet data to encode
 * @warn Caller must ensure that `buffer` can hold at least 3 elements.
 */
void encode_Ack(uint8_t* buffer, const Ack_t* packet);

/**
 * @brief Create `Ack_t` with data encoded in byte buffer.
 * @param pointer to `Ack_t` where decoded `Ack_t` should be stored.
 * @param buffer Data encoded as byte buffer.
 * @warn Caller must ensure that `buffer` has at least 3 elements.
 */
void decode_Ack(Ack_t* ack, const uint8_t* buffer);

void send_ping();
void send_anchor_response(const Ping_t* ping, int16_t rssi);
void acknowledge_packet(const AnchorResponse_t* anchor_response);

/**
 * @brief Estimate the current position from 3 TrackRepsonses from different Beacons.
 * @param a `AnchorResponse_t` from first Beacon.
 * @param b `AnchorResponse_t` from second Beacon.
 * @param c `AnchorResponse_t` from third Beacon.
 */
void estimate_position(const AnchorResponse_t* a, const AnchorResponse_t* b, const AnchorResponse_t* c);

/*
 * State management, scheduling, debug/info helpers
 */

/**
 * @brief Sets the `node_state` global to `new_state`.
 * @param new_state New `NodeState_t` value for `node_state`.
 */
static void SetState(NodeState_t new_state);

/**
 * @brief Add `TrackingProcess` to UTIL_SEQ task-queue. (Queue `TrackingProcess` for execution.)
 */
static void QueueLoraLocatorTask();

char* pt_toString(PacketType_t packet_type);

/**
 * @brief Print information about device and radio configuration. Called on initialization.
 */
static void print_metadata();

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void LoraLocator_Init(void)
{
    /* USER CODE BEGIN SubghzApp_Init_1 */
    LoraLocator_Config();

    /* Timers */
#if IS_ANCHOR_NODE == 0 && IS_END_NODE == 1
    if (UTIL_TIMER_Create(&interval_timer, INTERVAL_PERIOD_MS, UTIL_TIMER_PERIODIC, &OnIntervalEvent, NULL) != UTIL_TIMER_OK) {
        APP_LOG(TS_ON, VLEVEL_M, "Could not create interval timer.\n\r");
    }
#elif IS_ANCHOR_NODE == 1 && IS_END_NODE == 0
    // make interval_timer slightly shorter on anchor node to ensure anchor is already in receiver mode when end node starts transmitting
    if (UTIL_TIMER_Create(&interval_timer, INTERVAL_PERIOD_MS-100, UTIL_TIMER_PERIODIC, &OnIntervalEvent, NULL) != UTIL_TIMER_OK) {
        APP_LOG(TS_ON, VLEVEL_M, "Could not create interval timer.\n\r");
    }
#else
#error "Set atleast/only one of IS_ANCHOR_NODE and IS_END_NODE to 1."
#endif
    if (UTIL_TIMER_Create(&listen_timer, LISTEN_PERIOD_MS, UTIL_TIMER_ONESHOT, &OnListenEndEvent, NULL) != UTIL_TIMER_OK) {
        APP_LOG(TS_ON, VLEVEL_M, "Could not create listen timer.\n\r");
    }
    /* USER CODE END SubghzApp_Init_1 */

    /* Radio initialization */
    RadioEvents.TxDone = &OnTxDone;
    RadioEvents.RxDone = &OnRxDone;
    RadioEvents.TxTimeout = &OnTxTimeout;
    RadioEvents.RxTimeout = &OnRxTimeout;
    RadioEvents.RxError = &OnRxError;

    Radio.Init(&RadioEvents);

    /* USER CODE BEGIN SubghzApp_Init_2 */
    /*calculate random delay for synchronization*/
    random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/

    /* Radio Set frequency */
    Radio.SetChannel(RF_FREQUENCY);

    /* Radio configuration */
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
            LORA_SPREADING_FACTOR, LORA_CODINGRATE,
            LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
            true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_MS);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
            LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
            LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
            0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

    /*fills tx buffer*/
    memset(tx_buffer, 0x0, MAX_APP_BUFFER_SIZE);

    print_metadata();

    /*register task to to be run in while(1) after Radio IT*/
    UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, LoraLocator_Process);
    QueueLoraLocatorTask();
    /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/*
 * Event handlers
 */

static void OnListenEndEvent(void* context) {
    // Exit Rx mode
    Radio.Standby();
    // Enter Sleep mode
    Radio.Sleep();
}

static void OnIntervalEvent(void *context)
{
    SetState(NODE_STATE_INTERVAL_START);
    QueueLoraLocatorTask();
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
    /* USER CODE BEGIN OnTxDone */
    /* Update the State of the FSM*/
    State = TX;
    switch (tx_result.packet_type) {
    case PACKET_TYPE_PING: {
        const Ping_t* ping = (Ping_t*) tx_result.packet;
        APP_LOG(TS_ON, VLEVEL_L, ", TX, Ping, %u, %u,,\n\r", ping->device_id, ping->packet_id);
    }break;
    case PACKET_TYPE_ANCHOR_RESPONSE: {
        const AnchorResponse_t* r = (AnchorResponse_t*) tx_result.packet;
        APP_LOG(TS_ON, VLEVEL_L, ", TX, AnchorResponse, %c, %u, %d\n\r", r->anchor_id, r->packet_id, r->recv_rssi);
    }break;
    case PACKET_TYPE_ACK: {
        const Ack_t* ack = (Ack_t*) tx_result.packet;
        APP_LOG(TS_ON, VLEVEL_L, ", TX, Ack, %c, %u,,\n\r", ack->receiver_id, ack->packet_id);
    }break;
    }
    /* Tx was successful */
    tx_result.state = RESULT_OK;
    /* State change point for `node_state` FSM */
    SetState(NODE_STATE_TX_END);
    /* Run PingPong process in background*/
    QueueLoraLocatorTask();
    /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
    /* USER CODE BEGIN OnRxDone */
    /* Update the State of the FSM*/
    State = RX;

    /* Record payload Signal to noise ratio in Lora*/
    rx_result.metadata.snr = LoraSnr_FskCfo;
    /* Record payload size*/
    rx_result.metadata.payload_size = size;
    /* Record Received Signal Strength*/
    rx_result.metadata.rssi = rssi;

#if IS_ANCHOR_NODE == 0 && IS_END_NODE == 1
    if (size == ANCHOR_RESPONSE_SIZE) {
        rx_result.packet_type = PACKET_TYPE_ANCHOR_RESPONSE;
        rx_result.state = RESULT_OK;
        rx_result.packet = &rx_anchor_response;
        decode_AnchorResponse(rx_result.packet, payload);
        const AnchorResponse_t* r = (AnchorResponse_t*) rx_result.packet;
        APP_LOG(TS_ON, VLEVEL_L, ", RX, AnchorResponse, %c, %u, %d\n\r", r->anchor_id, r->packet_id, r->recv_rssi);
    } else {
        rx_result.state = RESULT_ERROR;
        rx_result.msg = "Could not decode received packet it had incorrect length.";
    }
#elif IS_ANCHOR_NODE == 1 && IS_END_NODE == 0
    if (size == PING_SIZE) {
        if (payload[0] == PACKET_TYPE_PING) {
            rx_result.packet_type = PACKET_TYPE_PING;
            rx_result.packet = &rx_ping;
            decode_Ping(rx_result.packet, payload);

            if (rx_result.packet == NULL) {
                rx_result.state = RESULT_ERROR;
                rx_result.msg = "Could not decode received packet because of wrong format.";
            } else {
                rx_result.state = RESULT_OK;
                rx_ping_rssi = rssi;
                APP_LOG(TS_ON, VLEVEL_L, ", RX, Ping, %u, %u, %d.\n\r", ((Ping_t*) rx_result.packet)->device_id, ((Ping_t*) rx_result.packet)->packet_id, rx_ping_rssi);
            }
        } else if (payload[0] == PACKET_TYPE_ACK) {
            rx_result.packet_type = PACKET_TYPE_ACK;
            rx_result.packet = &rx_ack;
            decode_Ack(rx_result.packet, payload);

            if (rx_result.packet == NULL) {
                rx_result.state = RESULT_ERROR;
                rx_result.msg = "Could not decode received packet because of wrong format.";
            } else {
                rx_result.state = RESULT_OK;
                APP_LOG(TS_ON, VLEVEL_L, ", RX, Ack, %c, %u,,\n\r", ((Ack_t*) rx_result.packet)->receiver_id, ((Ack_t*) rx_result.packet)->packet_id);
            }
        } else {
            rx_result.state = RESULT_ERROR;
            rx_result.msg = "Could not decode received packet it had incorrect discriminator.";
        }
    } else {
        rx_result.state = RESULT_ERROR;
        rx_result.msg = "Could not decode received packet it had incorrect length.";
    }
#else
#error "Set atleast/only one of IS_ANCHOR_NODE and IS_END_NODE to 1."
#endif

    /* State change point for `node_state` FSM */
    SetState(NODE_STATE_RX_END);

    /* Run PingPong process in background*/
    QueueLoraLocatorTask();
    /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
    /* USER CODE BEGIN OnTxTimeout */
    /* Update the State of the FSM*/
    State = TX_TIMEOUT;
    SetState(NODE_STATE_TX_END);
    tx_result.state = RESULT_ERROR;
    tx_result.msg = "TxError: TX Timeout.";
    /* Run PingPong process in background*/
    QueueLoraLocatorTask();
    /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
    /* USER CODE BEGIN OnRxTimeout */
    /* Update the State of the FSM*/
    State = RX_TIMEOUT;
    SetState(NODE_STATE_RX_END);
    rx_result.state = RESULT_ERROR;
    rx_result.msg = "RxError: RX Timeout.";
    /* Run PingPong process in background*/
    QueueLoraLocatorTask();
    /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
    /* USER CODE BEGIN OnRxError */
    /* Update the State of the FSM*/
    State = RX_ERROR;

    rx_result.state = RESULT_ERROR;
    rx_result.msg = "RxError: Error during packet reception.";
    SetState(NODE_STATE_RX_END);
    /* Run PingPong process in background*/
    QueueLoraLocatorTask();
    /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

#if (IS_ANCHOR_NODE == 0 && IS_END_NODE == 1)

void LoraLocator_Process(void)
{
    /* Process can be entered from different states:
     * 1) First execution of task     => NODE_STATE_INIT
     * 2) Interval timer elapsed      => NODE_STATE_INTERVAL_START
     * 3) After Rx()                  => NODE_STATE_RX_END
     * 4) After Tx()                  => NODE_STATE_TX_END
     *
     * The state the process is entered from is recorded in the node_state variable.
     */

    switch(node_state) {
    case NODE_STATE_INIT: {
        /* End node:
         * Start interval timer (start sending Ping_t)
         */
        UTIL_TIMER_StartWithPeriod(&interval_timer, INTERVAL_PERIOD_MS);
    }break;
    case NODE_STATE_INTERVAL_START: {
        // End node: Send Ping_t
        send_ping();
    }break;
    case NODE_STATE_RX_END: {
        /* check if ERROR or SUCCESS
         * SUCCESS@End node: Send ACK_t
         * ERROR@End node: log error
         */
        if (rx_result.state == RESULT_OK) {
            if (rx_result.packet_type == PACKET_TYPE_ANCHOR_RESPONSE) {
                acknowledge_packet(rx_result.packet);
            } else {
                APP_LOG(TS_ON, VLEVEL_M, "RX Error: Unexpected packet type. Was %s, expected %s!\n\r", pt_toString(rx_result.packet_type), pt_toString(PACKET_TYPE_ANCHOR_RESPONSE));
            }
        } else {
            APP_LOG(TS_ON, VLEVEL_M, "RX Error: %s\n\r", rx_result.msg);
            Radio.Standby();
            Radio.Sleep();
        }
    }break;
    case NODE_STATE_TX_END: {
        /* check if ERROR or SUCCESS
         * SUCCESS@End node:
         *   check if Ping_t or ACK_t:
         *        Ping_t => start Rx() + start listen_timer
         *        ACK_t => start Rx()
         * ERROR@End node: (log error) +
         *   check if Ping_t or ACK_t:
         *        Ping_t => go to sleep (wait for next interval_start)
         *        ACK_t => resend ACK_t? OR start Rx()
         */
         if (tx_result.state == RESULT_OK) {
             if (tx_result.packet_type == PACKET_TYPE_PING) {
                UTIL_TIMER_StartWithPeriod(&listen_timer, LISTEN_PERIOD_MS);
             }
             Radio.Rx(RX_TIMEOUT_MS);
         } else {
             APP_LOG(TS_ON, VLEVEL_M, "TX Error: %s\n\r", tx_result.msg);
             if (tx_result.packet_type == PACKET_TYPE_PING) {
                 Radio.Standby();
                 Radio.Sleep();
             } else {
                 Radio.Rx(RX_TIMEOUT_MS);
             }
         }
    }break;
    }
}

#elif (IS_ANCHOR_NODE == 1 && IS_END_NODE == 0)

void LoraLocator_Process(void) {
    /* Process can be entered from different states:
     * 1) First execution of task     => NODE_STATE_INIT
     * 2) Interval timer elapsed      => NODE_STATE_INTERVAL_START
     * 3) After Rx()                  => NODE_STATE_RX_END
     * 4) After Tx()                  => NODE_STATE_TX_END
     *
     * The state the process is entered from is recorded in the node_state variable.
     */

    switch(node_state) {
    case NODE_STATE_INIT: {
        /* Anchor:
         * Listen for first Ping_t to synchronize own interval_timer with
         * interval_timer of End node
         */
        Radio.Rx(ACQUIRE_PING_TIMEOUT_MS);
    }break;
    case NODE_STATE_INTERVAL_START: {
        // Anchor: Listen for Ping_t with PING_TIMEOUT_MS
        Radio.Rx(PING_TIMEOUT_MS);
    }break;
    case NODE_STATE_RX_END: {
        /* check if ERROR or SUCCESS
         * SUCCESS@Anchor:
         *   check if Ping_t or ACK_t:
         *        Ping_t => Send AnchorResponse_t
         *        ACK_t => check if correct ACK_t:
         *            correct => go to sleep
         *            invalid => resend AnchorResponse_t
         * ERROR@Anchor: (log error) + restart Rx()?
         */
        if (rx_result.state == RESULT_OK) {
            if (rx_result.packet_type == PACKET_TYPE_PING) {
                // synchronize interval_timer with ping reception -> TODO improve for multiple end nodes, so that multiple ping periods can be tracked
                UTIL_TIMER_StartWithPeriod(&interval_timer, INTERVAL_PERIOD_MS-100);
                UTIL_TIMER_StartWithPeriod(&listen_timer, LISTEN_PERIOD_MS);
                send_anchor_response(rx_result.packet, rx_result.metadata.rssi);
                anchor_response_retries = 0;
            } else if (rx_result.packet_type == PACKET_TYPE_ACK) {
                // go to sleep if max retry amount is reached
                if (anchor_response_retries > MAX_ANCHOR_RESPONSE_RETRIES) {
                    Radio.Standby();
                    Radio.Sleep();
                }

                // check if ACK is for this device and its last transmitted AnchorResponse
                if (((Ack_t*) rx_result.packet)->receiver_id == DEVICE_ID && ((Ack_t*) rx_result.packet)->packet_id == tx_anchor_response.packet_id) {
                    Radio.Standby();
                    Radio.Sleep();
                } else {
                    send_anchor_response(&rx_ping, rx_ping_rssi); // resend AnchorResponse_t for last received ping
                    anchor_response_retries++;
                }
            }
        } else {
            APP_LOG(TS_ON, VLEVEL_M, "RX Error: %s\n\r", rx_result.msg);
            if (UTIL_TIMER_IsRunning(&listen_timer)) {
                Radio.Rx(ACK_TIMEOUT_MS);
            } else {
                Radio.Rx(ACQUIRE_PING_TIMEOUT_MS);
            }
        }
    }break;
    case NODE_STATE_TX_END: {
        /* check if ERROR or SUCCESS
         * SUCCESS@Anchor: start Rx(ACK_TIMEOUT)
         * ERROR@Anchor: (log error) + go to sleep (wait for next Ping_t)
         */
        if (tx_result.state == RESULT_OK) {
            Radio.Rx(ACK_TIMEOUT_MS);
        } else {
            APP_LOG(TS_ON, VLEVEL_M, "TX Error: %s\n\r", tx_result.msg);
            Radio.Standby();
            Radio.Sleep();
        }
    }break;
    }
}

#else

#error "Set atleast/only one of IS_ANCHOR_NODE and IS_END_NODE to 1."

#endif

/*
 * Encoding/Decoding
 */
void encode_Ping(uint8_t *buffer, const Ping_t* packet) {
    if (buffer == NULL) {
        return;
    }

    buffer[0] = (uint8_t) packet->packet_type;
    buffer[1] = (uint8_t) packet->device_id;
    buffer[2] = packet->packet_id;
}

void decode_Ping(Ping_t *packet, const uint8_t *buffer) {
    if (buffer == NULL || packet == NULL) {
        packet = NULL;
        return;
    }
    if (buffer[1] > BEACONC) {
        packet = NULL;
        return;
    }

    packet->packet_type = buffer[0];
    packet->device_id = buffer[1];
    packet->packet_id = buffer[2];
}

void encode_AnchorResponse(uint8_t *buffer, const AnchorResponse_t* packet) {
    if (buffer == NULL) {
        return;
    }

    buffer[0] = (uint8_t) packet->anchor_id;
    buffer[1] = packet->packet_id;
    buffer[2] = (uint8_t) (packet->recv_rssi >> 8) & 0xFF;
    buffer[3] = (uint8_t) packet->recv_rssi & 0xFF;
}

void decode_AnchorResponse(AnchorResponse_t* packet, const uint8_t* buffer) {
    if (buffer == NULL || packet == NULL) {
        packet = NULL;
        return;
    }
    if (buffer[0] > BEACONC) {
        packet = NULL;
        return;
    }

    packet->anchor_id = buffer[0];
    packet->packet_id = buffer[1];
    packet->recv_rssi = buffer[2] << 8;
    packet->recv_rssi |= buffer[3];
}

void encode_Ack(uint8_t* buffer, const Ack_t* ack) {
    if (buffer == NULL || ack == NULL) {
        return;
    }

    buffer[0] = ack->packet_type;
    buffer[1] = ack->receiver_id;
    buffer[2] = ack->packet_id;
}

void decode_Ack(Ack_t* ack, const uint8_t* buffer) {
    if (buffer == NULL || ack == NULL) {
        return;
    }

    ack->packet_type = buffer[0];
    ack->receiver_id = buffer[1];
    ack->packet_id = buffer[2];
}

/*
 * Sending packets
 */
static uint8_t ping_id_counter = 1;

void send_ping() {
    tx_ping.packet_type = PACKET_TYPE_PING;
    tx_ping.device_id = DEVICE_ID;
    tx_ping.packet_id = ping_id_counter;
    ping_id_counter++;

    tx_result.packet_type = tx_ping.packet_type;
    tx_result.packet = &tx_ping;
    encode_Ping(tx_buffer, tx_result.packet);
    Radio.Send(tx_buffer, sizeof tx_ping);
}

void acknowledge_packet(const AnchorResponse_t* anchor_response) {
    tx_ack.packet_type = PACKET_TYPE_ACK;
    tx_ack.packet_id = anchor_response->packet_id;
    tx_ack.receiver_id = anchor_response->anchor_id;

    tx_result.packet_type = tx_ack.packet_type;
    tx_result.packet = &tx_ack;
    encode_Ack(tx_buffer, tx_result.packet);
    Radio.Send(tx_buffer, sizeof tx_ack);
}

void send_anchor_response(const Ping_t* ping, int16_t rssi) {
    tx_anchor_response.anchor_id = DEVICE_ID;
    tx_anchor_response.packet_id = ping->packet_id;
    tx_anchor_response.recv_rssi = rssi;

    tx_result.packet_type = PACKET_TYPE_ANCHOR_RESPONSE;
    tx_result.packet = &tx_anchor_response;
    encode_AnchorResponse(tx_buffer, tx_result.packet);
    Radio.Send(tx_buffer, sizeof tx_anchor_response);
}

/*
 * Math for position estimation
 */

#define RSSI0 50
#define PLE 32.0

float calculate_distance(int16_t rssi) {
    return pow(10, (rssi + RSSI0) / PLE);
}

void estimate_position(const AnchorResponse_t* a, const AnchorResponse_t* b, const AnchorResponse_t* c) {
    float distance_a = calculate_distance(a->recv_rssi);
    float distance_b = calculate_distance(b->recv_rssi);
    float distance_c = calculate_distance(c->recv_rssi);
    return;
}

/*
 * State management, scheduling, debug/info helpers
 */

static void SetState(NodeState_t new_state) {
    node_state = new_state;
}

static void QueueLoraLocatorTask() {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void print_metadata() {
    APP_LOG(TS_OFF, VLEVEL_L, "\n\r====================\n\r");
    APP_LOG(TS_OFF, VLEVEL_L, "LoRa Locator\n\r\n\r");
    /* Get App version*/
    APP_LOG(TS_OFF, VLEVEL_L, "APPLICATION_VERSION: V%X.%X.%X\r\n",
            (uint8_t)(APP_VERSION_MAIN),
            (uint8_t)(APP_VERSION_SUB1),
            (uint8_t)(APP_VERSION_SUB2));

    /* Get Middleware SubGhz_Phy info */
    APP_LOG(TS_OFF, VLEVEL_L, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
            (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
            (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
            (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

    /* Modulation info */
    APP_LOG(TS_OFF, VLEVEL_L, "---------------\n\r");
    APP_LOG(TS_OFF, VLEVEL_L, "LORA_MODULATION\n\r");
    APP_LOG(TS_OFF, VLEVEL_L, "LORA_BW: %d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
    APP_LOG(TS_OFF, VLEVEL_L, "LORA_SF: %d\n\r", LORA_SPREADING_FACTOR);

    /* Device configuration */
    APP_LOG(TS_OFF, VLEVEL_L, "--------------------\n\r");
    APP_LOG(TS_OFF, VLEVEL_L, "DEVICE_CONFIGURATION\n\r");
    APP_LOG(TS_OFF, VLEVEL_L, "DEVICE_TYPE: %s\n\r", IS_ANCHOR_NODE ? "Anchor" : "EndNode");
#if IS_ANCHOR_NODE == 1 && IS_END_NODE == 0
    APP_LOG(TS_OFF, VLEVEL_L, "DEVICE_ID: %c\n\r", DEVICE_ID);
#elif IS_ANCHOR_NODE == 0 && IS_END_NODE == 1
    APP_LOG(TS_OFF, VLEVEL_L, "DEVICE_ID: %u\n\r", DEVICE_ID);
#else
#error "Set atleast/only one of IS_ANCHOR_NODE and IS_END_NODE to 1."
#endif
    APP_LOG(TS_OFF, VLEVEL_L, "====================\n\r");

    APP_LOG(TS_OFF, VLEVEL_L, "rand: %d\n\r", random_delay);
}

char* pt_toString(PacketType_t packet_type) {
    switch (packet_type) {
    case PACKET_TYPE_PING: return "PING";
    case PACKET_TYPE_ANCHOR_RESPONSE: return "ANCHOR_RESPONSE";
    case PACKET_TYPE_ACK: return "ACK";
    default: return "UNKNOWN";
    }
}


/* USER CODE END PrFD */
