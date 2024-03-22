#pragma once

#include "Core/Inc/main.h"
#include "cmsis_os.h"

// A message in the transmit queue should be 1 larger than the maximum content of
// a Message i.e. 1 + k_headerSize (8) + k_maxPayloadBytes (142) + k_crcSize (CRC16 = 2)
#define TX_QUEUE_MSG_BUFFER_SIZE 153
// At 115200 baud 128 bytes will hold 10ms of data
// The UART DMA shares this buffer size
#define RX_QUEUE_MSG_BUFFER_SIZE 128

#ifdef __cplusplus
extern "C"
#endif
typedef __PACKED_STRUCT
{
    uint8_t length;
    uint8_t buffer[TX_QUEUE_MSG_BUFFER_SIZE];
} TxQueueMsg_t;

#ifdef __cplusplus
extern "C"
#endif
typedef __PACKED_STRUCT
{
    uint32_t length;
    uint8_t buffer[RX_QUEUE_MSG_BUFFER_SIZE];
} RxQueueMsg_t;

extern osMessageQueueId_t epuRxQueueHandle;
extern osMessageQueueId_t epuTxQueueHandle;

extern osMessageQueueId_t extRxQueueHandle;
extern osMessageQueueId_t extTxQueueHandle;
