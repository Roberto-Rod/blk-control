#include "Application/inc/UartTask.hpp"
#include "Application/inc/Bsp.hpp"

#include "Core/Inc/main.h"
#include "cmsis_os.h"

static USART_TypeDef *huart1 { USART1 };
static USART_TypeDef *huart2 { USART2 };

extern osSemaphoreId_t uartTxSemaphoreHandle;
extern osEventFlagsId_t uartEventsHandle;
extern uint32_t g_epuDMAHighWatermark;

namespace
{
    constexpr uint32_t k_disableRCUTxFlag { 0x00000001 };
    constexpr bool k_enable { true };
    constexpr bool k_disable { false };
}

static inline void usartISRHandler(USART_TypeDef *phuart, const char *errMsg)
{
    // Framing Error
    if (LL_USART_IsActiveFlag_FE(phuart))
    {
        LL_USART_ClearFlag_FE(phuart);
        debug_printf("%s FE\r\n", errMsg);
    }
    // Overrun Error
    else if (LL_USART_IsActiveFlag_ORE(phuart))
    {
        LL_USART_ClearFlag_ORE(phuart);
        debug_printf("%s ORE\r\n", errMsg);
    }
    else if (LL_USART_IsEnabledIT_IDLE(phuart) && LL_USART_IsActiveFlag_IDLE(phuart))
    {
        // Clear IDLE line flag
        LL_USART_ClearFlag_IDLE(phuart);
    }
}
extern "C"
{
    void USART1_IRQHandler(void)
    {
        usartISRHandler(huart1, "EXT");
    }

    void USART2_IRQHandler(void)
    {
        usartISRHandler(huart2, "EPU");
    }

    void DMA1_Channel2_3_IRQHandler(void)
    {
        // Channel 2 - huart1 (EXT) TX
        if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_2))
        {
            if (LL_DMA_IsActiveFlag_TE2(DMA1))
            {
                // Clear transfer error flag
                LL_DMA_ClearFlag_TE2(DMA1);
                (void)osSemaphoreRelease(uartTxSemaphoreHandle);
            }
            else if (LL_DMA_IsActiveFlag_TC2(DMA1))
            {
                // Clear transfer complete flag
                LL_DMA_ClearFlag_TC2(DMA1);
                (void)osSemaphoreRelease(uartTxSemaphoreHandle);
                (void)osEventFlagsSet(uartEventsHandle, k_disableRCUTxFlag);
            }
        }

        // Channel 3 - huart1 (EXT) RX
        if (LL_DMA_IsActiveFlag_TE3(DMA1))
        {
            // Clear transfer error flag
            LL_DMA_ClearFlag_TE3(DMA1);
            debug_printf("TE EXT\r\n");
        }
        else if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_3) && LL_DMA_IsActiveFlag_HT3(DMA1))
        {
            // Clear half transfer complete flag
            LL_DMA_ClearFlag_HT3(DMA1);
        }
        else if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_3) && LL_DMA_IsActiveFlag_TC3(DMA1))
        {
            // Clear transfer complete flag
            LL_DMA_ClearFlag_TC3(DMA1);
        }
    }

    void DMA1_Channel4_5_6_7_IRQHandler(void)
    {
        // Channel 4 - huart2 (EPU) TX
        if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_4))
        {
            if (LL_DMA_IsActiveFlag_TE4(DMA1))
            {
               // Clear transfer error flag
               LL_DMA_ClearFlag_TE4(DMA1);
               (void)osSemaphoreRelease(uartTxSemaphoreHandle);
            }
            else if (LL_DMA_IsActiveFlag_TC4(DMA1))
            {
               // Clear transfer complete flag
               LL_DMA_ClearFlag_TC4(DMA1);
               (void)osSemaphoreRelease(uartTxSemaphoreHandle);
            }
        }

        // Channel 5 - huart2 (EPU) RX
        if (LL_DMA_IsActiveFlag_TE5(DMA1))
        {
            // Clear transfer error flag
            LL_DMA_ClearFlag_TE5(DMA1);
            debug_printf("TE EPU\r\n");
        }
        else if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_HT5(DMA1))
        {
            // Clear half transfer complete flag
            LL_DMA_ClearFlag_HT5(DMA1);
        }
        else if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_TC5(DMA1))
        {
            // Clear transfer complete flag
            LL_DMA_ClearFlag_TC5(DMA1);
        }
    }
} // end extern "C"

void uartTaskRun()
{
    mercury::blackstar::UartTask uartTask;
    uartTask.run();
}

namespace
{
    void checkReceiver(RxQueueMsg_t *pQueueMsg,
                       uint8_t *pDMABuffer,
                       uint32_t& currentTail,
                       const size_t dmaBufferSize,
                       const uint32_t dmaBufferAvailableSpace,
                       const osMessageQueueId_t queue,
                       const char *errMsg)
    {
        // The DMA buffer (pointed to by pDMABuffer) is being written to in a circular manner by the hardware.
        // The buffer is of dmaBufferSize and the currentTail is the index of the last byte processed by this task.
        // dmaBufferAvailableSpace is read from the hardware (DMA_CNDTR NDT) and returns the number of bytes available
        // before the end of the buffer (since circular mode - the number of bytes before wrap around occurs)
        const uint32_t head { dmaBufferSize - dmaBufferAvailableSpace };
        if (head != currentTail)
        {
            const uint32_t count { (head > currentTail) ? (head - currentTail) : ((dmaBufferSize - currentTail) + head) };
            uint32_t nextTail { currentTail };
            for (uint32_t i = 0u; i < count; ++i)
            {
                pQueueMsg->buffer[i] = pDMABuffer[nextTail++];
                nextTail %= dmaBufferSize; // wrap around
            }
            pQueueMsg->length = count;
            // Place the message into the MessageTask's queue for processing
            osStatus_t msgStatus { osMessageQueuePut(queue, pQueueMsg, 0u, 0u) };
            if (msgStatus == osOK)
            {
                currentTail = nextTail;
            }
            else
            {
                debug_printf("%s RX Put Failed: 0x%x\r\n", errMsg, msgStatus);
                // currentTail remains the same so we should attempt to process the message again next time
            }
        }
    }

    void transmit(osMessageQueueId_t txQueueHandle, USART_TypeDef *phuart, uint32_t dmaChannel)
    {
        (void)osSemaphoreAcquire(uartTxSemaphoreHandle, osWaitForever);

        static TxQueueMsg_t data;
        osStatus_t status { osMessageQueueGet(txQueueHandle, &data, 0u, 0u) };
        if (status == osOK)
        {
            // Configure DMA
            LL_DMA_DisableChannel(DMA1, dmaChannel);
            LL_DMA_SetPeriphAddress(DMA1, dmaChannel, LL_USART_DMA_GetRegAddr(phuart, LL_USART_DMA_REG_DATA_TRANSMIT));
            LL_DMA_SetMemoryAddress(DMA1, dmaChannel, (uint32_t)data.buffer);
            LL_DMA_SetDataLength(DMA1, dmaChannel, data.length);

            // Clear all flags
            LL_DMA_ClearFlag_TC4(DMA1);
            LL_DMA_ClearFlag_HT4(DMA1);
            LL_DMA_ClearFlag_TE4(DMA1);

            // Start transfer
            LL_DMA_EnableIT_TC(DMA1, dmaChannel);
            LL_USART_EnableDMAReq_TX(phuart);
            LL_DMA_EnableChannel(DMA1, dmaChannel);
        }
    }
}

namespace mercury::blackstar
{
    void UartTask::run()
    {
        initialiseEPUReceiver();
        initialiseEXTReceiver();

        while(true)
        {
            checkEPUReceiver();
            checkEXTReceiver();

            if (osMessageQueueGetCount(epuTxQueueHandle))
            {
                transmit(epuTxQueueHandle, huart2, LL_DMA_CHANNEL_4);
            }
            if (osMessageQueueGetCount(extTxQueueHandle))
            {
                transmit(extTxQueueHandle, huart1, LL_DMA_CHANNEL_2);
            }
            osDelay(1u);
        }
    }

    void UartTask::initialiseEPUReceiver()
    {
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, LL_USART_DMA_GetRegAddr(huart2, LL_USART_DMA_REG_DATA_RECEIVE));
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)m_epuDMABuffer.data());
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, m_epuDMABuffer.size());

        LL_DMA_ClearFlag_TC5(DMA1);
        LL_DMA_ClearFlag_HT5(DMA1);
        LL_DMA_ClearFlag_TE5(DMA1);

        LL_USART_ClearFlag_FE(huart2);
        LL_USART_ClearFlag_ORE(huart2);

        // Enable HT & TC interrupts (huart2 - EPU)
        LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
        LL_USART_EnableDMAReq_RX(huart2);
        LL_USART_EnableIT_IDLE(huart2);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    }

    void UartTask::initialiseEXTReceiver()
    {
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(huart1, LL_USART_DMA_REG_DATA_RECEIVE));
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)m_extDMABuffer.data());
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, m_extDMABuffer.size());

        LL_DMA_ClearFlag_TC6(DMA1);
        LL_DMA_ClearFlag_HT6(DMA1);
        LL_DMA_ClearFlag_TE6(DMA1);

        LL_USART_ClearFlag_FE(huart1);
        LL_USART_ClearFlag_ORE(huart1);

        // Enable HT & TC interrupts (huart1 - EXT)
        LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_3);
        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
        LL_USART_EnableDMAReq_RX(huart1);
        LL_USART_EnableIT_IDLE(huart1);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    }

    void UartTask::checkEPUReceiver()
    {
        static RxQueueMsg_t epuMsg;
        checkReceiver(&epuMsg,
                      m_epuDMABuffer.data(),
                      m_epuTail,
                      m_epuDMABuffer.size(),
                      LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5),
                      epuRxQueueHandle,
                      "EPU");
#ifdef DEBUG
        g_epuDMAHighWatermark = std::max(epuMsg.length, g_epuDMAHighWatermark);
#endif
    }

    void UartTask::checkEXTReceiver()
    {
        static RxQueueMsg_t extMsg;
        checkReceiver(&extMsg,
                      m_extDMABuffer.data(),
                      m_extTail,
                      m_extDMABuffer.size(),
                      LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3),
                      extRxQueueHandle,
                      "EXT");
    }
}
