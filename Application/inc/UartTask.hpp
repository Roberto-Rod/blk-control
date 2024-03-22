#pragma once

#ifdef __cplusplus
extern "C"
#endif
void uartTaskRun();

#ifdef __cplusplus
#include "Application/inc/Queues.hpp"

#include <cstdint>
#include <array>

namespace mercury
{
    namespace blackstar
    {
        class UartTask
        {
        public:
            UartTask() = default;
            [[noreturn]] void run();

        private:
            void initialiseEPUReceiver();
            void initialiseEXTReceiver();
            void checkEPUReceiver();
            void checkEXTReceiver();

            // We are polling the DMA @ 1ms interval so in theory the length will be
            // 12 bytes @ 115200 baud. These buffer provide 10ms of storage...
            std::array<uint8_t, RX_QUEUE_MSG_BUFFER_SIZE> m_epuDMABuffer {};
            std::array<uint8_t, RX_QUEUE_MSG_BUFFER_SIZE> m_extDMABuffer {};
            uint32_t m_epuTail { 0u };
            uint32_t m_extTail { 0u };
        };
    }
}
#endif // __cplusplus
