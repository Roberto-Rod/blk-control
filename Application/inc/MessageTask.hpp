#pragma once

#ifdef __cplusplus
extern "C"
#endif
void messageTaskRun();

#ifdef __cplusplus
#include "Application/inc/MessageReceiver.hpp"
#include "Application/inc/Bsp.hpp"
#include "Application/inc/Queues.hpp"

#include "cmsis_os.h"

#include <cstdint>
#include <array>

namespace mercury
{
    namespace blackstar
    {
        void sendMessage(osMessageQueueId_t txQueueHandle, Message& message);

        class MessageTask
        {
        public:
            MessageTask() = default;
            [[noreturn]] void run();

        private:
            static constexpr uint32_t k_linkTimeOutTicks { 3000u }; // 3 second timeout

            void receivingMessage(osMessageQueueId_t queue, uint32_t count);
            void processMessage(Message& message);
            void receivingEXTMessage(osMessageQueueId_t queue, uint32_t count);
            void processEXTMessage(RxQueueMsg_t data);

            MessageReceiver m_epuReceiver { Bsp::getCRCCalculator(), Bsp::getOSLockForCRC() };

            static constexpr auto k_numberOfReceiveQueues { 1 };
            static constexpr auto k_epuQueueIndex { 0 };
            struct QueueStatus
            {
                uint32_t m_lastMessageTime { 0u };
                bool m_firstMessageReceived { false };
            };
            std::array<QueueStatus, k_numberOfReceiveQueues> m_queueStatus {};
        };
    }
}
#endif // __cplusplus
