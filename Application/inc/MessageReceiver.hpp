#pragma once

#include "Application/inc/Message.hpp"

#include <cstdint>
#include <array>

namespace mercury
{
    namespace blackstar
    {
        class MessageReceiver final
        {
        public:
            explicit MessageReceiver(CRCInterface& crc, OSLockInterface& osLock);

            // The universal reference TF is assuming a callback function in the format
            // [](Message& message) {}
            template <typename TF>
            void process(TF&& callback, const uint8_t byteReceived)
            {
                switch (m_state)
                {
                    case State::Idle:
                        inIdle(byteReceived);
                        break;

                    case State::ReceivingHeader:
                        inReceivingHeader(byteReceived);
                        break;

                    case State::ReceivingPayload:
                        inReceivingPayload(byteReceived);
                        break;

                    default:
                        break;
                }

                // This is transitory state that is entered in order to
                // issue the callback
                if (m_state == State::Processing)
                {
                    callback(m_message);
                    m_state = State::Idle;
                }
            }

            void reset();

        private:
            enum class State
            {
                Idle,
                ReceivingHeader,
                ReceivingPayload,
                // Transient state
                Processing
            };

            // States
            void inIdle(const uint8_t byteReceived);
            void inReceivingHeader(const uint8_t byteReceived);
            void inReceivingPayload(const uint8_t byteReceived);

            Message m_message;
            State m_state { State::Idle };
            int m_bytesToProcess { 0 };
        };
    }
}
