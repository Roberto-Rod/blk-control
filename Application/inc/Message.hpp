#pragma once

#include "Application/inc/MessageHeader.hpp"

#include <cstdint>
#include <array>

namespace mercury
{
    namespace blackstar
    {
        class Message
        {
        public:
            Message(CRCInterface& crc, OSLockInterface& osLock);
            Message(CRCInterface& crc, OSLockInterface& osLock, MessageHeader::MessageId id);
            virtual ~Message() = default;

            void serializeIn(uint8_t source);
            void resetSerializeIn();

            uint8_t serializeOut();
            void resetSerializeOut();

            bool checkCrc();
            void updateCrc();

            CRCInterface& m_crcCalculator;
            OSLockInterface& m_osLock;
            MessageHeader m_header;

            static constexpr auto k_maxPayloadBytes { 142U };
            std::array<uint8_t, k_maxPayloadBytes> m_payload {};

        private:
            uint16_t m_crc { 0 };
            uint8_t m_serializeInCount { 0 };
            uint8_t m_serializeOutCount { 0 };

            uint32_t calcCrc();
        };
    }
}
