#pragma once

#include "Application/inc/BspInterfaces.hpp"

#include <cstdint>

namespace mercury
{
    namespace blackstar
    {
        constexpr uint8_t k_startOfFrame { 0xA5u };
        constexpr uint8_t k_headerSize { 8u };
        constexpr uint8_t k_crcSize { sizeof(uint16_t) };

        class MessageHeader
        {
        public:
            MessageHeader(CRCInterface& crc, OSLockInterface& osLock);
            virtual ~MessageHeader() = default;

            enum class MessageStatus : uint8_t
            {
                NewCommand = 0u,
                Retransmit,
                Acknowledge,
                NotAcknowledge,
                ResponseOk,
                ResponseNotOk
            };

            // Message ID definitions see Table 3
            // KT-957-0413-00 K-CEMA Serial Protocol ICD
            enum class MessageId : uint8_t
            {
                Ping                        = 0x00u,
                GetSoftwareVersionNumber    = 0x01u,
                SetLedPattern               = 0x02u,
                SetBuzzerPattern            = 0x03u,
                Synchronize                 = 0x04u,
                ButtonStatus                = 0x05u,
                Zeroize                     = 0x06u,
                GetSlotNumber               = 0x07u,
                GetHardwareInformation      = 0x08u,
                ZeroiseStatus               = 0x09u,
                GetBuiltInTestInformation   = 0x0Au,
                GetUnitInformation          = 0x0Bu,
                SetUnitInformation          = 0x0Cu,
                SetLedBrightness            = 0x0Du,
                GetDynamicBatteryParameters = 0x80u,
                GetStaticBatteryParameters  = 0x81u,
                // Additional BlackStar messages
                SetKey                      = 0xFEu,
                GetKey                      = 0xFFu
            };

            void serializeIn(uint8_t source);
            void resetSerializeIn();

            uint8_t serializeOut();
            void resetSerializeOut();

            bool checkCrc();
            void updateCrc();

            CRCInterface& m_crcCalculator;
            OSLockInterface& m_osLock;

            uint8_t m_startOfFrame { k_startOfFrame };
            uint8_t m_messageSequenceNumber { 0u };
            uint8_t m_acknowledgeNumber { 0u };
            MessageStatus m_messageStatus { MessageStatus::NewCommand };
            uint8_t m_protocolVersion { 0u };
            MessageId m_messageId { MessageId::Ping };
            uint8_t m_payloadLength { 0u };
            uint16_t m_headerCrc { 0u };
            uint8_t m_serializeInCount { 0u };
            uint8_t m_serializeOutCount { 0u };

            static inline uint8_t s_lastSequenceNumberSent { 0u };

        private:
            uint32_t calcCrc();
        };
    }
}
