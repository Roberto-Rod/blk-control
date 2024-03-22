#pragma once

#include "Application/inc/Message.hpp"

#include <cstdint>

namespace mercury
{
    namespace blackstar
    {
        struct GetSoftwareVersionNumberPayload
        {
            uint8_t m_payloadFormatVersion { 0u };
            uint16_t m_major;
            uint16_t m_minor;
            uint16_t m_patch;
            uint32_t m_buildID;
        } __attribute__((packed));
        static_assert(sizeof(GetSoftwareVersionNumberPayload) < (Message::k_maxPayloadBytes - 2u)); // -2 for CRC
    }
}
