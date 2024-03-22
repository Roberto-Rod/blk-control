#include "Application/inc/Message.hpp"

#include <mutex>

namespace mercury::blackstar
{
    Message::Message(CRCInterface& crc, OSLockInterface& osLock)
        : m_crcCalculator { crc },
          m_osLock { osLock },
          m_header { crc, osLock }
    {
    }

    Message::Message(CRCInterface& crc, OSLockInterface& osLock, MessageHeader::MessageId id)
        : Message(crc, osLock)
    {
        m_header.m_messageId = id;
    }

    void Message::serializeIn(uint8_t source)
    {
        if (m_serializeInCount == m_header.m_payloadLength)
        {
            // Payload CRC LSB in penultimate byte
            m_crc = static_cast<uint16_t>(source);
        }
        else if (m_serializeInCount == m_header.m_payloadLength + 1)
        {
            // Payload CRC MSB in ultimate byte
            m_crc |= static_cast<uint16_t>(source) << 8;
        }
        else
        {
            // Payload
            m_payload[m_serializeInCount] = source;
        }

        // Allow count to go beyond payload to accommodate 2 CRC bytes
        if (m_serializeInCount < k_maxPayloadBytes + 1u)
        {
            ++m_serializeInCount;
        }
    }

    void Message::resetSerializeIn()
    {
        m_serializeInCount = 0;
    }

    uint8_t Message::serializeOut()
    {
        uint8_t value(0);

        // Allow count to go beyond payload to accommodate 2 CRC bytes
        if (m_serializeOutCount < k_maxPayloadBytes + 1)
        {
            if (m_serializeOutCount == m_header.m_payloadLength)
            {
                // Payload CRC LSB in penultimate byte
                value = static_cast<uint8_t>(m_crc);
            }
            else if (m_serializeOutCount == m_header.m_payloadLength + 1)
            {
                // Payload CRC MSB in ultimate byte
                value = static_cast<uint8_t>(m_crc >> 8);
            }
            else
            {
                // Payload
                value = m_payload[m_serializeOutCount];
            }
            ++m_serializeOutCount;
        }

        return value;
    }

    void Message::resetSerializeOut()
    {
        m_serializeOutCount = 0;
    }

    bool Message::checkCrc()
    {
        return calcCrc() == m_crc;
    }

    void Message::updateCrc()
    {
        m_header.updateCrc();
        m_crc = calcCrc();
    }

    uint32_t Message::calcCrc()
    {
        std::lock_guard<OSLockInterface> lock(m_osLock);
        m_crcCalculator.reset();

        uint32_t crc { m_crcCalculator.accumulate(reinterpret_cast<uint32_t*>(&m_payload), m_header.m_payloadLength) };
        return static_cast<uint16_t>(crc);
    }
}
