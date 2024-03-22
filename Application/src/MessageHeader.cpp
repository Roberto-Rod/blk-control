#include "Application/inc/MessageHeader.hpp"

#include <mutex>

namespace
{
    constexpr uint8_t k_startOfFrameIndex             { 0u };
    constexpr uint8_t k_messageSequenceNumberIndex    { 1u };
    constexpr uint8_t k_acknowledgeNumberIndex        { 2u };
    constexpr uint8_t k_statusAndProtocolVersionIndex { 3u };
    constexpr uint8_t k_messageIdIndex                { 4u };
    constexpr uint8_t k_payloadLengthIndex            { 5u };
    constexpr uint8_t k_headerCrcLSBIndex             { 6u };
    constexpr uint8_t k_headerCrcMSBIndex             { 7u };

    constexpr uint8_t k_messageStatusShift            { 5u };
    constexpr uint8_t k_messageStatusMask             { 0x7u };
    constexpr uint8_t k_protocolVersionStatusMask     { 0x1Fu };

    constexpr uint8_t toStatusProtocol(uint8_t status, uint8_t protocol)
    {
        return ((status & k_messageStatusMask) << k_messageStatusShift) | (protocol & k_protocolVersionStatusMask);
    }
}

namespace mercury::blackstar
{
    MessageHeader::MessageHeader(CRCInterface& crc, OSLockInterface& osLock)
        : m_crcCalculator { crc },
          m_osLock { osLock }
    {
    }

    void MessageHeader::serializeIn(uint8_t source)
    {
        switch (m_serializeInCount++)
        {
            case k_startOfFrameIndex:
                m_startOfFrame = source;
                break;

            case k_messageSequenceNumberIndex:
                m_messageSequenceNumber = source;
                break;

            case k_acknowledgeNumberIndex:
                m_acknowledgeNumber = source;
                break;

            case k_statusAndProtocolVersionIndex:
                m_messageStatus = static_cast<MessageStatus>((source >> k_messageStatusShift) & k_messageStatusMask);
                m_protocolVersion = (source & k_protocolVersionStatusMask);
                break;

            case k_messageIdIndex:
                m_messageId = static_cast<MessageId>(source);
                break;

            case k_payloadLengthIndex:
                m_payloadLength = source;
                break;

            case k_headerCrcLSBIndex:
                m_headerCrc = static_cast<uint16_t>(source);
                break;

            case k_headerCrcMSBIndex:
                m_headerCrc |= static_cast<uint16_t>(source) << 8;
                break;

            default:
                break;
        }
    }

    void MessageHeader::resetSerializeIn()
    {
        m_serializeInCount = 0u;
    }

    uint8_t MessageHeader::serializeOut()
    {
        uint8_t value { 0u };

        if (m_serializeOutCount < 8u)
        {
            switch (m_serializeOutCount++)
            {
                case k_startOfFrameIndex:
                    value = m_startOfFrame;
                    break;

                case k_messageSequenceNumberIndex:
                    value = m_messageSequenceNumber;
                    break;

                case k_acknowledgeNumberIndex:
                    value = m_acknowledgeNumber;
                    break;

                case k_statusAndProtocolVersionIndex:
                    value = toStatusProtocol(static_cast<uint8_t>(m_messageStatus), m_protocolVersion);
                    break;

                case k_messageIdIndex:
                    value = static_cast<uint8_t>(m_messageId);
                    break;

                case k_payloadLengthIndex:
                    value = m_payloadLength;
                    break;

                case k_headerCrcLSBIndex:
                    value = static_cast<uint8_t>(m_headerCrc);
                    break;

                case k_headerCrcMSBIndex:
                    value = static_cast<uint8_t>(m_headerCrc >> 8);
                    break;

                default:
                    break;
            }
        }

        return value;
    }

    void MessageHeader::resetSerializeOut()
    {
        m_serializeOutCount = 0u;
    }

    bool MessageHeader::checkCrc()
    {
        return calcCrc() == m_headerCrc;
    }

    void MessageHeader::updateCrc()
    {
        m_headerCrc = calcCrc();
    }

    uint32_t MessageHeader::calcCrc()
    {
        std::lock_guard<OSLockInterface> lock(m_osLock);
        m_crcCalculator.reset();

        // Serialize out
        const uint8_t statusProtocol { toStatusProtocol(static_cast<uint8_t>(m_messageStatus), m_protocolVersion) };
        uint8_t serialized[6] { m_startOfFrame, m_messageSequenceNumber, m_acknowledgeNumber,
                                statusProtocol, static_cast<uint8_t>(m_messageId), m_payloadLength };

        uint32_t crc { m_crcCalculator.accumulate(reinterpret_cast<uint32_t*>(&serialized[0]), sizeof(serialized)) };
        return static_cast<uint16_t>(crc);
    }
}
