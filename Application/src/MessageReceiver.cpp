#include "Application/inc/MessageReceiver.hpp"

namespace mercury::blackstar
{
    MessageReceiver::MessageReceiver(CRCInterface& crc, OSLockInterface& osLock)
        : m_message { crc, osLock }
    {
    }

    void MessageReceiver::reset()
    {
        m_state = State::Idle;
    }

    void MessageReceiver::inIdle(const uint8_t byteReceived)
    {
        // Looking for a message header, so just check the begin is start of frame
        if (byteReceived == k_startOfFrame)
        {
            m_message.m_header.resetSerializeIn();
            m_message.m_header.serializeIn(byteReceived);
            m_bytesToProcess = k_headerSize - 1;
            m_state = State::ReceivingHeader;
        }
    }

    void MessageReceiver::inReceivingHeader(const uint8_t byteReceived)
    {
        m_message.m_header.serializeIn(byteReceived);
        m_bytesToProcess--;
        if (m_bytesToProcess == 0)
        {
            // Read the whole header, so check the CRC
            if (m_message.m_header.checkCrc())
            {
                // Check the header structure - just get the payload length for now and add CRC length
                if (m_message.m_header.m_payloadLength > 0)
                {
                    m_message.resetSerializeIn();

                    m_bytesToProcess = m_message.m_header.m_payloadLength + k_crcSize;
                    m_state = State::ReceivingPayload;
                }
                else
                {
                    m_state = State::Processing;
                }
            }
            else
            {
                m_state = State::Idle;
            }
        }
    }

    void MessageReceiver::inReceivingPayload(const uint8_t byteReceived)
    {
        m_message.serializeIn(byteReceived);
        m_bytesToProcess--;
        if (m_bytesToProcess == 0)
        {
            // Read the whole payload & CRC
            if (m_message.checkCrc())
            {
                m_state = State::Processing;
            }
            else
            {
                m_state = State::Idle;
            }
        }
    }
}
