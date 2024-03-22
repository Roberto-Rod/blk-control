#include "Application/inc/ApplicationVersion.hpp"
#include "Application/inc/Bsp.hpp"
#include "Application/inc/MessageTask.hpp"
#include "Application/inc/Message.hpp"
#include "Application/inc/MessagePayloads.hpp"

#include <string>

extern uint32_t g_messageHighWatermark;

namespace
{
    // For further information, please see KT-957-0413-00 K-CEMA Serial Protocol ICD Rev A.1 Draft 9
    constexpr auto k_numberOfPatternsN { 1u };
    constexpr auto k_patternByte0 { 1u };
}

void messageTaskRun()
{
    mercury::blackstar::MessageTask messageTask;
    messageTask.run();
}

namespace mercury::blackstar
{
    constexpr bool isAcknowledgeExpected(MessageHeader::MessageStatus status)
    {
        return ((status == MessageHeader::MessageStatus::NewCommand) ||
                (status == MessageHeader::MessageStatus::Retransmit));
    }

    void sendMessage(osMessageQueueId_t txQueueHandle, Message& message)
    {
        // A message in the transmit queue should be 1 larger than the maximum content of
        // a Message i.e. 1 + k_headerSize + k_maxPayloadBytes + k_crcSize (CRC16)
        // The first byte will be used to put the actual length of data in this message
        const uint32_t queueMsgSize { osMessageQueueGetMsgSize(txQueueHandle) };

        uint8_t length { k_headerSize };
        if (message.m_header.m_payloadLength > 0)
        {
            length += (message.m_header.m_payloadLength + k_crcSize);
        }

        if (length < queueMsgSize)
        {
            // Allocate on the FreeRTOS Heap
            TxQueueMsg_t *pMem { reinterpret_cast<TxQueueMsg_t*>(pvPortMalloc(sizeof(TxQueueMsg_t))) };
            if (pMem)
            {
                pMem->length = length;
                // Add the header
                message.m_header.resetSerializeOut();
                for (uint8_t i = 0u; i < k_headerSize; ++i)
                {
                    pMem->buffer[i] = message.m_header.serializeOut();
                }
                // Add the message
                if (message.m_header.m_payloadLength > 0)
                {
                    message.resetSerializeOut();
                    for (int i = 0; i < (message.m_header.m_payloadLength + k_crcSize); ++i)
                    {
                        pMem->buffer[k_headerSize + i] = message.serializeOut();
                    }
                }
                // Send
                (void)osMessageQueuePut(txQueueHandle, pMem, 0u, osWaitForever);
                // Free the FreeRTOS task memory
                vPortFree(pMem);
            }
            else
            {
                debug_printf("Failed to allocate dynamic memory");
            }
        }
        else
        {
            debug_printf("Message queue size error");
        }
    }

    void MessageTask::run()
    {
        while (true)
        {
            if (uint32_t count { osMessageQueueGetCount(epuRxQueueHandle) }; count > 0u)
            {
                receivingMessage(epuRxQueueHandle, count);
            }
            else
            {
                for (auto i = 0; i < k_numberOfReceiveQueues; i++)
                {
                    if (m_queueStatus[i].m_firstMessageReceived)
                    {
                        if ((osKernelGetTickCount() - m_queueStatus[i].m_lastMessageTime) >= k_linkTimeOutTicks)
                        {
                            // Link timeout
                            auto& failedReceiver { m_epuReceiver };
                            failedReceiver.reset();
                            // No need to check again until a new message has been received
                            m_queueStatus[i].m_firstMessageReceived = false;
                        }
                    }
                }
            }
            (void)osThreadYield();
        }
    }

    void MessageTask::receivingEXTMessage(osMessageQueueId_t queue, uint32_t count)
    {
        static RxQueueMsg_t data;
        osMessageQueueGet(queue, &data, 0u, 0u);
        processEXTMessage(data);
    }

    void MessageTask::receivingMessage(osMessageQueueId_t queue, uint32_t count)
    {
#ifdef DEBUG
        g_messageHighWatermark = std::max(count, g_messageHighWatermark);
#endif
        static RxQueueMsg_t data;
        auto callback { [&](Message& message) { processMessage(message); } };
        auto& activeReceiver { m_epuReceiver };

        do
        {
            osStatus_t status { osMessageQueueGet(queue, &data, 0u, 0u) };
            if (status != osOK)
            {
                break;
            }
            for (uint8_t i = 0u; i < data.length; ++i)
            {
                activeReceiver.process(callback, data.buffer[i]);
            }
        } while (--count);
    }

    void MessageTask::processMessage(Message& message)
    {
        osMessageQueueId_t txQueueHandle { epuTxQueueHandle };
        uint8_t payloadLength { message.m_header.m_payloadLength };

        // Acknowledge
        const MessageHeader::MessageStatus messageStatus { message.m_header.m_messageStatus };
        if (isAcknowledgeExpected(messageStatus))
        {
            message.m_header.m_acknowledgeNumber = message.m_header.m_messageSequenceNumber;
            message.m_header.m_messageStatus = MessageHeader::MessageStatus::Acknowledge;
            message.m_header.m_payloadLength = 0;
            message.m_header.updateCrc();
            sendMessage(txQueueHandle, message);
        }

        // Then process
        if (messageStatus == MessageHeader::MessageStatus::NewCommand)
        {
            switch (message.m_header.m_messageId)
            {
				case MessageHeader::MessageId::GetSoftwareVersionNumber:
				{
					message.m_header.m_messageStatus = MessageHeader::MessageStatus::ResponseOk;
					message.m_header.m_payloadLength = sizeof(GetSoftwareVersionNumberPayload);
					auto pVersionPayload { reinterpret_cast<GetSoftwareVersionNumberPayload*>(message.m_payload.data()) };
					*pVersionPayload = {}; // initialise the GetSoftwareVersionNumberPayload structure to defaults
					pVersionPayload->m_major = k_applicationVersionMajor;
					pVersionPayload->m_minor = k_applicationVersionMinor;
					pVersionPayload->m_patch = k_applicationVersionPatch;
					pVersionPayload->m_buildID = 0; //k_buildID;
					message.updateCrc();
					sendMessage(txQueueHandle, message);
				}
				break;

                case MessageHeader::MessageId::GetKey:
                {
                	constexpr uint8_t k_keyLength { 32u };
                 	M41ST87W& tamper { Bsp::getInstance().getTamper() };
                   	// Read SRAM directly into message payload
                   	if (tamper.readSRAM(&message.m_payload[1], k_keyLength))
                   	{
						message.m_header.m_messageStatus = MessageHeader::MessageStatus::ResponseOk;
						message.m_header.m_payloadLength = k_keyLength + 1u;
						message.m_payload[0] = 0x01; // Payload version 1
						message.updateCrc();
						sendMessage(txQueueHandle, message);
                   	}
                }
                break;

                case MessageHeader::MessageId::SetKey:
                {
                	// Data starts at payload byte 1
                	if (payloadLength > 1u)
                	{
                		M41ST87W& tamper { Bsp::getInstance().getTamper() };
                		char *str { reinterpret_cast<char *>(&message.m_payload[1]) };
                		std::string key { str, payloadLength - 1u };
                		std::vector<uint8_t> data;
                	    auto bytes { reinterpret_cast<std::uint8_t *>(key.data()) };
                	    data.insert(data.end(), bytes, bytes + key.size());
                		tamper.resetSettings();
                		osDelay(10);
                		tamper.writeSRAM(data);
                	}
                }
                break;

                default:
                {

                }
                break;
            }
        }

        //Bsp::getInstance().setZeroiseHandler(ZeroiseHandlers::CSM);
        // Store last message time for all messages, this is all we do for ping...
        const auto index { k_epuQueueIndex };
        m_queueStatus[index] = { osKernelGetTickCount(), true };
    }

    void MessageTask::processEXTMessage(RxQueueMsg_t data)
    {
        constexpr uint8_t k_dataLength { 3 };
        constexpr uint8_t k_ASCII_A { 0x41 };
        constexpr uint8_t k_ASCII_C { 0x43 };
        constexpr uint8_t k_ASCII_R { 0x52 };
        constexpr uint8_t k_ASCII_U { 0x55 };
        constexpr std::array<uint8_t, 4> message { k_dataLength, k_ASCII_R, k_ASCII_C, k_ASCII_U };

        if (data.buffer[0] == k_ASCII_A)
        {
            (void)osMessageQueuePut(extTxQueueHandle, message.data(), 0u, osWaitForever);
        }
        (void)osMessageQueueReset(extRxQueueHandle);
    }
}
