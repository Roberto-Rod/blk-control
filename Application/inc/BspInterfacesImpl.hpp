#pragma once

#include "Application/inc/BspInterfaces.hpp"

namespace mercury
{
    namespace blackstar
    {
        class BspI2CTransmit : public I2CTransmitInterface
        {
        public:
            BspI2CTransmit() = default;
            ~BspI2CTransmit() = default;

            bool transmit(uint16_t devAddress, uint8_t *pData, uint16_t size, uint32_t timeout_ms) override;
        };

        class BspI2CReceive : public I2CReceiveInterface
        {
        public:
            BspI2CReceive() = default;
            ~BspI2CReceive() = default;

            bool receive(uint16_t devAddress, uint8_t *pData, uint16_t size, uint32_t timeout_ms) override;
        };

        class BspI2COSLock : public OSLockInterface
        {
        public:
            BspI2COSLock() = default;
            ~BspI2COSLock() = default;

            void lock() override;
            void unlock() override;
        };

        class BspOSDelay : public OSDelayInterface
        {
        public:
            BspOSDelay() = default;
            ~BspOSDelay() = default;

            void delay(const uint32_t delay_ms) override;
        };

        class BspCRCOSLock : public OSLockInterface
        {
        public:
            BspCRCOSLock() = default;
            ~BspCRCOSLock() = default;

            void lock() override;
            void unlock() override;
        };

        class BspCRC : public CRCInterface
        {
        public:
            BspCRC() = default;
            ~BspCRC() = default;

            void reset() override;
            uint32_t accumulate(uint32_t *pBuffer, uint32_t bufferLength) override;
        };
    }
}
