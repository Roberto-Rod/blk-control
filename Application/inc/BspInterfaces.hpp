#pragma once

#include <cstdint>

namespace mercury
{
    namespace blackstar
    {
        class I2CTransmitInterface
        {
        public:
            virtual ~I2CTransmitInterface() = default;

            virtual bool transmit(uint16_t devAddress, uint8_t *pData, uint16_t size, uint32_t timeout_ms = UINT32_MAX) = 0;

        protected:
            I2CTransmitInterface() = default;
        };

        class I2CReceiveInterface
        {
        public:
            virtual ~I2CReceiveInterface() = default;

            virtual bool receive(uint16_t devAddress, uint8_t *pData, uint16_t size, uint32_t timeout_ms = UINT32_MAX) = 0;

        protected:
            I2CReceiveInterface() = default;
        };

        class OSLockInterface
        {
        public:
            virtual ~OSLockInterface() = default;

            virtual void lock() = 0;
            virtual void unlock() = 0;

        protected:
            OSLockInterface() = default;
        };

        class OSDelayInterface
        {
        public:
            virtual ~OSDelayInterface() = default;

            virtual void delay(const uint32_t delay_ms) = 0;

        protected:
            OSDelayInterface() = default;
        };

        class CRCInterface
        {
        public:
            virtual ~CRCInterface() = default;

            virtual void reset() = 0;
            virtual uint32_t accumulate(uint32_t *pBuffer, uint32_t bufferLength) = 0;

        protected:
            CRCInterface() = default;
        };
    }
}
