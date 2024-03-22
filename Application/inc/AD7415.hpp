#pragma once

#include "Application/inc/BspInterfaces.hpp"

namespace mercury
{
    namespace blackstar
    {
        struct Temperature final
        {
            int16_t m_value;
            bool m_isValid;
        };

        class AD7415TemperatureSensorDriver final
        {
        public:
            AD7415TemperatureSensorDriver(const uint8_t slaveAddress,
                                          I2CTransmitInterface& transmit,
                                          I2CReceiveInterface& receive,
										  OSLockInterface& lock);
            ~AD7415TemperatureSensorDriver() = default;

            Temperature Read();

        private:
            uint8_t m_slaveAddress;
            I2CTransmitInterface& m_transmit;
            I2CReceiveInterface& m_receive;
            OSLockInterface& m_osLock;
        };
    }
}
