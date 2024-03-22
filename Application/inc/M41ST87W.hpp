#pragma once

#include "Application/inc/BspInterfaces.hpp"

#include <vector>

namespace mercury
{
    namespace blackstar
    {
        class M41ST87W final
        {
        public:
        	M41ST87W(const uint8_t slaveAddress,
                                   I2CTransmitInterface& transmit,
                                   I2CReceiveInterface& receive,
								   OSLockInterface& lock);
            ~M41ST87W() = default;

            bool writeSRAM(std::vector<uint8_t>& data);
            bool readSRAM(uint8_t *buffer, uint8_t length);
            bool resetSettings();

        private:
            static const uint32_t k_i2cTimeout_ms { 200u };
            static const uint8_t k_addrFlags { 0x0F };
            static const uint8_t k_addrChannel1Settings { 0x14 };
            static const uint8_t k_addrChannel2Settings { 0x15 };
            static const uint8_t k_addrSRAMBase { 0x20 };
            static const uint8_t k_SRAMSizeBytes { 128u };

            uint8_t m_slaveAddress;
            I2CTransmitInterface& m_transmit;
            I2CReceiveInterface& m_receive;
            OSLockInterface& m_osLock;
        };
    }
}
