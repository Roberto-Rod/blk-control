#include "Application/inc/AD7415.hpp"

#include <array>
#include <mutex>

namespace mercury::blackstar
{
    AD7415TemperatureSensorDriver::AD7415TemperatureSensorDriver(const uint8_t slaveAddress,
                                                                 I2CTransmitInterface& transmit,
                                                                 I2CReceiveInterface& receive,
																 OSLockInterface& lock)
        : m_slaveAddress { slaveAddress },
          m_transmit { transmit },
          m_receive { receive },
		  m_osLock { lock }
    {
    }

    Temperature AD7415TemperatureSensorDriver::Read()
    {
        constexpr uint8_t k_temperatureValueRegisterAddress { 0x00u };
        constexpr uint8_t k_configurationRegisterAddress { 0x01u };
        constexpr uint16_t k_temperatureValueReadLength { 2u };
        constexpr uint16_t k_configurationWriteLength { 2u };
        constexpr uint16_t k_addressRegisterWriteLength { 1u };
        constexpr uint32_t k_i2cTimeout_ms { 100u };
        // One-shot value: retain bit 6 (FLTR) = 1 (power-up default)
        //                 set bit 2 (ONE SHOT) = 1 (not stored)
        constexpr uint8_t k_oneshotValue { 0x44u };

        std::lock_guard<OSLockInterface> lock { m_osLock };

        Temperature temperature {};

        // Write the one-shot value to the configuration address
        std::array<uint8_t, k_configurationWriteLength> wrBuf { k_configurationRegisterAddress, k_oneshotValue };
        temperature.m_isValid = m_transmit.transmit(m_slaveAddress, wrBuf.data(), k_configurationWriteLength, k_i2cTimeout_ms);

        if (temperature.m_isValid)
        {
        	// Write 0x00 to the Address Pointer Register
        	wrBuf[0] = k_temperatureValueRegisterAddress;
        	temperature.m_isValid = m_transmit.transmit(m_slaveAddress, wrBuf.data(), k_addressRegisterWriteLength, k_i2cTimeout_ms);
        }

        if (temperature.m_isValid)
        {
            // Read the register
            std::array<uint8_t, k_temperatureValueReadLength> buf {};
            temperature.m_isValid = m_receive.receive(m_slaveAddress, buf.data(), k_temperatureValueReadLength, k_i2cTimeout_ms);

            if (temperature.m_isValid)
            {
                // Convert 8-bit buffer to 16-bit value and shift temperature data
                // bits to the correct position.
                uint16_t temp { static_cast<uint16_t>((buf[0] << 8u) & 0xFF00u) };
                temp |= static_cast<uint8_t>(buf[1]);
                temp >>= 6;

                // Handle positive/negative temperatures
                if (temp >= 512u)
                {
                    // Negative temperature
                    temperature.m_value = (static_cast<int16_t>(temp) - 1024u);
                }
                else
                {
                    // Positive temperature
                    temperature.m_value = static_cast<int16_t>(temp);
                }
            }
        }

        return temperature;
    }
}
