#include "Application/inc/M41ST87W.hpp"

#include <array>
#include <algorithm>
#include <vector>
#include <mutex>

namespace mercury::blackstar
{
	M41ST87W::M41ST87W(const uint8_t slaveAddress,
                       I2CTransmitInterface& transmit,
                       I2CReceiveInterface& receive,
					   OSLockInterface& lock)
        : m_slaveAddress { slaveAddress },
          m_transmit { transmit },
          m_receive { receive },
		  m_osLock { lock }
    {
    }

    bool M41ST87W::writeSRAM(std::vector<uint8_t>& data)
    {
    	bool ok { false };
    	size_t numberOfBytes { data.size() };
    	if (numberOfBytes <= k_SRAMSizeBytes)
    	{
    		std::lock_guard<OSLockInterface> lock(m_osLock);
			std::array<uint8_t, k_SRAMSizeBytes + 1> buffer;
			buffer[0] = k_addrSRAMBase;
			std::copy_n(data.begin(), numberOfBytes, buffer.begin() + 1);
			ok = m_transmit.transmit(m_slaveAddress, buffer.data(), numberOfBytes + 1, k_i2cTimeout_ms);
    	}
        return ok;
    }

    bool M41ST87W::readSRAM(uint8_t *buffer, uint8_t length)
    {
        bool ok { false };
        if (length <= k_SRAMSizeBytes)
        {
			std::lock_guard<OSLockInterface> lock { m_osLock };
			uint8_t registerAddress { k_addrSRAMBase };
			if (m_transmit.transmit(m_slaveAddress, &registerAddress, sizeof(registerAddress), k_i2cTimeout_ms))
			{
				ok = m_receive.receive(m_slaveAddress, buffer, length, k_i2cTimeout_ms);
			}
        }
        return ok;
    }

    bool M41ST87W::resetSettings()
    {
        // Bit 7: TEB (Tamper Enable)
        // Bit 6: TIE (Tamper Interrupt Enable)
        // Bit 5: TCM (Tamper Connect Mode)
        // Bit 4: TPM (Tamper Polarity Mode)
        // Bit 3: TDS (Tamper Detect Sampling)
        // Bit 2: TCHI (Tamper Current High)
        // Bit 1: CLR1EXT (RAM Clear External Bus)
        // Bit 0: CLR1 (RAM Clear)
        static const uint8_t k_channel1Settings { 0x95 }; // N.C.: TCM=0, TPM=1, TCHI=1
        static const uint8_t k_channel2Settings { 0xA5 }; // N.O.: TCM=1, TPM=0, TCHI=1

        bool ok { true };
        std::lock_guard<OSLockInterface> lock { m_osLock };

        uint8_t flagsCommand[2] { k_addrFlags, 0 };
        uint8_t channel1Command[2] { k_addrChannel1Settings, 0 };
        uint8_t channel2Command[2] { k_addrChannel2Settings, 0 };
        ok = m_transmit.transmit(m_slaveAddress, &channel1Command[0], sizeof(channel1Command), k_i2cTimeout_ms) && ok;
        ok = m_transmit.transmit(m_slaveAddress, &channel2Command[0], sizeof(channel2Command), k_i2cTimeout_ms) && ok;
        ok = m_transmit.transmit(m_slaveAddress, &flagsCommand[0], sizeof(flagsCommand), k_i2cTimeout_ms) && ok;
        channel1Command[1] = k_channel1Settings;
        channel2Command[1] = k_channel2Settings;
        ok = m_transmit.transmit(m_slaveAddress, &channel1Command[0], sizeof(channel1Command), k_i2cTimeout_ms) && ok;
        ok = m_transmit.transmit(m_slaveAddress, &channel2Command[0], sizeof(channel2Command), k_i2cTimeout_ms) && ok;
        return ok;
    }
}
