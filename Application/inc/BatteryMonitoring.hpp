#pragma once

#include "Application/inc/GpioDriverInterface.hpp"
#include "Application/inc/AD7415.hpp"

#include <functional>

namespace mercury
{
    namespace blackstar
    {
        class BatteryMonitoring final
        {
        public:
            struct StatusFlags
            {
                bool m_batteryStatusCharging : 1;
                bool m_batteryStatusFault : 1;
                uint8_t m_unused : 6;

                uint8_t raw() { return *(reinterpret_cast<uint8_t *>(this)); };

            } __attribute__((packed));

            BatteryMonitoring(AD7415TemperatureSensorDriver& driver,
                              GpioDriverInterface& gpioBatteryChargeStatus);
            ~BatteryMonitoring() = default;

            void monitor();
            StatusFlags getStatusFlags() const;
            Temperature getTemperature() const;

            template <typename TDisable>
            void update(TDisable&& disableChargingCallback)
            {
                updateTemperatureAndStatus();
                if (!isTemperatureInRange())
                {
                    disableChargingCallback();
                }
            }

        private:
            void updateTemperatureAndStatus();
            bool isTemperatureInRange() const;

            AD7415TemperatureSensorDriver& m_driver;
            GpioDriverInterface& m_gpioBatteryChargeStatus;

            // crude battery status monitoring properties
            uint8_t m_levelChangeCount { 0u };
            bool m_lastStat { false };

            Temperature m_temperature {};
            BatteryMonitoring::StatusFlags m_statusFlags { 0u };
        };
    }
}
