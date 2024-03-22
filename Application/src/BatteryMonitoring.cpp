#include "../inc/BatteryMonitoring.hpp"

namespace mercury::blackstar
{
    BatteryMonitoring::BatteryMonitoring(AD7415TemperatureSensorDriver& driver,
                                         GpioDriverInterface& gpioBatteryChargeStatus)
        : m_driver { driver },
          m_gpioBatteryChargeStatus { gpioBatteryChargeStatus }
    {
    }

    void BatteryMonitoring::monitor()
    {
        const bool stat { m_gpioBatteryChargeStatus.read() };
        if (m_lastStat != stat)
        {
            m_levelChangeCount++;
            m_lastStat = stat;
        }
    }

    BatteryMonitoring::StatusFlags BatteryMonitoring::getStatusFlags() const
    {
        return m_statusFlags;
    }

    Temperature BatteryMonitoring::getTemperature() const
    {
        return m_temperature;
    }

    void BatteryMonitoring::updateTemperatureAndStatus()
    {
        StatusFlags batteryStatusFlags { 0u };
        Temperature current { m_driver.Read() };

        // Check on new period, failure is a 2Hz signal
        // there should be 3 level changes per 825ms
        if (m_levelChangeCount >= 3u)
        {
           batteryStatusFlags.m_batteryStatusFault = true;
        }
        else if (m_lastStat == false) // Active LOW
        {
           batteryStatusFlags.m_batteryStatusCharging = true;
        }
        // then reset
        m_levelChangeCount = 0u;

        m_temperature = current;
        m_statusFlags = batteryStatusFlags;
    }

    bool BatteryMonitoring::isTemperatureInRange() const
    {
        if (m_temperature.m_isValid)
        {
            constexpr auto k_batteryTemperatureRangeLow_degC { -40 * 4 }; // -40 in resolution 0.25 °C
            constexpr auto k_batteryTemperatureRangeHigh_degC { 85 * 4 }; // 85 in resolution 0.25 °C

            return ((k_batteryTemperatureRangeLow_degC <= m_temperature.m_value) && (m_temperature.m_value <= k_batteryTemperatureRangeHigh_degC));
        }
        else
        {
            return false;
        }
    }
}
