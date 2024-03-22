#pragma once

#include "stm32l0xx_hal.h"

#include "Application/inc/BspInterfacesImpl.hpp"

#include "Application/inc/BspInterfaces.hpp"
#include "Application/inc/GpioDriver.hpp"
#include "Application/inc/BatteryMonitoring.hpp"
#include "Application/inc/M41ST87W.hpp"

#include <atomic>
#include <memory>
#include <array>
#include <map>

namespace mercury
{
    namespace blackstar
    {
        enum class Gpio : uint8_t
        {
            // Outputs
            ZerPwrHold,
            DirectLedDrive,
            nBattChrgLow,
            nBattChrgEn,
            EpuPowerEn,
            PwrButtonEn,
            RstButtonEn,

            // Inputs
            nIrqTamper,
            BattChrgStat,
            PwrGood5V,
            PwrGood5V5,
            EpuOn,
            ExtShutdown,
            nPowerFault,

            // GPIO count
            NumberOfGpios
        };

        class Bsp
        {
        public:
            static Bsp& getInstance();

            void initialise();

            void disableBatteryCharging();
            void holdSTM32PowerOn();
            void releaseSTM32PowerHold();
            void directLedDrive(bool state);
            void setEPUPowerOn();
            void setEPUPowerOff();
            void holdEPUPowerButton();
            void releaseEPUPowerButton();
            bool externalShutdownRequested();
            bool EPUIsPowered();

            static BatteryMonitoring& getMonitor();
            static M41ST87W& getTamper();
            static OSLockInterface& getOSLockForCRC();
            static CRCInterface& getCRCCalculator();

        private:
            Bsp() = default;

            void setZerPwrHold(bool state);
            std::atomic<bool> m_adcConversionComplete;
            bool m_adcInitialised;
            uint32_t m_adcVref_mV;
        };
    }
}
