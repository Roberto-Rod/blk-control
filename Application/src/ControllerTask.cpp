#include "Application/inc/ControllerTask.hpp"

#include "Core/Inc/main.h"
#include "cmsis_os.h"

void controllerTaskRun()
{
    mercury::blackstar::ControllerTask controllerTask;
    controllerTask.run();
}

namespace mercury::blackstar
{
    void ControllerTask::run()
    {
    	constexpr TickType_t k_powerOffWaitTicks { 10000u / portTICK_RATE_MS }; // Wait 10,000 ms between asserting power button and removing power
    	constexpr TickType_t k_STM32ReleaseWaitTicks { 500u / portTICK_RATE_MS }; // Wait 500 ms between asserting EPU power off and releasing STM32 power hold
    	constexpr TickType_t k_EPUPowerTimeoutTicks { 10000u / portTICK_RATE_MS }; // Allow 10,000 ms for EPU power rail to start after power applied
    	constexpr TickType_t k_InitialWaitTicks { 1000u / portTICK_RATE_MS }; // Wait 1,000 ms after moving to initial state before controlling EPU power
        constexpr TickType_t k_powerStatePeriodTicks { 250u / portTICK_RATE_MS }; // Monitor power state every 250 ms
        constexpr TickType_t k_batteryMonitorPeriodTicks { 50u / portTICK_RATE_MS }; // Monitor battery every 50 ms
        constexpr uint8_t k_batteryTemperaturePeriod { 20u }; // Update battery temperature every 20 x monitor battery period = 1 s

        enum class PowerState
        {
            Initial,
            PowerOffRequested,
            PowerOffEPU,
			PowerOffReleaseSTM32,
            PowerOnRequested,
			PowerOn
        };

        TickType_t lastBatteryUpdateTicks { 0u };
        TickType_t lastPowerStateUpdateTicks { 0u };
        TickType_t powerButtonPressedTicks { 0u };
        TickType_t powerOffInitiatedTicks { 0u };
        TickType_t powerOnInitiatedTicks { 0u };
        TickType_t initialTicks { xTaskGetTickCount() };
        uint8_t batteryPeriodCount { 0u };
        Bsp& bsp { Bsp::getInstance() };
        BatteryMonitoring& battery { Bsp::getMonitor() };
        PowerState powerState { PowerState::Initial };

        // Hold STM32 power on when the controller starts so that this controller can disable power
        // after performing a clean shutdown
        bsp.holdSTM32PowerOn();

        while(true)
        {
            TickType_t ticksNow { xTaskGetTickCount() };

            if ((ticksNow - lastPowerStateUpdateTicks) >= k_powerStatePeriodTicks)
            {
            	lastPowerStateUpdateTicks = ticksNow;

				switch (powerState)
				{
				case PowerState::Initial:
					// In the initial state, wait for power-up delay and then
					// work out which power state we are supposed to be in
					if ((ticksNow - initialTicks) >= k_InitialWaitTicks)
					{
						if (bsp.externalShutdownRequested())
						{
							// If we have just started then power should already be off, move straight
							// to power off state which will ensure that power is off
							powerState = PowerState::PowerOffEPU;
							powerOffInitiatedTicks = ticksNow;
						}
						else
						{
							powerState = PowerState::PowerOnRequested;
							powerOnInitiatedTicks = ticksNow;
						}
					}
					break;

				case PowerState::PowerOffRequested:
					// When power off has been requested release power button and wait before removing power
					bsp.releaseEPUPowerButton();
					if ((ticksNow - powerButtonPressedTicks) >= k_powerOffWaitTicks)
					{
						powerState = PowerState::PowerOffEPU;
						powerOffInitiatedTicks = ticksNow;
					}
					break;

				case PowerState::PowerOffEPU:
					// Remove EPU power
					bsp.setEPUPowerOff();
					if ((ticksNow - powerOffInitiatedTicks) >= k_STM32ReleaseWaitTicks)
					{
						powerState = PowerState::PowerOffReleaseSTM32;
					}
					break;

				case PowerState::PowerOffReleaseSTM32:
					// Remove STM32 power hold
					bsp.releaseSTM32PowerHold();
					initialTicks = ticksNow;
					powerState = PowerState::Initial;
					break;

				case PowerState::PowerOnRequested:
					// Apply power
					bsp.setEPUPowerOn();
					if (bsp.EPUIsPowered())
					{
						powerState = PowerState::PowerOn;
					}
					else if ((ticksNow - powerOnInitiatedTicks) >= k_EPUPowerTimeoutTicks)
					{
						powerState = PowerState::PowerOffEPU;
						powerOffInitiatedTicks = ticksNow;
					}
					break;

				case PowerState::PowerOn:
					// Respond to an external shutdown request
					if (bsp.externalShutdownRequested())
					{
						// Assert the power button, this will be held for one power state update period
						bsp.holdEPUPowerButton();
						powerButtonPressedTicks = ticksNow;
						powerState = PowerState::PowerOffRequested;
					}
					// If EPU has powered off then move to off state
					if (!bsp.EPUIsPowered())
					{
						powerState = PowerState::PowerOffEPU;
						powerOffInitiatedTicks = ticksNow;
					}
					break;
				}
            }

            // Check battery temperature, etc
            if ((ticksNow - lastBatteryUpdateTicks) >= k_batteryMonitorPeriodTicks)
            {
                lastBatteryUpdateTicks = ticksNow;
                battery.monitor();

                // Update battery temperature every N periods
                batteryPeriodCount++;
                if (batteryPeriodCount >= k_batteryTemperaturePeriod)
                {
                    batteryPeriodCount = 0u;
                    battery.update([&](){ bsp.disableBatteryCharging(); });
                }
            }

            osDelay(1u);
        }
    }
}

