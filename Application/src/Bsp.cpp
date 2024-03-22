#include "Application/inc/Bsp.hpp"
#include "Core/Inc/main.h"
#include "cmsis_os.h"

#include <tuple>

extern I2C_HandleTypeDef hi2c1;
// Mutex protects the access to the data memory transmitted over the I2C interface
extern osMutexId i2cMutexHandle;

extern CRC_HandleTypeDef hcrc;
extern osMutexId crcMutexHandle;

namespace
{
    using namespace mercury::blackstar;

    constexpr bool k_high { true };
    constexpr bool k_low { false };

    const std::map<Gpio, std::pair<GPIO_TypeDef*, uint16_t>>
        k_gpioPortPinMap { // Outputs
                           { Gpio::ZerPwrHold,        std::make_pair(ZER_PWR_HOLD_GPIO_Port, ZER_PWR_HOLD_Pin) },
                           { Gpio::DirectLedDrive,    std::make_pair(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin) },
                           { Gpio::nBattChrgLow,      std::make_pair(BATT_CHRG_LOW_GPIO_Port, BATT_CHRG_LOW_Pin) },
                           { Gpio::nBattChrgEn,       std::make_pair(BATT_CHRG_ENn_GPIO_Port, BATT_CHRG_ENn_Pin) },
                           { Gpio::EpuPowerEn,        std::make_pair(EPU_PWR_EN_GPIO_Port, EPU_PWR_EN_Pin) },
                           { Gpio::PwrButtonEn,       std::make_pair(PWR_BTN_ON_GPIO_Port, PWR_BTN_ON_Pin) },
                           { Gpio::RstButtonEn,       std::make_pair(RST_BTN_ON_GPIO_Port, RST_BTN_ON_Pin) },

                           // Inputs
                           { Gpio::nIrqTamper,        std::make_pair(IRQ_TAMPERn_GPIO_Port, IRQ_TAMPERn_Pin) },
                           { Gpio::BattChrgStat,      std::make_pair(BATT_CHRG_STAT_GPIO_Port, BATT_CHRG_STAT_Pin) },
                           { Gpio::PwrGood5V,         std::make_pair(PGOOD__5V_ZER_GPIO_Port, PGOOD__5V_ZER_Pin) },
                           { Gpio::PwrGood5V5,        std::make_pair(PGOOD__5V5_GPIO_Port, PGOOD__5V5_Pin) },
                           { Gpio::EpuOn,             std::make_pair(EPU_ON_GPIO_Port, EPU_ON_Pin) },
                           { Gpio::ExtShutdown,       std::make_pair(EXT_SHDN_GPIO_Port, EXT_SHDN_Pin) },
                           { Gpio::nPowerFault,       std::make_pair(PWR_FAULTn_GPIO_Port, PWR_FAULTn_Pin) } };

    BspI2CTransmit s_i2cTransmit {};
    BspI2CReceive s_i2cReceive {};
    BspI2COSLock s_i2cOsLock {};
    BspOSDelay s_osDelay {};

    constexpr uint8_t k_temperatureSensorSlaveAddress { 0x4Du };
    constexpr uint8_t k_tamperSlaveAddress { 0x68u };
    AD7415TemperatureSensorDriver s_tempSensorDriver { k_temperatureSensorSlaveAddress, s_i2cTransmit, s_i2cReceive, s_i2cOsLock };

    inline void setGpioState(Gpio gpio, bool state)
    {
        const auto& [port, pin] { k_gpioPortPinMap.at(gpio) };
        HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    inline bool getGpioState(Gpio gpio)
    {
        const auto& [port, pin] { k_gpioPortPinMap.at(gpio) };
        return HAL_GPIO_ReadPin(port, pin);
    }

    inline void gpioOff(Gpio gpio)
    {
        const auto& [port, pin] { k_gpioPortPinMap.at(gpio) };
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }

    inline void gpioOn(Gpio gpio)
    {
        const auto& [port, pin] { k_gpioPortPinMap.at(gpio) };
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    }

    GpioDriver s_gpioBatteryChargeStatus { [](){ return getGpioState(Gpio::BattChrgStat); },
                                           [](const bool state){} }; // read only
}

namespace mercury::blackstar
{
    // Bsp
    Bsp& Bsp::getInstance()
    {
        static Bsp bsp;
        return bsp;
    }

    void Bsp::initialise()
    {
    }

    void Bsp::disableBatteryCharging()
    {
        // OFF (Active Low)
        gpioOn(Gpio::nBattChrgEn);
    }

    BatteryMonitoring& Bsp::getMonitor()
    {
        static BatteryMonitoring s_battery { s_tempSensorDriver, s_gpioBatteryChargeStatus };
        return s_battery;
    }

    M41ST87W& Bsp::getTamper()
    {
    	static M41ST87W s_tamper { k_tamperSlaveAddress, s_i2cTransmit, s_i2cReceive, s_i2cOsLock };
    	return s_tamper;
    }

    OSLockInterface& Bsp::getOSLockForCRC()
    {
        static BspCRCOSLock s_crcOSLock {};
        return s_crcOSLock;
    }

    CRCInterface& Bsp::getCRCCalculator()
    {
        static BspCRC s_crcCalculator {};
        return s_crcCalculator;
    }

    void Bsp::holdSTM32PowerOn()
    {
        setZerPwrHold(k_high);
    }

    void Bsp::releaseSTM32PowerHold()
    {
        setZerPwrHold(k_low);
    }

    void Bsp::setZerPwrHold(bool state)
    {
        setGpioState(Gpio::ZerPwrHold, state);
    }

    void Bsp::directLedDrive(bool state)
    {
        setGpioState(Gpio::DirectLedDrive, state);
    }

    void Bsp::setEPUPowerOn()
    {
    	setGpioState(Gpio::EpuPowerEn, true);
    }

    void Bsp::setEPUPowerOff()
    {
    	setGpioState(Gpio::EpuPowerEn, false);
    }

    void Bsp::holdEPUPowerButton()
    {
    	setGpioState(Gpio::PwrButtonEn, true);
    }

    void Bsp::releaseEPUPowerButton()
    {
    	setGpioState(Gpio::PwrButtonEn, false);
    }

    bool Bsp::externalShutdownRequested()
    {
    	return getGpioState(Gpio::ExtShutdown);
    }

    bool Bsp::EPUIsPowered()
    {
    	return getGpioState(Gpio::EpuOn);
    }
}
