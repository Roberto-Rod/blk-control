#include "stm32l0xx_hal.h"

#include "Application/inc/BspInterfacesImpl.hpp"

#include "Core/Inc/main.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
// Mutex protects the access to the data memory transmitted over the I2C interface
extern osMutexId i2cMutexHandle;

extern CRC_HandleTypeDef hcrc;
extern osMutexId crcMutexHandle;

namespace mercury::blackstar
{
    // I2CTransmitInterface
    bool BspI2CTransmit::transmit(uint16_t devAddress, uint8_t *pData, uint16_t size, uint32_t timeout_ms)
    {
        HAL_StatusTypeDef status { HAL_I2C_Master_Transmit(&hi2c1, (devAddress << 1u), pData, size, timeout_ms) };
        return (status == HAL_OK);
    }

    // I2CReceiveInterface
    bool BspI2CReceive::receive(uint16_t devAddress, uint8_t *pData, uint16_t size, uint32_t timeout_ms)
    {
        HAL_StatusTypeDef status { HAL_I2C_Master_Receive(&hi2c1, (devAddress << 1u), pData, size, timeout_ms) };
        return (status == HAL_OK);
    }

    // OSLockInterface
    void BspI2COSLock::lock()
    {
        (void)osMutexAcquire(i2cMutexHandle, osWaitForever);
    }

    void BspI2COSLock::unlock()
    {
        (void)osMutexRelease(i2cMutexHandle);
    }

    void BspCRCOSLock::lock()
    {
        (void)osMutexAcquire(crcMutexHandle, osWaitForever);
    }

    void BspCRCOSLock::unlock()
    {
        (void)osMutexRelease(crcMutexHandle);
    }

    // OSDelayInterface
    void BspOSDelay::delay(const uint32_t delay_ms)
    {
        (void)osDelay(delay_ms);
    }

    // CRCInterface
    void BspCRC::reset()
    {
        HAL_CRC_Init(&hcrc);
        while (HAL_CRC_GetState(&hcrc) != HAL_CRC_STATE_READY) { }
    }

    uint32_t BspCRC::accumulate(uint32_t *pBuffer, uint32_t bufferLength)
    {
        return HAL_CRC_Accumulate(&hcrc, pBuffer, bufferLength);
    }
}

