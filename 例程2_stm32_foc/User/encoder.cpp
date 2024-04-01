#include "encoder.h"
#include "spi.h"

float angleLast = 0;
uint64_t angleTimestamp = 0;
float velocityLast = 0;
uint64_t velocityTimestamp = 0;
int32_t rotationCount = 0;
int32_t rotationCountLast = 0;

const static uint8_t RESOLUTION_BITS = 14;      // 分辨率
const static uint16_t COUNT_PER_ROTATE = 16384; // 14位  16384
const static uint16_t ANGLE_REG = 0x3FFF;       // spi存放数据地址
const static uint8_t COMMAND_PARITY_BIT = 15;   // 奇偶检验位
const static uint8_t COMMAND_RW_BIT = 14;       // 命令RW位
const static uint8_t DATA_START_BIT = 13;       // 数据开始位
const static uint16_t DATA_MASK = 0xFFFF >> (16 - RESOLUTION_BITS);

uint16_t command;
uint16_t registerVal;

SPI_HandleTypeDef *spiHandle = &hspi1;
Direction countDirection = Direction::CW;

// // 取绝对值
// inline float abs(float _v)
// {
//     return _v >= 0 ? _v : -_v;
// }

void Update()
{
    float angle = GetRawAngle();
    angleTimestamp = micros(); // 获取运行时间(ms)

    float deltaAngle = angle - angleLast;
    // If overflow happened track it as full rotation
    if (fabs(deltaAngle) > (0.8f * _2PI))
        rotationCount += (deltaAngle > 0) ? -1 : 1;

    angleLast = angle;
}

float GetVelocity()
{
    float time = (float)(angleTimestamp - velocityTimestamp) * 1e-6f;
    // Quick fix for strange cases (micros overflow)
    if (time <= 0)
        time = 1e-3f;

    // velocity calculation
    float vel = ((float)(rotationCount - rotationCountLast) * _2PI + (angleLast - velocityLast)) / time;

    // save variables for future pass
    velocityLast = angleLast;
    rotationCountLast = rotationCount;
    velocityTimestamp = angleTimestamp;

    return vel;
}

void VarInit()
{
    // Initialize all the internal variables of EncoderBase
    // to ensure a "smooth" startup (without a 'jump' from zero)
    GetRawAngle();
    delayMicroSeconds(1);

    velocityLast = GetRawAngle();
    velocityTimestamp = micros();
    HAL_Delay(1);

    GetRawAngle();
    delayMicroSeconds(1);

    angleLast = GetRawAngle();
    angleTimestamp = micros();
}

uint8_t SpiCalcEvenParity(uint16_t value)
{
    uint8_t cnt = 0;
    uint8_t i;

    for (i = 0; i < 16; i++)
    {
        if (value & 0x1) // 非0 cnt++
            cnt++;

        value >>= 1;
    }

    return cnt & 0x1;
}

void Encoder_Init()
{
    VarInit();

    command = ANGLE_REG | (1 << COMMAND_RW_BIT); // 0x43FF
    // Add a parity bit on the MSB
    command |= ((uint16_t)SpiCalcEvenParity(command) << COMMAND_PARITY_BIT); // 把15位 置0？
}

uint16_t SpiTransmitAndRead16Bits(uint16_t _dataTx)
{
    uint16_t dataRx;

    GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16U; // Chip select

#if 0
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &_dataTx, (uint8_t*) &dataRx, 1, HAL_MAX_DELAY);
#else
    /* Set the transaction information */
    spiHandle->pRxBuffPtr = (uint8_t *)(&dataRx);
    spiHandle->RxXferCount = 1;
    spiHandle->pTxBuffPtr = (uint8_t *)(&_dataTx);
    spiHandle->TxXferCount = 1;
    bool txAvailable = true;

    /* Check if the SPI is already enabled */
    if ((spiHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
        __HAL_SPI_ENABLE(spiHandle);

    /* Transmit and Receive data in 16 Bit mode */
    spiHandle->Instance->DR = *((uint16_t *)spiHandle->pTxBuffPtr);
    spiHandle->pTxBuffPtr += sizeof(uint16_t);
    spiHandle->TxXferCount--;

    while ((spiHandle->TxXferCount > 0U) || (spiHandle->RxXferCount > 0U))
    {
        /* Check TXE flag */
        if ((__HAL_SPI_GET_FLAG(spiHandle, SPI_FLAG_TXE)) &&
            (spiHandle->TxXferCount > 0U) &&
            (txAvailable == 1U))
        {
            spiHandle->Instance->DR = *((uint16_t *)spiHandle->pTxBuffPtr);
            spiHandle->pTxBuffPtr += sizeof(uint16_t);
            spiHandle->TxXferCount--;
            /* Next Data is a reception (Rx). Tx not allowed */
            txAvailable = false;
        }
        /* Check RXNE flag */
        if ((__HAL_SPI_GET_FLAG(spiHandle, SPI_FLAG_RXNE)) && (spiHandle->RxXferCount > 0U))
        {
            *((uint16_t *)spiHandle->pRxBuffPtr) = (uint16_t)spiHandle->Instance->DR;
            spiHandle->pRxBuffPtr += sizeof(uint16_t);
            spiHandle->RxXferCount--;
            /* Next Data is a Transmission (Tx). Tx is allowed */
            txAvailable = true;
        }
    }
#endif

    GPIOA->BSRR = GPIO_PIN_15;

    return dataRx;
}

uint16_t GetRawData()
{
    SpiTransmitAndRead16Bits(command);

    // 350ns is the required time for AMS sensors, 80ns for MA730, MA702
    // Need to be tuned for different CPU
    uint8_t _us = 10;
    while (_us--)
        __NOP();

    // This should shift data to the rightmost bits of the word
    registerVal = SpiTransmitAndRead16Bits(0x00) >> (1 + DATA_START_BIT - RESOLUTION_BITS); // 1+13-14

    // Return the data, stripping the non data (e.g. parity) bits
    return registerVal & DATA_MASK;
}

// 获取当前角度，0到6.28
float GetRawAngle()
{
    return ((float)GetRawData() / (float)COUNT_PER_ROTATE) * _2PI;
}

float GetFullAngle()
{
    return (float)rotationCount * _2PI + angleLast;
}
